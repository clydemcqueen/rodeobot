#include <rodeobot/w3.h>
#include <angles/angles.h>

namespace rodeobot {

//===========================================================================
// Globals
//===========================================================================

static LightSensor g_sensor_right         {50, -68.0/180 * M_PI, -58.0/180 * M_PI};
static LightSensor g_sensor_front_right   {50, -50.0/180 * M_PI, -40.0/180 * M_PI};
static LightSensor g_sensor_center_right  {50, -16.0/180 * M_PI,  -6.0/180 * M_PI};
static LightSensor g_sensor_center_left   {50,  -5.0/180 * M_PI,   5.0/180 * M_PI};
static LightSensor g_sensor_front_left    {50,  14.0/180 * M_PI,  24.0/180 * M_PI};
static LightSensor g_sensor_left          {50,  33.0/180 * M_PI,  43.0/180 * M_PI};

//===========================================================================
// Utilities
//===========================================================================

// Get a parameter from the parameter server, but clip to min & max.
void getParam(const ros::NodeHandle &nh, const std::string &name, double &val, double min, double max)
{
  nh.getParam(name, val);
  if (val < min)
  {
    ROS_WARN("param %s val %f is below min %f", name.c_str(), val, min);
    val = min;
  }
  if (val > max)
  {
    ROS_WARN("param %s val %f is above max %f", name.c_str(), val, max);
    val = max;
  }
}

// Return a smooth velocity. Works for +/foward/CCW and -/backward/CW.
double smooth(double goal, double v, double accel, double min, double max)
{
  assert(min > 0.0);
  assert(max > 0.0);
  assert(max > min);

  accel /= SPIN_RATE;

  if (std::abs(v - goal) < accel)
  {
    // Close enough -- pin to goal
    v = goal;
  }
  else
  {
    // Accelerate / decelerate
    if (v < goal)
      v += accel;
    else if (v > goal)
      v -= accel;
  }

  // Clip to max
  if (std::abs(v) > max)
    v = (v > 0.0) ? max : -max;

  // Clip to min
  if (std::abs(v) < min)
    v = 0.0;

  // ROS_INFO("goal %f, accel %f, min %f, max %f, v %f", goal, accel, min, max, v);
  return v;
}

// Return true if we hit something.
bool isCollision(const ca_msgs::BumperConstPtr &msg)
{
  if (msg->is_left_pressed)
  {
    ROS_WARN("[WANDER] Hit left bumper");
    return true;
  }

  if (msg->is_right_pressed)
  {
    ROS_WARN("[WANDER] Hit right bumper");
    return true;
  }

  return false;
}

// Return true if we almost hit something.
bool isTooClose(const ca_msgs::BumperConstPtr &msg)
{
  if (msg->is_light_left)
  {
    ROS_WARN("[WANDER] Too close: left light bumper");
    return true;
  }

  if (msg->is_light_front_left)
  {
    ROS_WARN("[WANDER] Too close: front left light bumper");
    return true;
  }

  if (msg->is_light_center_left)
  {
    ROS_WARN("[WANDER] Too close: center left light bumper");
    return true;
  }

  if (msg->is_light_right)
  {
    ROS_WARN("[WANDER] Too close: right light bumper");
    return true;
  }

  if (msg->is_light_front_right)
  {
    ROS_WARN("[WANDER] Too close: front right light bumper");
    return true;
  }

  if (msg->is_light_center_right)
  {
    ROS_WARN("[WANDER] Too close: center right light bumper");
    return true;
  }

  return false;
}

//===========================================================================
// Laser scan utilities
//===========================================================================

// Find the widest max/Inf/NaN arc.
void find(sensor_msgs::LaserScan &scan, float max, int &start, int &width)
{
  width = 0;

  // -1 means "not an arc"
  start = -1;
  int current_start = -1;

  for (int i = 0; i < scan.ranges.size() + 1; ++i) // Note: size()+1 iterations
  {
    if ((i < scan.ranges.size()) && (std::isinf(scan.ranges[i]) || std::isnan(scan.ranges[i]) || scan.ranges[i] > max))
    {
      // In an arc
      if (current_start < 0)
      {
        // Start a new arc
        current_start = i;
      }
    }
    else 
    {
      // Just past the end, or not in an arc
      if (current_start >= 0)
      {
        // Examine the arc that just ended
        if (i - current_start > width)
        {
          // Best arc so far
          start = current_start;
          width = i - current_start;
        }

        // End the arc
        current_start = -1;
      }
    }
  }
}

// Draw an arc so we can see it in rviz.
void draw(sensor_msgs::LaserScan &scan, float max, int start, int width)
{
  if (width <= 0)
    return;

  for (int i = 0; i < scan.ranges.size(); ++i)
    scan.ranges[i] = (i >= start && i < start + width) ?  max : NAN;
}

// Draw an arc so we can see it in rviz. Scan must be initialized!
void draw(sensor_msgs::LaserScan &scan, float max, LightSensor &sensor)
{
  int start = (sensor.start - scan.angle_min) / scan.angle_increment;
  int end = (sensor.end - scan.angle_min) / scan.angle_increment;

  for (int i = 0; i < scan.ranges.size(); ++i)
    if (i >= start && i < end)
      scan.ranges[i] = max;
}

//===========================================================================
// Wander
//===========================================================================

// TODO make enum
#define READY_SONG 0
#define GO_SONG 1
#define STOP_SONG 2
#define ERROR_SONG 3

Wander::Wander(ros::NodeHandle &nh, tf::TransformListener &tf):
  create_base_{true},
  nh_{nh},
  tf_{tf},
  state_{State::start},
  base_awake_{false},
  laser_awake_{false},
  x_prev_v_{0.0},
  r_prev_v_{0.0}
{
  nh.getParam("create_base", create_base_);
  getParam(nh, "x_min_v", x_min_v_, 0.0, 0.5);
  getParam(nh, "x_max_v", x_max_v_, 0.3, 5.0);
  getParam(nh, "x_accel", x_accel_, 0.5, 5.0);
  getParam(nh, "r_min_v", r_min_v_, 0.0, 0.5);
  getParam(nh, "r_max_v", r_max_v_, 0.4, 5.0);
  getParam(nh, "r_accel", r_accel_, 0.5, 5.0);
  getParam(nh, "r_epsilon", r_epsilon_, 0.01, r_accel_ / 10 - 0.001);
  getParam(nh, "scan_horizon", scan_horizon_, 0.5, 10.0);
  getParam(nh, "scan_width", scan_width_, 0.1, 1.0);

  // Set up publishers
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  laser1_pub_ = nh.advertise<sensor_msgs::LaserScan>("/scan1", 1);
  laser2_pub_ = nh.advertise<sensor_msgs::LaserScan>("/scan2", 1);
  if (create_base_)
  {
    define_song_pub_ = nh.advertise<ca_msgs::DefineSong>("/define_song", 5); // Queue = 5
    play_song_pub_ = nh.advertise<ca_msgs::PlaySong>("/play_song", 5); // Queue = 5
  }

  // Subscribe to camera (laser) messages
  laser_sub_ = nh.subscribe("/scan", 1, &Wander::laserCallback, this);

  // Subscribe to Create2 base messages
  if (create_base_)
  {
    bumper_sub_ = nh.subscribe("/bumper", 1, &Wander::bumperCallback, this);
    day_button_sub_ = nh.subscribe("/day_button", 1, &Wander::dayButtonCallback, this);
    minute_button_sub_ = nh.subscribe("/minute_button", 1, &Wander::minuteButtonCallback, this);
    mode_sub_ = nh.subscribe("/mode", 1, &Wander::modeCallback, this);
    wheeldrop_sub_ = nh.subscribe("/wheeldrop", 1, &Wander::wheeldropCallback, this);
  }
}

void Wander::laserCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
  // Don't block -- just skip this reading
  if (scan_mutex_.try_lock())
  {
    scan_ = *msg;
    scan_mutex_.unlock();
    laser_awake_ = true;

    // If we're running on a turtlebot base, use this as a sign that the base is awake, too.
    if(!create_base_)
      base_awake_ = true;
  }
}

void Wander::bumperCallback(const ca_msgs::BumperConstPtr &msg)
{
  close_right_        = msg->light_signal_right        > g_sensor_right.threshold;
  close_front_right_  = msg->light_signal_front_right  > g_sensor_front_right.threshold;
  close_center_right_ = msg->light_signal_center_right > g_sensor_center_right.threshold;
  close_center_left_  = msg->light_signal_center_left  > g_sensor_center_left.threshold;
  close_front_left_   = msg->light_signal_front_left   > g_sensor_front_left.threshold;
  close_left_         = msg->light_signal_left         > g_sensor_left.threshold;

  // printf("L:%d FL:%d CL:%d R:%d FR:%d CR:%d\r",
  //    close_left_, close_front_left_, close_center_left_,
  //    close_center_right_, close_front_right_, close_right_);

  //if (state_ != State::pause && (isCollision(msg) || isTooClose(msg)))
  if (state_ != State::pause && isCollision(msg))
  {
    emergencyStop();
  }
}

void Wander::wheeldropCallback(const std_msgs::EmptyConstPtr &msg)
{
  if (state_ != State::pause)
  {
    ROS_WARN("[WANDER] Wheel drop");
    emergencyStop();
  }
}

void Wander::dayButtonCallback(const std_msgs::EmptyConstPtr &msg)
{
  if (state_ != State::pause)
  {
    ROS_INFO("[WANDER] Day button: stop");
    playSong(STOP_SONG);
    state_ = State::pause;
    x_prev_v_ = 0.0; // Need this so we don't re-start too fast TODO possibly refactor
    r_prev_v_ = 0.0;
  }
}

void Wander::minuteButtonCallback(const std_msgs::EmptyConstPtr &msg)
{
  if (state_ == State::pause)
  {
    ROS_INFO("[WANDER] Minute button: go");
    playSong(GO_SONG);
    state_ = State::drive;
  }
}

#define C3 60
#define D3 62
#define E3 64
#define F3 65
#define G3 67
#define A3 69
#define B3 71
#define C4 72

// defineSong(TEST, {{36, 0.25}, {38, 0.25}, {40, 0.25}, {41, 0.25}, {43, 0.25}, {45, 0.25}, {47, 0.25}, {48, 0.25}});
// defineSong(TEST, {{48, 0.25}, {50, 0.25}, {52, 0.25}, {53, 0.25}, {55, 0.25}, {57, 0.25}, {59, 0.25}, {60, 0.25}});
// defineSong(TEST, {{72, 0.25}, {74, 0.25}, {76, 0.25}, {77, 0.25}, {79, 0.25}, {81, 0.25}, {83, 0.25}, {84, 0.25}});
// defineSong(TEST, {{84, 0.25}, {86, 0.25}, {88, 0.25}, {89, 0.25}, {91, 0.25}, {93, 0.25}, {95, 0.25}, {96, 0.25}});
// defineSong(TEST, {{60, 0.25}, {62, 0.25}, {64, 0.25}, {65, 0.25}, {67, 0.25}, {69, 0.25}, {71, 0.25}, {72, 0.25}});

void Wander::modeCallback(const ca_msgs::ModeConstPtr &msg)
{
  if (!base_awake_)
  {
    // Define our songs.
    defineSong(READY_SONG, {{C3, 0.25}, {E3, 0.25}, {G3, 0.25}});
    defineSong(GO_SONG,    {{E3, 0.25}, {G3, 0.25}});
    defineSong(STOP_SONG,  {{G3, 0.25}, {E3, 0.25}});
    defineSong(ERROR_SONG, {{G3, 0.25}, {E3, 0.25}, {C3, 0.25}});

    base_awake_ = true;
  }
}

void Wander::defineSong(int num, Song song)
{
  ROS_INFO("Define song %d", num);
  ca_msgs::DefineSong msg;
  msg.song = num;
  msg.length = song.size();
  for (int i = 0; i < song.size(); ++i)
  {
    msg.notes.push_back(song[i].note);
    msg.durations.push_back(song[i].duration);
  }
  define_song_pub_.publish(msg);
}

void Wander::playSong(int num)
{
  if (create_base_)
  {
    ROS_INFO("Play song %d", num);
    ca_msgs::PlaySong msg;
    msg.song = num;
    play_song_pub_.publish(msg);
  }
}

void Wander::emergencyStop()
{
  playSong(ERROR_SONG);

  // Stop quickly
  geometry_msgs::Twist msg;
  cmd_vel_pub_.publish(msg);

  state_ = State::pause;
  x_prev_v_ = 0.0;
  r_prev_v_ = 0.0;
}

#define RANGES_SIZE 100

void Wander::spinOnce(const ros::TimerEvent &event)
{
  if ((event.current_real - event.current_expected).toSec() > 2.0 / SPIN_RATE)
    ROS_WARN("This event is %f clock seconds late", (event.current_real - event.current_expected).toSec());

  if (event.profile.last_duration.toSec() > 1.0 / SPIN_RATE)
    ROS_WARN("Last event took %f wall seconds too long", event.profile.last_duration.toSec() - 1.0 / SPIN_RATE);

  if (state_ == State::start && base_awake_ && laser_awake_)
  {
    ROS_INFO("W3 awake!");
    playSong(READY_SONG);
    if (create_base_)
    {
      state_ = State::pause; // Wait for a human to hit the "go" button
    }
    else
    {
      state_ = State::drive; // Start driving
    }
  }

  if (state_ == State::start)
    return;

  // Lock the scan.
  scan_mutex_.lock();

  // Find the widest arc.
  int arc_start, arc_width;
  find(scan_, scan_horizon_, arc_start, arc_width);

  // Draw the arc for rviz.
  sensor_msgs::LaserScan scan1 = scan_;
  draw(scan1, scan_horizon_, arc_start, arc_width);
  laser1_pub_.publish(scan1);

  if (create_base_)
  {
    // Draw the IR signals for rviz.
    sensor_msgs::LaserScan scan2;
    scan2.header.frame_id = scan_.header.frame_id;
    scan2.angle_min = g_sensor_right.start;
    scan2.angle_max = g_sensor_left.end;
    scan2.angle_increment = (g_sensor_left.end - g_sensor_right.start) / RANGES_SIZE;
    scan2.time_increment = 0.0;
    scan2.range_min = 0.1;
    scan2.range_max = 10.0;
    scan2.ranges.assign(RANGES_SIZE, NAN);
    if (close_right_) 
      draw(scan2, 0.25, g_sensor_right);
    if (close_front_right_) 
      draw(scan2, 0.25, g_sensor_front_right);
    if (close_center_right_) 
      draw(scan2, 0.25, g_sensor_center_right);
    if (close_center_left_) 
      draw(scan2, 0.25, g_sensor_center_left);
    if (close_front_left_) 
      draw(scan2, 0.25, g_sensor_front_left);
    if (close_left_) 
      draw(scan2, 0.25, g_sensor_left);
    laser2_pub_.publish(scan2);
  }

  // If we're paused, we're done.
  if (state_ == State::pause)
  {
    scan_mutex_.unlock(); // TODO switch to a diff't mutex mech to increase robustness
    return;
  }

  // Calculate our motion.
  double x_target_v = 0.0;
  double r_target_v = 0.0;
  if (state_ == State::drive)
    driveMotion(arc_start, arc_width, x_target_v, r_target_v);
  else
    recoverMotion(arc_start, arc_width, x_target_v, r_target_v);

  // Smooth the motion.
  x_prev_v_ = smooth(x_target_v, x_prev_v_, x_accel_, x_min_v_, x_max_v_);
  r_prev_v_ = smooth(r_target_v, r_prev_v_, r_accel_, r_min_v_, r_max_v_);

  // Publish a velocity command.
  geometry_msgs::Twist twist;
  twist.linear.x = x_prev_v_;
  twist.angular.z = r_prev_v_;
  cmd_vel_pub_.publish(twist);

  // Unlock the scan.
  scan_mutex_.unlock();
}

// TODO inline?
bool Wander::isClose()
{
  return close_right_ || close_front_right_ || close_center_right_ || close_center_left_ || close_front_left_ || close_left_;
}

void Wander::driveMotion(int arc_start, int arc_width, double &x_target_v, double &r_target_v)
{
  if (isClose())
  {
    ROS_INFO("Too close, recover");
    state_ = State::recover;
    return;
  }
  
  double arc_d = std::sin(arc_width / 2 * scan_.angle_increment) * scan_horizon_ * 2;
  if (arc_d < scan_width_)
  {
    ROS_INFO("Arc too small %f vs. %f, recover", arc_d, scan_width_);
    state_ = State::recover;
    return;
  }

  // Calculate goal yaw.
  double goal_yaw = scan_.angle_min + scan_.angle_increment * (arc_start + arc_width / 2);
  // ROS_INFO("Goal yaw %f", goal_yaw);

  // Calculate target velocity.
  x_target_v = x_max_v_;
  r_target_v = 0.0;
  if (goal_yaw > r_epsilon_)
    r_target_v = r_max_v_;
  else if (goal_yaw < -r_epsilon_)
    r_target_v = -r_max_v_;
}

void Wander::recoverMotion(int arc_start, int arc_width, double &x_target_v, double &r_target_v)
{
  // TODO if we've been in recovery too long, go to pause
  
  if (arc_width == scan_.ranges.size() && !isClose())
  {
    ROS_INFO("Full arc and IR sensors are clear, drive");
    dir_ = Direction::none;
    state_ = State::drive;
    return;
  }

  // Pick a direction.
  if (dir_ == Direction::none)
  {
    dir_ = (rand() % 2 > 0) ? Direction::cw : Direction::ccw;
    ROS_INFO("Turning %s", dir_ == Direction::cw ? "clockwise" : "counterclockwise");
  }

  // Calculate angular velocity. Leave linear velocity at 0.0.
  r_target_v = (dir_ == Direction::ccw) ? r_max_v_ : -r_max_v_;
}

} // namespace rodeobot
