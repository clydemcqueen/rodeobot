#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, UInt8MultiArray

g_set_ascii_pub = None
g_voltage = None
g_charge_ratio = None

# Voltage updated.
def battery_voltage_callback(msg):
  g_voltage = msg.data

# Charge ratio updated. Update the display.
def battery_charge_ratio_callback(msg):
  g_charge_ratio = msg.data
  update_display()

# Convert 2 digit float into a pair of ints.
def float_to_decimal(f):
  if f < 0.0:
    return (0, 0)
  elif f > 99.9:
    return (9, 9)
  else
    tens = int(f / 10)
    return (tens, int(f - tens * 10))

# Update the display. 2 digits for voltage, 2 for charge ratio.
def update_display():
  display = UInt8MultiArray()
  vd = float_to_decimal(g_voltage)
  cd = float_to_decimal(g_charge_ratio)
  display.data[0] = vd[0] + 48
  display.data[1] = vd[1] + 48
  display.data[2] = cd[0] + 48
  display.data[3] = cd[1] + 48
  g_set_ascii_pub.publish(display)

if __name__ == '__main__':
  rospy.init_node('battery_monitor', anonymous=True)

  # Set up publishers.
  g_set_ascii_pub = rospy.Publisher('set_ascii', UInt8MultiArray, queue_size=1)

  # Set up subscribers.
  rospy.Subscriber('battery/voltage', Float32, battery_voltage_callback)
  rospy.Subscriber('battery/charge_ratio', Float32, battery_charge_ratio_callback)
  
  # Hand control to rospy.
  rospy.spin()
