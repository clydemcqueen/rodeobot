#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, UInt8MultiArray

g_set_ascii_pub = None
g_voltage = '00'
g_charge_ratio = '00'

# Voltage updated.
def battery_voltage_callback(msg):
  global g_voltage
  voltage = float_to_string(msg.data)
  if voltage != g_voltage:
    g_voltage = voltage
    update_display()

# Charge ratio updated.
def battery_charge_ratio_callback(msg):
  global g_charge_ratio
  charge_ratio = float_to_string(msg.data * 100)
  if charge_ratio != g_charge_ratio:
    g_charge_ratio = charge_ratio
    update_display()

# Convert 2 digit float into a string.
def float_to_string(f):
  if f < 0.0:
    return '00'
  elif f > 99.9:
    return '99'
  else:
    tens = int(f / 10)
    return str(tens) + (str(int(f - tens * 10)))

# Update the display. 2 digits for voltage, 2 for charge ratio.
def update_display():
  print 'updating display: {} {}'.format(g_voltage, g_charge_ratio)
  display = UInt8MultiArray()
  display.data = g_voltage + g_charge_ratio
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
