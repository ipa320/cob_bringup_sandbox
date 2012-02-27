#!/usr/bin/env python

import roslib; roslib.load_manifest('cob_hwmonitor')
import rospy
from serial import *
from cob_hwmonitor.msg import hw_msg

import random

read_message = hw_msg()

read_message.temp_1_curr = 200
read_message.temp_1_min = 200
read_message.temp_1_max = 200

read_message.temp_2_curr = 200
read_message.temp_2_min = 200
read_message.temp_2_max = 200

read_message.temp_3_curr = 200
read_message.temp_3_min = 200
read_message.temp_3_max = 200

read_message.temp_4_curr = 200
read_message.temp_4_min = 200
read_message.temp_4_max = 200

read_message.temp_5_curr = 200
read_message.temp_5_min = 200
read_message.temp_5_max = 200

read_message.temp_6_curr = 200
read_message.temp_6_min = 200
read_message.temp_6_max = 200


read_message.akku_voltage_curr = 40
read_message.akku_voltage_min = 40 
read_message.akku_voltage_max = 40

read_message.hals_motor_voltage_curr = 24
read_message.hals_motor_voltage_min = 24
read_message.hals_motor_voltage_max = 24

read_message.hals_logik_voltage_curr = 24
read_message.hals_logik_voltage_min = 24
read_message.hals_logik_voltage_max = 24

read_message.tablett_logik_voltage_curr = 24
read_message.tablett_logik_voltage_min = 24
read_message.tablett_logik_voltage_max = 24

read_message.arm_logik_voltage_curr = 24
read_message.arm_logik_voltage_min = 24
read_message.arm_logik_voltage_max = 24

read_message.tablett_motor_voltage_curr = 24
read_message.tablett_motor_voltage_min = 24
read_message.tablett_motor_voltage_max = 24


read_message.hals_motor_current_curr = 0
read_message.hals_motor_current_min = 0
read_message.hals_motor_current_max = 0

read_message.hals_logik_current_curr = 0
read_message.hals_logik_current_min = 0
read_message.hals_logik_current_max = 0

read_message.tablett_logik_current_curr = 0
read_message.tablett_logik_current_min = 0
read_message.tablett_logik_current_max = 0

read_message.arm_logik_current_curr = 0
read_message.arm_logik_current_min = 0
read_message.arm_logik_current_max = 0

read_message.tablett_motor_current_curr = 0
read_message.tablett_motor_current_min = 0
read_message.tablett_motor_current_max = 0

#s = Serial(port="/dev/ttyUSB0",baudrate=230400, bytesize=EIGHTBITS, parity=PARITY_NONE, stopbits=STOPBITS_ONE, timeout=3)
#s.open()
#send_buff_array=[0xFF,0x0E,0x00,0x00]#sending
#message = ""
#for i in send_buff_array:
#	message += chr(i)
#s.write(message)
#buff = s.read(6)
#read_buff_array= []
#for i in buff:
#	read_buff_array.append(ord(i))
#for i in range (len(read_buff_array)):
#	print "%0#x " % read_buff_array[i]
#send_buff_array=[0x88,0x0E,0x00,0x00]#sending
#message = ""
#for i in send_buff_array:
#	message += chr(i)
#s.write(message)
#buff = s.read(6)
#read_buff_array= []
#for i in buff:
#	read_buff_array.append(ord(i))
#for i in range (len(read_buff_array)):
#	print "%0#x " % read_buff_array[i]  
	
	
def hwpy():
    pub = rospy.Publisher('hwmonitor', hw_msg)
    rospy.init_node('hwpy')
    while not rospy.is_shutdown():
      for l in range(9):				# schleife specifier
	for k in range(8):					# schleife kanal
	
	  send_buff_array=[k,l,0,0]
	  message = ""
	  for i in send_buff_array:
	      message += chr(i)
	  s.write(message)
	  buff = s.read(6)
	  read_buff_array = []
	  for i in buff:
	      read_buff_array.append(ord(i))
	      
	  ##read_message.data = read_buff_array	
	  #voltage = 24 + random.randrange(-3,3)
	  #current = 5 + random.randrange(-2,2)
	  #temp = 220 + random.randrange(-40,40)
	  
	  if l == 0:
	    if k == 0:
	      read_message.temp_1_curr = read_buff_array
	    elif k == 1:
	      read_message.temp_2_curr = read_buff_array
	    elif k == 2:
	      read_message.temp_3_curr = read_buff_array
	    elif k == 3:
	      read_message.temp_4_curr = read_buff_array
	    elif k == 4:
	      read_message.temp_5_curr = read_buff_array
	    elif k == 5:
	      read_message.temp_6_curr = read_buff_array
	  elif l == 1:
	    if k == 0:
	      read_message.temp_1_min = read_buff_array
	    elif k == 1:
	      read_message.temp_2_min = read_buff_array
	    elif k == 2:
	      read_message.temp_3_min = read_buff_array
	    elif k == 3:
	      read_message.temp_4_min = read_buff_array
	    elif k == 4:
	      read_message.temp_5_min = read_buff_array
	    elif k == 5:
	      read_message.temp_6_min = read_buff_array
	  elif l == 2:
	    if k == 0:
	      read_message.temp_1_max = read_buff_array
	    elif k == 1:
	      read_message.temp_2_max = read_buff_array
	    elif k == 2:
	      read_message.temp_3_max = read_buff_array
	    elif k == 3:
	      read_message.temp_4_max = read_buff_array
	    elif k == 4:
	      read_message.temp_5_max = read_buff_array
	    elif k == 5:
	      read_message.temp_6_max = read_buff_array
	  elif l == 3:
	    if k == 0:
	      read_message.akku_voltage_curr = read_buff_array
	    elif k == 1:
	      read_message.hals_motor_voltage_curr = read_buff_array
	    elif k == 2:
	      read_message.hals_logik_voltage_curr = read_buff_array
	    elif k == 3:
	      read_message.tablett_logik_voltage_curr = read_buff_array
	    elif k == 6:
	      read_message.arm_logik_voltage_curr = read_buff_array
	    elif k == 7:
	      read_message.tablett_motor_voltage_curr = read_buff_array
	  elif l == 4:
	    if k == 0:
	      read_message.akku_voltage_min = read_buff_array
	    elif k == 1:
	      read_message.hals_motor_voltage_min = read_buff_array
	    elif k == 2:
	      read_message.hals_logik_voltage_min = read_buff_array
	    elif k == 3:
	      read_message.tablett_logik_voltage_min = read_buff_array
	    elif k == 6:
	      read_message.arm_logik_voltage_min = read_buff_array
	    elif k == 7:
	      read_message.tablett_motor_voltage_min = read_buff_array
	  elif l == 5:
	    if k == 0:
	      read_message.akku_voltage_max = read_buff_array
	    elif k == 1:
	      read_message.hals_motor_voltage_max = read_buff_array
	    elif k == 2:
	      read_message.hals_logik_voltage_max = read_buff_array
	    elif k == 3:
	      read_message.tablett_logik_voltage_max = read_buff_array
	    elif k == 6:
	      read_message.arm_logik_voltage_max = read_buff_array
	    elif k == 7:
	      read_message.tablett_motor_voltage_max = read_buff_array
	  elif l == 6:
	    if k == 1:
	      read_message.hals_motor_current_curr = read_buff_array
	    elif k == 2:
	      read_message.hals_logik_current_curr = read_buff_array
	    elif k == 3:
	      read_message.tablett_logik_current_curr = read_buff_array
	    elif k == 6:
	      read_message.arm_logik_current_curr = read_buff_array
	    elif k == 7:
	      read_message.tablett_motor_current_curr = read_buff_array
	  elif l == 7:
	    if k == 1:
	      read_message.hals_motor_current_min = read_buff_array
	    elif k == 2:
	      read_message.hals_logik_current_min = read_buff_array
	    elif k == 3:
	      read_message.tablett_logik_current_min = read_buff_array
	    elif k == 6:
	      read_message.arm_logik_current_min = read_buff_array
	    elif k == 7:
	      read_message.tablett_motor_current_min = read_buff_array
	  elif l == 8:
	    if k == 1:
	      read_message.hals_motor_current_max = read_buff_array
	    elif k == 2:
	      read_message.hals_logik_current_max = read_buff_array
	    elif k == 3:
	      read_message.tablett_logik_current_max = read_buff_array
	    elif k == 6:
	      read_message.arm_logik_current_max = read_buff_array
	    elif k == 7:
	      read_message.tablett_motor_current_max = read_buff_array	
	  
      pub.publish(read_message)
      rospy.sleep(1.0)
	  
	  
if __name__ == '__main__':
    try:
        hwpy()
    except rospy.ROSInterruptException: pass
