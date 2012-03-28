#!/usr/bin/env python

import roslib; roslib.load_manifest('cob_hwmonitor')
import rospy
import sqlite3
from serial import *
from cob_hwmonitor.msg import hw_msg
import random

						  # initialize message
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
						  # initialize database
db = sqlite3.connect('/home/uhr-eh/git/care-o-bot/cob_apps/cob_hwmonitor/db/hwmonitor.db')
cursor = db.cursor()
cursor.execute('CREATE TABLE IF NOT EXISTS "hwmonitor" ("monitoring_point" ,"current_value" INTEGER,"minimum_value" INTEGER,"maximum_value" INTEGER)')

						  # initialize serial port

#s = Serial(port="/dev/ttyUSB0",baudrate=230400, bytesize=EIGHTBITS, parity=PARITY_NONE, stopbits=STOPBITS_ONE, timeout=3)
#s.open()

						  # sending dummy messages

#send_buff_array=[0xFF,0x0E,0x00,0x00]
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
#send_buff_array=[0x88,0x0E,0x00,0x00]
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
      for l in range(9):				# loop specifier
	for k in range(8):					# loop kanal
	
	  #send_buff_array=[k,l,0,0]
	  #message = ""
	  #for i in send_buff_array:
	  #    message += chr(i)
	  #s.write(message)
	  #buff = s.read(6)
	  #read_buff_array = []
	  #for i in buff:
	  #    read_buff_array.append(ord(i))
	      
	  ##read_message.data = read_buff_array	
	  voltage = 24 + random.randrange(-6,6)
	  current = 5 + random.randrange(-4,4)
	  temp = 220 + random.randrange(-40,40)
	  
	  if l == 0:
	    if k == 0:
	      read_message.temp_1_curr = temp#read_buff_array
	    elif k == 1:
	      read_message.temp_2_curr = temp#read_buff_array
	    elif k == 2:
	      read_message.temp_3_curr = temp#read_buff_array
	    elif k == 3:
	      read_message.temp_4_curr = temp#read_buff_array
	    elif k == 4:
	      read_message.temp_5_curr = temp#read_buff_array
	    elif k == 5:
	      read_message.temp_6_curr = temp#read_buff_array
	  elif l == 1:
	    if k == 0:
	      read_message.temp_1_min = 200#read_buff_array
	    elif k == 1:
	      read_message.temp_2_min = 200#read_buff_array
	    elif k == 2:
	      read_message.temp_3_min = 200#read_buff_array
	    elif k == 3:
	      read_message.temp_4_min = 200#read_buff_array
	    elif k == 4:
	      read_message.temp_5_min = 200#read_buff_array
	    elif k == 5:
	      read_message.temp_6_min = 200#read_buff_array
	  elif l == 2:
	    if k == 0:
	      read_message.temp_1_max = 240#read_buff_array
	    elif k == 1:
	      read_message.temp_2_max = 240#read_buff_array
	    elif k == 2:
	      read_message.temp_3_max = 240#read_buff_array
	    elif k == 3:
	      read_message.temp_4_max = 240#read_buff_array
	    elif k == 4:
	      read_message.temp_5_max = 240#read_buff_array
	    elif k == 5:
	      read_message.temp_6_max = 240#read_buff_array
	  elif l == 3:
	    if k == 0:
	      read_message.akku_voltage_curr = voltage#read_buff_array
	    elif k == 1:
	      read_message.hals_motor_voltage_curr = voltage#read_buff_array
	    elif k == 2:
	      read_message.hals_logik_voltage_curr = voltage#read_buff_array
	    elif k == 3:
	      read_message.tablett_logik_voltage_curr = voltage#read_buff_array
	    elif k == 6:
	      read_message.arm_logik_voltage_curr = voltage#read_buff_array
	    elif k == 7:
	      read_message.tablett_motor_voltage_curr = voltage#read_buff_array
	  elif l == 4:
	    if k == 0:
	      read_message.akku_voltage_min = 20#read_buff_array
	    elif k == 1:
	      read_message.hals_motor_voltage_min = 20#read_buff_array
	    elif k == 2:
	      read_message.hals_logik_voltage_min = 20#read_buff_array
	    elif k == 3:
	      read_message.tablett_logik_voltage_min = 20#read_buff_array
	    elif k == 6:
	      read_message.arm_logik_voltage_min = 20#read_buff_array
	    elif k == 7:
	      read_message.tablett_motor_voltage_min = 20#read_buff_array
	  elif l == 5:
	    if k == 0:
	      read_message.akku_voltage_max = 28#read_buff_array3
	    elif k == 1:
	      read_message.hals_motor_voltage_max = 28#read_buff_array3
	    elif k == 2:
	      read_message.hals_logik_voltage_max = 28#read_buff_array3
	    elif k == 3:
	      read_message.tablett_logik_voltage_max = 28#read_buff_array3
	    elif k == 6:
	      read_message.arm_logik_voltage_max = 28#read_buff_array3
	    elif k == 7:
	      read_message.tablett_motor_voltage_max = 28#read_buff_array3
	  elif l == 6:
	    if k == 1:
	      read_message.hals_motor_current_curr = current#read_buff_array
	    elif k == 2:
	      read_message.hals_logik_current_curr = current#read_buff_array
	    elif k == 3:
	      read_message.tablett_logik_current_curr = current#read_buff_array
	    elif k == 6:
	      read_message.arm_logik_current_curr = current#read_buff_array
	    elif k == 7:
	      read_message.tablett_motor_current_curr = current#read_buff_array
	  elif l == 7:
	    if k == 1:
	      read_message.hals_motor_current_min = 2#read_buff_array2
	    elif k == 2:
	      read_message.hals_logik_current_min = 2#read_buff_array2
	    elif k == 3:
	      read_message.tablett_logik_current_min = 2#read_buff_array2
	    elif k == 6:
	      read_message.arm_logik_current_min = 2#read_buff_array2
	    elif k == 7:
	      read_message.tablett_motor_current_min = 2#read_buff_array2
	  elif l == 8:
	    if k == 1:
	      read_message.hals_motor_current_max = 8#read_buff_array3
	    elif k == 2:
	      read_message.hals_logik_current_max = 8#read_buff_array3
	    elif k == 3:
	      read_message.tablett_logik_current_max = 8#read_buff_array3
	    elif k == 6:
	      read_message.arm_logik_current_max = 8#read_buff_array3
	    elif k == 7:
	      read_message.tablett_motor_current_max = 8#read_buff_array3
      
      cursor.execute('DELETE from hwmonitor')
      for t in [("Sensor 1" , read_message.temp_1_curr ,read_message.temp_1_min , read_message.temp_1_max),
		("Sensor 2" , read_message.temp_2_curr , read_message.temp_2_min , read_message.temp_2_max),
		("Sensor 3" , read_message.temp_3_curr , read_message.temp_3_min , read_message.temp_3_max),
		("Sensor 4" , read_message.temp_4_curr , read_message.temp_4_min , read_message.temp_4_max),
		("Sensor 5" , read_message.temp_5_curr , read_message.temp_5_min , read_message.temp_5_max),
		("Sensor 6" , read_message.temp_6_curr , read_message.temp_6_min , read_message.temp_6_max),
		("Akkuspannung" ,read_message.akku_voltage_curr , read_message.akku_voltage_min , read_message.akku_voltage_max),
		("Hals Motor Spannung" , read_message.hals_motor_voltage_curr , read_message.hals_motor_voltage_min , read_message.hals_motor_voltage_max),
		("Hals Logik Spannung" , read_message.hals_logik_voltage_curr , read_message.hals_logik_voltage_min , read_message.hals_logik_voltage_max),
		("Tablett Logik Spannung" , read_message.tablett_logik_voltage_curr , read_message.tablett_logik_voltage_min , read_message.tablett_logik_voltage_max),
		("Arm Logik Spannung" , read_message.arm_logik_voltage_curr , read_message.arm_logik_voltage_min , read_message.arm_logik_voltage_max),
		("Tablett Motor Spannung" , read_message.tablett_motor_voltage_curr , read_message.tablett_motor_voltage_min , read_message.tablett_motor_voltage_max),
		("Hals Logik Strom" , read_message.hals_logik_current_curr , read_message.hals_logik_current_min , read_message.hals_logik_current_max),
		("Hals Motor Strom" , read_message.hals_motor_current_curr , read_message.hals_motor_current_min , read_message.hals_motor_current_max),
		("Tablett Logik Strom" , read_message.tablett_logik_current_curr , read_message.tablett_logik_current_min , read_message.tablett_logik_current_max),
		("Arm Logik Strom" , read_message.arm_logik_current_curr , read_message.arm_logik_current_min , read_message.arm_logik_current_max),
		("Tablett Motor Strom" ,read_message.tablett_logik_current_curr , read_message.tablett_logik_current_min , read_message.tablett_logik_current_max)
	       ]:
	  cursor.execute('INSERT into hwmonitor values (?,?,?,?)',t)
      db.commit()      
      pub.publish(read_message)
      rospy.sleep(1.0)
	  
	  
if __name__ == '__main__':
    try:
        hwpy()
    except rospy.ROSInterruptException: pass
