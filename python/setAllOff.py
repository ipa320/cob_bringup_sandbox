#!/usr/bin/python


from serial import *


s = Serial(port="/dev/ttyUSB0",baudrate=230400, bytesize=EIGHTBITS, parity=PARITY_NONE, stopbits=STOPBITS_ONE, timeout=3)
print "Set Switch OFF"
s.open()

print "Disable Channel 1"
send_buff_array=[0x80,0x0A,0x00,0x00]#sending
message = ""
for i in send_buff_array:
	message += chr(i)
s.write(message)
buff = s.read(6)
read_buff_array= []
for i in buff:
	read_buff_array.append(ord(i))

for i in range (len(read_buff_array)):
	print "%0#x " % read_buff_array[i]

print "Disable Channel 2"
send_buff_array=[0x81,0x0A,0x00,0x00]#sending
message = ""
for i in send_buff_array:
	message += chr(i)
s.write(message)
buff = s.read(6)
read_buff_array= []
for i in buff:
	read_buff_array.append(ord(i))

for i in range (len(read_buff_array)):
	print "%0#x " % read_buff_array[i]

print "Disable Channel 3"
send_buff_array=[0x82,0x0A,0x00,0x00]#sending
message = ""
for i in send_buff_array:
	message += chr(i)
s.write(message)
buff = s.read(6)
read_buff_array= []
for i in buff:
	read_buff_array.append(ord(i))

for i in range (len(read_buff_array)):
	print "%0#x " % read_buff_array[i]

print "Disable Channel 4"
send_buff_array=[0x83,0x0A,0x00,0x00]#sending
message = ""
for i in send_buff_array:
	message += chr(i)
s.write(message)
buff = s.read(6)
read_buff_array= []
for i in buff:
	read_buff_array.append(ord(i))

for i in range (len(read_buff_array)):
	print "%0#x " % read_buff_array[i]

print "Disable Channel 5"
send_buff_array=[0x84,0x0A,0x00,0x00]#sending
message = ""
for i in send_buff_array:
	message += chr(i)
s.write(message)
buff = s.read(6)
read_buff_array= []
for i in buff:
	read_buff_array.append(ord(i))

for i in range (len(read_buff_array)):
	print "%0#x " % read_buff_array[i]

print "Disable Channel 6"
send_buff_array=[0x85,0x0A,0x00,0x00]#sending
message = ""
for i in send_buff_array:
	message += chr(i)
s.write(message)
buff = s.read(6)
read_buff_array= []
for i in buff:
	read_buff_array.append(ord(i))

for i in range (len(read_buff_array)):
	print "%0#x " % read_buff_array[i]

print "Disable Channel 7"
send_buff_array=[0x86,0x0A,0x00,0x00]#sending
message = ""
for i in send_buff_array:
	message += chr(i)
s.write(message)
buff = s.read(6)
read_buff_array= []
for i in buff:
	read_buff_array.append(ord(i))

for i in range (len(read_buff_array)):
	print "%0#x " % read_buff_array[i]

print "Disable Channel 8"
send_buff_array=[0x87,0x0A,0x00,0x00]#sending
message = ""
for i in send_buff_array:
	message += chr(i)
s.write(message)
buff = s.read(6)
read_buff_array= []
for i in buff:
	read_buff_array.append(ord(i))

for i in range (len(read_buff_array)):
	print "%0#x " % read_buff_array[i]