import serial, time

# send commands to device, format: char {0-255 (optional)} {0-255 (optional)}
# press ENTER to see buffered output

# change port number to match device
ser = serial.Serial('/dev/ttyUSB0', 115200)
time.sleep(2)

while 1:
	while ser.inWaiting() > 0:
		line = ser.readline().strip()
		print(line)

	str = raw_input('> ').split()
	if len(str) > 0:
		cmd = str[0]
		
		val1 = None
		val2 = None
		if len(str) > 1:
			val1 = str[1]
		if len(str) > 2:
			val2 = str[2]

		ser.write(cmd)
		if not val1 == None:
			ser.write(chr(int(val1)))
			if not val2 == None:
				ser.write(chr(int(val2)))
		ser.write("\n")
	
	time.sleep(0.1)
	

	
	

