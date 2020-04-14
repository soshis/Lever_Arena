#~/usr/bin/python
#Function for reading an MCP3208 ADC from a raspberry pi
#Spencer Boyer boyer2@uw.edu University of Washington
#Adapted from http://www.paulschow.com/2013/08/monitoring-temperatures-using-raspberry.html

import spidev

class mcp3208:

	#initialize mcp3208 on chip select device
	def __init__(self,device):
		
		self.values = [-1] * 8	
		self.spi = spidev.SpiDev()
		self.spi.open(0,device)
	
	#Read MCP3208 and return array of value of each channel
	def readADC(self):
		for channel in range(0,8):
			self.values[channel] = self.readChannel(channel)
		return self.values		

	#Read single channel of MCP3208
	def readChannel(self, channel):
		if (channel > 7 or channel < 0):
			return -1
		readBytes = self.spi.xfer2([6 + ((channel&4) >> 2),(channel&3) <<6, 0])
		#with default spi configuration mcp3208 is LSB first
		self.values[channel] = (((readBytes[1] & 15) << 8) + readBytes[2])
		return self.values[channel]

	#Close the spi device
	def __del__(self):
		self.spi.close()
		return 0

