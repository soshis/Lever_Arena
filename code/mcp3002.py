#!/usr/bin/python

import spidev #SPI 

#return handle to the device
def init(device):
	spi = spidev.SpiDev(0,0)
	spi.max_speed_hz = 1200000
	spi.open(0,device)
	return spi
	

# function to read from device spi
# [adc1,adc2] = readMCP3002(SPI device)
def read(spi):
	data = spi.xfer2([0x70,0])
	adc1 = ((data[0] & 3) << 8) + data[1]
	data = spi.xfer2([0x60,0])
	adc2 = ((data[0] & 3) << 8) + data[1]
	#spi.close()
	return [adc1,adc2]
