#!/usr/bin/env python
import json
import serial
import time
import minimalmodbus
import paho.mqtt.client as mqtt
import signal
import sys
import gc
from time import sleep
# This program is different from the original as it runs continually and reports data to the HA MQTT server instead of being evoked by HA directly

MQTTServer = "127.0.0.1"
MQTTPort = 1883

class SofarInverter:
	Inverter_Freq = 0
	Battery_ChargeDischargePwr = 0
	Battery_Cycles = 0
	Battery_ChrgLevel = 0
	Battery_Temp = 0
	Grid_IO_Pwr = 0
	House_Consumption_Pwr = 0
	Internal_IO_Pwr = 0
	PV_Generation_Pwr = 0
	EPS_Output_V = 0
	EPS_Output_Pwr = 0
	TodayGeneratedSolar_Wh = 0
	TodaySoldSolar_Wh = 0
	TodayBoughtGrid_Wh = 0
	TodayConsumption_Wh = 0
	TotalLoadConsumptionH = 0
	TotalLoadConsumption = 0
	InverterInternalTemp = 0
	InverterHeatsinkTemp = 0

class Instrument:
	def __init__(self):
		### SET /dev/ttyUSB* to your device, check it with "dmesg | grep tty"
		instrument = minimalmodbus.Instrument('/dev/ttyUSB2', 1) # port name, slave address 

		instrument.serial.baudrate = 9600   # Baud
		instrument.serial.bytesize = 8
		instrument.serial.parity   = serial.PARITY_NONE
		instrument.serial.stopbits = 1
		instrument.serial.timeout  = 0.5   # seconds

def on_connect(client, userdata, flags, rc):
	print("Connected with result code "+str(rc))
# Subscribe to all reqstate messages
	client.subscribe("sensors/sofar/command")
# This will call on message if it hears anything

def on_message(client, userdata, msg):
	print("\n MQTT Command of Topic: "+msg.topic+ " recieved with payload: "+str(msg.payload))
	sleep(1)
	
# Create MQTT Client
client = mqtt.Client()
# Link Callbacks with the above functions
client.on_connect = on_connect
client.on_message = on_message
client.username_pw_set("sofar","sofar")
client.connect(MQTTServer,MQTTPort,30)
client.loop_start()

def signal_handler(signal,frame):
	print("Cleaning Up then Shutting Down")
	client.loop_stop()
	instrument.close()
	sys.exit(1)

signal.signal(signal.SIGINT, signal_handler)

def readAllData(instrument, pvdevice):
	success = False # Intialise Success/Failure Flag to ensure full data is only uploaded if all data is received.
	pvdevice.Inverter_Freq = instrument.read_register(0x20c, 0, functioncode=3, signed=False) / 100.0 # read inverter frequency
	pvdevice.Battery_ChargeDischargePwr = instrument.read_register(0x20d, 0, functioncode=3, signed=True) * 10.0 # read battery charge discharge pwr
	pvdevice.Battery_Cycles = instrument.read_register(0x22c, 0, functioncode=3, signed=False) # read times batteries have been cycled
	pvdevice.Battery_ChrgLevel = instrument.read_register(0x210, 0, functioncode=3, signed=False) # read battery charge level
	pvdevice.Battery_Temp = instrument.read_register(0X211, 0, functioncode=3, signed=True) #Read Battery Temperature as Signed 16-Bit
	pvdevice.Grid_IO_Pwr = instrument.read_register(0x212, 0, functioncode=3, signed=True) * 10.0 # read total grid pwr
	pvdevice.House_Consumption_Pwr = instrument.read_register(0x213, 0, functioncode=3, signed=False) * 10.0 # read load pwr
	pvdevice.Internal_IO_Pwr = instrument.read_register(0x214, 0, functioncode=3, signed=True) * 10.0 # read internal system I/O pwr
	pvdevice.PV_Generation_Pwr = instrument.read_register(0x215, 0, functioncode=3, signed=False) * 10.0 # read SolarPV Generation Pwr
	pvdevice.EPS_Output_V = instrument.read_register(0x216, 0, functioncode=3, signed=False) / 10.0 # read EPS Output volts
	pvdevice.EPS_Output_Pwr = instrument.read_register(0x217, 0, functioncode=3, signed=False) * 10.0 # read EPS Output pwr
	pvdevice.TodayGeneratedSolar_Wh = instrument.read_register(0x218, 0, functioncode=3, signed=False) * 10.0 # read Today's generation Wh
	pvdevice.TodaySoldSolar_Wh = instrument.read_register(0x219, 0, functioncode=3, signed=False) * 10.0 # read Today's Generation sold Wh
	pvdevice.TodayBoughtGrid_Wh = instrument.read_register(0x21a, 0, functioncode=3, signed=False) * 10.0 # read Today's Power bought Wh
	pvdevice.TodayConsumption_Wh = instrument.read_register(0x21b, 0, functioncode=3, signed=False) * 10.0 # read Today's consumption bought Wh
	pvdevice.TotalLoadConsumptionH = instrument.read_register(0x222, 0, functioncode=3, signed=False) * 0xffff # Total Load Consumption kWh HighByte
	pvdevice.TotalLoadConsumption = TotalLoadConsumptionH + instrument.read_register(0x223, 0, functioncode=3, signed=False) # Total Load Consumption kWh LowByte
	pvdevice.InverterInternalTemp = instrument.read_register(0x238, 0, functioncode=3, signed=False) # Inverter Internal Temperature
	pvdevice.InverterHeatsinkTemp = instrument.read_register(0x239, 0, functioncode=3, signed=False) # Inverter Heatsink Temperature
	success = True

def valuesToStrings(pvdevice):
	Grid_PwrStr = str(pvdevice.Grid_IO_Pwr)
	House_Consumption_PwrStr = str(pvdevice.House_Consumption_Pwr)
	Internal_IO_PwrStr = str(pvdevice.Internal_IO_Pwr)
	PV_Generation_PwrStr = str(pvdevice.PV_Generation_Pwr)
	EPS_Output_VStr = str(pvdevice.EPS_Output_V)
	EPS_Output_PwrStr = str(pvdevice.EPS_Output_Pwr)
	TodayGenerated_WhStr = str(pvdevice.TodayGeneratedSolar_Wh)
	TodaySold_WhStr = str(pvdevice.TodaySoldSolar_Wh)
	TodayBought_WhStr = str(pvdevice.TodayBoughtGrid_Wh)
	TodayConsumption_WhStr = str(pvdevice.TodayConsumption_Wh)
	Battery_ChargeDischargePwrStr = str(pvdevice.Battery_ChargeDischargePwr)
	Battery_CyclesStr = str(pvdevice.Battery_Cycles)
	Battery_ChrgLevelStr = str(pvdevice.Battery_ChrgLevel)
	InverterInternalTempStr = str(pvdevice.InverterInternalTemp)
	InverterHeatsinkTempStr = str(pvdevice.InverterHeatsinkTemp)
	TotalLoadConsumptionStr = str(pvdevice.TotalLoadConsumption)
	Inverter_FreqStr = str(float("{:.1f}".format(pvdevice.Inverter_Freq)))

def printStrings(valuesToStrings):
		##Debug Print
	print("Grid Power " + valuesToStrings.Grid_PwrStr + "W")
	print("House Consumption " + valuesToStrings.House_Consumption_PwrStr + "W")
	print("Solar PV Generation " + valuesToStrings.PV_Generation_PwrStr + "W")
	print("Internal I/O Power " + valuesToStrings.Internal_IO_PwrStr + "W")
	print("EPS Output Volts " + valuesToStrings.EPS_Output_VStr + "V")
	print("EPS Output Power " + valuesToStrings.EPS_Output_PwrStr + "W")
	print("Today's Generation " + valuesToStrings.TodayGenerated_WhStr + "Wh")
	print("Today's Sold " + valuesToStrings.TodaySold_WhStr + "Wh")
	print("Today's Bought " + valuesToStrings.TodayBought_WhStr + "Wh")
	print("Today's Consumption " + valuesToStrings.TodayConsumption_WhStr + "Wh")
	print("All time Consumption " + valuesToStrings.TotalLoadConsumptionStr + "kW")
	print("Battery Charge/Discharge Power " + valuesToStrings.Battery_ChargeDischargePwrStr + "W")
	print("Battery Cycles " + valuesToStrings.Battery_CyclesStr)
	print("Battery Charge Level " + valuesToStrings.Battery_ChrgLevelStr + "%")
	print("Inverter Internal Temp " + valuesToStrings.InverterInternalTempStr + "C")
	print("Inverter Heatsink Temp " + valuesToStrings.InverterHeatsinkTempStr + "C")
	print("Grid Frequency " + valuesToStrings.Inverter_FreqStr + "Hz")

def mqttPublish(valuesToStrings):
	topic = "sensors/sofar/" +  "grid_power"
	client.publish(topic,valuesToStrings.Grid_PwrStr,qos=0,retain=False)
	topic = "sensors/sofar/" +  "house_consumption"
	client.publish(topic,valuesToStrings.House_Consumption_PwrStr,qos=0,retain=False)


mqttcounts=0
serErrorCount = 0
instrument = Instrument()
inverter = SofarInverter()

while serErrorCount != 11:
	

	try:
		readAllData(instrument, inverter)
	except:
		print("Unable to read Data From Inverter.")
		#ser.close()
		#ser = serial.Serial(SerialDevice, SerialBaud, timeout=20)
		#print("ser closed and reopened")
		#repairTRX()
		#print("TRX Repaired")
	#print(str(jsondata))

	try:
		
		#print("Read ID:" + str(id)+ " Watts " +str(watts) + " State " + str(state))
		mqttcounts = mqttcounts +1
		print("Publish to MQTT "+ str(mqttcounts))
		valuesToStrings(pvdevice)
		printStrings(valuesToStrings)
		mqttPublish(valuesToStrings)
		
	except:
		print("\n Error Occured in MQTT Publish")
	
	gc.collect()
	
	
	if serErrorCount == 10:
			print("max errors reached")		

	sleep(5)
	

client.loop_stop()
