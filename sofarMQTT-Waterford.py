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
	def __init__(self):
		self.Inverter_Freq = 0
		self.Battery_ChargeDischargePwr = 0
		self.Battery_Cycles = 0
		self.Battery_ChrgLevel = 0
		self.Battery_Temp = 0
		self.Grid_IO_Pwr = 0
		self.House_Consumption_Pwr = 0
		self.Internal_IO_Pwr = 0
		self.PV_Generation_Pwr = 0
		self.EPS_Output_V = 0
		self.EPS_Output_Pwr = 0
		self.TodayGeneratedSolar_Wh = 0
		self.TodaySoldSolar_Wh = 0
		self.TodayBoughtGrid_Wh = 0
		self.TodayConsumption_Wh = 0
		self.TotalLoadConsumptionH = 0
		self.TotalLoadConsumption = 0
		self.InverterInternalTemp = 0
		self.InverterHeatsinkTemp = 0
		self.Battery_Voltage = 0
		self.Battery_Current = 0

	def setStrings(self):
		self.Grid_PwrStr = str(self.Grid_IO_Pwr)
		self.House_Consumption_PwrStr = str(self.House_Consumption_Pwr)
		self.Internal_IO_PwrStr = str(self.Internal_IO_Pwr)
		self.PV_Generation_PwrStr = str(self.PV_Generation_Pwr)
		self.EPS_Output_VStr = str(self.EPS_Output_V)
		self.EPS_Output_PwrStr = str(self.EPS_Output_Pwr)
		self.TodayGenerated_WhStr = str(self.TodayGeneratedSolar_Wh)
		self.TodaySold_WhStr = str(self.TodaySoldSolar_Wh)
		self.TodayBought_WhStr = str(self.TodayBoughtGrid_Wh)
		self.TodayConsumption_WhStr = str(self.TodayConsumption_Wh)
		self.Battery_ChargeDischargePwrStr = str(self.Battery_ChargeDischargePwr)
		self.Battery_CyclesStr = str(self.Battery_Cycles)
		self.Battery_ChrgLevelStr = str(self.Battery_ChrgLevel)
		self.InverterInternalTempStr = str(self.InverterInternalTemp)
		self.InverterHeatsinkTempStr = str(self.InverterHeatsinkTemp)
		self.TotalLoadConsumptionStr = str(self.TotalLoadConsumption)
		self.Inverter_FreqStr = str(float("{:.1f}".format(self.Inverter_Freq)))
		self.Battery_VoltageStr = str(self.Battery_Voltage)
		self.Battery_CurrentStr = str(self.Battery_Current)



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
	pvdevice.TotalLoadConsumption = pvdevice.TotalLoadConsumptionH + instrument.read_register(0x223, 0, functioncode=3, signed=False) # Total Load Consumption kWh LowByte
	pvdevice.InverterInternalTemp = instrument.read_register(0x238, 0, functioncode=3, signed=False) # Inverter Internal Temperature
	pvdevice.InverterHeatsinkTemp = instrument.read_register(0x239, 0, functioncode=3, signed=False) # Inverter Heatsink Temperature
	pvdevice.Battery_Voltage = instrument.read_register(0x20E, 0, functioncode=3, signed=False) / 10 # read Battery DC Voltage
	pvdevice.Battery_Current = instrument.read_register(0x20F, 0, functioncode=3, signed=True) /100 # read Battery DC Current
	success = True


def printStrings(pvdevice):
		##Debug Print
	print("Grid Power " + pvdevice.Grid_PwrStr + "W")
	print("House Consumption " + pvdevice.House_Consumption_PwrStr + "W")
	print("Solar PV Generation " + pvdevice.PV_Generation_PwrStr + "W")
	print("Internal I/O Power " + pvdevice.Internal_IO_PwrStr + "W")
	print("EPS Output Volts " + pvdevice.EPS_Output_VStr + "V")
	print("EPS Output Power " + pvdevice.EPS_Output_PwrStr + "W")
	print("Today's Generation " + pvdevice.TodayGenerated_WhStr + "Wh")
	print("Today's Sold " + pvdevice.TodaySold_WhStr + "Wh")
	print("Today's Bought " + pvdevice.TodayBought_WhStr + "Wh")
	print("Today's Consumption " + pvdevice.TodayConsumption_WhStr + "Wh")
	print("All time Consumption " + pvdevice.TotalLoadConsumptionStr + "kWh")
	print("Battery Charge/Discharge Power " + pvdevice.Battery_ChargeDischargePwrStr + "W")
	print("Battery Cycles " + pvdevice.Battery_CyclesStr)
	print("Battery Charge Level " + pvdevice.Battery_ChrgLevelStr + "%")
	print("Battery Voltage " + pvdevice.Battery_VoltageStr + "V")
	print("Battery Current " + pvdevice.Battery_CurrentStr + "A")
	print("Inverter Internal Temp " + pvdevice.InverterInternalTempStr + "C")
	print("Inverter Heatsink Temp " + pvdevice.InverterHeatsinkTempStr + "C")
	print("Grid Frequency " + pvdevice.Inverter_FreqStr + "Hz")

def mqttPublish(pvdevice):
	topic = "sensors/sofar/" +  "grid_power"
	client.publish(topic,pvdevice.Grid_PwrStr,qos=0,retain=False)
	topic = "sensors/sofar/" +  "house_consumption"
	client.publish(topic,pvdevice.House_Consumption_PwrStr,qos=0,retain=False)
	topic = "sensors/sofar/" +  "pv_generation"
	client.publish(topic,pvdevice.PV_Generation_PwrStr,qos=0,retain=False)
	topic = "sensors/sofar/" +  "batt_power"
	client.publish(topic,pvdevice.Battery_ChargeDischargePwrStr,qos=0,retain=False)
	topic = "sensors/sofar/" +  "batt_soc"
	client.publish(topic,pvdevice.Battery_ChrgLevelStr,qos=0,retain=False)
	topic = "sensors/sofar/" +  "batt_voltage"
	client.publish(topic,pvdevice.Battery_VoltageStr,qos=0,retain=False)
	topic = "sensors/sofar/" +  "batt_current"
	client.publish(topic,pvdevice.Battery_CurrentStr,qos=0,retain=False)

mqttcounts=0
serErrorCount = 0
inverter = SofarInverter()
instrument = minimalmodbus.Instrument('/dev/ttyUSB2', 1) # port name, slave address 

instrument.serial.baudrate = 9600   # Baud
instrument.serial.bytesize = 8
instrument.serial.parity   = serial.PARITY_NONE
instrument.serial.stopbits = 1
instrument.serial.timeout  = 0.5   # seconds


while serErrorCount != 11:
	
	
	

	try:
		print("Read Data "+ str(mqttcounts))
		readAllData(instrument, inverter)
		mqttcounts = mqttcounts +1
		serErrorCount = 0
	except:
		serErrorCount = serErrorCount + 1
		wait = serErrorCount * 10
		print("Unable to read Data From Inverter, wait " + str(wait) +"s for retry. Error count: " + str( serErrorCount))
		sleep(10)
		#ser.close()
		#ser = serial.Serial(SerialDevice, SerialBaud, timeout=20)
		#print("ser closed and reopened")
		#repairTRX()
		#print("TRX Repaired")
	#print(str(jsondata))
	if serErrorCount == 0:
		try:
			print("Publish to MQTT "+ str(mqttcounts))
			inverter.setStrings()
			printStrings(inverter)
			mqttPublish(inverter)
		except:
			print("\n Error Occured in MQTT Publish")
	
	gc.collect()
	
	
	if serErrorCount == 10:
			print("max errors reached")		

	sleep(5)
	

client.loop_stop()
