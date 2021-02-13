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

### SET /dev/ttyUSB* to your device, check it with "dmesg | grep tty"
instrument = minimalmodbus.Instrument('/dev/ttyUSB2', 1) # port name, slave address 

instrument.serial.baudrate = 9600   # Baud
instrument.serial.bytesize = 8
instrument.serial.parity   = serial.PARITY_NONE
instrument.serial.stopbits = 1
instrument.serial.timeout  = 0.5   # seconds
success = False # Intialise Success/Failure Flag to ensure full data is only uploaded if all data is received.
# Init all rows
Inverter_Freq = instrument.read_register(0x20c, 0, functioncode=3, signed=False) / 100.0 # read inverter frequency

Battery_ChargeDischargePwr = instrument.read_register(0x20d, 0, functioncode=3, signed=True) * 10.0 # read battery charge discharge pwr
Battery_Cycles = instrument.read_register(0x22c, 0, functioncode=3, signed=False) # read times batteries have been cycled
Battery_ChrgLevel = instrument.read_register(0x210, 0, functioncode=3, signed=False) # read battery charge level
Battery_Temp = instrument.read_register(0X211, 0, functioncode=3, signed=True) #Read Battery Temperature as Signed 16-Bit

Grid_IO_Pwr = instrument.read_register(0x212, 0, functioncode=3, signed=True) * 10.0 # read total grid pwr

House_Consumption_Pwr = instrument.read_register(0x213, 0, functioncode=3, signed=False) * 10.0 # read load pwr

Internal_IO_Pwr = instrument.read_register(0x214, 0, functioncode=3, signed=True) * 10.0 # read internal system I/O pwr

PV_Generation_Pwr = instrument.read_register(0x215, 0, functioncode=3, signed=False) * 10.0 # read SolarPV Generation Pwr

EPS_Output_V = instrument.read_register(0x216, 0, functioncode=3, signed=False) / 10.0 # read EPS Output volts
EPS_Output_Pwr = instrument.read_register(0x217, 0, functioncode=3, signed=False) * 10.0 # read EPS Output pwr

TodayGeneratedSolar_Wh = instrument.read_register(0x218, 0, functioncode=3, signed=False) * 10.0 # read Today's generation Wh
TodaySoldSolar_Wh = instrument.read_register(0x219, 0, functioncode=3, signed=False) * 10.0 # read Today's Generation sold Wh
TodayBoughtGrid_Wh = instrument.read_register(0x21a, 0, functioncode=3, signed=False) * 10.0 # read Today's Power bought Wh
TodayConsumption_Wh = instrument.read_register(0x21b, 0, functioncode=3, signed=False) * 10.0 # read Today's consumption bought Wh

TotalLoadConsumptionH = instrument.read_register(0x222, 0, functioncode=3, signed=False) * 0xffff # Total Load Consumption kWh HighByte
TotalLoadConsumption = TotalLoadConsumptionH + instrument.read_register(0x223, 0, functioncode=3, signed=False) # Total Load Consumption kWh LowByte

InverterInternalTemp = instrument.read_register(0x238, 0, functioncode=3, signed=False) # Inverter Internal Temperature
InverterHeatsinkTemp = instrument.read_register(0x239, 0, functioncode=3, signed=False) # Inverter Heatsink Temperature

	# try:

	##Flag to stream all data to EmonHub
success = True
Grid_PwrStr = str(Grid_IO_Pwr)
House_Consumption_PwrStr = str(House_Consumption_Pwr)
Internal_IO_PwrStr = str(Internal_IO_Pwr)
PV_Generation_PwrStr = str(PV_Generation_Pwr)
EPS_Output_VStr = str(EPS_Output_V)
EPS_Output_PwrStr = str(EPS_Output_Pwr)
TodayGenerated_WhStr = str(TodayGeneratedSolar_Wh)
TodaySold_WhStr = str(TodaySoldSolar_Wh)
TodayBought_WhStr = str(TodayBoughtGrid_Wh)
TodayConsumption_WhStr = str(TodayConsumption_Wh)
Battery_ChargeDischargePwrStr = str(Battery_ChargeDischargePwr)
Battery_CyclesStr = str(Battery_Cycles)
Battery_ChrgLevelStr = str(Battery_ChrgLevel)
InverterInternalTempStr = str(InverterInternalTemp)
InverterHeatsinkTempStr = str(InverterHeatsinkTemp)
Inverter_FreqStr = str(float("{:.1f}".format(Inverter_Freq)))
##Debug Print
print("Grid Power " + Grid_PwrStr + "W")
print("House Consumption " + House_Consumption_PwrStr + "W")
print("Solar PV Generation " + PV_Generation_PwrStr + "W")
print("Internal I/O Power " + Internal_IO_PwrStr + "W")
print("EPS Output Volts " + EPS_Output_VStr + "V")
print("EPS Output Power " + EPS_Output_PwrStr + "W")
print("Today's Generation " + TodayGenerated_WhStr + "Wh")
print("Today's Sold " + TodaySold_WhStr + "Wh")
print("Today's Bought " + TodayBought_WhStr + "Wh")
print("Today's Consumption " + TodayConsumption_WhStr + "Wh")
print("All time Consumption " + str(TotalLoadConsumption) + "kW")
print("Battery Charge/Discharge Power " + Battery_ChargeDischargePwrStr + "W")
print("Battery Cycles " + Battery_CyclesStr)
print("Battery Charge Level " + Battery_ChrgLevelStr + "%")
print("Inverter Internal Temp " + InverterInternalTempStr + "C")
print("Inverter Heatsink Temp " + InverterHeatsinkTempStr + "C")
print("Grid Frequency " + Inverter_FreqStr + "Hz")
mqttcounts=0

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

def readData():
	success = False # Intialise Success/Failure Flag to ensure full data is only uploaded if all data is received.

	Inverter_Freq = instrument.read_register(0x20c, 0, functioncode=3, signed=False) / 100.0 # read inverter frequency

	Battery_ChargeDischargePwr = instrument.read_register(0x20d, 0, functioncode=3, signed=True) * 10.0 # read battery charge discharge pwr
	Battery_Cycles = instrument.read_register(0x22c, 0, functioncode=3, signed=False) # read times batteries have been cycled
	Battery_ChrgLevel = instrument.read_register(0x210, 0, functioncode=3, signed=False) # read battery charge level
	Battery_Temp = instrument.read_register(0X211, 0, functioncode=3, signed=True) #Read Battery Temperature as Signed 16-Bit

	Grid_IO_Pwr = instrument.read_register(0x212, 0, functioncode=3, signed=True) * 10.0 # read total grid pwr

	House_Consumption_Pwr = instrument.read_register(0x213, 0, functioncode=3, signed=False) * 10.0 # read load pwr

	Internal_IO_Pwr = instrument.read_register(0x214, 0, functioncode=3, signed=True) * 10.0 # read internal system I/O pwr

	PV_Generation_Pwr = instrument.read_register(0x215, 0, functioncode=3, signed=False) * 10.0 # read SolarPV Generation Pwr

	EPS_Output_V = instrument.read_register(0x216, 0, functioncode=3, signed=False) / 10.0 # read EPS Output volts
	EPS_Output_Pwr = instrument.read_register(0x217, 0, functioncode=3, signed=False) * 10.0 # read EPS Output pwr

	TodayGeneratedSolar_Wh = instrument.read_register(0x218, 0, functioncode=3, signed=False) * 10.0 # read Today's generation Wh
	TodaySoldSolar_Wh = instrument.read_register(0x219, 0, functioncode=3, signed=False) * 10.0 # read Today's Generation sold Wh
	TodayBoughtGrid_Wh = instrument.read_register(0x21a, 0, functioncode=3, signed=False) * 10.0 # read Today's Power bought Wh
	TodayConsumption_Wh = instrument.read_register(0x21b, 0, functioncode=3, signed=False) * 10.0 # read Today's consumption bought Wh

	TotalLoadConsumptionH = instrument.read_register(0x222, 0, functioncode=3, signed=False) * 0xffff # Total Load Consumption kWh HighByte
	TotalLoadConsumption = TotalLoadConsumptionH + instrument.read_register(0x223, 0, functioncode=3, signed=False) # Total Load Consumption kWh LowByte

	InverterInternalTemp = instrument.read_register(0x238, 0, functioncode=3, signed=False) # Inverter Internal Temperature
	InverterHeatsinkTemp = instrument.read_register(0x239, 0, functioncode=3, signed=False) # Inverter Heatsink Temperature

	success = True

	Grid_PwrStr = str(Grid_IO_Pwr)
	House_Consumption_PwrStr = str(House_Consumption_Pwr)
	Internal_IO_PwrStr = str(Internal_IO_Pwr)
	PV_Generation_PwrStr = str(PV_Generation_Pwr)
	EPS_Output_VStr = str(EPS_Output_V)
	EPS_Output_PwrStr = str(EPS_Output_Pwr)
	TodayGenerated_WhStr = str(TodayGeneratedSolar_Wh)
	TodaySold_WhStr = str(TodaySoldSolar_Wh)
	TodayBought_WhStr = str(TodayBoughtGrid_Wh)
	TodayConsumption_WhStr = str(TodayConsumption_Wh)
	Battery_ChargeDischargePwrStr = str(Battery_ChargeDischargePwr)
	Battery_CyclesStr = str(Battery_Cycles)
	Battery_ChrgLevelStr = str(Battery_ChrgLevel)
	InverterInternalTempStr = str(InverterInternalTemp)
	InverterHeatsinkTempStr = str(InverterHeatsinkTemp)
	Inverter_FreqStr = str(float("{:.1f}".format(Inverter_Freq)))
		##Debug Print
	print("Grid Power " + Grid_PwrStr + "W")
	print("House Consumption " + House_Consumption_PwrStr + "W")
	print("Solar PV Generation " + PV_Generation_PwrStr + "W")
	print("Internal I/O Power " + Internal_IO_PwrStr + "W")
	print("EPS Output Volts " + EPS_Output_VStr + "V")
	print("EPS Output Power " + EPS_Output_PwrStr + "W")
	print("Today's Generation " + TodayGenerated_WhStr + "Wh")
	print("Today's Sold " + TodaySold_WhStr + "Wh")
	print("Today's Bought " + TodayBought_WhStr + "Wh")
	print("Today's Consumption " + TodayConsumption_WhStr + "Wh")
	print("All time Consumption " + str(TotalLoadConsumption) + "kW")
	print("Battery Charge/Discharge Power " + Battery_ChargeDischargePwrStr + "W")
	print("Battery Cycles " + Battery_CyclesStr)
	print("Battery Charge Level " + Battery_ChrgLevelStr + "%")
	print("Inverter Internal Temp " + InverterInternalTempStr + "C")
	print("Inverter Heatsink Temp " + InverterHeatsinkTempStr + "C")
	print("Grid Frequency " + Inverter_FreqStr + "Hz")
	try:
		
		#print("Read ID:" + str(id)+ " Watts " +str(watts) + " State " + str(state))
		mqttcounts = mqttcounts +1
		print("Publish to MQTT "+ str(mqttcounts))
		#Build the MQTT Names
		topic = "sensors/sofar/" +  "grid_power"
		client.publish(topic,Grid_PwrStr,qos=0,retain=False)
		topic = "sensors/sofar/" +  "house_consumption"
		client.publish(topic,House_Consumption_PwrStr,qos=0,retain=False)
	except:
		print("\n Error Occured in MQTT Publish")
	
	gc.collect()


serErrorCount = 0


while serErrorCount != 11:
	

	try:
		readData()
	except:
		print("Unable to read Data From Inverter.")
		#ser.close()
		#ser = serial.Serial(SerialDevice, SerialBaud, timeout=20)
		#print("ser closed and reopened")
		#repairTRX()
		#print("TRX Repaired")
	#print(str(jsondata))
	
	
	if serErrorCount == 10:
			print("max errors reached")		

	sleep(5)
	

client.loop_stop()
