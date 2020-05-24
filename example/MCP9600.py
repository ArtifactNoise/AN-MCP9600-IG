from PyMCP2221A import PyMCP2221A
import time
import json


class MCP9600:
    def __init__(self,VID = 0x04D8,PID = 0x00DD,devnum = 0):
        self.mcp2221a = PyMCP2221A.PyMCP2221A(VID = 0x04D8,PID = 0x00DD,devnum = 0)
        self.mcp2221a.I2C_Init()
        self.address = 0x60
        self.Type_K = 0x00
        self.Type_J = 0x01
        self.Type_T = 0x02
        self.Type_N = 0x03
        self.Type_S = 0x04
        self.Type_E = 0x05
        self.Type_B = 0x06
        self.Type_R = 0x07

        self.Cold_Junction_0625C = 0
        self.Cold_Junction_25C = 1
        self.ADC_Measurement_18bit = 0
        self.ADC_Measurement_16bit = 1
        self.ADC_Measurement_14bit = 2
        self.ADC_Measurement_12bit = 3
        self.BurstMode_Temperature_1 = 0
        self.BurstMode_Temperature_2 = 1
        self.BurstMode_Temperature_4 = 2
        self.BurstMode_Temperature_8 = 3
        self.BurstMode_Temperature_16 = 4
        self.BurstMode_Temperature_32 = 5
        self.BurstMode_Temperature_64 = 6
        self.BurstMode_Temperature_128 = 7
        self.Shutdown_Modes_Nomal = 0
        self.Shutdown_Modes_Shutdown = 1
        self.Shutdown_Modes_Burst = 2

    def DeviceID_Revision(self):
        data = [reg["DeviceID_Revision"]]
        self.mcp2221a.I2C_Write_No_Stop(self.address,data)
        rdata = self.mcp2221a.I2C_Read_Repeated(self.address,2)
        return rdata[0]<<8 | rdata[1]<<0

    def Thermocouple_Sensor_Configuration(self,Thermocouple_Type,Filter_Coefficient):
        data = [reg["Thermocouple_Sensor_Configuration"],Thermocouple_Type<<4 | Filter_Coefficient]
        self.mcp2221a.I2C_Write_No_Stop(self.address,data)
        rdata = self.mcp2221a.I2C_Read_Repeated(self.address,1)
        return rdata[0]


    def Device_Configuration(self,Cold_Junction_Resolution,
                                                ADC_Measurement_Resolution,
                                                Burst_Mode_Temperature_Samples,
                                                Shutdown_Modes
                                                ):
        data = [reg["Device_Configuration"],
                Cold_Junction_Resolution<<7 | 
                ADC_Measurement_Resolution<<6 | 
                Burst_Mode_Temperature_Samples<<3 | 
                Shutdown_Modes ]
        self.mcp2221a.I2C_Write_No_Stop(self.address,data)
        rdata = self.mcp2221a.I2C_Read_Repeated(self.address,1)
        return rdata[0]
    def Hot_Junction_Temperature(self):
        data = [reg["Hot_Junction_Temperature"]]
        self.mcp2221a.I2C_Write_No_Stop(self.address,data)
        rdata = self.mcp2221a.I2C_Read_Repeated(self.address,2)
        return (rdata[0]<<8 | rdata[1])/16

str = '''
{
        "Hot_Junction_Temperature":0,
        "Junctions_Temperature_Delta":1,
        "Cold_Junction_Temperature":2,
        "Raw_Data_ADC":3,
        "STATUS_MCP9600":4,
        "Thermocouple_Sensor_Configuration":5,
        "Device_Configuration":6,
        "Alert_1_Configuration":8,
        "Alert_2_Configuration":9,
        "Alert_3_Configuration":10,
        "Alert_4_Configuration":11,
        "Alert_1_Hysteresis":12,
        "Alert_2_Hysteresis":13,
        "Alert_3_Hysteresis":14,
        "Alert_4_Hysteresis":15,
        "Alert_1_Limit":16,
        "Alert_2_Limit":17,
        "Alert_3_Limit":18,
        "Alert_4_Limit":19,
        "DeviceID_Revision":32
}      

'''

reg = json.loads(str)

mcp9600 = MCP9600()

print("0x%04X" % mcp9600.DeviceID_Revision())
print("0x%04X" % mcp9600.Thermocouple_Sensor_Configuration(Thermocouple_Type=mcp9600.Type_K,Filter_Coefficient=0x00))
print("0x%04X" % mcp9600.Device_Configuration(  Cold_Junction_Resolution=mcp9600.Cold_Junction_25C,
                                                ADC_Measurement_Resolution=mcp9600.ADC_Measurement_18bit,
                                                Burst_Mode_Temperature_Samples=mcp9600.BurstMode_Temperature_1,
                                                Shutdown_Modes = mcp9600.Shutdown_Modes_Nomal))

while True:
        #print (data)
        #temp =( data[0]*16 + data[1]/16)
        #ftemp = temp*1.8+32
        temp = mcp9600.Hot_Junction_Temperature()
        print ("temp in C : %.2f temp" %temp)
        #print ("temp in f : %.2f ftemp" %ftemp)

#mcp9600.I2C_Write(address,[reg["Thermocouple_Sensor_Configuration"],0x07])
#mcp9600.I2C_Write(address,[reg["Device_Configuration"],0x7c])

#        rdata = mcp9600.I2C_Read(address,1)
#        print("0x%04X" %  (rdata[0]) )
#        rdata = mcp9600.I2C_Read(address,1)
#        print("0x%04X" %  (rdata[0]) )





"""
# Get I2C bus
#bus.write_byte_data(address, 0xC0, 0x00)
# config sensor for thermocouple type and filter
bus.write_byte_data(address, 0x05, 0x07)mcp2221.I2C_Write(addrs,[int(jdata["Thermocouple_Sensor_Configuration"],16),0x07])

# config sensor for sample and mode
bus.write_byte_data(address, 0x06, 0x7c)mcp2221.I2C_Write(addrs,[int(jdata["Thermocouple_Sensor_Configuration"],16),0x07])

#bus.write_byte(address,0xC0)

while True:
        bus.write_byte(address,0x00)
        data = bus.read_i2c_block_data(address, 0x00, 2)
        print (data)
        temp =( data[0]*16 + data[1]/16)
        ftemp = temp*1.8+32
        print ("temp in C : %.2f temp" %temp)
        print ("temp in f : %.2f ftemp" %ftemp)
time.sleep(1)

"""
