from bluepy.btle import UUID, Peripheral, DefaultDelegate, AssignedNumbers
import struct
import math
import json
import serial
import re
from time import gmtime, strftime
import pickle
import pandas as pd
import sklearn

def _TI_UUID(val):
    return UUID("%08X-0451-4000-b000-000000000000" % (0xF0000000+val))

# Sensortag versions
AUTODETECT = "-"
SENSORTAG_V1 = "v1"
SENSORTAG_2650 = "CC2650"

class SensorBase:
    # Derived classes should set: svcUUID, ctrlUUID, dataUUID
    sensorOn  = struct.pack("B", 0x01)
    sensorOff = struct.pack("B", 0x00)

    def __init__(self, periph):
        self.periph = periph
        self.service = None
        self.ctrl = None
        self.data = None

    def enable(self):
        if self.service is None:
            self.service = self.periph.getServiceByUUID(self.svcUUID)
        if self.ctrl is None:
            self.ctrl = self.service.getCharacteristics(self.ctrlUUID) [0]
        if self.data is None:
            self.data = self.service.getCharacteristics(self.dataUUID) [0]
        if self.sensorOn is not None:
            self.ctrl.write(self.sensorOn,withResponse=True)

    def read(self):
        return self.data.read()

    def disable(self):
        if self.ctrl is not None:
            self.ctrl.write(self.sensorOff)

    # Derived class should implement _formatData()

def calcPoly(coeffs, x):
    return coeffs[0] + (coeffs[1]*x) + (coeffs[2]*x*x)

class IRTemperatureSensor(SensorBase):
    svcUUID  = _TI_UUID(0xAA00)
    dataUUID = _TI_UUID(0xAA01)
    ctrlUUID = _TI_UUID(0xAA02)

    zeroC = 273.15 # Kelvin
    tRef  = 298.15
    Apoly = [1.0,      1.75e-3, -1.678e-5]
    Bpoly = [-2.94e-5, -5.7e-7,  4.63e-9]
    Cpoly = [0.0,      1.0,      13.4]

    def __init__(self, periph):
        SensorBase.__init__(self, periph)
        self.S0 = 6.4e-14

    def read(self):
        '''Returns (ambient_temp, target_temp) in degC'''

        # See http://processors.wiki.ti.com/index.php/SensorTag_User_Guide#IR_Temperature_Sensor
        (rawVobj, rawTamb) = struct.unpack('<hh', self.data.read())
        tAmb = rawTamb / 128.0
        Vobj = 1.5625e-7 * rawVobj

        tDie = tAmb + self.zeroC
        S   = self.S0 * calcPoly(self.Apoly, tDie-self.tRef)
        Vos = calcPoly(self.Bpoly, tDie-self.tRef)
        fObj = calcPoly(self.Cpoly, Vobj-Vos)

        tObj = math.pow( math.pow(tDie,4.0) + (fObj/S), 0.25 )
        return (tAmb, tObj - self.zeroC)


class IRTemperatureSensorTMP007(SensorBase):
    svcUUID  = _TI_UUID(0xAA00)
    dataUUID = _TI_UUID(0xAA01)
    ctrlUUID = _TI_UUID(0xAA02)

    SCALE_LSB = 0.03125;
 
    def __init__(self, periph):
        SensorBase.__init__(self, periph)

    def read(self):
        '''Returns (ambient_temp, target_temp) in degC'''
        # http://processors.wiki.ti.com/index.php/CC2650_SensorTag_User's_Guide?keyMatch=CC2650&tisearch=Search-EN
        (rawTobj, rawTamb) = struct.unpack('<hh', self.data.read())
        tObj = (rawTobj >> 2) * self.SCALE_LSB;
        tAmb = (rawTamb >> 2) * self.SCALE_LSB;
        return (tAmb, tObj)

class AccelerometerSensor(SensorBase):
    svcUUID  = _TI_UUID(0xAA10)
    dataUUID = _TI_UUID(0xAA11)
    ctrlUUID = _TI_UUID(0xAA12)

    def __init__(self, periph):
        SensorBase.__init__(self, periph)
        if periph.firmwareVersion.startswith("1.4 "):
            self.scale = 64.0
        else:
            self.scale = 16.0

    def read(self):
        '''Returns (x_accel, y_accel, z_accel) in units of g'''
        x_y_z = struct.unpack('bbb', self.data.read())
        return tuple([ (val/self.scale) for val in x_y_z ])

class MovementSensorMPU9250(SensorBase):
    svcUUID  = _TI_UUID(0xAA80)
    dataUUID = _TI_UUID(0xAA81)
    ctrlUUID = _TI_UUID(0xAA82)
    sensorOn = None
    GYRO_XYZ =  7
    ACCEL_XYZ = 7 << 3
    MAG_XYZ = 1 << 6
    ACCEL_RANGE_2G  = 0 << 8
    ACCEL_RANGE_4G  = 1 << 8
    ACCEL_RANGE_8G  = 2 << 8
    ACCEL_RANGE_16G = 3 << 8

    def __init__(self, periph):
        SensorBase.__init__(self, periph)
        self.ctrlBits = 0

    def enable(self, bits):
        SensorBase.enable(self)
        self.ctrlBits |= bits
        self.ctrl.write( struct.pack("<H", self.ctrlBits) )

    def disable(self, bits):
        self.ctrlBits &= ~bits
        self.ctrl.write( struct.pack("<H", self.ctrlBits) )

    def rawRead(self):
        dval = self.data.read()
        return struct.unpack("<hhhhhhhhh", dval)

class AccelerometerSensorMPU9250:
    def __init__(self, sensor_):
        self.sensor = sensor_
        self.bits = self.sensor.ACCEL_XYZ | self.sensor.ACCEL_RANGE_4G
        self.scale = 8.0/32768.0 # TODO: why not 4.0, as documented?

    def enable(self):
        self.sensor.enable(self.bits)

    def disable(self):
        self.sensor.disable(self.bits)

    def read(self):
        '''Returns (x_accel, y_accel, z_accel) in units of g'''
        rawVals = self.sensor.rawRead()[3:6]
        return tuple([ v*self.scale for v in rawVals ])



class HumiditySensor(SensorBase):
    svcUUID  = _TI_UUID(0xAA20)
    dataUUID = _TI_UUID(0xAA21)
    ctrlUUID = _TI_UUID(0xAA22)

    def __init__(self, periph):
        SensorBase.__init__(self, periph)

    def read(self):
        '''Returns (ambient_temp, rel_humidity)'''
        (rawT, rawH) = struct.unpack('<HH', self.data.read())
        temp = -46.85 + 175.72 * (rawT / 65536.0)
        RH = -6.0 + 125.0 * ((rawH & 0xFFFC)/65536.0)
        return (temp, RH)

class HumiditySensorHDC1000(SensorBase):
    svcUUID  = _TI_UUID(0xAA20)
    dataUUID = _TI_UUID(0xAA21)
    ctrlUUID = _TI_UUID(0xAA22)

    def __init__(self, periph):
        SensorBase.__init__(self, periph)

    def read(self):
        '''Returns (ambient_temp, rel_humidity)'''
        (rawT, rawH) = struct.unpack('<HH', self.data.read())
        temp = -40.0 + 165.0 * (rawT / 65536.0)
        RH = 100.0 * (rawH/65536.0)
        return (temp, RH)

class MagnetometerSensor(SensorBase):
    svcUUID  = _TI_UUID(0xAA30)
    dataUUID = _TI_UUID(0xAA31)
    ctrlUUID = _TI_UUID(0xAA32)

    def __init__(self, periph):
        SensorBase.__init__(self, periph)

    def read(self):
        '''Returns (x, y, z) in uT units'''
        x_y_z = struct.unpack('<hhh', self.data.read())
        return tuple([ 1000.0 * (v/32768.0) for v in x_y_z ])
        # Revisit - some absolute calibration is needed

class MagnetometerSensorMPU9250:
    def __init__(self, sensor_):
        self.sensor = sensor_
        self.scale = 4912.0 / 32760
        # Reference: MPU-9250 register map v1.4

    def enable(self):
        self.sensor.enable(self.sensor.MAG_XYZ)

    def disable(self):
        self.sensor.disable(self.sensor.MAG_XYZ)

    def read(self):
        '''Returns (x_mag, y_mag, z_mag) in units of uT'''
        rawVals = self.sensor.rawRead()[6:9]
        return tuple([ v*self.scale for v in rawVals ])

class BarometerSensor(SensorBase):
    svcUUID  = _TI_UUID(0xAA40)
    dataUUID = _TI_UUID(0xAA41)
    ctrlUUID = _TI_UUID(0xAA42)
    calUUID  = _TI_UUID(0xAA43)
    sensorOn = None

    def __init__(self, periph):
       SensorBase.__init__(self, periph)

    def enable(self):
        SensorBase.enable(self)
        self.calChr = self.service.getCharacteristics(self.calUUID) [0]

        # Read calibration data
        self.ctrl.write( struct.pack("B", 0x02), True )
        (c1,c2,c3,c4,c5,c6,c7,c8) = struct.unpack("<HHHHhhhh", self.calChr.read())
        self.c1_s = c1/float(1 << 24)
        self.c2_s = c2/float(1 << 10)
        self.sensPoly = [ c3/1.0, c4/float(1 << 17), c5/float(1<<34) ]
        self.offsPoly = [ c6*float(1<<14), c7/8.0, c8/float(1<<19) ]
        self.ctrl.write( struct.pack("B", 0x01), True )


    def read(self):
        '''Returns (ambient_temp, pressure_millibars)'''
        (rawT, rawP) = struct.unpack('<hH', self.data.read())
        temp = (self.c1_s * rawT) + self.c2_s
        sens = calcPoly( self.sensPoly, float(rawT) )
        offs = calcPoly( self.offsPoly, float(rawT) )
        pres = (sens * rawP + offs) / (100.0 * float(1<<14))
        return (temp,pres)

class BarometerSensorBMP280(SensorBase):
    svcUUID  = _TI_UUID(0xAA40)
    dataUUID = _TI_UUID(0xAA41)
    ctrlUUID = _TI_UUID(0xAA42)

    def __init__(self, periph):
        SensorBase.__init__(self, periph)

    def read(self):
        (tL,tM,tH,pL,pM,pH) = struct.unpack('<BBBBBB', self.data.read())
        temp = (tH*65536 + tM*256 + tL) / 100.0
        press = (pH*65536 + pM*256 + pL) / 100.0
        return (temp, press)

class GyroscopeSensor(SensorBase):
    svcUUID  = _TI_UUID(0xAA50)
    dataUUID = _TI_UUID(0xAA51)
    ctrlUUID = _TI_UUID(0xAA52)
    sensorOn = struct.pack("B",0x07)

    def __init__(self, periph):
       SensorBase.__init__(self, periph)

    def read(self):
        '''Returns (x,y,z) rate in deg/sec'''
        x_y_z = struct.unpack('<hhh', self.data.read())
        return tuple([ 250.0 * (v/32768.0) for v in x_y_z ])

class GyroscopeSensorMPU9250:
    def __init__(self, sensor_):
        self.sensor = sensor_
        self.scale = 500.0/65536.0

    def enable(self):
        self.sensor.enable(self.sensor.GYRO_XYZ)

    def disable(self):
        self.sensor.disable(self.sensor.GYRO_XYZ)

    def read(self):
        '''Returns (x_gyro, y_gyro, z_gyro) in units of degrees/sec'''
        rawVals = self.sensor.rawRead()[0:3]
        return tuple([ v*self.scale for v in rawVals ])

class KeypressSensor(SensorBase):
    svcUUID = UUID(0xFFE0)
    dataUUID = UUID(0xFFE1)
    ctrlUUID = None
    sensorOn = None

    def __init__(self, periph):
        SensorBase.__init__(self, periph)
 
    def enable(self):
        SensorBase.enable(self)
        self.char_descr = self.service.getDescriptors(forUUID=0x2902)[0]
        self.char_descr.write(struct.pack('<bb', 0x01, 0x00), True)

    def disable(self):
        self.char_descr.write(struct.pack('<bb', 0x00, 0x00), True)

class OpticalSensorOPT3001(SensorBase):
    svcUUID  = _TI_UUID(0xAA70)
    dataUUID = _TI_UUID(0xAA71)
    ctrlUUID = _TI_UUID(0xAA72)

    def __init__(self, periph):
       SensorBase.__init__(self, periph)

    def read(self):
        '''Returns value in lux'''
        raw = struct.unpack('<h', self.data.read()) [0]
        m = raw & 0xFFF;
        e = (raw & 0xF000) >> 12;
        return 0.01 * (m << e)

class BatterySensor(SensorBase):
    svcUUID  = UUID("0000180f-0000-1000-8000-00805f9b34fb")
    dataUUID = UUID("00002a19-0000-1000-8000-00805f9b34fb")
    ctrlUUID = None
    sensorOn = None

    def __init__(self, periph):
       SensorBase.__init__(self, periph)

    def read(self):
        '''Returns the battery level in percent'''
        val = ord(self.data.read())
        return val

class SensorTag(Peripheral):
    def __init__(self,addr,version=AUTODETECT):
        Peripheral.__init__(self,addr)
        if version==AUTODETECT:
            svcs = self.discoverServices()
            if _TI_UUID(0xAA70) in svcs:
                version = SENSORTAG_2650
            else:
                version = SENSORTAG_V1

        fwVers = self.getCharacteristics(uuid=AssignedNumbers.firmwareRevisionString)
        if len(fwVers) >= 1:
            self.firmwareVersion = fwVers[0].read().decode("utf-8")
        else:
            self.firmwareVersion = u''

        if version==SENSORTAG_V1:
            self.IRtemperature = IRTemperatureSensor(self)
            self.accelerometer = AccelerometerSensor(self)
            self.humidity = HumiditySensor(self)
            self.magnetometer = MagnetometerSensor(self)
            self.barometer = BarometerSensor(self)
            self.gyroscope = GyroscopeSensor(self)
            self.keypress = KeypressSensor(self)
            self.lightmeter = None
        elif version==SENSORTAG_2650:
            self._mpu9250 = MovementSensorMPU9250(self)
            self.IRtemperature = IRTemperatureSensorTMP007(self)
            self.accelerometer = AccelerometerSensorMPU9250(self._mpu9250)
            self.humidity = HumiditySensorHDC1000(self)
            self.magnetometer = MagnetometerSensorMPU9250(self._mpu9250)
            self.barometer = BarometerSensorBMP280(self)
            self.gyroscope = GyroscopeSensorMPU9250(self._mpu9250)
            self.keypress = KeypressSensor(self)
            self.lightmeter = OpticalSensorOPT3001(self)
            self.battery = BatterySensor(self)

class KeypressDelegate(DefaultDelegate):
    BUTTON_L = 0x02
    BUTTON_R = 0x01
    ALL_BUTTONS = (BUTTON_L | BUTTON_R)

    _button_desc = { 
        BUTTON_L : "Left button",
        BUTTON_R : "Right button",
        ALL_BUTTONS : "Both buttons"
    } 

    def __init__(self):
        DefaultDelegate.__init__(self)
        self.lastVal = 0

    def handleNotification(self, hnd, data):
        # NB: only one source of notifications at present
        # so we can ignore 'hnd'.
        val = struct.unpack("B", data)[0]
        down = (val & ~self.lastVal) & self.ALL_BUTTONS
        if down != 0:
            self.onButtonDown(down)
        up = (~val & self.lastVal) & self.ALL_BUTTONS
        if up != 0:
            self.onButtonUp(up)
        self.lastVal = val

    def onButtonUp(self, but):
        print ( "** " + self._button_desc[but] + " UP")

    def onButtonDown(self, but):
        print ( "** " + self._button_desc[but] + " DOWN")

def main():
    import time
    import sys
    import argparse
    from time import gmtime, strftime
    import serial
    import pickle
    import pandas as pd
    import os
    from sklearn.externals import joblib

    parser = argparse.ArgumentParser()
    parser.add_argument('host', action='store',help='MAC of BT device')
    parser.add_argument('-n', action='store', dest='count', default=0,
            type=int, help="Number of times to loop data")
    parser.add_argument('-t',action='store',type=float, default=5.0, help='time between polling')
    parser.add_argument('-T','--temperature', action="store_true",default=False)
    parser.add_argument('-A','--accelerometer', action='store_true',
            default=False)
    parser.add_argument('-H','--humidity', action='store_true', default=False)
    parser.add_argument('-M','--magnetometer', action='store_true',
            default=False)
    parser.add_argument('-B','--barometer', action='store_true', default=False)
    parser.add_argument('-G','--gyroscope', action='store_true', default=False)
    parser.add_argument('-K','--keypress', action='store_true', default=False)
    parser.add_argument('-L','--light', action='store_true', default=False)
    parser.add_argument('-P','--battery', action='store_true', default=False)
    parser.add_argument('--all', action='store_true', default=False)

    arg = parser.parse_args(sys.argv[1:])

    print('Connecting to ' + arg.host)
    tag = SensorTag(arg.host)

    # Enabling selected sensors
    if arg.temperature or arg.all:
        tag.IRtemperature.enable()
    if arg.humidity or arg.all:
        tag.humidity.enable()
    if arg.barometer or arg.all:
        tag.barometer.enable()
    if arg.accelerometer or arg.all:
        tag.accelerometer.enable()
    if arg.magnetometer or arg.all:
        tag.magnetometer.enable()
    if arg.gyroscope or arg.all:
        tag.gyroscope.enable()
    if arg.battery or arg.all:
        tag.battery.enable()
    if arg.keypress or arg.all:
        tag.keypress.enable()
        tag.setDelegate(KeypressDelegate())
    if arg.light and tag.lightmeter is None:
        print("Warning: no lightmeter on this device")
    if (arg.light or arg.all) and tag.lightmeter is not None:
        tag.lightmeter.enable()

    # Some sensors (e.g., temperature, accelerometer) need some time for initialization.
    # Not waiting here after enabling a sensor, the first read value might be empty or incorrect.
    #time.sleep(1.0)
    time.sleep(0.01)
    file_name = str(int(time.time())) + ".json"
    f = open(file_name, "w")
    counter=0
    
    #ser=serial.Serial('/dev/ttyACM1', 115200, timeout=5)
    #ser.write("AT+SEND=123\r")
    #ser.close()

    #date = strftime("%Y/%m/%d", gmtime())
    #time = strftime("%H:%M:%S", gmtime())
    
    while True:
        date = strftime("%Y/%m/%d", gmtime())
        time = strftime("%H:%M:%S", gmtime())
        #datetime = [{"date":date,"time":time}]
        if arg.humidity or arg.all:
           #print(datetime)
           #print("Humidity:",tag.humidity.read())
           hum = tag.humidity.read()
           temp = [{"date":date,"time":time,"Temp'C":hum[0]}]
           humRH = [{"date":date,"time":time,"humRH":hum[1]}]
           ser=serial.Serial('/dev/ttyACM1', 115200, timeout=5)
           ser.write("AT+SEND="+str(json.dumps(temp))+"\r")
           ser.close()
           ser=serial.Serial('/dev/ttyACM1', 115200, timeout=5)
           ser.write("AT+SEND="+str(json.dumps(hum))+"\r")
           ser.close()
           print(temp)
           print(humRH)
           f.write(json.dumps(temp))
           f.write(json.dumps(hum))
        if arg.barometer or arg.all: 
           #print(datetime)
           #print("Barometer: ", tag.barometer.read())
           baro = tag.barometer.read()
           baro_js = [{"date":date,"time":time,"baro'C":baro[0]}]
           barohPa_js = [{"date":date,"time":time,"barohPa":baro[1]}]
           ser=serial.Serial('/dev/ttyACM1', 115200, timeout=5)
           ser.write("AT+SEND="+str(json.dumps(baro_js))+"\r"+"\r")
           ser.close()
           ser=serial.Serial('/dev/ttyACM1', 115200, timeout=5)
           ser.write("AT+SEND="+str(json.dumps(barohPa_js))+"\r"+"\r")
           ser.close()
           print(baro_js)
           print(barohPa_js)
           f.write(json.dumps(baro_js))
           f.write(json.dumps(barohPa_js))
        if arg.accelerometer or arg.all:
           #datetime = [{"date":date,"time":time}]
           #print(datetime)
           #print("Accelerometer: ", tag.accelerometer.read())
           accel = tag.accelerometer.read()
           accelX = [{"date":date,"time":time,"accelX":accel[0]}]
           accelY = [{"date":date,"time":time,"accelY":accel[1]}]
           accelZ = [{"date":date,"time":time,"accelZ":accel[2]}]
           ser=serial.Serial('/dev/ttyACM1', 115200, timeout=5)
           ser.write("AT+SEND="+str(json.dumps(accelX))+"\r"+"\r")
           ser.close()
           ser=serial.Serial('/dev/ttyACM1', 115200, timeout=5)
           ser.write("AT+SEND="+str(json.dumps(accelY))+"\r"+"\r")
           ser.close()
           ser=serial.Serial('/dev/ttyACM1', 115200, timeout=5)
           ser.write("AT+SEND="+str(json.dumps(accelZ))+"\r"+"\r")
           ser.close()
           print(accelX)
           print(accelY)
           print(accelZ)
           f.write(json.dumps(accelX))
           f.write(json.dumps(accelY))
           f.write(json.dumps(accelZ))
        if arg.magnetometer or arg.all:
           #print(datetime)
           #print("Magnetometer: ", tag.magnetometer.read())
           mag = tag.magnetometer.read()
           magX = [{"date":date,"time":time,"magX":mag[0]}]
           magY = [{"date":date,"time":time,"magY":mag[1]}]
           magZ = [{"date":date,"time":time,"magZ":mag[2]}]
           ser=serial.Serial('/dev/ttyACM1', 115200, timeout=5)
           ser.write("AT+SEND="+str(json.dumps(magX))+"\r")
           ser.close()
           ser=serial.Serial('/dev/ttyACM1', 115200, timeout=5)
           ser.write("AT+SEND="+str(json.dumps(magY))+"\r")
           ser.close()
           ser=serial.Serial('/dev/ttyACM1', 115200, timeout=5)
           ser.write("AT+SEND="+str(json.dumps(magZ))+"\r")
           ser.close()
           print(magX)
           print(magY)
           print(magZ)
           f.write(json.dumps(magX))
           f.write(json.dumps(magY))
           f.write(json.dumps(magZ))
        if arg.gyroscope or arg.all:
           #print(datetime)
           #print("Gyroscope: ", tag.gyroscope.read())
           gyro = tag.gyroscope.read()
           gyroX = [{"date":date,"time":time,"gyroX":gyro[0]}]
           gyroY = [{"date":date,"time":time,"gyroY":gyro[1]}]
           gyroZ = [{"date":date,"time":time,"gyroZ":gyro[2]}]
           ser=serial.Serial('/dev/ttyACM1', 115200, timeout=5)
           ser.write("AT+SEND="+str(json.dumps(gyroX))+"\r")
           ser.close()
           ser=serial.Serial('/dev/ttyACM1', 115200, timeout=5)
           ser.write("AT+SEND="+str(json.dumps(gyroY))+"\r")
           ser.close()
           ser=serial.Serial('/dev/ttyACM1', 115200, timeout=5)
           ser.write("AT+SEND="+str(json.dumps(gyroZ))+"\r")
           ser.close()
           print(gyroX)
           print(gyroY)
           print(gyroZ)
           f.write(json.dumps(gyroX))
           f.write(json.dumps(gyroY))
           f.write(json.dumps(gyroZ))
        if (arg.light or arg.all) and tag.lightmeter is not None:
           #print(datetime)
           #print("Light: ", tag.lightmeter.read())
           li = tag.lightmeter.read()
           liLX = [{"date":date,"time":time,"liLX":tag.lightmeter.read()}]
           ser=serial.Serial('/dev/ttyACM1', 115200, timeout=5)
           ser.write("AT+SEND="+str(json.dumps(liLX))+"\r")
           ser.close()
           print(liLX)
           f.write(json.dumps(liLX))
        if arg.battery or arg.all:
           #print(datetime)
           #print("Battery: ", tag.battery.read())
           #ba = tag.battery.read()
           battery = [{"date":date,"time":time,"battary":tag.battery.read()}]
           ser=serial.Serial('/dev/ttyACM1', 115200, timeout=5)
           ser.write("AT+SEND="+str(json.dumps(battery))+"\r")
           ser.close()
           print(battery)
           f.write(json.dumps(battery))
        if arg.all:
           X_test = pd.DataFrame({'accelX':[accel[0]],'accelY':[accel[1]],'accelZ':[accel[2]],'gyroX':gyro[0],'gyroY':gyro[1],'gyroZ':gyro[2]})
           #print(X_test)
           clf = joblib.load("finalized_model.pkl")
           predict = clf.predict(X_test)
           #print(predict)
           #print(str(predict[0]))
           status = [{"date":date,"time":time,"status":str(predict[0])}]
           print(status)
           ser=serial.Serial('/dev/ttyACM1', 115200, timeout=5)
           ser.write("AT+SEND="+str(json.dumps(status))+"\r")
           ser.close()
           
            

        if counter >= arg.count and arg.count != 0:
           break          
           #js8 = [temp,hum,baro_js,barohPa_js,accelX,accelY,accelZ,magX,gyroX,gyroY,gyroZ,liLX,battery]
           #ser=serial.Serial('/dev/ttyACM1', 115200, timeout=5)
           #ser.write("AT+SEND="+str(json.dumps(js8))+"\r")
           #ser.close()
           #f.write(json.dumps(js8))
        counter += 1
        #tag.waitForNotifications(arg.t)
        tag.waitForNotifications(0.1)
        #print("==================================")
    tag.disconnect()
    del tag

if __name__ == "__main__":
    main()
