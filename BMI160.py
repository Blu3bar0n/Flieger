import time
import smbus2
import KONST
import BME280

class BMI160():

    # Globales Koord: x-achse => Osten; y- Achse => Norden; z-Achse => unten
    # Fahrzeug Koord: x-Achse => Vorne; y-Achse => Rechts; z-Achse => unten
    BMI160_I2C_ADDR = 0x69
    BMI_PARTID = 0xD1
    BMM_PARTID = 0x32
    CMD = 0x7E
    ACC_SET_PMU_MODE = 0x11 #Sets acc power mode normal (11)/ low power (12)
    GYR_SET_PMU_MODE = 0x15 #Sets gyro power mode normal (15)/ fast start-up (16) / suspend (17)
    MAG_SET_PMU_MODE = 0x19 #Sets mag power mode suspend (18)/ normal (1A) oder 19?/ low power (1B) oder 1a?
    FOC = 0x03
    ACC_CONF = 0x40
    ACC_RANGE = 0x41
    GYR_CONF = 0x42
    GYR_RANGE = 0x43
    CONF_DATA = 0b00101010 #Output data rate: 400 Hz/ Normal bandwith parameter and undersampling parameter
    ACC_RANGE_DATA = 0b00001000 
    ACC_RANGE_DATA_IN_M_S = 8 * 9.81
    GYR_RANGE_DATA = 0x00000001 
    GYR_RANGE_DATA_IN_GRAD_SEC = 1000
    ZWEI_BYTE_MAX = 65535 
    DREI_BYTE_MAX = 16777215  
    SENSORTIME_INCREMENT = 0.000039  #39us
    FOC_CONF = 0x69
    FOC_CONF_DATA = 0b01101111 #emebles gyro fast offsset copensation x -> -1g
    MAG_CONF = 0x44
    MAG_CONF_DATA = 0b00000110
    MAG_IF_ADRESS = 0x4B
    MAG_IF_MODE = 0x4C
    MAG_IF_READ = 0x4D
    MAG_IF_WRITE = 0x4E
    MAG_IF_DATA_TO_WRITE = 0x4F
    MAG_IF_ADRESS_DATA = 0b00100000 # ???????????? 20 ist reset von register aber : 20 statt 10 da in mag if letzter wert al 0 reserviert ist
    MAG_IF_MODE_MANUAL = 0b10001101
    MAG_IF_MODE_AUTO = 0b00000011
    MAG_IF_DATA_TRIGGER_DATA = 0x06
    MAG_IF_DATA_TRIGGER_EXTERNAL_ADRESS = 0x44
    MAG_IF_READ_ADDRESS = 0x42
    MAG_POWER = 0x4B
    MAG_POWER_DATA = 0b00000001
    MAG_OPERATION_MODE = 0x4C
    MAG_OPERATION_MODE_DATA = 0b00000001

    BMM150_I2C_ADDR = 0x10
    BMM_CONTROL = MAG_OPERATION_MODE
    BMM_CONTROL_DATA = MAG_OPERATION_MODE_DATA
    BMM_STATUS = 0x48
    BMM_DATA = 0x42
    BMM_AXES_ENABLE = 0x4E
    BMM_AXES_ENABLE_DATA = 0b00000111

    BMM150_REGULAR_REPXY = 4
    BMM150_REP_XY_ADDR = 0x51
    BMM150_REGULAR_REPZ = 14
    BMM150_REP_Z_ADDR = 0x52
    BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL = -4096
    BMM150_ZAXIS_HALL_OVERFLOW_ADCVAL = -16384
    BMM150_DIG_X1 = 0x5D
    BMM150_DIG_Y1 = 0x5E
    BMM150_DIG_Z4_LSB = 0x62
    BMM150_DIG_Z4_MSB = 0x63
    BMM150_DIG_X2 = 0x64
    BMM150_DIG_Y2 = 0x65
    BMM150_DIG_Z2_LSB = 0x68
    BMM150_DIG_Z2_MSB = 0x69
    BMM150_DIG_Z1_LSB = 0x6A
    BMM150_DIG_Z1_MSB = 0x6B
    BMM150_DIG_XYZ1_LSB = 0x6C
    BMM150_DIG_XYZ1_MSB = 0x6D
    BMM150_DIG_Z3_LSB = 0x6E
    BMM150_DIG_Z3_MSB = 0x6F
    BMM150_DIG_XY2 = 0x70
    BMM150_DIG_XY1 = 0x71
    BMM150_OVERFLOW_OUTPUT_FLOAT = 0.0

    dig_x1 = 0
    dig_y1 = 0
    dig_x2 = 0
    dig_y2 = 0
    dig_z1 = 0
    dig_z2 = 0
    dig_z3 = 0
    dig_z4 = 0
    dig_xy1 = 0
    dig_xy2 = 0
    dig_xyz1 = 0



    gyr = [0.0,0.0,0.0]
    acc = [0.0,0.0,0.0]
    mag = [0.0,0.0,0.0]
    magKallib = [0.0, 0.0,0.0]
    sensortime = 0
    sensortimeOldAcc = 0
    sensortimeOldGyr = 0
    bus = smbus2.SMBus(5)
    bme280 = BME280.BME280(bus)
    kallibx = 0.0
    kalliby = 0.0
    kallibz = 0.0
    akallibx = 0.0
    akalliby = 0.0
    akallibz = 0.0
    tmonCFold = 0.0
    #temperatur = 0.0
    barhoehe = 0.0
    druck = 0.0
    #minAcc = 0.05
    tmonCFoldBME = 0.0

    dataToSend = [[0.0,0.0,0.0, 0.0, 0], [0.0,0.0,0.0, 0.0, 0], [0.0,0.0,0.0, 0.0, 0], [0.0, 0]] # gyr, acc, mag, [x,y,z,dTime,isNewData], [alt über null,isNewData]

    def Kallib(self):
        print ("kalib gestartet")
        time.sleep(1)
        print ("messung gestartet")
        self.kallibx = 0.0
        self.kalliby = 0.0
        self.kallibz = 0.0
        kalibdauer = 5
        timer = 0
        timeold = int(time.strftime("%S"))
        sumx = self.gyr[0]
        sumy = self.gyr[1]
        sumz = self.gyr[2]
        countx = 1.0
        county = 1.0
        countz = 1.0
        asumx = self.gyr[0]
        asumy = self.gyr[1]
        asumz = self.gyr[2]
        acountx = 1.0
        acounty = 1.0
        acountz = 1.0
        while(timer <= kalibdauer):
            if(timeold != int(time.strftime("%S"))):
                timer = timer +1
                timeold = int(time.strftime("%S"))
            self.ReadOnce()
            sumx = sumx + self.gyr[0]
            countx = countx + 1
            sumy = sumy + self.gyr[1]
            county = county + 1
            sumz = sumz + self.gyr[2]
            countz = countz + 1
            asumx = asumx + self.acc[0]
            acountx = acountx + 1
            asumy = asumy + self.acc[1]
            acounty = acounty + 1
            asumz = asumz + self.acc[2]
            acountz = acountz + 1
        self.kallibx = sumx/countx
        self.kalliby = sumy/county
        self.kallibz = sumz/countz
        self.akallibx = asumx/acountx
        self.akalliby = asumy/acounty
        self.akallibz = asumz/acountz - 9.81
        self.acc = [0.0,0.0,0.0]
        self.gyr = [0.0,0.0,0.0]
        self.mag = [0.0,0.0,0.0]
        print ("kalib beendet")
        print (self.kallibz)
        print (countz)
        print (sumz)



    def InitRead(self):
        print ("BMI160 read init")
        self.bus.write_byte_data(self.BMI160_I2C_ADDR,self.CMD,self.ACC_SET_PMU_MODE)
        time.sleep(0.01)
        self.bus.write_byte_data(self.BMI160_I2C_ADDR,self.CMD,self.GYR_SET_PMU_MODE)
        time.sleep(0.1)
        err = self.bus.read_byte_data(self.BMI160_I2C_ADDR,0x02)
        if (err != 0):
            print("INIT0 Error in Err-Reg [", bin(err), "]")
        self.bus.write_byte_data(self.BMI160_I2C_ADDR,self.CMD,self.MAG_SET_PMU_MODE)
        time.sleep(0.01)
        err = self.bus.read_byte_data(self.BMI160_I2C_ADDR,0x02)
        if (err != 0):
            print ("INIT1 Error in Err-Reg [", bin(err), "]")
        self.bus.write_byte_data(self.BMI160_I2C_ADDR,self.FOC_CONF,self.FOC_CONF_DATA)
        self.bus.write_byte_data(self.BMI160_I2C_ADDR,self.ACC_CONF,self.CONF_DATA)
        self.bus.write_byte_data(self.BMI160_I2C_ADDR,self.ACC_RANGE,self.ACC_RANGE_DATA)
        self.bus.write_byte_data(self.BMI160_I2C_ADDR,self.GYR_CONF,self.CONF_DATA)
        self.bus.write_byte_data(self.BMI160_I2C_ADDR,self.GYR_RANGE,self.GYR_RANGE_DATA)
        #set up bmm150
        self.bus.write_byte_data(self.BMM150_I2C_ADDR,self.MAG_POWER,  self.MAG_POWER_DATA)
        time.sleep(0.1)
        print("bmm150 running status", self.bus.read_i2c_block_data(self.BMM150_I2C_ADDR,self.MAG_POWER,1))
        self.bus.write_byte_data(self.BMM150_I2C_ADDR,self.BMM_CONTROL,  self.BMM_CONTROL_DATA)
        self.bus.write_byte_data(self.BMM150_I2C_ADDR,self.BMM_AXES_ENABLE,  self.BMM_AXES_ENABLE_DATA)
        self.bus.write_byte_data(self.BMM150_I2C_ADDR,self.BMM150_REP_XY_ADDR,  self.BMM150_REGULAR_REPXY)
        self.bus.write_byte_data(self.BMM150_I2C_ADDR,self.BMM150_REP_Z_ADDR,  self.BMM150_REGULAR_REPZ)
        self.read_trim_registers()

        self.bus.write_byte_data(self.BMI160_I2C_ADDR,0x77,0b11000000) #Enable Offset
        self.bus.write_byte_data(self.BMI160_I2C_ADDR,self.CMD,self.FOC) #unabhngig von Range_Data
        time.sleep(0.25)

        sensortime = self.bus.read_i2c_block_data(self.BMI160_I2C_ADDR,0x18,3)
        sensortime = sensortime[0] + (sensortime[1]<<8) + (sensortime[2]<<16)
        self.sensortimeOldAcc = sensortime
        self.sensortimeOldGyr = sensortime

    def ReadOnce(self):
        try:
            id = self.bus.read_byte_data(self.BMI160_I2C_ADDR, 0x00)
            #print "Device ID [", hex(id), "]"
            if (id != self.BMI_PARTID):
                print ("Wrong Device ID [", hex(id), "]")
                return
            err = self.bus.read_byte_data(self.BMI160_I2C_ADDR,0x02)
            if (err != 0):
                print ("Error in Err-Reg Read Once[", bin(err), "]")
                #self.bus.write_byte_data(self.BMI160_I2C_ADDR,self.CMD,0xB6)
                #self.InitRead()
                return
            #print "err: ", bin(err)
            sensortimel = self.bus.read_i2c_block_data(self.BMI160_I2C_ADDR,0x18,3)
            self.sensortime = sensortimel[0] + (sensortimel[1]<<8) + (sensortimel[2]<<16)
            #print "sensortime: ",sensortime
            #pmu_status = self.bus.read_byte_data(self.BMI160_I2C_ADDR,0x03)
            #print ("pmu_status: ", bin(pmu_status))
            status = self.bus.read_byte_data(self.BMI160_I2C_ADDR,0x1B)
            #print ("status: ",(0b01000000 & status))

            if ((0b01000000 & status) > 0): #neue GYR daten auslesen und auf istwinkel integrieren
                #print "new gyr data"
                gyr_raw = self.bus.read_i2c_block_data(self.BMI160_I2C_ADDR,0x0C,6)
                lgyr = [gyr_raw[0] + (gyr_raw[1]<<8), gyr_raw[2] + (gyr_raw[3]<<8), gyr_raw[4] + (gyr_raw[5]<<8)]

                faktor = 1.0 / (self.ZWEI_BYTE_MAX / 2.0) * self.GYR_RANGE_DATA_IN_GRAD_SEC
                if (lgyr[0] > self.ZWEI_BYTE_MAX/2):
                    lgyr[0] = lgyr[0] - self.ZWEI_BYTE_MAX
                if (lgyr[1] > self.ZWEI_BYTE_MAX/2):
                    lgyr[1] = lgyr[1] - self.ZWEI_BYTE_MAX
                if (lgyr[2] > self.ZWEI_BYTE_MAX/2):
                    lgyr[2] = lgyr[2] - self.ZWEI_BYTE_MAX
                #self.gyr = [lgyr[0] * faktor - self.kallibx, lgyr[1] * faktor - self.kalliby, lgyr[2] * faktor - self.kallibz]      #Sensorposition
                self.gyr = [lgyr[2] * faktor - self.kallibx, -lgyr[1] * faktor - self.kalliby, lgyr[0] * faktor - self.kallibz]     #Konvertiert in NED (North-East-Down)
                if (self.sensortime < self.sensortimeOldGyr):
                    self.sensortimeOldGyr = self.sensortimeOldGyr - self.DREI_BYTE_MAX
                deltaTime = self.sensortime - self.sensortimeOldGyr
                self.sensortimeOldGyr = self.sensortime
                deltaTime = deltaTime * self.SENSORTIME_INCREMENT

                if(self.dataToSend[0][4] == 0):
                    self.dataToSend[0][4] = 1
                    for i in range(0, 3):
                       self.dataToSend[0][i] = self.gyr[i]
                    self.dataToSend[0][3] = deltaTime
                else:
                    self.dataToSend[0][4] = 1
                    #print("gyr data in Queue")
                    for i in range(0, 3):
                       self.dataToSend[0][i] = (self.dataToSend[0][i] * self.dataToSend[0][3] + self.gyr[i] * deltaTime)/(deltaTime + self.dataToSend[0][3])
                    self.dataToSend[0][3] = deltaTime + self.dataToSend[0][3]

            if ((0b10000000 & status) > 0): # Acc auslesen
                #print ("new acc data")
                acc_raw = self.bus.read_i2c_block_data(self.BMI160_I2C_ADDR,0x12,6)
                lacc = [acc_raw[0] + (acc_raw[1]<<8), acc_raw[2] + (acc_raw[3]<<8), acc_raw[4] + (acc_raw[5]<<8)]
                faktor = 1.0 / (self.ZWEI_BYTE_MAX / 2.0) * self.ACC_RANGE_DATA_IN_M_S
                if (lacc[0]> self.ZWEI_BYTE_MAX/2):
                    lacc[0] = lacc[0] - self.ZWEI_BYTE_MAX
                if (lacc[1]> self.ZWEI_BYTE_MAX/2):
                    lacc[1] = lacc[1] - self.ZWEI_BYTE_MAX
                if (lacc[2]> self.ZWEI_BYTE_MAX/2):
                    lacc[2] = lacc[2] - self.ZWEI_BYTE_MAX
                #self.acc = [lacc[0] * faktor-self.akallibx, lacc[1] * faktor-self.akalliby, lacc[2] * faktor-self.akallibz] #Sensorposition
                self.acc = [-lacc[2] * faktor-self.akallibx, lacc[1] * faktor-self.akalliby, -lacc[0] * faktor-self.akallibz] #Konvertiert in NED (North-East-Down)
                #print ("acc x,y,z: ",self.acc)
                #print(lacc[0])
                if (self.sensortime < self.sensortimeOldAcc):
                    self.sensortimeOldAcc = self.sensortimeOldAcc - self.DREI_BYTE_MAX
                deltaTime = self.sensortime - self.sensortimeOldAcc
                self.sensortimeOldAcc = self.sensortime
                deltaTime = deltaTime * self.SENSORTIME_INCREMENT

                if(self.dataToSend[1][4] == 0):
                    self.dataToSend[1][4] = 1
                    for i in range(0, 3):
                       self.dataToSend[1][i] = self.acc[i]
                    self.dataToSend[1][3] = deltaTime
                else:
                    self.dataToSend[1][4] = 1
                    for i in range(0, 3):
                       self.dataToSend[1][i] = (self.dataToSend[1][i] * self.dataToSend[1][3] + self.acc[i] * deltaTime)/(deltaTime + self.dataToSend[1][3])
                    self.dataToSend[1][3] = deltaTime + self.dataToSend[1][3]

                #self.Positionsbestimmung(deltaTime)

        except KeyboardInterrupt: 
            exit()
        except Exception as e:
            log = open(KONST.Filename,"a")
            errmsg = "Unexpectet Error in BMI160:" + '\n' + str(e) + '\n'
            log.write(errmsg)
            log.close()
            if(KONST.ethconn):
                raise
            else:
                if (str(e) == "[Errno 6] No such device or address"):
                    self.InitRead()
                pass

    def ReadOnceMag(self):
        try:
            id = self.bus.read_byte_data(self.BMM150_I2C_ADDR, 0x40)
            #print ("Device ID BMM150[", hex(id), "]")
            if (id != self.BMM_PARTID):
                print ("Wrong Device ID BMM150[", hex(id), "]")
                return
            sensortimel = self.bus.read_i2c_block_data(self.BMI160_I2C_ADDR,0x18,3)
            self.sensortime = sensortimel[0] + (sensortimel[1]<<8) + (sensortimel[2]<<16)
            #print "sensortime: ",sensortime
            #pmu_status = self.bus.read_byte_data(self.BMI160_I2C_ADDR,0x03)
            #print ("pmu_status: ", bin(pmu_status))
            status = self.bus.read_byte_data(self.BMM150_I2C_ADDR,self.BMM_STATUS)
            #print ("status: ",(0b00000001 & status))

            if ((0b00000001 & status) > 0): #neue MAG daten auslesen 
                mag_raw = self.bus.read_i2c_block_data(self.BMM150_I2C_ADDR,self.BMM_DATA,8)
                #print("raw", mag_raw[4], (mag_raw[5]))
                mag_b = [0, 0, 0, 0, 0, 0]
                mag_b[0] = int(mag_raw[0]>>3)
                mag_b[1] = (int(mag_raw[1] & 0b01111111)<<5)
                mag_b[2] = int(mag_raw[2]>>3)
                mag_b[3] = (int(mag_raw[3] & 0b01111111)<<5)
                mag_b[4] = int(mag_raw[4]>>1)
                mag_b[5] = (int(mag_raw[5] & 0b01111111)<<7)
                #print("mag_b", mag_b[4],  mag_b[5])
                lmag = [ (mag_b[0] + mag_b[1]), (mag_b[2] + mag_b[3]),  (mag_b[4] + mag_b[5])]
                if((mag_raw[1] & 0b10000000) > 0):
                    lmag[0] = lmag[0] + self.BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL 
                if((mag_raw[3] & 0b10000000) > 0):
                    lmag[1] = lmag[1] + self.BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL 
                if((mag_raw[5] & 0b10000000) > 0):
                    lmag[2] = lmag[2] + self.BMM150_ZAXIS_HALL_OVERFLOW_ADCVAL
                rhall = (mag_raw[6]>>2) + (mag_raw[7]<<6)
                #print(lmag, rhall)
                self.mag[2] = - self.Compensate_x(lmag[0], rhall)# - self.magKallib[0]
                self.mag[1] = - self.Compensate_y(lmag[1], rhall)# - self.magKallib[1] #///////////////richtig kompensiert?
                self.mag[0] = - self.Compensate_z(lmag[2], rhall)# - self.magKallib[2]
                if(1): #1für std; 0 für callib aufnahmen
                    self.mag_n = self.mag
                    self.mag_n[0] = self.mag_n[0] -(-36.751599)
                    self.mag_n[1] = self.mag_n[1] -(2.567309)
                    self.mag_n[2] = self.mag_n[2] -(-16.930631)
                    
                    self.mag[0] = self.mag_n[0] * (0.025025) + self.mag_n[1] * (-0.000275) + self.mag_n[2] * (-0.000179)
                    self.mag[1] = self.mag_n[0] * (-0.000275) + self.mag_n[1] * (0.026307) + self.mag_n[2] * (0.000610)
                    self.mag[2] = self.mag_n[0] * (-0.000179) + self.mag_n[1] * (0.000610) + self.mag_n[2] * (0.028161)
                else:
                    log = open("/home/odroid/Desktop/ericWorkspace/Python/LOG/LogMagForCallib","a") #log Für magneto tool
                    logstr = str(self.mag[0]) + "\t" + str(self.mag[1]) + "\t" + str(self.mag[2]) +"\n"
                    print(logstr)
                    log.write(logstr)
                    log.close()

                #print("compensiert", self.mag)
                self.dataToSend[2][4] = 1
                for i in range(0, 3):
                   self.dataToSend[2][i] = self.mag[i]

        except KeyboardInterrupt: 
            exit()
        except Exception as e:
            log = open(KONST.Filename,"a")
            errmsg = "Unexpectet Error in BMI160:" + '\n' + str(e) + '\n'
            log.write(errmsg)
            log.close()
            if(KONST.ethconn):
                raise
            else:
                if (str(e) == "[Errno 6] No such device or address"):
                    self.InitRead()
                pass

    def Compensate_x(self, mag_data_x, data_rhall):
        if((mag_data_x != self.BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL) and (data_rhall != 0) and ( self.dig_xyz1 != 0)):
            process_comp_x0 = (( self.dig_xyz1) * 16384.0/ data_rhall)
            retval = (process_comp_x0 - 16384.0)
            process_comp_x1 = ( self.dig_xy2) * (retval * retval / 268435456.0)
            process_comp_x2 = process_comp_x1 + retval * ( self.dig_xy1) / 16384.0
            process_comp_x3 = ( self.dig_x2) + 160.0
            process_comp_x4 = mag_data_x * ((process_comp_x2 + 256.0) * process_comp_x3)
            retval = ((process_comp_x4 / 8192.0) + (( self.dig_x1) * 8.0)) / 16.0
        else:
            print("overflow x")
            retval = self.BMM150_OVERFLOW_OUTPUT_FLOAT
        return retval

    def Compensate_y(self,  mag_data_y, data_rhall):
        if ((mag_data_y != self.BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL) and (data_rhall != 0) and (self.dig_xyz1 != 0)) :
            process_comp_y0 = (self.dig_xyz1) * 16384.0 / data_rhall
            retval = process_comp_y0 - 16384.0
            process_comp_y1 = (self.dig_xy2) * (retval * retval / 268435456.0)
            process_comp_y2 = process_comp_y1 + retval * (self.dig_xy1) / 16384.0
            process_comp_y3 = (self.dig_y2) + 160.0
            process_comp_y4 = mag_data_y * (((process_comp_y2) + 256.0) * process_comp_y3)
            retval = ((process_comp_y4 / 8192.0) + ((self.dig_y1) * 8.0)) / 16.0
        else:
            print("overflow y")
            retval = self.BMM150_OVERFLOW_OUTPUT_FLOAT
        return retval

    def Compensate_z(self,  mag_data_z, data_rhall):
        if ((mag_data_z != self.BMM150_ZAXIS_HALL_OVERFLOW_ADCVAL) and (self.dig_z2 != 0) and (self.dig_z1 != 0) and (self.dig_xyz1 != 0) and (data_rhall != 0)):
            process_comp_z0 = (mag_data_z) - (self.dig_z4)
            process_comp_z1 = (data_rhall) - (self.dig_xyz1)
            process_comp_z2 = ((self.dig_z3) * process_comp_z1)
            process_comp_z3 = (self.dig_z1) * (data_rhall) / 32768.0
            process_comp_z4 = (self.dig_z2) + process_comp_z3
            process_comp_z5 = (process_comp_z0 * 131072.0) - process_comp_z2
            retval = (process_comp_z5 / ((process_comp_z4) * 4.0)) / 16.0
        else :
            print("overflow z")
            retval = self.BMM150_OVERFLOW_OUTPUT_FLOAT
        return retval

    def read_trim_registers(self):
        trim_x1y1 = self.bus.read_i2c_block_data(self.BMM150_I2C_ADDR,self.BMM150_DIG_X1,2)
        trim_xyz_data = self.bus.read_i2c_block_data(self.BMM150_I2C_ADDR,self.BMM150_DIG_Z4_LSB,4)
        trim_xy1xy2 = self.bus.read_i2c_block_data(self.BMM150_I2C_ADDR,self.BMM150_DIG_Z2_LSB,10)
        self.dig_x1 = trim_x1y1[0]
        self.dig_y1 = trim_x1y1[1]
        self.dig_x2 = trim_xyz_data[2]
        self.dig_y2 = trim_xyz_data[3]
        temp_msb = (trim_xy1xy2[3]) << 8
        self.dig_z1 = (temp_msb | trim_xy1xy2[2])
        temp_msb = (trim_xy1xy2[1]) << 8
        self.dig_z2 = (temp_msb | trim_xy1xy2[0])
        temp_msb = (trim_xy1xy2[7]) << 8
        self.dig_z3 = (temp_msb | trim_xy1xy2[6])
        temp_msb = (trim_xyz_data[1]) << 8
        self.dig_z4 = (temp_msb | trim_xyz_data[0])
        self.dig_xy1 = trim_xy1xy2[9]
        self.dig_xy2 = trim_xy1xy2[8]
        temp_msb = ((trim_xy1xy2[5] & 0x7F)) << 8
        self.dig_xyz1 = (temp_msb | trim_xy1xy2[4])

    def MagKallib(self, qchild_bmi):
        self.magKallib = [0.0, 0.0,0.0]
        timeold = time.monotonic()
        sumvorne = [0.0, 0.0, 0.0]
        count = 0
        while((time.monotonic() - timeold) < 1.5):
            self.ReadOnceMag()
            for i in range(0, 3):
                sumvorne[i] = sumvorne[i] + self.mag[i]
            count = count + 1
        for i in range(0, 3):
            sumvorne[i] = sumvorne[i] / count
            print(sumvorne[i], count)
        qchild_bmi.send(1)
        print("bmm Voren Done")

        while( qchild_bmi.poll() == False):
            time.sleep(0.01)
            self.ReadOnceMag()
        while( qchild_bmi.poll() == True):
            qchild_bmi.recv()

        timeold = time.monotonic()
        sumrechts = [0.0, 0.0, 0.0]
        count = 0
        while((time.monotonic() - timeold) < 1.5):
            self.ReadOnceMag()
            for i in range(0, 3):
                sumrechts[i] = sumrechts[i] + self.mag[i]
            count = count + 1
        for i in range(0, 3):
            sumrechts[i] = sumrechts[i] / count
            print(sumrechts[i], count)
        qchild_bmi.send(2)
        print("bmm Rechts Done")

        while( qchild_bmi.poll() == False):
            time.sleep(0.01)
            self.ReadOnceMag()
        while( qchild_bmi.poll() == True):
            qchild_bmi.recv()

        timeold = time.monotonic()
        sumhinten = [0.0, 0.0, 0.0]
        count = 0
        while((time.monotonic() - timeold) < 1.5):
            self.ReadOnceMag()
            for i in range(0, 3):
                sumhinten[i] = sumhinten[i] + self.mag[i]
            count = count + 1
        for i in range(0, 3):
            sumhinten[i] = sumhinten[i] / count
            print(sumhinten[i], count)
        qchild_bmi.send(3)
        print("bmm Hinten Done")

        while( qchild_bmi.poll() == False):
            time.sleep(0.01)
            self.ReadOnceMag()
        while( qchild_bmi.poll() == True):
            qchild_bmi.recv()

        timeold = time.monotonic()
        sumlinks = [0.0, 0.0, 0.0]
        count = 0
        while((time.monotonic() - timeold) < 1.5):
            self.ReadOnceMag()
            for i in range(0, 3):
                sumlinks[i] = sumlinks[i] + self.mag[i]
            count = count + 1
        for i in range(0, 3):
            sumlinks[i] = sumlinks[i] / count
            print(sumlinks[i], count)
        qchild_bmi.send(4)
        print("bmm Links Done")
        for i in range(0, 3):
            self.magKallib[i] = (sumvorne[i] + sumrechts[i] + sumhinten[i] + sumlinks[i]) / 4

        kallibtxt = open(KONST.MAGKALLIBFILENAME,"w")
        print("self.magKallib", self.magKallib)
        print(sumvorne)
        print(sumrechts)
        print(sumhinten)
        print(sumlinks)
        msg = str(self.magKallib[0]) + '\n' + str(self.magKallib[1]) + '\n' + str(self.magKallib[2]) + '\n'
        kallibtxt.write(msg)
        kallibtxt.close()
    
    def get_altitude(self, pressure, seaLevel):
        atmospheric = pressure / 100.0
        return 44330.0 * (1.0 - pow(atmospheric/seaLevel, 0.1903))
    
    def ReadOnceBME280(self):
        tmon = round(time.monotonic(), 2) #alle 0,01 s
        if(tmon-self.tmonCFoldBME != 0.0): 
            self.tmonCFoldBME = tmon
            #self.temperatur = self.bme280.read_temperature()
            self.druck = self.bme280.read_pressure()
            self.barhoehe = self. get_altitude(self.druck, 1024.25)
            self.dataToSend[3][0] = self.barhoehe
            self.dataToSend[3][1] = 1
            #print("self.barhoehe", self.barhoehe)

def Process(qchild_bmi, qparent_bmi):
    print ("in Prozess BMI")
    bmi160 = BMI160()
    bmi160.InitRead()
    while(bmi160.gyr[2] == 0):
        bmi160.ReadOnce()
    time.sleep(0.5)
    bmi160.Kallib()
    qchild_bmi.send(0)
    kallibtxt = open(KONST.MAGKALLIBFILENAME,"r")
    bmi160.magKallib[0] = float(kallibtxt.readline())
    bmi160.magKallib[1] = float(kallibtxt.readline())
    bmi160.magKallib[2] = float(kallibtxt.readline())
    kallibtxt.close()
    while(qchild_bmi.poll() == False):
        time.sleep(0.01)
        magkallibstate = -1
    while(magkallibstate != 0):
        bmi160.ReadOnceMag()
        if (qchild_bmi.poll() == True):
            magkallibstate =  qchild_bmi.recv()
        #print("hkjhkadfjafbdkjghskdhgjksdhkdjhgkj")
        if(magkallibstate==1):
            bmi160.MagKallib(qchild_bmi)
            magkallibstate  = -1
    print("bmi160.magKallib", bmi160.magKallib)
    try:
        print ("BMI einsatzbereit")
        #timeold = 0.0
        while(True): # dauer pro schleife < 0,0034 s
            #print(time.monotonic()-timeold)
            #timeold = time.monotonic()
            #print("asdfnkjasdjkndkjf")
            bmi160.ReadOnce()
            bmi160.ReadOnceMag()
            bmi160.ReadOnceBME280()
            if(bmi160.dataToSend[0][4] == 1 or bmi160.dataToSend[1][4] == 1 or bmi160.dataToSend[2][4] == 1 or bmi160.dataToSend[3][1] == 1):
                if (qparent_bmi.poll() == False):
                    qchild_bmi.send(bmi160.dataToSend)
                    #print("bmi send")
                    for i in range(0, 3):
                        bmi160.dataToSend[i][4] = 0
                        #print(i, bmi160.dataToSend[1][i])
                    bmi160.dataToSend[3][1] = 0
    except KeyboardInterrupt: 
        exit()
    except:
        print ("Unexpected Error BMI")
        raise
