import time
import math
import KONST
import GPSDevice
from geopy import distance

#import multitasking
#import signal
#import sys
#places   degrees          distance
#-------  -------          --------
#0        1                111  km
#1        0.1              11.1 km
#2        0.01             1.11 km
#3        0.001            111  m
#4        0.0001           11.1 m
#5        0.00001          1.11 m
#6        0.000001         11.1 cm
#7        0.0000001        1.11 cm
#8        0.00000001       1.11 mm

class SENS():
    acc_raw = [0.0,0.0,0.0, 0.0, 0]
    accOhneG = [0.0,0.0,0.0]
    accogWorld = [0.0,0.0,0.0]
    vVehicle = [0.0, 0.0, 0.0]
    vWorld = [0.0, 0.0, 0.0]
    pos = [0.0, 0.0, 0.0]                    #pos[lat,lon, meter ueber ground)
    gyr_raw =[0.0,0.0,0.0, 0.0, 0]
    gyrWorld = [0.0, 0.0,0.0]
    mag_raw = [0.0,0.0,0.0, 0.0, 0]
    magKallib = [0.0, 0.0,0.0]
    magWorld = [0.0, 0.0, 0.0]
    istgyr = [0.0,0.0,0.0]
    tmonCFold = 0.0
    tmonCFoldMag = 0.0
    tmonCFoldAcc = 0.0
    tmonCFoldBarhoehe =0.0
    gps = GPSDevice.GPSDevice()
    lastValidLat = 0.0
    lastValidLon = 0.0
    lastValidTmon = 0.0
    lastValidtime = u'2018-10-02T14:55:05.000Z'
    dposLastValidOhneFilter = [0.0,0.0,0.0]
    barHoehe_raw = [0.0, 0]
    barHoehe_rawOld = 0.0
    nurBarHoehe = 0.0
    hoeheBarFilter = 0.0
    dataToSend = [[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0], [0.0]] #gyr,acc,mag,accOhneG,istgyr,vVehicle,vWorld,pos,gps,yagyr
    dataToSendReg= [[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]] #gyr,acc,accOhneG,istgyr,vVehicle,vWorld,pos
    yagyr = 0.0
    
    def ShiftInRange(self,wert,unten,oben):
        if(wert < unten):
            wert = wert + abs(oben-unten)
        if(wert > oben):
            wert = wert - abs(oben-unten)
        return wert

    def Positionsbestimmung(self):
        gV = [0, 0, 9.81]
        winkel = [self.istgyr[0] / 180 * math.pi, self.istgyr[1] / 180 * math.pi, self.istgyr[2] / 180 * math.pi]
        gV = KONST.RMatrixWeltZuFahrzeug(gV, winkel)
        #print("winkel",winkel)
        #print("gV", gV)
        for i in range(0, 3):
            self.accOhneG[i] = self.acc_raw[i] - gV[i]
        #print(self.accOhneG)
        
        for i in range(0, 3):
            self.vVehicle[i] = self.vVehicle[i] + self.acc_raw[3] * self.accOhneG[i] 
        
        self.accogWorld = KONST.RMatrixFahrzeugZuWelt(self.accOhneG, winkel)
        for i in range(0, 3):
            self.vWorld[i] = self.vWorld[i] + self.acc_raw[3] * self.accogWorld[i] 
        dx = self.vWorld[0] * self.acc_raw[3] + self.accogWorld[0] / 2 * self.acc_raw[3] * self.acc_raw[3]
        dy = self.vWorld[1] * self.acc_raw[3] + self.accogWorld[1] / 2 * self.acc_raw[3] * self.acc_raw[3]
        dx = dx/KONST.ERDRADIUS
        dy = dy/KONST.ERDRADIUS
        self.pos[0] = self.pos[0] + math.sin(dx)*180/math.pi
        self.pos[1] = self.pos[1] + math.sin(dy)*180/math.pi
        self.pos[2] = self.pos[2] + self.vWorld[2] * self.acc_raw[3] + self.accogWorld[2] / 2 * self.acc_raw[3] * self.acc_raw[3]
        #print (self.accogWorld[0], self.vWorld[0], self.pos[0])
        self.dposLastValidOhneFilter[0] = self.dposLastValidOhneFilter[0] + math.sin(dx)*180/math.pi
        self.dposLastValidOhneFilter[1] = self.dposLastValidOhneFilter[1] + math.sin(dy)*180/math.pi
        self.dposLastValidOhneFilter[2] = self.dposLastValidOhneFilter[2] + self.vWorld[2] * self.acc_raw[3] + self.accogWorld[2] / 2 * self.acc_raw[3] * self.acc_raw[3]
    
    def ComplementaryFilter(self):
        tmon = round(time.monotonic(), 2) #alle 0,01 s
        #print(tmon, self.tmonCFold)
        if(tmon-self.tmonCFold > 0.011): #alle 0,02s
            self.tmonCFold = tmon
            #print(tmon)
            accBetrag = math.sqrt(self.acc_raw[0]*self.acc_raw[0] + self.acc_raw[1]*self.acc_raw[1] + self.acc_raw[2]*self.acc_raw[2])
            #print(accBetrag)
            if (5.0 < accBetrag and accBetrag < 15.0):
                ragyr = math.atan2(self.acc_raw[1], self.acc_raw[2]) * 180 / math.pi #roll from Acc
                self.istgyr[0] = self.istgyr[0] * 0.98 + ragyr * 0.02   
                #print (ragyr , self.istgyr[0])
                pagyr = -math.atan2(self.acc_raw[0], self.acc_raw[2]) * 180 / math.pi #pitch from Acc
                self.istgyr[1] = self.istgyr[1] * 0.98 + pagyr * 0.02 
                #print (pagyr , self.istgyr[1])
        
        if(tmon-self.tmonCFoldMag > 0.141): #alle 0,15s
            self.tmonCFoldMag = tmon
            normalenVector = [0, 0, 1] # normalenvektor welcher senkrecht auf der ebene zum boden steht
            winkel = [self.istgyr[0] / 180 * math.pi, self.istgyr[1] / 180 * math.pi, self.istgyr[2] / 180 * math.pi]
            normalenVector = KONST.RMatrixWeltZuFahrzeug(normalenVector, winkel)
            multNV = 1#ist 1 spaart rechenleistung... normalenVector[0] * normalenVector[0] + normalenVector[1] * normalenVector[1] + normalenVector[2] * normalenVector[2]
            #print("normalenVector: ", normalenVector)
            faktor = ((self.mag_raw[0]) * normalenVector[0] + self.mag_raw[1] * normalenVector[1] + self.mag_raw[2] * normalenVector[2])/(multNV) #x_0 = 0 wenn kallibriert
            corrMag_raw = [0.0, 0.0, 0.0]
            corrMag_raw[0] = self.mag_raw[0] - faktor * normalenVector[0] #lot zwischen den achsen wieder in [lat, lon, m über ground] 
            corrMag_raw[1] = self.mag_raw[1] - faktor * normalenVector[1]
            corrMag_raw[2] = self.mag_raw[2] - faktor * normalenVector[2]
            
            self.yagyr = math.atan2(corrMag_raw[1], corrMag_raw[0]) * 180 / math.pi #yaw corr von mag
            #self.yagyr = math.atan2(self.mag_raw[1], self.mag_raw[0]) * 180 / math.pi #yaw von mag
            #print("yaw mag",self.yagyr)
            if(abs(self.istgyr[2] - self.yagyr)<180):
                self.istgyr[2] = self.istgyr[2] * 0.95 + self.yagyr * 0.05
            else:
                tmp_istgyr2 = self.ShiftInRange(self.istgyr[2],0,360)
                self.yagyr = self.ShiftInRange(self.yagyr,0,360)
                self.istgyr[2] = tmp_istgyr2 * 0.95 + self.yagyr * 0.05
                self.istgyr[2] = self.ShiftInRange(self.istgyr[2],-180,180)
            
    def ComplementaryFilterAcc(self):
        tmon = round(time.monotonic(), 2) #alle 0,01 s
        if(tmon-self.tmonCFoldAcc > 0.011): #alle 0,02s
            self.tmonCFoldAcc = tmon
            tmpx = self.lastValidLat + self.dposLastValidOhneFilter[0]
            tmpy = self.lastValidLon + self.dposLastValidOhneFilter[1]
            self.pos[0] = self.pos[0] * 0.98 + tmpx * 0.02
            self.pos[1] = self.pos[1] * 0.98 + tmpy * 0.02
            
    
    def VWordFilter(self):
        dt = time.monotonic() - self.lastValidTmon
        if(dt < 2.5 and dt > 0.1):
            dx = distance.distance([self.gps.lat, self.gps.lon],[self.lastValidLat, self.gps.lon]).km
            dy = distance.distance([self.gps.lat, self.gps.lon],[self.gps.lat, self.lastValidLon]).km
            vx = dx/dt * 1000
            vy = dy/dt * 1000
            if((self.gps.lat - self.lastValidLat)<0):
                vx = -vx
            if((self.gps.lon - self.lastValidLon)<0):
                vy = -vy
            #print("vx,vy", vx, vy)
            self.vWorld[0] = 0.75 * self.vWorld[0] + 0.25 * vx
            self.vWorld[1] = 0.75 * self.vWorld[1] + 0.25 * vy

    
    def CalcEuler(self):
        winkel = [self.istgyr[0] / 180 * math.pi, self.istgyr[1] / 180 * math.pi, self.istgyr[2] / 180 * math.pi]
        self.gyrWorld = KONST.WGeschwindigkeitRMatrixFahrzeugZUWelt(self.gyr_raw, winkel)
        self.istgyr[0] = self.istgyr[0] + self.gyr_raw[3] * self.gyrWorld[0] 
        self.istgyr[0] = self.ShiftInRange(self.istgyr[0],-180,180)
        #print ("istgyr[0], gyr[0]: ",self.istgyr[0],self.gyrWorld[0])
        self.istgyr[1] = self.istgyr[1] + self.gyr_raw[3] * self.gyrWorld[1] 
        self.istgyr[1] = self.ShiftInRange(self.istgyr[1],-180,180)
        #print ("istgyr[1], gyr[1]: ",self.istgyr[1],self.gyrWorld[1])
        self.istgyr[2] = self.istgyr[2] + self.gyr_raw[3] * self.gyrWorld[2] 
        self.istgyr[2] = self.ShiftInRange(self.istgyr[2],-180,180)
        #print ("istgyr[2], gyr[2]: ",self.istgyr[2],self.gyrWorld[2])
        #print(self.istgyr)
    
    def Barhoehe(self):
        deltaH = self.barHoehe_raw[0]-self.barHoehe_rawOld
        self.nurBarHoehe = self.nurBarHoehe + deltaH
        #print("self.nurBarHoehe", self.nurBarHoehe)
        self.barHoehe_rawOld = self.barHoehe_raw[0]
    
    def BarhoeheFilter(self):
        tmon = round(time.monotonic(), 2) #alle 0,01 s
        if(tmon-self.tmonCFoldBarhoehe > 0.091): #alle 0,1s
            self.pos[2] = self.pos[2] * 0.8 + self.nurBarHoehe * 0.2
            dHdt = (self.nurBarHoehe-self.hoeheBarFilter) / (tmon-self.tmonCFoldBarhoehe)
            self.hoeheBarFilter = self.nurBarHoehe
            self.vWorld[2] = 0.9 * self.vWorld[2] + 0.1* dHdt
            #print("self.accOhneG[2]", self.accOhneG[2])
            #print("self.nurBarHoehe", self.nurBarHoehe)
            #print("dHdt", dHdt)
            #print("self.vWorld[2]", self.vWorld[2])
            #print("self.pos[2]", self.pos[2])
            self.tmonCFoldBarhoehe = tmon
            
            
            #hack für vVehicle
            winkel = [self.istgyr[0] / 180 * math.pi, self.istgyr[1] / 180 * math.pi, self.istgyr[2] / 180 * math.pi]
            V = KONST.RMatrixWeltZuFahrzeug(self.vWorld, winkel)
            for i in range(0, 3):
                self.vVehicle[i] = 0.98 * self.vVehicle[i] + 0.02 * V[i]
    
def Process(qparent_sens,  qchild_sens, qparent_bmi,  qchild_bmi, q_gps, qparent_sens_reg,  qchild_sens_reg):
    try:
        print("In Sensorfusion")
        Sens = SENS()
        stateSens = -1
        while(stateSens != 0):
            if (qchild_sens.poll() == True):
                stateSens=  qchild_bmi.recv()
            if (q_gps.empty() == False):
                Sens.gps = q_gps.get()
                if(isinstance(Sens.gps.lat, float)):
                    Sens.lastValidLat = Sens.gps.lat
                    Sens.lastValidLon = Sens.gps.lon
                    Sens.lastValidTmon = time.monotonic()
                    if(Sens.pos[0] == 0):
                        Sens.pos[0] = Sens.lastValidLat
                        Sens.pos[1] = Sens.lastValidLon
                        qchild_sens.send(1)
                        print("GPS Gefunden")
                    else:
                        Sens.pos[0] = Sens.pos[0]/2 + Sens.lastValidLat/2
                        Sens.pos[1] = Sens.pos[1]/2 + Sens.lastValidLon/2
        bmi160Data =  qparent_bmi.recv()
        yagyr = math.atan2(bmi160Data[2][1], bmi160Data[2][0]) * 180 / math.pi #pitch from Acc
        print("yaw mag", yagyr)
        Sens.istgyr[2] = yagyr
        Sens.barHoehe_rawOld = bmi160Data[3][0]
        print("bmi160Data[3][0]sdfgd", bmi160Data[3][0])
        #init für kopf
        SCHRITTWEITE = 0.0050
        tol = SCHRITTWEITE*1.1
        newRun = time.monotonic()+SCHRITTWEITE
        #init end kopf
        while (True):
            try:
                #kopf für while Timing
                timetest = newRun - time.monotonic()
                if(timetest>0 and timetest < tol):
                    time.sleep(timetest)
                else:
                    #print("Sensorfusion timetest<0 oder > Schrerittweite +0.001")
                    #print(timetest)
                    newRun =  time.monotonic()+SCHRITTWEITE
                #print("dauer für den letzten zeitschritt")
                newRun = newRun+SCHRITTWEITE#Hz Timing
                #ende kopf
                #///////////////////////////////Get Data///////////////////
                if (q_gps.empty() == False):
                    Sens.gps = q_gps.get()
                    if(isinstance(Sens.gps.lat, float)):
                        Sens.VWordFilter()
                        Sens.lastValidLat = Sens.gps.lat
                        Sens.lastValidLon = Sens.gps.lon
                        Sens.lastValidTmon = time.monotonic()
                        for i in range(0, 3):
                            Sens.dposLastValidOhneFilter[i] = 0.0
                if (qparent_bmi.poll() == True):
                    bmi160Data =  qparent_bmi.recv()
                    #print("data in sens",bmi160Data[0][4] )
                    for i in range(0, 5):
                        if(bmi160Data[0][4]):
                           Sens.gyr_raw[i] = bmi160Data[0][i]
                           #print(Sens.gyr_raw[i])
                        if(bmi160Data[1][4]):
                            Sens.acc_raw[i] = bmi160Data[1][i]
                        if(bmi160Data[2][4]):
                            Sens.mag_raw[i] = bmi160Data[2][i]
                    if(bmi160Data[3][1]):
                        Sens.barHoehe_raw[0] = bmi160Data[3][0]
                        Sens.barHoehe_raw[1] = bmi160Data[3][1]
                    
                #/////////////////////////Process Data////////////////////
                if(Sens.gyr_raw[4] == 1):
                    Sens.CalcEuler()
                    Sens.gyr_raw[4] = 0
                if(Sens.acc_raw[4] == 1):
                    Sens.Positionsbestimmung()
                    Sens.acc_raw[4] = 0
                if(Sens.barHoehe_raw[1]):
                    Sens.Barhoehe()
                    Sens.barHoehe_raw[1] = 0
                #///////////////////Filter Data//////////////////
                Sens.ComplementaryFilter()
                Sens.ComplementaryFilterAcc()
                Sens.BarhoeheFilter() #enthält auch hack für vVehicle
                #////////////////////Send Data
                #daten für main
                if (qparent_sens.poll() == False):
                    for i in range(0, 3):
                        Sens.dataToSend[0][i]  = Sens.gyr_raw[i] #[gyr,acc,mag,accOhneG,istgyr,vVehicle,vWorld,pos,gps,yagyr ]
                        Sens.dataToSend[1][i]  = Sens.acc_raw[i]
                        Sens.dataToSend[2][i]  = Sens.mag_raw[i]
                        Sens.dataToSend[3][i]  = Sens.accOhneG[i]
                        Sens.dataToSend[4][i]  = Sens.istgyr[i]
                        Sens.dataToSend[5][i]  = Sens.vVehicle[i]
                        Sens.dataToSend[6][i]  = Sens.vWorld[i]
                        Sens.dataToSend[7][i]  = Sens.pos[i]
                    Sens.dataToSend[8][0]  = Sens.gps.lat
                    Sens.dataToSend[8][1]  = Sens.gps.lon
                    Sens.dataToSend[9][0]  = Sens.yagyr
                    #print("istgyr:",  Sens.istgyr)  
                    qchild_sens.send(Sens.dataToSend)
                #daten für regelung
                if (qparent_sens_reg.poll() == False):
                    for i in range(0, 3):
                        Sens.dataToSendReg[0][i]  = Sens.gyr_raw[i] #[gyr,acc,mag,accOhneG,istgyr,vVehicle,vWorld,pos,gps ]
                        Sens.dataToSendReg[1][i]  = Sens.accOhneG[i]
                        Sens.dataToSendReg[2][i]  = Sens.istgyr[i]
                        Sens.dataToSendReg[3][i]  = Sens.vVehicle[i]
                        Sens.dataToSendReg[4][i]  = Sens.vWorld[i]
                        Sens.dataToSendReg[5][i]  = Sens.pos[i]
                    #print("istgyr:",  Sens.istgyr)  
                    qchild_sens_reg.send(Sens.dataToSendReg)
                
            except KeyboardInterrupt: 
                exit()
            except Exception as e:
                log = open(KONST.Filename,"a")
                errmsg = "Unexpectet Error in Sensorfusion:" + '\n' + str(e) + '\n'
                log.write(errmsg)
                log.close()
                if(KONST.ethconn):
                    raise
                else:
                    pass
    except KeyboardInterrupt: 
        exit()
    except:
        print ("Unexpected Error BMI")
        raise
