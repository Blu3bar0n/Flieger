
import time
#import math
import KONST
import PID
import maestro

class REGELUNG():
    pidRoll = PID.PID()
    kp = 3.5
    ki = 0.5
    kd = 0.6
    pidRoll.setKp(kp)
    pidRoll.setKi(ki)
    pidRoll.setKd(kd)
    pidRoll.setWindup(0.25)

    pidPitch = PID.PID()
    kpPitch = 9.0
    kiPitch = 0.5
    kdPitch = 1.0
    pidPitch.setKp(kpPitch)
    pidPitch.setKi(kiPitch)
    pidPitch.setKd(kdPitch)
    pidPitch.setWindup(0.25)
    
    pidYaw = PID.PID()
    kpYaw = 9.0
    kiYaw = 0.5
    kdYaw = 1.0
    pidYaw.setKp(kpYaw)
    pidYaw.setKi(kiYaw)
    pidYaw.setKd(kdYaw)
    pidYaw.setWindup(0.25)
    
    acc = [0.0,0.0,0.0]
    accOhneG = [0.0,0.0,0.0]
    accogWorld = [0.0,0.0,0.0]
    vVehicle = [0.0, 0.0, 0.0]
    vWorld = [0.0, 0.0, 0.0]
    pos = [0.0, 0.0, 0.0]
    gyr = [0.0,0.0,0.0]
    gyrWorld = [0.0, 0.0,0.0]
    mag = [0.0,0.0,0.0]
    magKallib = [0.0, 0.0,0.0]
    magWorld = [0.0, 0.0, 0.0]
    istgyr = [0.0,0.0,0.0]
    trajektorie = [0.0,0.0,0.0]
    state = 0
    channel = KONST.CHDEFREG
    oldChSend = KONST.OLDCHSENDDEF
    servoCh5off = -55
    servo = maestro.Controller()
    servo.setOffset(5,servoCh5off)
    servo.setRange(5,1000+servoCh5off,2000+servoCh5off)
    print("servo gestartet")
    
    
    def ShiftInRange(self,wert,unten,oben):
        if(wert < unten):
            wert = wert + abs(oben-unten)
        if(wert > oben):
            wert = wert - abs(oben-unten)
        return wert
    
    def MaxAbsWert(wert,unten,oben):
        if wert < unten:
            wert = unten
        if wert > oben:
            wert = oben
        return wert
    
    def FiSoEx(self, arr=None):
        tmp = KONST.tmpdef
        if arr == None:
            for i in range(1,10):
                tmp[i] = self.channel[i]
        else:
            for i in range(1,10):
                if((i in arr) == False):
                    tmp[i] = self.channel[i]
        return tmp
    
    def ControllRoll(self, sollRoll):
        istRoll = self.istgyr[0]
        abweichung = sollRoll - istRoll
        abweichung = self.ShiftInRange(abweichung,-180,180)
        abweichung = abweichung/180
        #print "abweichung", abweichung
        self.pidRoll.update(abweichung)
        steuerung = self.MaxAbsWert(self.pidRoll.output,-1,1)
        if(abweichung < 0):
            if(self.gyr_raw[0] > KONST.MINWRONGANGLESPEED and abs(abweichung)*180 > 8):
                steuerung = 1
        else:
            if(self.gyr_raw[0] < (- KONST.MINWRONGANGLESPEED) and abs(abweichung)*180 > 8):
                steuerung = -1
        #print("pwm: ",pwm)
        return steuerung
        
    def ControllYaw(self, sollYaw):
        istYaw = self.istgyr[2]
        abweichung = sollYaw - istYaw
        abweichung = self.ShiftInRange(abweichung,-180,180)
        abweichung = abweichung/180
        #print "abweichung", abweichung
        self.pidYaw.update(abweichung)
        steuerung = self.MaxAbsWert(self.pidYaw.output,-1,1)
        if(abweichung < 0):
            if(self.gyr_raw[0] > KONST.MINWRONGANGLESPEED and abs(abweichung)*180 > 8):
                steuerung = 1
        else:
            if(self.gyr_raw[0] < (- KONST.MINWRONGANGLESPEED) and abs(abweichung)*180 > 8):
                steuerung = -1
        #print("pwm: ",pwm)
        return steuerung
    
    def ControllSpeed(self):
        steuerung = 0.0001 * self.istgyr[1] * self.istgyr[1] + 0.0208 * self.istgyr[1] + 0.25
        if(KONST.ethconn == True):
            steuerung = -1.0
        return steuerung
    
    def ControllPitch(self, sollPitch):
        istPitch = self.istgyr[1]
        if(sollPitch>1000):
            abweichung = 0
        else:
            abweichung = sollPitch - istPitch
        
        abweichung = self.ShiftInRange(abweichung,-180,180)
        abweichung = abweichung/180
        #print "abweichung", abweichung
        self.pidPitch.update(abweichung)
        steuerung = self.MaxAbsWert(self.pidPitch.output,-1,1)
        if(abweichung < 0):
            if(self.gyr_raw[1] > KONST.MINWRONGANGLESPEED and abs(abweichung)*180 > 8):
                steuerung = 1
        else:
            if(self.gyr_raw[1] < (- KONST.MINWRONGANGLESPEED) and abs(abweichung)*180 > 8):
                steuerung = -1
        #print("pwm: ",pwm)
        if(sollPitch>1000):
            steuerung = sollPitch / 1000 - 5
            
        return steuerung
    
    def Landeklappen (self):
        if (self.channel[7] == -1):
            self.servo.setOffset(1,0)
            self.servo.setOffset(5,0 + self.servoCh5off)
            #print "0"
        else:
            if (self.channel[10]!= 1):
                #print "100"
                self.servo.setOffset(1,-self.landelappen)
                self.servo.setOffset(5,self.landelappen + self.servoCh5off)
            else:
                self.servo.setOffset(1,0)
                self.servo.setOffset(5,0 + self.servoCh5off)

    def ServoOut(self, arr=None):
        if(arr == None):
            arr = self.channel
#        if ((abs(self.istgyr[1])> KONST.MAXPITCH - KONST.MAXPITCHPUFFER) and (self.state != 0)):
#            if (abs(self.istgyr[1])> KONST.MAXPITCH): #pitch zu krass
#                if(self.istgyr[1]> 0):
#                    self.channel[2] = -1
#                    arr[2] = self.channel[2]
#                else:
#                    self.channel[2] = 1
#                    arr[2] = self.channel[2]
#            else:
#                if(self.istgyr[1]> 0):       #pitch hoch und bewegung in die falsche Richtung
#                    if(self.gyr_raw[1] > KONST.MINWRONGANGLESPEED):
#                        self.channel[2] = -1
#                        arr[2] = self.channel[2]
#                else:
#                    if(self.gyr_raw[1] < (- KONST.MINWRONGANGLESPEED)):
#                        self.channel[2] = 1
#                        arr[2] = self.channel[2]
        #print(arr[1],  oldChSend[1])
        for i in range(1,10):
            if(arr[i] != self.oldChSend[i]):
                self.servo.setTarget(i,arr[i])
                self.oldChSend[i] = arr[i]
                #print("fbadhfbajkbfkjagssffiasgizgfija")


def Process(qparent_reg,  qchild_reg):
    reg = REGELUNG()

    print("Pids gesetzt")
    try:
       #init für kopf
        SCHRITTWEITE = 0.010
        tol = SCHRITTWEITE*1.1
        newRun = time.monotonic()+SCHRITTWEITE
        #init end kopf
        while (True):
            try:
                #kopf für while Timing
                test = newRun - time.monotonic()
                if(test>0 and test < tol):
                    time.sleep(test)
                else:
                    print("regelung test<0 oder > Schrerittweite +0.001")
                    print(test)
                    newRun =  time.monotonic()+SCHRITTWEITE
                #print("dauer für den letzten zeitschritt")
                newRun = newRun+SCHRITTWEITE#Hz Timing
                #ende kopf
                #///////////////////////////////Get Data///////////////////
                if (qchild_reg.poll() == True):
                    dataFromRegleung = qchild_reg.recv()
                    reg.state = dataFromRegleung[0][0]
                    for i in range (1, 7):
                        reg.channel[i] = dataFromRegleung[1][i]
                    for i in range(0, 3):
                        reg.gyr[i]  = dataFromRegleung[2][i]              #[state, channels, gyr,accOhneG,istgyr,vVehicle,pos, Trajektorie ]
                        reg.accOhneG[i] = dataFromRegleung[3][i]
                        reg.istgyr[i] = dataFromRegleung[4][i] 
                        reg.vVehicle[i] = dataFromRegleung[5][i]
                        reg.pos[i] = dataFromRegleung[6][i] 
                        reg.trajektorie = dataFromRegleung[7][i] 
                #/////////////////////////////////////Verarbeitung////////////////
                if(reg.state == 0):
                    reg.ServoOut()
                if(reg.state == 1):
                    p = reg.ControllPitch(0)
                    r = reg.ControllRoll(0)
                    reg.channel[1] = r
                    reg.channel[5] = r
                    reg.channel[2] = p
                    reg.ServoOut()
                if(reg.state == 2):
                    r = reg.ControllRoll(reg.trajektorie[0])
                    p = reg.ControllPitch(reg.trajektorie[1])
                    y = reg.ControllYaw(reg.trajektorie[2])
                    s = reg.ControllSpeed()
                    reg.channel[1] = r
                    reg.channel[5] = r
                    reg.channel[2] = p
                    reg.channel[4] = y
                    reg.channel[3] = s
                    reg.ServoOut()
                #///////////Send back to Log////////////////////
                if (qparent_reg.poll() == False):
                    dataFromRegleung = qchild_reg.send(reg.channel)
                
            except KeyboardInterrupt: 
                exit()
            except Exception as e:
                log = open(KONST.Filename,"a")
                errmsg = "Unexpectet Error in Regelung:" + '\n' + str(e) + '\n'
                log.write(errmsg)
                log.close()
                if(KONST.ethconn):
                    raise
                else:
                    pass
        print("In Regelung")
    except KeyboardInterrupt: 
        exit()
    except:
        print ("Unexpected Error Regelung")
        raise
