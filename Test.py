import FSiA10B
import GPSDevice
import BMI160
#import sys
import time
#import os
import multiprocessing as mp
import LED
import KONST
import h5py
from pathlib import Path
import Sensorfusion
import Regelung
import Trajektorie
import math

eth = open("/sys/class/net/eth0/operstate","r")
streth = eth.read(2)
eth.close()
print (streth)
if(streth == "up"):
    KONST.ethconn = True
landelappen = 200
maxSpeed = 75
minSpeedForDropSteuerung = 15
taster = [0, 0]
lastValidLat = 48.807
lastValidLon = 9.262
lastValidCours = 0.0
lastValidSpeed = 0.0

print("start")


#fsia10b = FSiA10B.FSiA10B()
channel = KONST.CHDEF
chSend = KONST.CHSENDDEF
gps = GPSDevice.GPSDevice()
sens = Sensorfusion.SENS()
trajek = Trajektorie.TRAJEKTORIE()
print("obj generiert")

maxAbsGyr = [0.0,0.0,0.0]
maxAbsAcc = [0.0,0.0,0.0]
geschrieben = False
kalibdone = False

#init log
switchOld = -1
VRBOld = 0
countLogfile = 0
FILENAME = "/home/odroid/Desktop/ericWorkspace/Python/LOG/Log: "
DATUM = time.strftime("%Y:%m:%d")
FILENAME = FILENAME  + DATUM +" " 
check_file = Path(FILENAME +str(countLogfile) + ".txt")
while (check_file.is_file()):
    countLogfile = countLogfile + 1
    check_file = Path(FILENAME +str(countLogfile) + ".txt")
FILENAMEH5 = FILENAME + str(countLogfile) + "h5" + ".hdf5"
FILENAME = FILENAME + str(countLogfile) + ".txt"
KONST.Filename = FILENAME
fileH5 = h5py.File(FILENAMEH5, 'w')
h5group_ChannelUndWinkel = fileH5.create_group(u"ChannelUndWinkel")
maxH5countH5cuw = 64000 #mit 50hz reicht das 21min
maxH5ycuw = 36
dsetH5cuw = h5group_ChannelUndWinkel.create_dataset("ch1-5, istgyr1-3", (maxH5countH5cuw,maxH5ycuw), chunks=True)#, dtype='float64') #mit 100hz reicht das 1h
countH5cuw = 0
tmonCFoldH5 = round(time.monotonic(), 2)
if(tmonCFoldH5 > 2000000.0):
    print("Monotonic overflow! Warnung")
    exit()
#timeOld = 0

#print(type(maxAbsGyr[1]), dsetH5cuw.dtype)
logstr = ""
# init log ende

dataForRegleung = [[0.0], [0.0, 0.0,0.0,-1.0,0.0,0.0,-1.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0], [0.0,0.0,0.0],[0.0,0.0,0.0] ] #[state, channels, gyr,accOhneG,istgyr,vVehicle,pos, Trajektorie ]

#////////////////////////////funktionen//////////////////////////////////
def FiSoEx(arr=None):
    tmp = KONST.tmpdef
    if arr == None:
        for i in range(1,10):
            tmp[i] = channel[i]
    else:
        for i in range(1,10):
            if((i in arr) == False):
                tmp[i] = channel[i]
    return tmp

def ShiftInRange(wert,unten,oben):
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


def SleepAndFlushPipe(pipe):
    while(pipe.poll):
        pipe.recv()

#////////////////////////////funktionen//////////////////////////////////
if __name__ == '__main__':
    
    mp.set_start_method('spawn') #nur einmal angeben!!
    q_gps = mp.Queue()
    p_gps = mp.Process(target=GPSDevice.Process,  args=(q_gps, ))
    p_gps.start()
    print("GPS alive: ", p_gps.is_alive())
    qparent_fs,  qchild_fs = mp.Pipe()
    p_fs = mp.Process(target=FSiA10B.Process,  args=(qchild_fs,qparent_fs ))
    p_fs.start()
    print("FSiA10B alive: ", p_fs.is_alive())
    qparent_bmi,  qchild_bmi = mp.Pipe()
    p_bmi = mp.Process(target=BMI160.Process,  args=(qchild_bmi, qparent_bmi))
    p_bmi.start()
    print("BMI160 alive: ", p_bmi.is_alive())
    qparent_led,  qchild_led = mp.Pipe()
    p_led = mp.Process(target=LED.Process,  args=(qchild_led, qparent_led))
    p_led.start()
    print("Led alive: ", p_led.is_alive())
    qparent_sens,  qchild_sens = mp.Pipe()
    p_sens = mp.Process(target=Sensorfusion.Process,  args=(qparent_sens,  qchild_sens, qparent_bmi,  qchild_bmi, q_gps))
    p_sens.start()
    print("Sensorfusion alive: ", p_sens.is_alive())
    qparent_reg,  qchild_reg = mp.Pipe()
    p_reg = mp.Process(target=Regelung.Process,  args=(qparent_reg,  qchild_reg))
    p_reg.start()
    print("Regelung alive: ", p_reg.is_alive())
    print("Prozesse gestartet")
    
    print("Standby, trck beide Taster um fortzufahren")
    kallibstate = -1
    while(taster[0] != KONST.BT):
        if (qparent_bmi.poll() == True):
            kallibstate =  qparent_bmi.recv()
        if(kallibstate==0 and gps.lat != None):
            if (qparent_led.poll() == True):
                taster =  qparent_led.recv()
                #print(taster)
            if (qchild_led.poll() == False):
                qparent_led.send(KONST.standby)
            if (taster[0] == KONST.KT):
                chSend = KONST.CHSENDDEF
            if (taster[0] == KONST.TL):
                chSend = [0.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]
            if (taster[0] == KONST.TR):
                chSend = [0.0,1.0,1.0,-1.0,1.0,1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]
            #print(taster[0])
            #print(chSend[1])
            if(qchild_reg.poll() == False):
                dataForRegleung[0][0] = 0
                for i in range (1, 7):
                   dataForRegleung[1][i] = chSend[i]
                qparent_reg.send(dataForRegleung)
        else:
            qparent_led.send(KONST.kallib)
        time.sleep(0.05)
    channel = KONST.CHDEF
    #/////////////////////////MAG Kallib ///////////////////////////
    print("MAG Kallib")
    tasterReleased = 0
    gpsUp = 0
    while(taster[0] != KONST.TL or tasterReleased == 0 or kallibstate != 0 ):#or gpsUp == 0):
        if (qparent_led.poll() == True):
            taster =  qparent_led.recv()
        if (taster[0] == KONST.KT):
            tasterReleased = 1
        if (qparent_sens.poll() == True):
            gpsUp = qparent_sens.recv()
        if (qchild_led.poll() == False):
            if(gpsUp == 0):
                qparent_led.send(KONST.magkallib)
            else:
                qparent_led.send(KONST.magkallibwpos)
        if(tasterReleased == 1 and taster[0] == KONST.TRL):#Start MAG Kallib und Vorne
            print("Start MAG KAllib => VORNE")
            qparent_bmi.send(1)
            qparent_led.send(KONST.kallib)
            time.sleep(1)
            while( qparent_bmi.poll() == False):
                time.sleep(0.01)
            qparent_led.send(KONST.magkallib2)
            kallibstate =  qparent_bmi.recv()
            print("MAG KAllib => RECHTS")
            while(taster[0] != KONST.KT):
                if (qparent_led.poll() == True):
                    taster =  qparent_led.recv()
            while(taster[0] != KONST.TR):#Rechts
                if (qparent_led.poll() == True):
                    taster =  qparent_led.recv()
            qparent_bmi.send(2)
            qparent_led.send(KONST.kallib)
            time.sleep(1)
            while( qparent_bmi.poll() == False):
                time.sleep(0.01)
            qparent_led.send(KONST.magkallib3)
            kallibstate =  qparent_bmi.recv()
            print("MAG KAllib => HINTEN")
            while(taster[0] != KONST.KT):
                if (qparent_led.poll() == True):
                    taster =  qparent_led.recv()
            while(taster[0] != KONST.TR): #Hinten
                if (qparent_led.poll() == True):
                    taster =  qparent_led.recv()
            qparent_bmi.send(3)
            qparent_led.send(KONST.kallib)
            time.sleep(1)
            while( qparent_bmi.poll() == False):
                time.sleep(0.01)
            qparent_led.send(KONST.magkallib4)
            kallibstate =  qparent_bmi.recv()
            print("MAG KAllib => LINKS")
            while(taster[0] != KONST.KT):
                if (qparent_led.poll() == True):
                    taster =  qparent_led.recv()
            while(taster[0] != KONST.TR): #Links
                if (qparent_led.poll() == True):
                    taster =  qparent_led.recv()
            qparent_bmi.send(4)
            qparent_led.send(KONST.kallib)
            time.sleep(1)
            while( qparent_bmi.poll() == False):
                time.sleep(0.01)
            qparent_led.send(KONST.magkallib)
            kallibstate =  qparent_bmi.recv()
            if(kallibstate != 4):
                print("ERROR ind der Kallib")
            else:
                kallibstate = 0
            print("MAG KAllib ENDE")
    #////////ENDE KALLIBS ///////////////////////////////////////////////
    qparent_bmi.send(0)
    qparent_bmi.send(0)
    qparent_sens.send(0)
    qparent_led.send(KONST.manuell)
    #qparent_sens.send(1)
    try:
        print("while start")
        while(True ):
            try:
                
                #print("run")
#                if (q_gps.empty() == False):
#                    gps = q_gps.get()
#                    if(isinstance(gps.lat, float)):
#                        lastValidLat = gps.lat
#                        lastValidLon = gps.lon
#                    if(isinstance(gps.cours, float)):
#                        lastValidCours = gps.cours
#                    if(isinstance(gps.speed, float)):
#                        lastValidSpeed = gps.speed
                if (qparent_fs.poll() == True):
                   channel =  qparent_fs.recv()
#                if (qparent_bmi.poll() == True):
#                   bmi160 =  qparent_bmi.recv()
                if (qparent_reg.poll() == True):
                    chSend =  qparent_reg.recv()
                if (qparent_led.poll() == True):
                    taster =  qparent_led.recv()
                if (qparent_sens.poll() == True):
                    #print(qparent_sens.recv())
                    recvData =  qparent_sens.recv()
                    for i in range(0, 3):
                        sens.gyr_raw[i]  = recvData[0][i] #[gyr,acc,mag,accOhneG,istgyr,vVehicle,vWorld,pos,gps ]
                        sens.acc_raw[i]  = recvData[1][i]
                        sens.mag_raw[i]  = recvData[2][i]
                        sens.accOhneG[i]  = recvData[3][i]
                        sens.istgyr[i]  = recvData[4][i]
                        sens.vVehicle[i]  = recvData[5][i]
                        sens.vWorld[i]  = recvData[6][i]
                        sens.pos[i]  = recvData[7][i]
                    sens.gps.lat = recvData[8][0]
                    sens.gps.lon  = recvData[8][1]
                    #print("pos:", sens.pos)
                    #print("Vehicle:", sens.vVehicle)
                    #qparent_sens.send(1)
                    for i in range(0, 3):
                        dataForRegleung[2][i] = sens.gyr_raw[i]                #[state, channels, gyr,accOhneG,istgyr,vVehicle,pos, Trajektorie ]
                        dataForRegleung[3][i] = sens.accOhneG[i]
                        dataForRegleung[4][i] = sens.istgyr[i]
                        dataForRegleung[5][i] = sens.vVehicle[i]
                        dataForRegleung[6][i] = sens.pos[i]
                #print(channel[10])
                if (channel[10]== -1):    #//////////////////////////////SWC oben
                    qparent_led.send(KONST.manuell)
                    if (switchOld != -1):
                        logstr = logstr + "Flugsteuerung: Manuell"+'\n'
                        switchOld = -1
                    if(qchild_reg.poll() == False):
                        dataForRegleung[0][0] = 0
                        for i in range (1, 7):
                           dataForRegleung[1][i] = channel[i]
                        qparent_reg.send(dataForRegleung)
                    
                if (channel[10] == 0):    #//////////////////////////////////////SWC mitte
                    qparent_led.send(KONST.halbAutonom)
                    if (switchOld != 0):
                        logstr = logstr +"Flugsteuerung: roll control"+'\n'
                        switchOld = 0
                    if(qchild_reg.poll() == False):
                        dataForRegleung[0][0] = 1
                        for i in range (1, 7):
                           dataForRegleung[1][i] = channel[i]
                        qparent_reg.send(dataForRegleung)
                
                if (channel[10] == 1):#/////////////////////////////////SWC unten
                    qparent_led.send(KONST.vollAutonom)
                    if (switchOld != 1):
                        logstr = logstr + "Flugsteuerung: autonom"+'\n'
                        switchOld = 1
                        trajek.SetOldWaypoint(sens.pos)
                        n_Wp = [0.0, 0.0, 0.0, 0.0, 0.0]
                        n_Wp[0] = sens.pos[0] + math.cos(sens.istgyr[2] / 180 * math.pi) *  0.004 #440m
                        n_Wp[1] = sens.pos[1] + math.sin(sens.istgyr[2] / 180 * math.pi) *  0.004 #440m
                        n_Wp[2] = sens.pos[2]
                        n_Wp[4] = 1
                        trajek.SetNewWaypoint(n_Wp)
#                    if (channel[8] == 1 and VRBOld != channel[6]):
#                        delta = VRBOld - channel[6]
#                        VRBOld = channel[6]
#                        kp = kp + delta * 3.0
#                        if (kp < 0):
#                            kp = 0.0
#                        pidRoll.setKp(kp)
#                        logstr = logstr + "Neuer Kp: "+str(kp)+'\n'
#                    if (channel[9] == 1 and VRBOld != channel[6]):
#                        delta = VRBOld - channel[6]
#                        VRBOld = channel[6]
#                        ki = ki + delta * 3.0
#                        if (ki < 0):
#                            ki = 0.0
#                        pidRoll.setKp(ki)
#                        logstr = logstr + "Neuer Ki: "+str(ki)+'\n'
#                    if (channel[7]== 1 and VRBOld != channel[6]):
#                        delta = VRBOld - channel[6]
#                        VRBOld = channel[6]
#                        kd = kd + delta * 3.0
#                        if (kd < 0):
#                            kd = 0.0
#                        pidRoll.setKp(ki)
#                        logstr = logstr + "Neuer Kd"+str(kd)+'\n'
#                    if (channel[7] == -1 and channel[8] == -1 and channel[9] == -1):
#                        VRBOld = channel[6]
                    dataForRegleung[7] = trajek.StupidControl(sens.pos, sens.istgyr)
                    if(qchild_reg.poll() == False):
                        dataForRegleung[0][0] = 2
                        for i in range (1, 7):
                           dataForRegleung[1][i] = channel[i]
                        qparent_reg.send(dataForRegleung)
                
                # ///////////////////////////////////log///////////////////
                for i in range (0,3):
                    if (maxAbsAcc[i] < abs(sens.acc_raw[i])):
                        maxAbsAcc[i] = abs(sens.acc_raw[i])
                    if (maxAbsGyr[i] < abs(sens.gyr_raw[i])):
                        maxAbsGyr[i] = abs(sens.gyr_raw[i])
                    i = i+1
                if (int(time.strftime("%S"))%5 == 0):
                    if (geschrieben == False):
                        logstr = logstr +time.strftime("%H:%M:%S")+'\n'
                        for i in range (0,3):
                            logstr = logstr + "    max acc, gyr "+str(i)+" : "+str(maxAbsAcc[i])+"	"+str(maxAbsGyr[i])+'\n'
                            maxAbsAcc[i] = 0.0
                            maxAbsGyr[i] = 0.0
                            #print "max acc, gyr",i,":",maxAbsAcc[i],maxAbsGyr[i]
                            i = i + 1
                        logstr = logstr + "         Lat: "+str(sens.gps.lat)+" posx: "+str(sens.pos[0])+'\n'
                        logstr = logstr + "         Lon: "+str(sens.gps.lon)+" posy: "+str(sens.pos[1])+'\n'
                        logstr = logstr + "         Speed: "+str(sens.gps.speed)+'\n'
                        logstr = logstr + "         Alt: "+str(sens.gps.alt)+" posz: "+str(sens.pos[2])+'\n'
                        print(logstr)
                        log = open(FILENAME,"a")
                        log.write(logstr)
                        log.close()
                        logstr = ""
                        geschrieben = True
                        eth = open("/sys/class/net/eth0/operstate","r")
                        streth = eth.read(2)
                        eth.close()
                        if(streth == "up"):
                            KONST.ethconn = True
                        else:
                            KONST.ethconn = False
                else:
                    geschrieben = False
                tmonH5 = round(time.monotonic(), 2) #100Hz
                if(tmonH5-tmonCFoldH5 > 0.03):#25Hz
                    tmonCFoldH5 = tmonH5
                    if (countH5cuw<maxH5countH5cuw):
                        dsetH5cuw[countH5cuw, 0] = chSend[1]
                        dsetH5cuw[countH5cuw, 1] = chSend[2]
                        dsetH5cuw[countH5cuw, 2] = chSend[3]
                        dsetH5cuw[countH5cuw, 3] = chSend[4]
                        dsetH5cuw[countH5cuw, 4] = chSend[5]
                        dsetH5cuw[countH5cuw, 5] = taster[1]
                        for i in range(0, 3):
                            dsetH5cuw[countH5cuw, 6+i] = sens.istgyr[i]
                            dsetH5cuw[countH5cuw, 9+i] = sens.gyr_raw[i]
                            dsetH5cuw[countH5cuw, 12+i] = sens.acc_raw[i]
                            dsetH5cuw[countH5cuw, 15+i] = sens.accOhneG[i]
                            dsetH5cuw[countH5cuw, 18+i] = sens.accogWorld[i]
                            dsetH5cuw[countH5cuw, 21+i] = sens.vVehicle[i]
                            dsetH5cuw[countH5cuw, 24+i] = sens.vWorld[i]
                            dsetH5cuw[countH5cuw, 27+i] = sens.pos[i]
                        if(isinstance(sens.gps.lat, float)):
                            dsetH5cuw[countH5cuw, 30] = sens.gps.lat
                            dsetH5cuw[countH5cuw, 31] = sens.gps.lon
                        dsetH5cuw[countH5cuw, 35] = channel[10] + 2
                        #print("asfafsaf")
                        #print(countH5cuw, dsetH5cuw[countH5cuw, 35])
                        #print("x", sens.accogWorld[0]*100)
                        #print("y", sens.accogWorld[1]*100)
                        countH5cuw = countH5cuw + 1
                #//////////////////////////////log//////////////////////////////
                #print(time.monotonic()-timeOld)
                #timeOld = time.monotonic()
                if(taster[0] == KONST.BTL):
                    raise KeyboardInterrupt
            except KeyboardInterrupt:
                print("Keyboarinterrupt1")
                qparent_led.send(KONST.DEAD)
                raise
            except Exception as e:
                log = open(FILENAME,"a")
                errmsg = "Unexpectet Error in Main:" + '\n' + str(e)
                log.write(errmsg)
                log.close()
                if(KONST.ethconn):
                    raise
                else:
                    if(qchild_reg.poll() == False):
                        dataForRegleung[0][0] = 0
                        for i in range (1, 7):
                           dataForRegleung[1][i] = KONST.CHDEF[i]
                        qparent_reg.send(dataForRegleung)
                    pass

    except KeyboardInterrupt:
        print("Keyboarinterrupt2")
#        if (countH5cuw<maxH5countH5cuw):
#            dsetH5cuw.resize((countH5cuw, maxH5ycuw))
        print("v close")
        fileH5.close()
        print("n close")
        p_gps.join()
        p_fs.join()
        p_bmi.join()
        p_led.join()
        dataForRegleung[0][0] = 0
        for i in range (1, 7):
           dataForRegleung[1][i] = KONST.CHDEF[i]
        qparent_reg.send(dataForRegleung)
        print("Programmm beendet")
        exit()
fileH5.close()
    
