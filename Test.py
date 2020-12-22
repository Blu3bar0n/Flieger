import FSiA10B
import GPSDevice
import BMI160
import sys
import time
#import os
import multiprocessing as mp
import LED
import GUI
import KONST
import h5py
from pathlib import Path
import Sensorfusion
if ("noREG" in str(sys.argv)):
    print("REG nicht initialisiert")
else:
    import Regelung
import Trajektorie
import math
#from colorama import Fore, Style 

try:
    eth = open("/sys/class/net/eth0/operstate","r")
    streth = eth.read(2)
    #print (streth)
    eth.close()
except FileNotFoundError:
    print("not Found: /sys/class/net/eth0/operstate")
if(streth == "up"):
    KONST.ethconn = True
else:
    try:
        eth = open("/sys/class/net/wlan0/operstate","r")
        streth = eth.read(2)
        eth.close()
    except FileNotFoundError:
        print("not Found: /sys/class/net/wlan0/operstate")
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
n_Wp = [0.0, 0.0, 0.0, 0.0, 0.0]

#print("start")


#fsia10b = FSiA10B.FSiA10B()
channel = KONST.CHDEF
chSend = KONST.CHSENDDEF
gps = GPSDevice.GPSDevice()
sens = Sensorfusion.SENS()
trajek = Trajektorie.TRAJEKTORIE()
#print("obj generiert")

maxAbsGyr = [0.0,0.0,0.0]
maxAbsAcc = [0.0,0.0,0.0]
geschrieben = False
kalibdone = False
dposStupidControll = [0.0, 0.0, 0.0, 0.0, 0.0] # in m

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
oldLogstr = ""
# init log ende

dataForRegleung = [[0.0], [0.0, 0.0,0.0,-1.0,0.0,0.0,-1.0],[0.0,0.0,0.0], [0.0] ] #[state, channels, Trajektorie, zusatzservo ]

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
    if ("noGPS" in str(sys.argv)):
        print("GPS nicht gestartet")
    else:
        p_gps.start()
    print("GPS alive: ", p_gps.is_alive())
    qparent_fs,  qchild_fs = mp.Pipe()
    p_fs = mp.Process(target=FSiA10B.Process,  args=(qchild_fs,qparent_fs ))
    if ("noFSiA10B" in str(sys.argv)):
        print("FSiA10B nicht gestartet")
    else:
        p_fs.start()
    print("FSiA10B alive: ", p_fs.is_alive())
    qparent_bmi,  qchild_bmi = mp.Pipe()
    p_bmi = mp.Process(target=BMI160.Process,  args=(qchild_bmi, qparent_bmi))
    if ("noBMI" in str(sys.argv)):
        print("BMI nicht gestartet")
    else:
        p_bmi.start()
    print("BMI160 alive: ", p_bmi.is_alive())
    qparent_led,  qchild_led = mp.Pipe()
    p_led = mp.Process(target=LED.Process,  args=(qchild_led, qparent_led))
    if ("noLED" in str(sys.argv)):
        print("LED nicht gestartet")
    else:
        p_led.start()
    print("Led alive: ", p_led.is_alive())
    qparent_sens,  qchild_sens = mp.Pipe()
    qparent_sens_reg,  qchild_sens_reg = mp.Pipe()
    p_sens = mp.Process(target=Sensorfusion.Process,  args=(qparent_sens,  qchild_sens, qparent_bmi,  qchild_bmi, q_gps, qparent_sens_reg,  qchild_sens_reg))
    if ("noSENS" in str(sys.argv)):
        print("SENS nicht gestartet")
    else:p_sens.start()
    print("Sensorfusion alive: ", p_sens.is_alive())
    qparent_reg,  qchild_reg = mp.Pipe()
    if ("noREG" in str(sys.argv)):
        print("REG nicht gestartet")
    else:
        p_reg = mp.Process(target=Regelung.Process,  args=(qparent_reg,  qchild_reg, qparent_sens_reg,  qchild_sens_reg))
        p_reg.start()
        print("Regelung alive: ", p_reg.is_alive())
    qparent_gui,  qchild_gui = mp.Pipe()
    p_gui = mp.Process(target=GUI.Process,  args=(qparent_gui, qchild_gui))
    if ("noGUI" in str(sys.argv)):
        print("GUI nicht gestartet")
    else:
        p_gui.start()
    print("GUI alive: ", p_gui.is_alive())
    print("Prozesse gestartet")
    
    print("Standby, tuerck beide Taster um fortzufahren")
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
            if (qchild_reg.poll() == False):
                qparent_led.send(KONST.kallib)
        if (qparent_sens.poll() == True):
            qparent_sens.recv()
        time.sleep(0.05)
    channel = KONST.CHDEF
    for i in range (1, 7):
       dataForRegleung[1][i] = channel[i]
    qparent_reg.send(dataForRegleung)

 #////////ENDE KALLIBS ///////////////////////////////////////////////
    qparent_bmi.send(0)
    qparent_bmi.send(0)
    qparent_sens.send(0)
    qparent_led.send(KONST.manuell)
    #qparent_sens.send(1)
    #/////////////////wait for gps or override///////////////////////
    print("wait for gps oder ignorieren durch halten des linken tasters")
    gpsUp = 0
    while(taster[0] != KONST.TLL and gpsUp == 0):
        if (qparent_led.poll() == True):
            taster =  qparent_led.recv()
        if (qparent_sens.poll() == True):
            #print(qparent_sens.recv())
            recvData =  qparent_sens.recv()
            for i in range(0, 3):
                #print("i", i)
                #print("type(sens.gyr_raw[i])", type(sens.gyr_raw[i]))
                #print("type(recvData[0][i])", type(recvData[0][i]))
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
            sens.yagyr = recvData[9][0]
            if(isinstance(sens.gps.lat, float)):
                gpsUp = 1
                #print("gpsUp = 1")
            
    #/////////////////wait for gps or override///////////////////////
    try:
        print("while start")
        #init für kopf
        SCHRITTWEITE = 0.010
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
                    #print("Test timetest<0 oder > Schrerittweite +0.001")
                    #print(timetest)
                    newRun =  time.monotonic()+SCHRITTWEITE
                #print("dauer für den letzten zeitschritt")
                newRun = newRun+SCHRITTWEITE#Hz Timing
                #ende kopf
                
                #print("run")
                if (qparent_fs.poll() == True):
                   channel =  qparent_fs.recv()
                if (qparent_reg.poll() == True):
                    chSend =  qparent_reg.recv()
                if (qparent_led.poll() == True):
                    taster =  qparent_led.recv()
                if (qparent_sens.poll() == True):
                    #print(qparent_sens.recv())
                    recvData =  qparent_sens.recv()
                    for i in range(0, 3):
                        #print("i", i)
                        #print("type(sens.gyr_raw[i])", type(sens.gyr_raw[i]))
                        #print("type(recvData[0][i])", type(recvData[0][i]))
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
                    sens.yagyr = recvData[9][0]
                    #print("istgyr:", sens.istgyr)
                    #print("Vehicle:", sens.vVehicle)
                    #qparent_sens.send(1)
                #print(channel[10])
                if (channel[10]== -1):    #//////////////////////////////SWC oben
                    if (qchild_led.poll() == False):
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
                    if (qchild_led.poll() == False):
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
                    if (qchild_led.poll() == False):
                        qparent_led.send(KONST.vollAutonom)
                    if (switchOld != 1):
                        logstr = logstr + "Flugsteuerung: autonom"+'\n'
                        switchOld = 1
                        trajek.SetOldWaypoint(sens.pos)
                        n_Wp[0] = sens.pos[0] + math.cos(sens.istgyr[2] / 180 * math.pi) *  0.004 #440m
                        n_Wp[1] = sens.pos[1] + math.sin(sens.istgyr[2] / 180 * math.pi) *  0.004 #440m
                        n_Wp[2] = sens.pos[2]
                        n_Wp[4] = 1
                        trajek.SetNewWaypoint(n_Wp)
                    dataForRegleung[2] = trajek.StupidControl(sens.pos, sens.istgyr, dposStupidControll)
                    
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
                        #print(logstr)
                        log = open(FILENAME,"a")
                        log.write(logstr)
                        log.close()
                        oldLogstr = logstr
                        logstr = ""
                        geschrieben = True
                        try:
                            eth = open("/sys/class/net/eth0/operstate","r")
                            streth = eth.read(2)
                            #print (streth)
                            eth.close()
                        except FileNotFoundError:
                            print("not Found: /sys/class/net/eth0/operstate")
                        if(streth == "up"):
                            KONST.ethconn = True
                        else:
                            try:
                                eth = open("/sys/class/net/wlan0/operstate","r")
                                streth = eth.read(2)
                                eth.close()
                            except FileNotFoundError:
                                print("not Found: /sys/class/net/wlan0/operstate")
                            #print (streth)
                            if(streth == "up"):
                                KONST.ethconn = True
                            else:
                                KONST.ethconn = False
                else:
                    geschrieben = False
                tmonH5 = round(time.monotonic(), 2) #100Hz
                if(tmonH5-tmonCFoldH5 > 0.19):#5Hz
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
                            if(countH5cuw>1):
                                dsetH5cuw[countH5cuw, 27+i] = sens.pos[i]-dsetH5cuw[1, 27+i]
                            else:
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
                    #"gui"
                    akkuSpannung = (taster[1] - 1575) / 165 * 100
                    akkuSpannung = round(akkuSpannung, 2)
                    if(p_gui.is_alive()): # update gui wenn prozess noch lebendig
                        guiDate = [akkuSpannung, [channel[0], channel[10], channel[1]], [sens.gps.lat, sens.gps.lon], [sens.pos[0], sens.pos[1], sens.pos[2]], [round(sens.vWorld[0], 2), round(sens.vWorld[1], 2), round(sens.vWorld[2], 2)], [round(sens.istgyr[0], 2), round(sens.istgyr[1], 2), round(sens.istgyr[2], 2)] , [round(sens.gyr_raw[0], 2), round(sens.gyr_raw[1], 2), round(sens.gyr_raw[2], 2)] , -1, KONST.ethconn, [0, 0, 0]]
                        if(qchild_gui.poll() == False):
                            qparent_gui.send(guiDate)
                        
                    if(akkuSpannung <= 0.0):
                        if(akkuSpannung > -300):
                            print("!!!!!!!!!!! akkuSpannung zu tief !!!!!!!!!!!!!: ",  akkuSpannung)
                            print("!!!!!!!!!!! akkuSpannung zu tief !!!!!!!!!!!!!: ",  akkuSpannung)
                            print("!!!!!!!!!!! akkuSpannung zu tief !!!!!!!!!!!!!: ",  akkuSpannung)
                            print("!!!!!!!!!!! akkuSpannung zu tief !!!!!!!!!!!!!: ",  akkuSpannung)
                            print("!!!!!!!!!!! akkuSpannung zu tief !!!!!!!!!!!!!: ",  akkuSpannung)
                #//////////////////////////////log/////////////////////////////
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
    
