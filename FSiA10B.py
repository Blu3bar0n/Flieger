import serial
import KONST
import time
#import wiringpi as wpi
#import sys
#import multitasking
#import signal

class FSiA10B():


    # kill all tasks on ctrl-c
    #signal.signal(signal.SIGINT, multitasking.killall)
    channel = [0.0,0.0,0.0,-1.0,0.0,0.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]
    ser = serial.Serial('/dev/ttySAC0', 115200, serial.EIGHTBITS, serial.PARITY_NONE,serial.STOPBITS_ONE)
##    wpi.wiringPiSetup()
##    wpi.pinMode(16,0)
    
    #@multitasking.task
    def read (self, qchild_fs, qparent_fs):
        #ser = serial.Serial('/dev/ttySAC0', 115200, serial.EIGHTBITS, serial.PARITY_NONE,serial.STOPBITS_ONE)
        #timeold = time.monotonic()
        #init für kopf
        SCHRITTWEITE = 0.0070
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
                    print("test<0 oder > Schrerittweite +0.001")
                    print(test)
                    newRun =  time.monotonic()+SCHRITTWEITE
                #print("dauer für den letzten zeitschritt")
                newRun = newRun+SCHRITTWEITE#Hz Timing
                #ende kopf
                #print("zeitschritt: ", time.monotonic()-timeold)#zeitschritt
                #timeold = time.monotonic()
                lenght = self.ser.in_waiting
#                if(wpi.digitalRead(16) == True):
#                    print("kommt shit an")
#                else:
#                    print("kommt kein shit an!!!!!!!!")
                
                #print("lenght:", lenght)
                if (lenght > 0 and lenght < 70):
                    print("lenght:", lenght)
                    line = self.ser.read(lenght)
                    if (len(line) == 32):
                        #for i in range(0,16):          #Debug in hex
                            #print( int(line[i*2]))#.encode('hex')," ",line[i*2+1].encode('hex')
                        intarr = []            
                        for i in range(0,lenght):
                            intarr.append(int(line[i]))#.encode('hex'),16))

                        data = []
                        summe = 0
                        for i in range(0,16):
                            intarr[i*2+1] = intarr[i*2+1]<<8
                            data.append(intarr[i*2] + intarr[i*2+1])
                            summe = summe + data[i]
                            #print ("Channel",i,": ",data[i])       #debug in dezimal
                        
                        if (data[0] != 16416):
                            print ("Read() failed startbyte")
                            return 0
                        summe = summe - data[0] - data[15]
                        #if (data[15] != summe):
                            #print "Read() failed summe:",summe,"Checksum:",data[15]
                            #return 0

                        for i in range(0,16):
                            self.channel[i] = (data[i]-1500.0)/500.0
                        if (qparent_fs.poll() == False):
                            qchild_fs.send(self.channel)
                        print("FSiA10B channel[10]", self.channel[10])
                    else:
                        print("buffer flushed",  lenght)

                        self.ser.reset_input_buffer()
                elif(lenght > 0):
                    if(0):
                        line = self.ser.read(lenght)
                        intarr = []            
                        for i in range(0,lenght):
                            intarr.append(int(line[i]))#.encode('hex'),16))

                        data = []

                        summe = 0
                        for i in range(0,int(lenght/2)):
                            intarr[i*2+1] = intarr[i*2+1]<<8
                            data.append(intarr[i*2] + intarr[i*2+1])
                            summe = summe + data[i]
                            print ("Channel",i,": ",data[i])       #debug in dezimal
                        print("buffer flushed",  lenght)
                    else:
                        print("buffer flushed",  lenght)
                        self.ser.reset_input_buffer()
            except KeyboardInterrupt: 
                self.StopRead()
                exit()
            except Exception as e:
                log = open(KONST.Filename,"a")
                errmsg = "Unexpectet Error in FSiA10B:" + '\n' + str(e)
                log.write(errmsg)
                log.close()
                if(KONST.ethconn):
                    raise
                else:
                    pass

    def GetChannel(self,numCh):
        return self.channel[numCh]

    def StopRead (self):
        self.ser.close()

    def StartRead(self):
        self.ser = serial.Serial('/dev/ttySAC0', 115200, serial.EIGHTBITS, serial.PARITY_NONE,serial.STOPBITS_ONE)

def Process(qchild_fs, qparent_fs):
    print ("In Process FSiA10B")
    fs = FSiA10B()
    fs.read(qchild_fs, qparent_fs)
    
