import serial
import KONST
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
        while (True):
            try:
                lenght = self.ser.in_waiting
#                if(wpi.digitalRead(16) == True):
#                    print("kommt shit an")
#                else:
#                    print("kommt kein shit an!!!!!!!!")
                
                #print("lenght:", lenght)
                if (lenght > 0):
                    #print("lenght:", lenght)
                    line = self.ser.read(lenght)
                    if (len(line) == 32):
                        for i in range(0,16):          #Debug in hex
                            #print( int(line[i*2]))#.encode('hex')," ",line[i*2+1].encode('hex')
                            i = i+1
                        intarr = []            
                        for i in range(0,lenght):
                            intarr.append(int(line[i]))#.encode('hex'),16))
                            i = i+1

                        data = []
                        summe = 0
                        for i in range(0,16):
                            intarr[i*2+1] = intarr[i*2+1]<<8
                            data.append(intarr[i*2] + intarr[i*2+1])
                            summe = summe + data[i]
                            #print ("Channel",i,": ",data[i])       #debug in dezimal
                            i = i+1
                        
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
                        #print(self.channel[1])
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
    
