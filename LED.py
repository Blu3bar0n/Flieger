import wiringpi as wpi
import time
import KONST

def Process(qchild_led,qparent_led):
    wpi.wiringPiSetup()
    print ("wpi setup done")
    wpi.pinMode(27,1) #gelb
    wpi.pinMode(5,1) #gruen
    wpi.pinMode(6,1) #rot
    wpi.pinMode(23, 0) #links
    wpi.pinMode(26,0) #rechts
    wpi.pinMode(25,0) #adc      11,89V = 3600
    showstate = [0, 0, 0, 0]
    taster = [0, 0]
    timeold = time.monotonic()
    timeoldBT = time.monotonic()
    timeLedch = time.monotonic()
    countLed = 0
    lastLow = True
    Voltage = [12.0, 12.0, 12.0, 12.0, 12.0,12.0, 12.0, 12.0, 12.0, 12.0]
    sumV = 12.0
    try:
        while True:
            time.sleep(0.1) #Timing für 10 Hz nicht präziese und langweilig
            #print (sumV)
            if (qchild_led.poll() == True):
                showstate =  qchild_led.recv()
                #print("showstate", showstate)
            if (qparent_led.poll() == False):
                qchild_led.send(taster)
            if (timeold > time.monotonic()):
               timeold = time.monotonic() 
            if (timeoldBT > time.monotonic()):
               timeoldBT = time.monotonic() 
            if(wpi.digitalRead(23) == True):
                if ((timeold + 3.0) > time.monotonic()):
                    taster[0]= 1
                else:
                    taster[0]= 4
            if(wpi.digitalRead(26) == True):
                if ((timeold + 3.0) > time.monotonic()):
                    taster[0]= 2
                else:
                    taster[0]= 5
            if(wpi.digitalRead(23) == True and wpi.digitalRead(26) == True):
                timeold = time.monotonic()
                if ((timeoldBT + 3.0) > time.monotonic()):
                    taster[0]= 3
                else:
                    taster[0]= 6
            else:
                timeoldBT = time.monotonic()
                if(wpi.digitalRead(23) == False and wpi.digitalRead(26) == False):
                    taster[0]= 0
                    timeold = time.monotonic()
            #print(taster)
            timeLedch = time.monotonic()
            if(lastLow == True):
                if((timeLedch - int(timeLedch))>0.5):
                    lastLow = False
                    countLed = countLed + 1
                    for i in range(0, 9):
                        Voltage[i] = Voltage[i+1]
                    Voltage[9] = wpi.analogRead(25)*0.0033
                    sumV = 0.0
                    for i in range(0,10):
                        sumV = sumV + Voltage[i]
                    sumV = sumV / 10
                    taster[1] = sumV
            else:
                if((timeLedch - int(timeLedch))<0.5):
                    lastLow = True
                    countLed = countLed + 1
                    for i in range(0, 9):
                        Voltage[i] = Voltage[i+1]
                    Voltage[9] = wpi.analogRead(25)#*0.0033
                    sumV = 0.0
                    for i in range(0,10):
                        sumV = sumV + Voltage[i]
                    sumV = sumV / 10
                    taster[1] = sumV
            if (showstate == KONST.standby):
                if (countLed > 3):
                    countLed = 0
                if(countLed == 0):
                    wpi.digitalWrite(5, 0)
                    wpi.digitalWrite(27, 0)
                    wpi.digitalWrite(6, 0)
                if(countLed == 1):
                    wpi.digitalWrite(5, 1)
                    wpi.digitalWrite(27,0)
                    wpi.digitalWrite(6, 0)
                if(countLed == 2):
                    wpi.digitalWrite(5, 0)
                    wpi.digitalWrite(27,1)
                    wpi.digitalWrite(6, 0) 
                if(countLed == 3):
                    wpi.digitalWrite(5, 0)
                    wpi.digitalWrite(27, 0)
                    wpi.digitalWrite(6, 1)
            if(showstate == KONST.DEAD):
                wpi.digitalWrite(5, 1)
                wpi.digitalWrite(27, 1)
                wpi.digitalWrite(6, 1)
            
            if(showstate[3]==0):
                if(countLed > 6):
                    countLed = 1
                if (countLed%2 == 0):
                    if (countLed/2 <= showstate[0]):
                        wpi.digitalWrite(5, 1)
                    if (countLed/2 <= showstate[1]):
                        wpi.digitalWrite(27, 1)
                    if (countLed/2 <= showstate[2]):
                        wpi.digitalWrite(6, 1)
                else:
                    wpi.digitalWrite(5, 0)
                    wpi.digitalWrite(27,0)
                    wpi.digitalWrite(6, 0)
            
    
    except KeyboardInterrupt: 
        wpi.digitalWrite(5, 0)
        wpi.digitalWrite(27, 0)
        wpi.digitalWrite(6, 0)
        exit()
    except:
        print ("Unexpected Error led")
        raise
    
