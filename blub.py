import time
#init für kopf
SCHRITTWEITE = 0.0050
tol = SCHRITTWEITE*1.1
tol2 = tol - SCHRITTWEITE
newRun = time.monotonic()+SCHRITTWEITE
#init end kopf
while (True):
    #kopf für while Timing
    timetest = newRun - time.monotonic()
    print("timetest", timetest)
    if(timetest>0 and timetest < tol):
        time.sleep(timetest)
    else:
        #print("Sensorfusion timetest<0 oder > Schrerittweite +0.001")
        #print(timetest)
        newRun =  time.monotonic()+SCHRITTWEITE
    #print("dauer für den letzten zeitschritt")
    newRun = newRun+SCHRITTWEITE#Hz Timing
    #ende kopf
    print("nR", newRun)
    print(time.monotonic())
    time.sleep(0.0020)
