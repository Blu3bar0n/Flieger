import h5py
from os import listdir
import os
from pathlib import Path

#fileH5 = h5py.File("/home/odroid/Desktop/ericWorkspace/Python/LOG/ReadLog/Input/Log: 2016:02:11 2h5.hdf5", 'w')
#h5group_ChannelUndWinkel = fileH5.create_group(u"ChannelUndWinkel")
#dsetH5cuw = h5group_ChannelUndWinkel.create_dataset("ch1-5, istgyr1-3", (64000,36), chunks=True)#, dtype='float64') #mit 100hz reicht das 1h
#dsetH5cuw[0, 35] = 1.0
#dsetH5cuw[1, 35] = 1.0
#dsetH5cuw[2, 35] = 1.0
#dsetH5cuw[3, 35] = 1.0
#dsetH5cuw[4, 35] = 1.0
#fileH5.close()

dir = "/home/odroid/Desktop/ericWorkspace/Python/LOG/ReadLog/Input/"
onlyfiles = listdir(dir)                        #("/home/odroid/Desktop/ericWorkspace/Python/LOG/ReadLog/Input/*.hdf5")
print(onlyfiles)
if(len(onlyfiles) != 1):
    print("Keine oder zu viele Dateien im Input")
else:
    filename = str(onlyfiles[0])
    strh5 = ".hdf5"
    if(strh5 in filename):
        fileH5 = h5py.File(dir+filename, 'r')
        print(fileH5.keys())
        #dsetH5cuw = fileH5['ChannelUndWinkel/ch1-5, istgyr1-3']
        dsetH5cuw = fileH5['/ChannelUndWinkel/ch1-5, istgyr1-3']
        numrows = len(dsetH5cuw)    # 3 rows in your example
        numcols = len(dsetH5cuw[0])
        print("Reihen", numrows)
        print("Spalten", numcols)
        check_file = Path("/home/odroid/Desktop/ericWorkspace/Python/LOG/ReadLog/Output/Output.txt")
        if (check_file.is_file()):
            os.remove("/home/odroid/Desktop/ericWorkspace/Python/LOG/ReadLog/Output/Output.txt") 
        outputtxt = open("/home/odroid/Desktop/ericWorkspace/Python/LOG/ReadLog/Output/Output.txt", "a")
        for i in range(0, numrows):
            line = ""
            if (dsetH5cuw[i, 35] == 0):
                break
            for j in range(0, numcols):
                line = line + str(dsetH5cuw[i, j]) + " "
            line = line + '\n'
            outputtxt.write(line)
        fileH5.close()
        outputtxt.close()
        os.remove(dir+filename)
    else:
        print("Keine .hdf5 Dateien im Input")
print("done")




#FILENAME = "/home/odroid/Desktop/ericWorkspace/Python/LOG/Log: "
#DATUM = time.strftime("%Y:%m:%d")
#FILENAME = FILENAME  + DATUM +" " 
#check_file = Path(FILENAME +str(countLogfile) + ".txt")
#while (check_file.is_file()):
#    countLogfile = countLogfile + 1
#    check_file = Path(FILENAME +str(countLogfile) + ".txt")
#FILENAME = FILENAME + str(countLogfile) + ".txt"
#KONST.Filename = FILENAME
#FILENAMEH5 = FILENAME + str(countLogfile) + "h5" + ".hdf5"
#fileH5 = h5py.File(FILENAMEH5, 'w')
#h5group_ChannelUndWinkel = fileH5.create_group(u"ChannelUndWinkel")
#maxH5countH5cuw = 64000 #mit 50hz reicht das 21min
#dsetH5cuw = h5group_ChannelUndWinkel.create_dataset("ch1-5, istgyr1-3", (maxH5countH5cuw,35))#, dtype='float64') #mit 100hz reicht das 1h
#countH5cuw = 0
#countH5cuw50Hz = 0
#tmonCFoldH5 = round(time.monotonic(), 2)



#                tmonH5 = round(time.monotonic(), 2) #100Hz
#                if(tmonH5-tmonCFoldH5 > 0):
#                    tmonCFoldH5 = tmonH5
#                    if(countH5cuw50Hz == 0):#fuer 50Hz
#                        countH5cuw50Hz = 0
#                        if (countH5cuw<maxH5countH5cuw):
#                            dsetH5cuw[countH5cuw, 0] = chSend[1]
#                            dsetH5cuw[countH5cuw, 1] = chSend[2]
#                            dsetH5cuw[countH5cuw, 2] = chSend[3]
#                            dsetH5cuw[countH5cuw, 3] = chSend[4]
#                            dsetH5cuw[countH5cuw, 4] = chSend[5]
#                            dsetH5cuw[countH5cuw, 5] = taster[1]
#                            for i in range(0, 3):
#                                dsetH5cuw[countH5cuw, 6+i] = bmi160.istgyr[i]
#                                dsetH5cuw[countH5cuw, 9+i] = bmi160.gyr[i]
#                                dsetH5cuw[countH5cuw, 12+i] = bmi160.acc[i]
#                                dsetH5cuw[countH5cuw, 15+i] = bmi160.accOhneG[i]
#                                dsetH5cuw[countH5cuw, 18+i] = bmi160.accogWorld[i]
#                                dsetH5cuw[countH5cuw, 21+i] = bmi160.vVehicle[i]
#                                dsetH5cuw[countH5cuw, 24+i] = bmi160.vWorld[i]
#                                dsetH5cuw[countH5cuw, 27+i] = bmi160.pos[i]
#                            dsetH5cuw[countH5cuw, 30] = gps.lat
#                            dsetH5cuw[countH5cuw, 31] = gps.lon
#                            dsetH5cuw[countH5cuw, 32] = gps.cours
#                            dsetH5cuw[countH5cuw, 33] = gps.latErr
#                            dsetH5cuw[countH5cuw, 34] = gps.lonErr
#                            #print("x", bmi160.accogWorld[0]*100)
#                            #print("y", bmi160.accogWorld[1]*100)
#                            countH5cuw = countH5cuw + 1
#                    else:
#                        countH5cuw50Hz = 1
#                #//////////////////////////////log////////////////////
