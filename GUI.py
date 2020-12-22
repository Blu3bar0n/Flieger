from tkinter import *
#import time
#import subprocess

def UpdateLabelLeft(Label, text):
    if not(isinstance(text, str)):
        text = str(text)
    string = Label.cget("text")
    split = string.split(":")
    string = text+":"+split[-1]
    Label.config(text = string)
    
def UpdateLabelRight(Label, text):
    if not(isinstance(text, str)):
        text = str(text)
    string = Label.cget("text")
    split = string.split(":")
    string = split[0]+":"+text
    Label.config(text = string)
    
def Update(mpPipesParent,mpPipesChild, tk, labelAkku,  labelFunke,  labelFunkestate, labelFunkeCh, labelGps, labelGpsLat, labelGpsLon, labelX, labelY, labelZ, labelXv, labelYv, labelZv, labelRoll, labelPitch, labelYaw, labelRollv, labelPitchv, labelYawv, labelLaser, labelEth, Debug1, Debug2, Debug3):
    if (mpPipesChild.poll() == True):
        Data =  mpPipesChild.recv()
        #akku
        UpdateLabelRight(labelAkku, Data[0])
        if(Data[0] > 0):
            labelAkku.config(foreground="green")
        else:
            labelAkku.config(foreground="red")
        #Funke
        UpdateLabelRight(labelFunke, Data[1][0])
        UpdateLabelRight(labelFunkestate, Data[1][1])
        UpdateLabelRight(labelFunkeCh, Data[1][2])
        if(Data[1][0] == 29.832):
            UpdateLabelLeft(labelFunke, "Up")
            labelFunke.config(foreground="green")
        else:
            UpdateLabelLeft(labelFunke, "Down")
            labelFunke.config(foreground="red")
        #GPS
        UpdateLabelRight(labelGpsLat, Data[2][0])
        UpdateLabelRight(labelGpsLon, Data[2][1])
        #UpdateLabelRight(labelFunkeCh, Data[2][2])
        if(isinstance(Data[2][0], float)):
            UpdateLabelLeft(labelGps, "Up")
            labelGps.config(foreground="green")
        else:
            UpdateLabelLeft(labelGps, "Down")
            labelGps.config(foreground="red")
        #Position
        UpdateLabelRight(labelX, Data[3][0])
        UpdateLabelRight(labelY, Data[3][1])
        UpdateLabelRight(labelZ, Data[3][2])
        #speed
        UpdateLabelRight(labelXv, Data[4][0])
        UpdateLabelRight(labelYv, Data[4][1])
        UpdateLabelRight(labelZv, Data[4][2])
        #winkel
        UpdateLabelRight(labelRoll, Data[5][0])
        UpdateLabelRight(labelPitch, Data[5][1])
        UpdateLabelRight(labelYaw, Data[5][2])
        #winkelv
        UpdateLabelRight(labelRollv, Data[6][0])
        UpdateLabelRight(labelPitchv, Data[6][1])
        UpdateLabelRight(labelYawv, Data[6][2])
        #Laser  
        UpdateLabelRight(labelLaser, Data[7])
        #Eth
        UpdateLabelRight(labelEth, Data[8])
        if(Data[8]):
            labelEth.config(foreground="green")
        else:
            labelEth.config(foreground="red")
        #Debug
        UpdateLabelRight(Debug1, Data[9][0])
        UpdateLabelRight(Debug2, Data[9][1])
        UpdateLabelRight(Debug3, Data[9][2])
    tk.after(200,lambda:Update(mpPipesParent,mpPipesChild, tk, labelAkku,  labelFunke,  labelFunkestate, labelFunkeCh, labelGps, labelGpsLat, labelGpsLon, labelX, labelY, labelZ, labelXv, labelYv, labelZv, labelRoll, labelPitch, labelYaw, labelRollv, labelPitchv, labelYawv, labelLaser, labelEth, Debug1, Debug2, Debug3))

def Process(mpPipesParent,mpPipesChild):
   print("In Process Autobuild")
   #format gui
   labelNameLength = 8
   labelLength = 14
   tk = Tk()
   Label(tk, text="Akku:", width=labelNameLength,  anchor=W).grid(row=0, column=0)
   Label(tk, text="Funke:", width=labelNameLength,  anchor=W).grid(row=1, column=0)
   Label(tk, text="GPS:", width=labelNameLength,  anchor=W).grid(row=2, column=0)
   Label(tk, text="Position", width=labelNameLength,  anchor=W).grid(row=3, column=0)
   Label(tk, text="v:", width=labelNameLength,  anchor=W).grid(row=4, column=0)
   Label(tk, text="Winkel:", width=labelNameLength,  anchor=W).grid(row=5, column=0)
   Label(tk, text="vWinkel:", width=labelNameLength,  anchor=W).grid(row=6, column=0)
   Label(tk, text="Laser:", width=labelNameLength,  anchor=W).grid(row=7, column=0)
   Label(tk, text="Ethconn:", width=labelNameLength,  anchor=W).grid(row=8, column=0)
   Label(tk, text="Debug:", width=labelNameLength,  anchor=W).grid(row=9, column=0)
   
   labelAkku = Label(tk, text="Netzteil:", width=labelLength,  anchor=W)
   labelAkku.grid(row=0, column=1)
   
   labelFunke = Label(tk, text="down:", width=labelLength,  anchor=W)
   labelFunke.grid(row=1, column=1)
   labelFunke.config(foreground="red")
   labelFunkestate = Label(tk, text="state:", width=labelLength,  anchor=W)
   labelFunkestate.grid(row=1, column=2)
   labelFunkeCh = Label(tk, text="Bsp. Ch:", width=labelLength,  anchor=W)
   labelFunkeCh.grid(row=1, column=3)
   
   labelGps = Label(tk, text="down:", width=labelLength,  anchor=W)
   labelGps.grid(row=2, column=1)
   labelGps.config(foreground="red")
   labelGpsLat = Label(tk, text="lat:", width=labelLength,  anchor=W)
   labelGpsLat.grid(row=2, column=2)
   labelGpsLon = Label(tk, text="lon:", width=labelLength,  anchor=W)
   labelGpsLon.grid(row=2, column=3)
   
   labelX = Label(tk, text="x:", width=labelLength,  anchor=W)
   labelX.grid(row=3, column=1)
   labelY = Label(tk, text="y:", width=labelLength,  anchor=W)
   labelY.grid(row=3, column=2)
   labelZ = Label(tk, text="z:", width=labelLength,  anchor=W)
   labelZ.grid(row=3, column=3)
   
   labelXv = Label(tk, text="xv:", width=labelLength,  anchor=W)
   labelXv.grid(row=4, column=1)
   labelYv = Label(tk, text="yv:", width=labelLength,  anchor=W)
   labelYv.grid(row=4, column=2)
   labelZv = Label(tk, text="zv:", width=labelLength,  anchor=W)
   labelZv.grid(row=4, column=3)
   
   labelRoll = Label(tk, text="Roll:", width=labelLength,  anchor=W)
   labelRoll.grid(row=5, column=1)
   labelPitch = Label(tk, text="Pitch:", width=labelLength,  anchor=W)
   labelPitch.grid(row=5, column=2)
   labelYaw = Label(tk, text="Yaw:", width=labelLength,  anchor=W)
   labelYaw.grid(row=5, column=3)
   
   labelRollv = Label(tk, text="Rollv:", width=labelLength,  anchor=W)
   labelRollv.grid(row=6, column=1)
   labelPitchv = Label(tk, text="Pitchv:", width=labelLength,  anchor=W)
   labelPitchv.grid(row=6, column=2)
   labelYawv = Label(tk, text="Yawv:", width=labelLength,  anchor=W)
   labelYawv.grid(row=6, column=3)
   
   labelLaser = Label(tk, text="m:", width=labelLength,  anchor=W)
   labelLaser.grid(row=7, column=1)
   
   labelEth = Label(tk, text="connected:", width=labelLength,  anchor=W)
   labelEth.grid(row=8, column=1)
   labelEth.config(foreground="green")
   
   Debug1 = Label(tk, text=":", width=labelLength,  anchor=W)
   Debug1.grid(row=9, column=1)
   Debug2 = Label(tk, text=":", width=labelLength,  anchor=W)
   Debug2.grid(row=9, column=2)
   Debug3 = Label(tk, text=":", width=labelLength,  anchor=W)
   Debug3.grid(row=9, column=3)
   
   Button(tk,  text='QuitGui', command=tk.destroy).grid(row=20,column=0)
   tk.after(1000,lambda:Update(mpPipesParent,mpPipesChild, tk, labelAkku,  labelFunke,  labelFunkestate, labelFunkeCh, labelGps, labelGpsLat, labelGpsLon, labelX, labelY, labelZ, labelXv, labelYv, labelZv, labelRoll, labelPitch, labelYaw, labelRollv, labelPitchv, labelYawv, labelLaser, labelEth, Debug1, Debug2, Debug3))
   mainloop( )

if __name__ == '__main__':
   Process("error","error")
