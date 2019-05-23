#import KONST
from geopy import distance
import math

class TRAJEKTORIE():
    oldWaypoint = [0.0, 0.0, 0.0]
    nextWaypoint = [0.0, 0.0, 0.0, 0.0, 0.0]
    pitchDependingOnRollmin = 15
    pitchDependingOnRollmax = 50
    pitchTooLargeTooRoll = 20
    maxRollTooControllYaw = 60
    def NextWaypointOnLineBetweenOldAndNewWaypoint(self, istPos):
        #print("NextWaypoint")
        dWp = [0.0, 0.0, 0.0] #vektor welcher von oldWaypoint auf nextWaypoint zeigt in [lat, lon, m über ground] 
        dWp[0] = self.nextWaypoint[0] - self.oldWaypoint[0]#(distance.distance([self.nextWaypoint[0], self.oldWaypoint[1]],[self.oldWaypoint[0], self.oldWaypoint[1]]).km) * 1000
        dWp[1] = self.nextWaypoint[1] - self.oldWaypoint[1]#(distance.distance([self.oldWaypoint[0], self.nextWaypoint[1]],[self.oldWaypoint[0], self.oldWaypoint[1]]).km) * 1000
#        if((self.nextWaypoint[0] - self.oldWaypoint[0],)<0):
#            dWp[0] = -dWp[0]
#        if((self.nextWaypoint[1] - self.oldWaypoint[1])<0):
#            dWp[1] = -dWp[1]
        dposWp = [0.0, 0.0, 0.0] #vektor welcher von istPos auf oldWaypoint zeigt in [lat, lon, m über ground] 
        dposWp[0] = istPos[0] - self.oldWaypoint[0]#(distance.distance([istPos[0], self.oldWaypoint[1]],[self.oldWaypoint[0], self.oldWaypoint[1]]).km) * 1000
        dposWp[1] = istPos[1] - self.oldWaypoint[1]#(distance.distance([self.oldWaypoint[0], istPos[1]],[self.oldWaypoint[0], self.oldWaypoint[1]]).km) * 1000
#        if((istPos[0] - self.oldWaypoint[0],)<0):
#            dposWp[0] = -dposWp[0]
#        if((istPos[1] - self.oldWaypoint[1])<0):
#            dposWp[1] = -dposWp[1]
        n_Wp = [0.0, 0.0, 0.0] #[lat, lon, m über ground
        faktor = ((dposWp[0]) * dWp[0]+ (dposWp[1]) * dWp[1])/(dWp[0] * dWp[0] + dWp[1] * dWp[1]) # einheiten lat lon
        n_Wp[0] = self.oldWaypoint[0] + faktor * dWp[0] #lot zwischen den achsen wieder in [lat, lon, m über ground] 
        n_Wp[1] = self.oldWaypoint[1] + faktor * dWp[1]
        
        dWp_norm = [0.0, 0.0, 0.0] #[lat, lon, m über ground] 
        faktor2 = math.sqrt(dWp[0] * dWp[0] + dWp[1] * dWp[1])
        dWp_norm[0] = dWp[0] / faktor2
        dWp_norm[1] = dWp[1] / faktor2
        n_Wp[0] = n_Wp[0] + 0.0004 * dWp_norm[0]  #4        0.0001           11.1 m
        n_Wp[1] = n_Wp[1] + 0.0004 * dWp_norm[1]
        
        dposN_Wp = [0.0, 0.0, 0.0] #vektor welcher von oldWaypoint auf n_wp  zeigt in [lat, lon, m über ground] 
        dposN_Wp[0] = n_Wp[0] - self.oldWaypoint[0]
        dposN_Wp[1] = n_Wp[1] - self.oldWaypoint[1]
        faktor3 = math.sqrt(dposN_Wp[0] * dposN_Wp[0] + dposN_Wp[1] * dposN_Wp[1])
        faktor3 = faktor3 / faktor2
        if((faktor3) < 1):
            n_Wp[2] = self.nextWaypoint[2] * faktor3 + self.oldWaypoint[2] * (1 - faktor3)
        else:
            n_Wp[0] = self.nextWaypoint[0]
            n_Wp[1] = self.nextWaypoint[1]
            n_Wp[2] = self.nextWaypoint[2]
        
        return n_Wp

    def ShiftInRange(self,wert,unten,oben):
        if(wert < unten):
            wert = wert + abs(oben-unten)
        if(wert > oben):
            wert = wert - abs(oben-unten)
        return wert    
        
    def StupidControl(self, istPos_n, istGyr_n, dposStupidControll):
        istPos = [0.0, 0.0, 0.0]
        istGyr = [0.0, 0.0, 0.0]
        for i in range (0, 3):
            istPos[i] = istPos_n[i]
            istGyr[i] = istGyr_n[i]
        #print("StupidControll")
        if(self.nextWaypoint[4]==0): #kein wegpunkt gesetzt => nur stabilisierung
            istGyr[0] = 0.0
            istGyr[1] = 0.0 
            return istGyr
        if(self.nextWaypoint[4]==1):
            #///////////////////sollwerte////////////////
            sollPos = self.NextWaypointOnLineBetweenOldAndNewWaypoint(istPos)
            #print("sollPos", sollPos)
            dpos = [0.0, 0.0, 0.0]
            dpos[0] = (distance.distance([sollPos[0], istPos[1]],[istPos[0], istPos[1]]).km) * 1000
            dpos[1] = (distance.distance([istPos[0], sollPos[1]],[istPos[0], istPos[1]]).km) * 1000
            dpos[2] = sollPos[2] - istPos[2]
            if((sollPos[0] - istPos[0])<0):
                dpos[0] = -dpos[0]
            if((sollPos[1] - istPos[1])<0):
                dpos[1] = -dpos[1]
            #print("dpos", dpos)
            for i in range(0, 3):
                dposStupidControll[i] = round( dpos[i], 2)
            Yaw = math.atan2(dpos[1], dpos[0]) * 180 / math.pi
            Yaw = self.ShiftInRange(Yaw, -180, 180)
            distOverGround = math.sqrt(dpos[0] * dpos[0] + dpos[1] * dpos[1])
            Pitch = math.atan2(dpos[2], distOverGround) * 180 / math.pi
            Pitch = self.ShiftInRange(Pitch, -180, 180)
            #print("Pitch,Yaw: ", Pitch,  Yaw)
            #///////////////////differenz////////////////
            dPitch = Pitch - istGyr[1]
            dYaw = Yaw - istGyr[2]
            dYaw = self.ShiftInRange(dYaw, -180, 180)
            dposStupidControll[3] = round( dPitch, 2)
            dposStupidControll[4] = round( dYaw, 2)
            #print("dP,dY: ", dPitch,  dYaw)
            if(abs(Pitch) > 30):
                if(Pitch > 0):
                    Pitch = 30
                else:
                    Pitch = -30
            if(abs(istGyr[1]) > 30): #notfall Pitch unter berücksichtichtung von roll
                retdata = [0.0, 0.0, 0.0] # SollWinkel für Regelung
                faktor = 1.0/(self.pitchDependingOnRollmax - self.pitchDependingOnRollmin)
                retdata[1] = - faktor * abs(istGyr[0]) + self.pitchDependingOnRollmax * faktor
                if(retdata[1] > 1):
                    retdata[1] = 1
                if(retdata[1] < 0):
                    retdata[1] = 0
                retdata[1] = istGyr[1] - retdata[1] * istGyr[1]
                #evtl Pitch hinzufgen
                retdata[2] = istGyr[2]
                return retdata
            if(abs(dYaw) < 15):#close Yaw
                retdata = [0.0, 0.0, 0.0]
                if(abs(istGyr[1]) < self.pitchDependingOnRollmin): # ideal Roll
                    retdata[1] = Pitch
                    retdata[2] = Yaw
                    return retdata
            if(abs(dPitch) > self.pitchTooLargeTooRoll): #krasser Pitch unterschied
                retdata = [0.0, 0.0, 0.0]
                faktor = 1.0/(self.pitchDependingOnRollmax - self.pitchDependingOnRollmin)
                retdata[1] = - faktor * abs(istGyr[0]) + self.pitchDependingOnRollmax * faktor
                if(retdata[1] > 1):
                    retdata[1] = 1
                if(retdata[1] < 0):
                    retdata[1] = 0
                dPitch = MaxAbsWert(dPitch, -20, 20)
                retdata[1] = istGyr[1] - retdata[1] * dPitch
                retdata[2] = istGyr[2]
                return retdata
            #eindrehen weil dyaw groß und keine der oberen greift
            retdata = [0.0, 0.0, 0.0]
            retdata[0] = dYaw
            retdata[0] = MaxAbsWert(retdata[0], - self.maxRollTooControllYaw, self.maxRollTooControllYaw)
            retdata[1] = dYaw/45
            retdata[1] = MaxAbsWert(retdata[1], -1, 1) #regelt einschlag von höhenruder... max bei 45grad abweichung von yaw
            faktor = istGyr[0]/(self.maxRollTooControllYaw - 10)
            faktor = MaxAbsWert(faktor, -1, 1)#regelt einschlag je nach roll... max bei (self.maxRollTooControllYaw - 10) grad
            retdata[1] = retdata[1] * faktor
            if(dPitch>0):
                faktorpitch = dPitch/self.pitchTooLargeTooRoll
                retdata[1] = (retdata[1] + faktorpitch)/2
            retdata[1] = (retdata[1]+5)*1000 # formatierung fuer separierung zur unterscheidung zwischen yaw und pitch regelung
            retdata[2] = istGyr[2]
            return retdata
            
        
    def SetOldWaypoint(self, n_w):
        for i in range (0, 3):
            self.oldWaypoint[i] = n_w[i]
    def SetNewWaypoint(self, n_w):
        for i in range (0, 5):
            self.nextWaypoint[i] = n_w[i]

def MaxAbsWert(wert,unten,oben):
    if wert < unten:
        wert = unten
    if wert > oben:
        wert = oben
    return wert
