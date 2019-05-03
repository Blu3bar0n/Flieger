#import KONST
from geopy import distance
import math

class TRAJEKTORIE():
    oldWaypoint = [0.0, 0.0, 0.0]
    nextWaypoint = [0.0, 0.0, 0.0, 0.0, 0.0]
    pitchDependingOnRollmin = 15
    pitchDependingOnRollmax = 60
    
    def NextWaypointManMode(self, istPos):
        print("NextWaypoint")
        dWp = [0.0, 0.0, 0.0]
        dWp[0] = (distance.distance([self.nextWaypoint[0], self.oldWaypoint[1]],[self.oldWaypoint[0], self.oldWaypoint[1]]).km) * 1000
        dWp[1] = (distance.distance([self.oldWaypoint[0], self.nextWaypoint[1]],[self.oldWaypoint[0], self.oldWaypoint[1]]).km) * 1000
        if((self.nextWaypoint[0] - self.oldWaypoint[0],)<0):
            dWp[0] = -dWp[0]
        if((self.nextWaypoint[1] - self.oldWaypoint[1])<0):
            dWp[1] = -dWp[1]
        dposWp = [0.0, 0.0, 0.0]
        dposWp[0] = (distance.distance([istPos[0], self.oldWaypoint[1]],[self.oldWaypoint[0], self.oldWaypoint[1]]).km) * 1000
        dposWp[1] = (distance.distance([self.oldWaypoint[0], istPos[1]],[self.oldWaypoint[0], self.oldWaypoint[1]]).km) * 1000
        if((istPos[0] - self.oldWaypoint[0],)<0):
            dposWp[0] = -dposWp[0]
        if((istPos[1] - self.oldWaypoint[1])<0):
            dposWp[1] = -dposWp[1]
        n_Wp = [0.0, 0.0, 0.0]
        faktor = ((dposWp[0] - istPos[0]) * dWp[0]+ (dposWp[1] - istPos[1]) * dWp[1])/(dWp[0] * dWp[0] + dWp[1] * dWp[1])
        n_Wp[0] = istPos[0] + faktor * dWp[0]
        n_Wp[1] = istPos[1] + faktor * dWp[1]
        faktor2 = math.sqrt(dWp[0] * dWp[0] + dWp[1] * dWp[1])
        dWp[0] = dWp[0] / faktor2
        dWp[1] = dWp[1] / faktor2
        n_Wp[0] = n_Wp[0] + 20 * dWp[0]
        n_Wp[1] = n_Wp[1] + 20 * dWp[1]
        n_Wp[2] = self.nextWaypoint[2] - istPos[2]
        return n_Wp

    def ShiftInRange(self,wert,unten,oben):
        if(wert < unten):
            wert = wert + abs(oben-unten)
        if(wert > oben):
            wert = wert - abs(oben-unten)
        return wert    
        
    def StupidControl(self, istPos, istGyr):
        #print("StupidControll")
        if(self.nextWaypoint[4]==0):
            istGyr[0] = 0.0
            istGyr[1] = 0.0 
            return istGyr
        if(self.nextWaypoint[4]==1):
            #///////////////////sollwerte////////////////
            sollPos = self.NextWaypointManMode(istPos)
            dpos = [0.0, 0.0, 0.0]
            dpos[0] = (distance.distance([sollPos[0], istPos[1]],[istPos[0], istPos[1]]).km) * 1000
            dpos[1] = (distance.distance([istPos[0], sollPos[1]],[istPos[0], istPos[1]]).km) * 1000
            dpos[2] = sollPos[2] - istPos[2]
            if((sollPos[0] - istPos[0],)<0):
                dpos[0] = -dpos[0]
            if((sollPos[1] - istPos[1])<0):
                dpos[1] = -dpos[1]
            Pitch = math.atan2(dpos[1], dpos[0]) * 180 / math.pi
            Pitch = self.ShiftInRange(Pitch, -180, 180)
            Yaw = -math.atan2(dpos[0], dpos[2]) * 180 / math.pi
            Yaw = self.ShiftInRange(Yaw, -180, 180)
            #///////////////////differenz////////////////
            dPitch = Pitch - istGyr[1]
            dYaw = Yaw - istGyr[2]
            dYaw = self.ShiftInRange(dYaw, -180, 180)
            print("dP,dY: ", dPitch,  dYaw)
            if(abs(dPitch) > 30):
                if(dPitch > 0):
                    dPitch = 30
                else:
                    dPitch = -30
            if(abs(istGyr[1]) > 30): #notfall Pitch
                retdata = [0.0, 0.0, 0.0]
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
            if(dPitch > 30): #krasser Pitch unterschied
                retdata = [0.0, 0.0, 0.0]
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
            retdata = [0.0, 0.0, 0.0]
            retdata[0] = dYaw
            if(retdata[0] >80):
                retdata[0] = 80
            if(retdata[0] < -80):
                retdata[0] = -80
            retdata[1] = dYaw/45
            if(retdata[1] >1):
                retdata[1] = 1
            if(retdata[1] < -1):
                retdata[1] = -1
            faktor = istGyr[0]/60
            if(faktor >1):
                faktor = 1
            if(faktor < -1):
                faktor = -1
            retdata[1] = retdata[1] * faktor
            retdata[1] = (retdata[1]+5)*1000 # formatierung fuer separierung
            retdata[2] = istGyr[2]
            return retdata
            
        
    def SetOldWaypoint(self, n_w):
        self.oldWaypoint = n_w
    def SetNewWaypoint(self, n_w):
        self.newWaypoint = n_w

