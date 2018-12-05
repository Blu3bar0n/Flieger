import math
standby = [0, 0, 0, 1]
manuell = [3, 1, 0, 0]
halbAutonom = [3, 2, 0,0 ]
vollAutonom = [3, 3, 0,0 ]
kallib = [3, 3, 3, 0]
magkallib = [3, 2, 3 , 0]
magkallib2 = [2, 3, 3, 0]
magkallib3 = [3, 1, 3, 0]
magkallib4 = [1, 3, 3, 0]
magkallibwpos = [3, 2, 1 , 0]
DEAD = [0, 0, 0, 2]
CHDEF = [0.0,0.0,0.0,-1.0,0.0,0.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]
CHDEFREG = [0.0,0.0,0.0,-1.0,0.0,0.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]
CHSENDDEF = [0.0,0.0,0.0,-1.0,0.0,0.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]
OLDCHSENDDEF = [0.0,0.0,0.0,-1.0,0.0,0.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]
tmpdef = [0.0,0.0,0.0,-1.0,0.0,0.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]
KT = 0 # keine Taster
TL = 1 #Taster Links
TR = 2 #Taster Rechts
BT =3  #Beide Taster
TLL = 4 #Taster Links lang
TRL = 5 #Taster Rechts lang
BTL =6  #Beide Taster lang
MAXPITCH = 40 # 35 grad = 70% steigung 
MAXPITCHPUFFER = 10
MINWRONGANGLESPEED = 10
ethconn = False #eth0 nicht angeschlossen
Filename = ""
MAGKALLIBFILENAME = "/home/odroid/Desktop/ericWorkspace/Python/LOG/aMagKallib.txt"
ERDRADIUS = 6371000 # in m
Flughoehe = 50

def RMatrixWeltZuFahrzeug(vektor, winkel): #http://www.chrobotics.com/library/understanding-euler-angles
    w = winkel[2]                 #in Rad nicht in Grad !!!!!
    o = winkel[1]
    q = winkel[0]
    n_v =[0, 0, 0]
    n_v[0] = (math.cos(w) * math.cos(o)) * vektor[0] + (math.cos(o) * math.sin(w)) * vektor[1] + (- math.sin(o)) * vektor[2]
    n_v[1] = (math.cos(w) * math.sin(q) * math.sin(o) - math.cos(q) * math.sin(w)) * vektor[0] + (math.cos(q) * math.cos(w) + math.sin(q) * math.sin(w) * math.sin(o)) * vektor[1] + (math.cos(o) * math.sin(q)) * vektor[2]  
    n_v[2] = (math.sin(q)* math.sin(w) + math.cos(q) * math.cos(w) * math.sin(o)) * vektor[0]   + (math.cos(q) * math.sin(w) * math.sin(o) - math.cos(w) * math.sin(q)) * vektor[1] + (math.cos(q) * math.cos(o)) * vektor[2]  
    return n_v
    
def RMatrixFahrzeugZuWelt(vektor, winkel): #http://www.chrobotics.com/library/understanding-euler-angles
    q = winkel[0]                #in Rad nicht in Grad !!!!!
    o = winkel[1]
    w =winkel[2]
    n_v =[0, 0, 0]
    n_v[0] = (math.cos(o) * math.cos(w)) * vektor[0]   + (math.sin(q) * math.sin(o) * math.cos(w) - math.cos(q) * math.sin(w)) * vektor[1] + (math.cos(q)* math.sin(o) * math.cos(w) + math.sin(q) * math.sin(w)) * vektor[2]  
    n_v[1] = (math.cos(o) * math.sin(w)) * vektor[0]   + (math.sin(q) * math.cos(o) * math.sin(w) + math.cos(q) * math.cos(w)) * vektor[1] + (math.cos(q) * math.sin(w) * math.sin(o) - math.cos(w) * math.sin(q)) * vektor[2]  
    n_v[2] = (- math.sin(o)) * vektor[0]   + (math.cos(o) * math.sin(q)) * vektor[1] + (math.cos(q) * math.cos(o)) * vektor[2]  
    return n_v
    
def WGeschwindigkeitRMatrixFahrzeugZUWelt(VGesch, winkel): #http://www.princeton.edu/~stengel/Quaternions.pdf    https://robotics.stackexchange.com/questions/9502/use-data-from-gyroscope-to-calculate-orientation
    #w = winkel[2]                        #in Rad nicht in Grad !!!!!
    o = winkel[1]
    q = winkel[0]
    n_vg =[0, 0, 0]
    n_vg[0] = (1) * VGesch[0]   + (math.sin(q) * math.tan(o)) * VGesch[1] + (math.cos(q) * math.tan(o)) * VGesch[2]  
    n_vg[1] = (0) * VGesch[0]   + (math.cos(q)) * VGesch[1] + (- math.sin(q)) * VGesch[2]  
    n_vg[2] = (0) * VGesch[0]   + (math.sin(q) / math.cos(o)) * VGesch[1] + (math.cos(q) / math.cos(o)) * VGesch[2]  
    return n_vg





#places   degrees          distance
#-------  -------          --------
#0        1                111  km
#1        0.1              11.1 km
#2        0.01             1.11 km
#3        0.001            111  m
#4        0.0001           11.1 m
#5        0.00001          1.11 m
#6        0.000001         11.1 cm
#7        0.0000001        1.11 cm
#8        0.00000001       1.11 mm








