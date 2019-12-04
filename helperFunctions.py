<<<<<<< HEAD
import time
=======
#Calculate final position function
def calcBallFinalPosition(input):
>>>>>>> af29416f625d4b84a7c3a1bc63374722653f1f33

#IK funtion for arm
def invK():




<<<<<<< HEAD
#Store ball location history
pos_hist = [[], [], [], [], [], [], []]
def addSensors(aX, aY, aZ, bX, bY, bZ):
    pos_hist[0].append(time.time()) # ADD TIME STAMP
    pos_hist[1].append(aX)
    pos_hist[2].append(aY)
    pos_hist[3].append(aZ)
    pos_hist[4].append(bX)
    pos_hist[5].append(bY)
    pos_hist[6].append(bZ)
=======
#IK funtion for arm
def calcArmPosition():

>>>>>>> af29416f625d4b84a7c3a1bc63374722653f1f33

#Calculate final position function
def calcPosition():
    fX = 0.0
    fY = 0.0
    fZ = 0.0

    return fX, fY, fZ