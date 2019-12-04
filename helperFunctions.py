import time

#IK funtion for arm
def invK():

#Store ball location history
pos_hist = [[], [], [], [], [], [], []] # time, arm x, arm y, arm z, ball x, ball y, ball z
history_length = 100
TIME = 0
ARM_X = 1
ARM_Y = 2
ARM_Z = 3
BALL_X = 4
BALL_Y = 5
BALL_Z = 6
# ROBOT SETTINGS
SET_X = 3
LOWEST_Y = 1
HIGHEST_Y = 2
LOWEST_Z = -1
HIGHEST_Z = 1
def addSensors(aX, aY, aZ, bX, bY, bZ):
    pos_hist[TIME].append(time.time()) # ADD TIME STAMP
    pos_hist[ARM_X].append(aX)
    pos_hist[ARM_Y].append(aY)
    pos_hist[ARM_Z].append(aZ)
    pos_hist[BALL_X].append(bX)
    pos_hist[BALL_Y].append(bY)
    pos_hist[BALL_Z].append(bZ)
    if(len(pos_hist) > history_length):
        pos_hist = pos_hist[int(len(pos_hist)/2):] # cut list in half, but keeping the most recent

#Calculate final arm position function
def calcPosition():
    # ONLY RUN IF POSSIBLE
    if len(pos_hist[TIME]) < 2:
        return None
    # CALCULATE LATEST VEL X & TIME LEFT
    dX = pos_hist[BALL_X][-1] - pos_hist[BALL_X][-2]
    dT = (pos_hist[TIME][-1] - pos_hist[TIME][-2])
    vX = dX/dT
    distanceLeft = pos_hist[BALL_X][-1] - pos_hist[ARM_X][-1]
    timeLeft = distanceLeft / vX
    # CALCULATE VELOCITY & INTERCEPT
    vY = (pos_hist[BALL_Y][-1] - pos_hist[BALL_Y][-2])/dT
    interceptY = pos_hist[BALL_Y][-2] + vY*timeLeft - 4.9*timeLeft**2 #Y_0 + VY*T - 4.9T^2
    # CALCULATE END POSITION FOR Z
    dZ = pos_hist[BALL_Z][-1] - pos_hist[BALL_Z][-2]
    vZ = dZ / dT
    interceptZ = pos_hist[BALL_Z][-1] + vZ * timeLeft
    # CHECK IF THIS IS NOT A CATCHABLE BALL
    if interceptY < LOWEST_Y or interceptY > HIGHEST_Y:
        return None
    if interceptZ < LOWEST_Z or interceptZ > HIGHEST_Z:
        return None
    # CALCULATE FINAL POSITIONS
    fX = SET_X
    fY = interceptY
    fZ = interceptZ
    return fX, fY, fZ