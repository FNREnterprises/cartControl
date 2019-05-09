
import time
import datetime
import sys
import cv2
import numpy as np
from enum import Enum, unique
import logging

# configuration values for cart arduino infrared distance limits
SHORT_RANGE_MIN = 10
SHORT_RANGE_MAX = 20
LONG_RANGE_MIN = 14
LONG_RANGE_MAX = 34
delayBetweenDistanceMeasurements = 2    # value 1 caused unstable analog read values from distance sensor (2.4.2019)
finalDockingMoveDistance = 8            # distance to move forward after seeing activated docking switch

"""
static sensorDefinition sensorDefinitions[DISTANCE_SENSORS_COUNT]{
	{ VL_NAH,  "VL_NAH ", A21, NAH, true, A0, VL, -1 },
	{ VL_FERN, "VL_FERN", A21, FERN, true, A2, VL, -3 },
	{ VR_NAH,  "VR_NAH ", A41, NAH, true, A1, VR, 4 },
	{ VR_FERN, "VR_FERN", A21, FERN, true, A3, VR, -5 },
	{ LM_NAH,  "LM_NAH ", A21, NAH, true, A13, LM, -1 },
	{ RM_NAH,  "RM_NAH ", A21, NAH, true, A7, RM, 0 },
	{ HL_NAH,  "HL_NAH ", A21, NAH, true, A8, HL, 2 },
	{ HL_FERN, "HL_FERN", A21, FERN, true, A9, HL, 0 },
	{ HR_NAH,  "HR_NAH ", A21, NAH, true, A10, HR, 3 },
	{ HR_FERN, "HR_FERN", A21, FERN, true, A11, HR, 3 }
"""
#                              FL    FR   L  R   BL    BR
#                             n  f  n  f  n  n  n  f  n  f
distanceSensorCorrections = [-3,-5, 4,-5, 4, 3, 5, 8, 5, 0]


##################################################################
# needs to be the same as in navManager
##################################################################
@unique
class cMoveState(Enum):
    """
    list of move states for single move and move sequence
    """
    PENDING = 0
    IN_PROGRESS = 1
    INTERRUPTED = 2
    FINISHED = 3
    FAILED = 4


##################################################################
# cartControl can run as slave of navManager or in standalone mode
##################################################################
standAloneMode = False  # False -> slave mode,  True -> standalone mode

MY_IP = "192.168.0.17"
MY_RPC_PORT = 20001
MY_NAME = 'cartControl'


# values for arduino to calculate distance mm/s from speed value
# (values depend on battery level too, see excel sheet for calc)
SPEED_FACTOR = 1.49
SPEED_OFFSET = 44.6
SPEED_FACTOR_SIDEWAY = 0.5
SPEED_FACTOR_DIAGONAL = 0.63


NUM_DISTANCE_SENSORS = 10
NUM_MEASUREMENTS_PER_SCAN = 11

distanceList = np.zeros((NUM_DISTANCE_SENSORS, NUM_MEASUREMENTS_PER_SCAN))
obstacleInfo = []

distanceSensors = []
# VL nah
distanceSensors.append(
    {'sensorID': 0, 'direction': 'forward', 'position': 'left', 'range': 'short', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (0, 0), 'installed': True, 'servoIndex': 0, 'color': 'red',
     'rotation': -180})
# VL fern
distanceSensors.append(
    {'sensorID': 1, 'direction': 'forward', 'position': 'left', 'range': 'long', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (0, 1), 'installed': True, 'servoIndex': 0, 'color': 'blue',
     'rotation': -180})
# VR nah
distanceSensors.append(
    {'sensorID': 2, 'direction': 'forward', 'position': 'right', 'range': 'short', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (1, 0), 'installed': True, 'servoIndex': 1, 'color': 'red',
     'rotation': -180})
# VR fern
distanceSensors.append(
    {'sensorID': 3, 'direction': 'forward', 'position': 'right', 'range': 'long', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (1, 1), 'installed': True, 'servoIndex': 1, 'color': 'blue',
     'rotation': -180})
# LM nah
distanceSensors.append(
    {'sensorID': 4, 'direction': 'left', 'position': 'middle', 'range': 'short', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (2, 0), 'installed': True, 'servoIndex': 2, 'color': 'red',
     'rotation': 90})
# RM nah
distanceSensors.append(
    {'sensorID': 5, 'direction': 'right', 'position': 'middle', 'range': 'short', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (2, 1), 'installed': True, 'servoIndex': 2, 'color': 'red',
     'rotation': -90})
# HL nah
distanceSensors.append(
    {'sensorID': 6, 'direction': 'backward', 'position': 'left', 'range': 'short', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (3, 0), 'installed': True, 'servoIndex': 3, 'color': 'red', 'rotation': 0})
# HL fern
distanceSensors.append(
    {'sensorID': 7, 'direction': 'backward', 'position': 'left', 'range': 'long', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (3, 1), 'installed': True, 'servoIndex': 3, 'color': 'blue',
     'rotation': 0})
# HR nah
distanceSensors.append(
    {'sensorID': 8, 'direction': 'backward', 'position': 'right', 'range': 'short', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (3, 0), 'installed': True, 'servoIndex': 4, 'color': 'red', 'rotation': 0})
# HR fern
distanceSensors.append(
    {'sensorID': 9, 'direction': 'backward', 'position': 'right', 'range': 'long', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (3, 1), 'installed': True, 'servoIndex': 4, 'color': 'blue',
     'rotation': 0})


class Direction(Enum):
    STOP = 0
    FORWARD = 1
    FOR_DIAG_RIGHT = 2
    FOR_DIAG_LEFT = 3
    LEFT = 4
    RIGHT = 5
    BACKWARD = 6
    BACK_DIAG_RIGHT = 7
    BACK_DIAG_LEFT = 8
    ROTATE_LEFT = 9
    ROTATE_RIGHT = 10


# odometry is only active when cart is moving
cartMoving = False
_cartMoveTimeout = 0
cartRotating = False
_requestedCartSpeed = 0
_cartSpeedFromOdometry = 0

_movementBlocked = False
_timeMovementBlocked = None
_moveDistanceRequested = 0
_moveStartX = 0
_moveStartY = 0
rotateStartDegrees = 0
_targetOrientation = 0
_moveStartTime = None
_lastCartCommand = ""
_orientationBeforeMove = 0
_moveDirection = Direction.STOP

# Current cart position (center of cart) relative to map center, values in mm
imuDegrees = 0
imuDegreesCorrection = 0
_imuPitch = 0.0
_imuRoll = 0.0

cartLocationX = 0
cartLocationY = 0
_cartTargetLocationX = 0
_cartTargetLocationY = 0
cartLastPublishTime = time.time()
lastLocationSaveTime = time.time()


CENTER_OF_CART_ROTATION_X = 0
CENTER_OF_CART_ROTATION_Y = -30

_lastBatteryCheckTime = time.time()
_batteryStatus = None
arduino = None
arduinoStatus = 0
_mVolts12V = 0
_mVolts6V = 0

taskStarted = time.time()
_f = None


kinectIp = MY_IP
kinectPort = 20003
kinectConnection = None
monitoringWithKinect = False
firstKinectConnectionAttempt = True

# def startlog():
#    logging.basicConfig(filename="cartControl.log", level=logging.INFO, format='%(asctime)s - %(name)s - %(message)s', filemode="w")
cartReady = False
clientList = []

import rpcSend      # problem when importing at start of file in gui.py

def log(msg, publish=True):

    logtime = str(datetime.datetime.now())[11:22]
    logging.info(f"{logtime} - {msg}")
    print(f"{logtime} - {msg}")

    if publish:
        rpcSend.publishLog("cartControl - " + msg)


def saveImg(img, frameNr):
    try:
        cv2.imwrite(f"C:/cartControl/floorImages/floor_{frameNr}.jpg", img)
    except:
        log(f"cartGlobal, saveImg exception {sys.exc_info()[0]}")



def evalTrigDegrees(orientation, moveDirection: Direction):
    """
    set moveDegrees based on moveDirection
    cart orientation 0 is to the right
    :param orientation:
    :param moveDirection:
    :return:
    """
    #log(f"evalTrigDegrees, orientation: {orientation}, moveDirection: {moveDirection}")

    moveDegrees = None
    if moveDirection == Direction.FORWARD:
        moveDegrees = 0
    elif moveDirection == Direction.BACKWARD:
        moveDegrees = 180
    elif moveDirection == Direction.LEFT:
        moveDegrees = 90
    elif moveDirection == Direction.RIGHT:
        moveDegrees = -90
    elif moveDirection == Direction.FOR_DIAG_LEFT:
        moveDegrees = 45
    elif moveDirection == Direction.FOR_DIAG_RIGHT:
        moveDegrees = -45
    elif moveDirection == Direction.BACK_DIAG_LEFT:
        moveDegrees = 135
    elif moveDirection == Direction.BACK_DIAG_RIGHT:
        moveDegrees = -135
    if moveDegrees is None:
        return None
    else:
        return (orientation + moveDegrees) % 360  # make 0 degrees pointing to the right


def getSensorName(sensorID):
    return ["", "FL near", "FL far", "FR near", "FR far", "left", "right", "BL near", "BL far", "BR near", "BL far"][sensorID + 1]


def signedAngleDifference(start, end):
    """
    calculate angle difference in range -180 .. 180 between start and end degrees in range 0 .. 360
    """
    diff = end - start
    d = abs(diff) % 360
    value = 360 - d if d > 180 else d
    sign = 1 if (0 <= diff <= 180) or (-180 >= diff >= -360) else -1
    return sign * value

