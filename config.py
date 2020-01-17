
import time
import datetime
import sys
import cv2
from enum import Enum, unique
import logging

from PyQt5.QtCore import Qt

serverName = 'cartControl'
serverReady = False

# configuration values for cart arduino infrared distance limits
FLOOR_MAX_OBSTACLE = 3  # cm
FLOOR_MAX_ABYSS = 4     # cm
delayBetweenDistanceMeasurements = 2    # value 1 caused unstable analog read values from distance sensor (2.4.2019)
finalDockingMoveDistance = 12            # distance to move forward after seeing activated docking switch

cartDocked = False

# currently set by auto calibration in cart
#                              FL     FR    L   R    BL     BR
#                             n  f   n  f   n   n   n  f   n  f
#distanceSensorCorrections = [-1,-5,  4,-5,  4,  3,  5, 8,  5, -1]

d415Cfg = None
d415Handle = None

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
pcName = None
pcIP = None
MY_RPC_PORT = 20001

taskName = 'cartControl'


# values for arduino to calculate distance mm/s from speed value
# (values depend on battery level too, see excel sheet for calc)
SPEED_FACTOR = 1.49
SPEED_OFFSET = 44.6
SPEED_FACTOR_SIDEWAY = 0.5
SPEED_FACTOR_DIAGONAL = 0.63


obstacleInfo = []


streamD415 = None
D415streaming = False


class MoveDirection(Enum):
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
_moveStartTime = None
_lastCartCommand = ""
_orientationBeforeMove = 0
_moveDirection = MoveDirection.STOP

# Current cart position (center of cart) relative to map center, values in mm
platformImuYaw = 0
platformImuYawCorrection = 0
platformImuPitch = 0.0
platformImuRoll = 0.0

headImuYaw = 0              # yaw of head in relation to robot
# add to cartYaw for getting the orientation in relation to the map
headImuPitch = 0.0
headImuRoll = 0.0
headImuSet = False

###########################################
# cart status values
###########################################
cartStateChanged: bool = False

cartStatus: str = "unknown"
cartStatusColor = ("black","lightgray")

currentCommand: str = "STOP"

cartLocationX = 0
cartLocationY = 0
cartOrientation = 0

cartTargetLocationX = 0
cartTargetLocationY = 0
cartTargetOrientation = 0

cartLastPublishTime = time.time()
lastLocationSaveTime = time.time()

cartSensorUpdate: bool = False

distanceSensorObstacle = "-"
distanceSensorAbyss = "-"

distanceRequested = 0
distanceMoved = 0


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


distanceMonitoring = False

# robotControl connection
robotControlIp = pcIP
robotControlPort = 20004
robotControlConnection = None
robotControlConnectionFirstTry = True

# ground watch position of head for depth
pitchGroundWatchDegrees = -35   # head.neck

# ahead watch position of head for depth
pitchAheadWatchDegrees = -15   # head.neck


# the depth cam image includes the cart front when looking down
# use a registered cart front image to locate the cart front in the current image
# to register a new cart front image delete the existing cart front file and use a situation without obstacles
# check for the cart front only in the lower half of the image (cartFrontMinRow .. image height)
cartFrontShape = None
cartFrontLine = None
cartFrontBorderX = 4
cartFrontRows = 30
cartFrontSearchRows = 60      # number of rows with expected cart front in ground view
cartFrontRowShift = 0
cartFrontColShift = 0

cartLength2 = 0.29      # meters, distance from cart center to cart front
robotWidth = 0.6        # meters
robotWidth2 = robotWidth/2        # meters
robotHeight = 1.7       # meters
robotBaseZ = 0.88       # standard table height (could be dynamic, not fully implemented yet)
robotNeckZ = 0.63       # neck rotation point above base
robotNeckY = 0.09       # neck position in relation to cart center

D415_FOV_Horizontal = 69.4  # degrees
D415_FOV_Vertical = 42.5    # degrees
D415_Rows = 240
D415_Rows2 = D415_Rows/2
D415_Cols = 428

D415_Z = 0.095              # meters, cam position above neck axis
D415_Y = 0.15               # meters, cam distance in front of neck axis
D415_MountingPitch = -29    # degrees, cam is mounted with a downward pointing angle
distOffsetCamFromCartFront = 0.1    # meters
flagInForwardMove = False


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



def evalCartDegrees(orientation, moveDirection: MoveDirection):
    """
    set moveDegrees based on moveDirection
    cart orientation 0 is to the right
    :param orientation:
    :param moveDirection:
    :return:
    """
    log(f"evalCartDegrees, orientation: {orientation}, moveDirection: {moveDirection}")

    moveDegrees = None
    if moveDirection == MoveDirection.FORWARD:
        moveDegrees = 0
    elif moveDirection == MoveDirection.BACKWARD:
        moveDegrees = 180
    elif moveDirection == MoveDirection.LEFT:
        moveDegrees = 90
    elif moveDirection == MoveDirection.RIGHT:
        moveDegrees = -90
    elif moveDirection == MoveDirection.FOR_DIAG_LEFT:
        moveDegrees = 45
    elif moveDirection == MoveDirection.FOR_DIAG_RIGHT:
        moveDegrees = -45
    elif moveDirection == MoveDirection.BACK_DIAG_LEFT:
        moveDegrees = 135
    elif moveDirection == MoveDirection.BACK_DIAG_RIGHT:
        moveDegrees = -135
    if moveDegrees is None:
        return None
    else:
        return (orientation + moveDegrees) % 360  # make 0 degrees pointing to the right


def getSensorName(sensorID):
    return ["front left", "front center", "front right", "back left", "back center", "back right", "left front", "right front", "left back", "right back"][sensorID ]


def signedAngleDifference(start, end):
    """
    calculate angle difference in range -180 .. 180 between start and end degrees in range 0 .. 360
    """
    diff = end - start
    d = abs(diff) % 360
    value = 360 - d if d > 180 else d
    sign = 1 if (0 <= diff <= 180) or (-180 >= diff >= -360) else -1
    return sign * value

