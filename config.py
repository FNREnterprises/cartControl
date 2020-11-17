
import time
import datetime
import sys
import cv2
import logging
import marvinglobal.marvinglobal as mg

processName = 'cartControl'
share:mg.MarvinShares = None   # shared data

state = mg.State()
location = mg.Location()
movement = mg.Movement()
sensorTestData = mg.SensorTestData()
floorOffset = mg.FloorOffset()
obstacleDistance = mg.ObstacleDistance()
platformImu = mg.ImuData()
headImu = mg.ImuData()
battery6V = mg.Battery(5)
battery12V = mg.Battery(10)

simulateArduino = True

# configuration values for cart arduino infrared distance limits
DISTANCE_UNKNOWN = 999
FLOOR_MAX_OBSTACLE = 15  # mm
FLOOR_MAX_ABYSS = 20     # mm
NUM_REPEATED_MEASURES = 7
DELAY_BETWEEN_ANALOG_READS = 20         # value 1 caused unstable analog read values from distance sensor (2.4.2019)
MIN_SCAN_CYCLE_DURATION = 80            # when moving maintain a minimal delay between sensor reads
finalDockingMoveDistance = 12           # distance to move forward after seeing activated docking switch


# values for arduino to calculate distance mm/s from speed value
# (values depend on battery level too, see excel sheet for calc)
SPEED_FACTOR = 1.49
SPEED_OFFSET = 44.6
SPEED_FACTOR_SIDEWAY = 0.5
SPEED_FACTOR_DIAGONAL = 0.63

obstacleInfo = []

# flags for gui update sections
floorOffsetDataChanged: bool = False
distanceDataChanged: bool = False


# Current cart position (center of cart) relative to map center, values in mm
#platformImuYaw = 0
#platformImuYawCorrection = 0
#platformImuPitch = 0.0
#platformImuRoll = 0.0

#headImuYaw = 0              # yaw of head in relation to robot
# add to cartYaw for getting the orientation in relation to the map
#headImuPitch = 0.0
#headImuRoll = 0.0
#headImuSet = False



CENTER_OF_CART_ROTATION_X = 0
CENTER_OF_CART_ROTATION_Y = -30

_lastBatteryCheckTime = time.time()
_batteryStatus = None
arduino = None
arduinoConnEstablished = 0
_mVolts12V = 0
_mVolts6V = 0

taskStarted = time.time()
_f = None


distanceMonitoring = False

# robotControl connection
robotControlIp = '192.168.0.35'
robotControlAuthKey = b'marvin'
servoRequestQueue = None
servoStaticDict = None
servoCurrentDict = None
#robotControlConnection = None
#robotControlConnectionFirstTry = True

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

cartReady = False
clientList = []


# threads
startupThread = None
msgThread = None
navThread = None



def log(msg, publish=True):

    logtime = str(datetime.datetime.now())[11:22]
    logging.info(f"{logtime} - {msg}")
    print(f"{logtime} - {msg}")

    if publish:
        # forward to central logger, not used yet
        pass


def saveImg(img, frameNr):
    try:
        cv2.imwrite(f"C:/cartControl/floorImages/floor_{frameNr}.jpg", img)
    except:
        log(f"cartGlobal, saveImg exception {sys.exc_info()[0]}")


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

