
import time
import datetime
import sys
import cv2
import logging
from marvinglobal import marvinglobal as mg
from marvinglobal import marvinShares as ms
from marvinglobal import cartClasses
from marvinglobal import distanceSensorClasses

processName = 'cartControl'
marvinShares = ms.MarvinShares()

configurationMaster = cartClasses.Configuration()
stateMaster = cartClasses.State()
locationMaster = mg.Location()
movementMaster = cartClasses.Movement()

usSensorsMaster = [distanceSensorClasses.UsSensor(i) for i in range(mg.NUM_US_DISTANCE_SENSORS)]

swipingIrSensorsMaster = [distanceSensorClasses.SwipingIrSensor(i) for i in range(mg.NUM_SWIPING_IR_DISTANCE_SENSORS)]
#IrSensorsMaster = {}  # [cartClasses.IrDistanceSensor] * mg.NUM_SWIPING_IR_DISTANCE_SENSORS

staticIrSensorsMaster = [distanceSensorClasses.StaticIrSensor(i) for i in range(mg.NUM_STATIC_IR_DISTANCE_SENSORS)]

floorOffsetMaster = [distanceSensorClasses.FloorOffset()
                     for _ in range(mg.NUM_IR_DISTANCE_SENSORS)]
#irSensorReferenceDistanceMaster = [cartClasses.IrSensorReferenceDistance() for _ in range(mg.NUM_IR_DISTANCE_SENSORS)]

#obstacleDistanceMaster = cartClasses.ObstacleDistance()
platformImuMaster = cartClasses.ImuData()
headImuMaster = cartClasses.ImuData()
battery6VMaster = cartClasses.Battery(5)
battery12VMaster = cartClasses.Battery(10)

simulateArduino = False

# configuration values for cart arduino infrared distance limits
DISTANCE_UNKNOWN = 999

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
arduinoConnEstablished:bool = False
_mVolts12V = 0
_mVolts6V = 0

taskStarted = time.time()
_f = None


distanceMonitoring = False

# robotControl connection
robotControlIp = '192.168.0.35'
robotControlAuthKey = b'marvin'
skeletonRequestQueue = None
servoStaticDict = None
servoCurrentDict = None
#robotControlConnection = None
#robotControlConnectionFirstTry = True



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

D415_Z = 0.095              # meters, cam position above neck axis
D415_Y = 0.15               # meters, cam distance in front of neck axis
D415_MountingPitch = -29    # degrees, cam is mounted with a downward pointing angle


cartReady = False
clientList = []


# threads
startupThread = None
msgThread = None
navThread = None


def log(msg, publish=True):

    logtime = str(datetime.datetime.now())[11:23]
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


def getIrSensorName(sensorId):
    if sensorId < 10:
        return ["swipeFrontLeft  ",
                "swipeFrontCenter",
                "swipeFrontRight ",
                "swipeBackLeft   ",
                "swipeBackCenter ",
                "swipeBackRight  ",
                "staticLeftFront ",
                "staticRightFront",
                "staticLeftBack  ",
                "staticRightBack "][sensorId]
    else:
        return f"invalid sensorId {sensorId}"


def getUsSensorName(sensorID):
    return ["left", "middleLeft", "middleRight", "Right"][sensorID]


def signedAngleDifference(start, end):
    """
    calculate angle difference in range -180 .. 180 between start and end degrees in range 0 .. 360
    """
    diff = end - start
    d = abs(diff) % 360
    value = 360 - d if d > 180 else d
    sign = 1 if (0 <= diff <= 180) or (-180 >= diff >= -360) else -1
    return sign * value

