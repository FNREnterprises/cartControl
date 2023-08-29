
import os
import time
from typing import Tuple, List
import numpy as np
import simplejson as json
import pickle

import marvinglobal.marvinglobal as mg
import marvinglobal.cartClasses
import config
import arduinoSend
import distanceSensors

persistedLocation = "cartLocation.json"

# the local reference of the cart information
# changes get sent to marvinData
# other processes in need of cart information get updated by the shareManager

def publishState():
    #updStmt:Tuple[mg.SharedDataItems,marvinglobal.cartClasses.State] = (mg.SharedDataItems.CART_STATE, config.stateMaster)
    updStmt = {'msgType': mg.SharedDataItems.CART_STATE, 'sender': config.processName,
               'info': config.stateMaster}
    config.marvinShares.updateSharedData(updStmt)

def publishLocation():
    #updStmt:Tuple[mg.SharedDataItems, marvinglobal.mg.Location] = (mg.SharedDataItems.CART_LOCATION, config.locationMaster)
    updStmt = {'msgType': mg.SharedDataItems.CART_LOCATION, 'sender': config.processName,
               'info': config.locationMaster}
    config.marvinShares.updateSharedData(updStmt)

def publishMovement():
    #updStmt:Tuple[mg.SharedDataItems, marvinglobal.cartClasses.Movement] = (mg.SharedDataItems.CART_MOVEMENT, config.movementMaster)
    updStmt = {'msgType': mg.SharedDataItems.CART_MOVEMENT, 'sender': config.processName,
               'info': config.movementMaster}
    config.marvinShares.updateSharedData(updStmt)

def publishPlatformImu():
    #updStmt:Tuple[mg.SharedDataItems,marvinglobal.cartClasses.ImuData] = (mg.SharedDataItems.PLATFORM_IMU, config.platformImuMaster)
    updStmt = {'msgType': mg.SharedDataItems.PLATFORM_IMU, 'sender': config.processName,
               'info': config.platformImuMaster}
    config.marvinShares.updateSharedData(updStmt)

def publishHeadImu():
    #updStmt:Tuple[mg.SharedDataItems,marvinglobal.cartClasses.ImuData] = (mg.SharedDataItems.HEAD_IMU, config.headImuMaster)
    updStmt = {'msgType': mg.SharedDataItems.HEAD_IMU, 'sender': config.processName,
               'info': config.headImuMaster}
    config.marvinShares.updateSharedData(updStmt)


def publishObstacleDistances():
    #updStmt:Tuple[mg.SharedDataItems,marvinglobal.cartClasses.ObstacleDistance] = (mg.SharedDataItems.OBSTACLE_DISTANCE, config.obstacleDistanceMaster)
    updStmt = {'msgType': mg.SharedDataItems.OBSTACLE_DISTANCE, 'sender': config.processName,
               'info': config.obstacleDistanceMaster}
    config.marvinShares.updateSharedData(updStmt)


#cartLastPublishTime = time.time()
#timePersisted = time.time()

def initiateMove(moveDirectionEnum:mg.MoveDirection, speed, distanceMm, protected=True):

    # a protected FORWARD move includes monitoring of obstacles with the headCam
    if protected and moveDirectionEnum == mg.MoveDirection.FORWARD:
        # check for running image processing
        if "imageProcessing" not in config.marvinShares.processDict.keys():
            config.log(f"protected forward move needs a running image processing")
            return False

        # request image processing to monitor move
        config.marvinShares.imageProcessingRequestQueue.put(mg.ImageProcessingCommands.START_MONITOR_FORWARD_MOVE)

    config.movementMaster.moveDirectionEnum = moveDirectionEnum
    config.movementMaster.speed = speed
    config.movementMaster.distanceRequested = min(distanceMm, 2500)     # limit distance for single move command
    config.movementMaster.relAngleRequested = 0
    config.movementMaster.protected = protected
    config.movementMaster.startX = config.locationMaster.x
    config.movementMaster.startY = config.locationMaster.y
    config.movementMaster.startYaw = config.locationMaster.yaw
    config.movementMaster.moveAngle = config.movementMaster.evalMoveAngle(moveDirectionEnum)
    config.movementMaster.targetX = config.movementMaster.startX + int(distanceMm * np.cos(np.radians(config.movementMaster.moveAngle)))
    config.movementMaster.targetY = config.movementMaster.startY + int(distanceMm * np.sin(np.radians(config.movementMaster.moveAngle)))
    config.movementMaster.targetYaw = config.locationMaster.yaw
    config.movementMaster.moveStartTime = time.time()
    config.movementMaster.maxDuration = int(min(((distanceMm / speed) * 1500) + 1500, 15000))
    config.movementMaster.reasonStopped = "-"
    config.movementMaster.blockEvent = mg.CartMoveBlockEvents.FREE_PATH
    publishMovement()

    config.stateMaster.cartMoving = True
    config.stateMaster.cartRotating = False
    config.stateMaster.currentCommand = "MOVE"
    publishState()

    arduinoSend.sendMoveCommand()
    return True


def terminateMove(distanceMoved, yaw, reason):

    # stop move monitoring if it was requested
    # dumy comment to test git hub updates to fnrenterprise fork project
    if config.movementMaster.protected:
        config.marvinShares.imageProcessingRequestQueue.put(mg.ImageProcessingCommands.STOP_MONITOR_FORWARD_MOVE)

    config.stateMaster.cartMoving = False
    config.stateMaster.cartRotating = False
    config.stateMaster.currentCommand = "STOP"
    publishState()

    config.movementMaster.distanceMoved = distanceMoved
    config.movementMaster.reasonStopped = reason
    publishMovement()

    if abs(yaw - config.locationMaster.yaw) > 1:
        config.locationMaster.yaw = yaw
        publishLocation()

    # config.log("<-A " + recv, publish=False)  # stop and reason
    config.log(f"!S4 cart move stopped, {reason}")

    # if sensorTest is active stop it
    distanceSensors.sensorInTest = None


def initiateRotation(relativeAngle, speed=200):

    config.movementMaster.moveDirectionEnum = mg.MoveDirection.ROTATE_LEFT if relativeAngle > 0 else mg.MoveDirection.ROTATE_RIGHT
    config.movementMaster.speed = speed
    config.movementMaster.distanceRequested = 0
    config.movementMaster.relAngleRequested = relativeAngle
    config.movementMaster.protected = False

    config.movementMaster.startX = config.locationMaster.x
    config.movementMaster.startY = config.locationMaster.y
    config.movementMaster.startYaw = config.locationMaster.yaw
    config.movementMaster.moveAngle = 0
    config.movementMaster.targetX = config.locationMaster.x
    config.movementMaster.targetY = config.locationMaster.y
    config.movementMaster.targetYaw = config.locationMaster.yaw + relativeAngle
    config.movementMaster.moveStartTime = time.time()
    config.movementMaster.maxDuration = 2500 + abs(relativeAngle) * 50;
    config.movementMaster.blockEvent = mg.CartMoveBlockEvents.FREE_PATH
    publishMovement()

    config.stateMaster.cartMoving = False
    config.stateMaster.cartRotating = True
    config.stateMaster.currentCommand = "ROTATE"
    publishState()

    arduinoSend.sendRotateCommand()


def terminateRotation(reason):
    config.stateMaster.cartMoving = False
    config.stateMaster.cartRotating = False
    config.stateMaster.currentCommand = "STOP"
    publishState()

    config.movementMaster.reasonStopped = reason
    publishMovement()

    config.log(f"!S3 cart rotation stopped, {reason}")


def updateCartLocation(yaw, distance):
    # only update when values have a significant change
    if abs(yaw - config.locationMaster.yaw) < 1 and abs(distance - config.movementMaster.distanceMoved) < 5:
        return
    #config.log(f"!P1, update cart location, yaw: {yaw}, distance: {distance}, move angle: {config.movementMaster.moveAngle}")
    config.locationMaster.x = config.movementMaster.startX + int(config.movementMaster.distanceMoved * np.cos(np.radians(config.movementMaster.moveAngle)))
    config.locationMaster.y = config.movementMaster.startY + int(config.movementMaster.distanceMoved * np.sin(np.radians(config.movementMaster.moveAngle)))
    #config.marvinShares.updateSharedData((mg.SharedDataItems.CART_LOCATION, config.locationMaster))
    publishLocation()
    saveCartLocation()

    config.movementMaster.angleRotated = config.movementMaster.startYaw - yaw
    config.movementMaster.distanceMoved = distance
    #config.marvinShares.updateSharedData((mg.SharedDataItems.CART_MOVEMENT, config.movementMaster))
    publishMovement()


def updateCartRotation(yaw):
    # only update when values have a significant change
    if abs(yaw - config.locationMaster.yaw) < 1:
        return
    config.log(f"!P2, update cart rotation, yaw: {yaw}")
    config.locationMaster.yaw = yaw
    #config.marvinShares.updateSharedData((mg.SharedDataItems.CART_LOCATION, config.locationMaster))
    publishLocation()
    saveCartLocation()

    config.movementMaster.rotated = config.movementMaster.startYaw - yaw
    #config.marvinShares.updateSharedData((mg.SharedDataItems.CART_MOVEMENT, config.movementMaster))
    publishMovement()


def saveCartLocation():
    # avoid too many saves
    if time.time() - config.locationMaster.timePersisted < 2:
        return

    # Saving the location objects:
    cartData = {'posX': int(config.locationMaster.x), 'posY': int(config.locationMaster.y), 'yaw': int(config.locationMaster.yaw)}
    #if cart.location.x == 0 and cart.location.y == 0:
    #    config.log(f"CART LOCATION SAVED AS 0,0")
    filename = persistedLocation
    with open(filename, "w") as write_file:
        json.dump(cartData, write_file)
    config.locationMaster.timePersisted = time.time()
    #config.log(f"cart location saved")


def loadCartLocation():
    """
    we load the cart location from the file system
    as the carts imu is reset with each start of the cart the carts orientation might be 0
    set an orientationCorrection value to account for this
    :return:
    """
    # Getting back the last saved cart data
    config.log(f"load cart location")
    config.locationMaster = mg.Location()
    filename = persistedLocation
    if os.path.exists(filename):
        with open(filename, "r") as read_file:
            cartData = json.load(read_file)

        config.locationMaster.x = cartData['posX']
        config.locationMaster.y = cartData['posY']
        config.platformImuMaster.yawCorrection = (cartData['yaw'] - config.platformImuMaster.yaw) % 360
        config.locationMaster.yaw = (cartData['yaw'] + config.platformImuMaster.yawCorrection) % 360

    else:
        config.locationMaster.x = 0
        config.locationMaster.y = 0
        config.locationMaster.yaw = 0
        config.platformImuMaster.yawCorrection = 0

    publishPlatformImu()
    publishLocation()

    config.log(f"imuYaw: {config.platformImuMaster.yaw}," +
        f"cartYawCorrection: {config.platformImuMaster.yawCorrection}," +
        f"roll: {config.platformImuMaster.roll}," +
        f"pitch: {config.platformImuMaster.pitch}, " +
        f"cartX: {config.locationMaster.x}, " +
        f"cartY: {config.locationMaster.y}, " +
        f"lastYaw: {config.locationMaster.yaw}")
