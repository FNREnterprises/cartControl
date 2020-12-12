
import os
import time
from typing import Tuple, List
import numpy as np
import simplejson as json

import marvinglobal.marvinglobal as mg
import marvinglobal.cartClasses
import config
import arduinoSend

persistedLocation = "cartLocation.json"

# the local reference of the cart information
# changes get sent to marvinData
# other processes in need of cart information get updated by the shareManager

def publishState():
    updStmt:Tuple[mg.SharedDataItem,marvinglobal.cartClasses.State] = (mg.SharedDataItem.CART_STATE, config.stateLocal)
    config.share.updateSharedData(updStmt)

def publishLocation():
    updStmt:Tuple[mg.SharedDataItem, marvinglobal.cartClasses.Location] = (mg.SharedDataItem.CART_LOCATION, config.locationLocal)
    config.share.updateSharedData(updStmt)

def publishMovement():
    updStmt:Tuple[mg.SharedDataItem, marvinglobal.cartClasses.Movement] = (mg.SharedDataItem.CART_MOVEMENT, config.movementLocal)
    config.share.updateSharedData(updStmt)

def publishPlatformImu():
    updStmt:Tuple[mg.SharedDataItem,marvinglobal.cartClasses.ImuData] = (mg.SharedDataItem.PLATFORM_IMU, config.platformImuLocal)
    config.share.updateSharedData(updStmt)

def publishHeadImu():
    updStmt:Tuple[mg.SharedDataItem,marvinglobal.cartClasses.ImuData] = (mg.SharedDataItem.HEAD_IMU, config.headImuLocal)
    config.share.updateSharedData(updStmt)


def publishObstacleDistances():
    updStmt:Tuple[mg.SharedDataItem,marvinglobal.cartClasses.ObstacleDistance] = (mg.SharedDataItem.OBSTACLE_DISTANCE, config.obstacleDistanceLocal)
    config.share.updateSharedData(updStmt)


#cartLastPublishTime = time.time()
#lastLocationSaveTime = time.time()

def initiateMove(moveDirection, speed, distanceMm, protected=True):

    # a protected FORWARD move includes monitoring of obstacles with the headCam
    if protected and moveDirection == mg.MoveDirection.FORWARD:
        # request image processing to monitor move
        config.share.imageProcessingQueue.put(mg.START_MONITOR_FORWARD_MOVE)

    config.movementLocal.moveDirection = moveDirection
    config.movementLocal.speed = speed
    config.movementLocal.distanceRequested = min(distanceMm, 2500)     # limit distance for single move command
    config.movementLocal.rotationRequested = 0
    config.movementLocal.protected = protected
    config.movementLocal.startX = config.locationLocal.x
    config.movementLocal.startY = config.locationLocal.y
    config.movementLocal.startYaw = config.locationLocal.yaw
    config.movementLocal.moveAngle = config.movementLocal.evalMoveAngle(moveDirection)
    config.movementLocal.targetX = config.movementLocal.startX + int(distanceMm * np.cos(np.radians(config.movementLocal.moveAngle)))
    config.movementLocal.targetY = config.movementLocal.startY + int(distanceMm * np.sin(np.radians(config.movementLocal.moveAngle)))
    config.movementLocal.targetYaw = config.locationLocal.yaw
    config.movementLocal.moveStartTime = time.time()
    config.movementLocal.maxDuration = int(min(((distanceMm / speed) * 1500) + 1500, 15000))
    config.movementLocal.reasonStopped = "-"
    publishMovement()

    config.stateLocal.cartMoving = True
    config.stateLocal.cartRotating = False
    config.stateLocal.currentCommand = "MOVE"
    publishState()

    arduinoSend.sendMoveCommand()


def terminateMove(distanceMoved, yaw, reason):

    # stop move monitoring if it was requested
    if config.movementLocal.protected:
        config.share.imageProcessingQueue.put(mg.STOP_MONITOR_FORWARD_MOVE)

    config.stateLocal.cartMoving = False
    config.stateLocal.cartRotating = False
    config.stateLocal.currentCommand = "STOP"
    publishState()

    config.movementLocal.distanceMoved = distanceMoved
    config.movementLocal.reasonStopped = reason
    publishMovement()

    if abs(yaw - config.locationLocal.yaw) > 1:
        config.locationLocal.yaw = yaw
        publishLocation()

    # config.log("<-A " + recv, publish=False)  # stop and reason
    config.log(f"!S4 cart move stopped, {reason}")

    # if sensorTest is active stop it
    #if distanceSensors.sensorInTest is not None:
    #    distanceSensors.sensorInTest = None


def initiateRotation(relativeAngle, speed=200):

    config.movementLocal.moveDirection = mg.MoveDirection.ROTATE_LEFT if relativeAngle > 0 else mg.MoveDirection.ROTATE_RIGHT
    config.movementLocal.speed = speed
    config.movementLocal.distanceRequested = 0
    config.movementLocal.rotationRequested = relativeAngle
    config.movementLocal.protected = False

    config.movementLocal.startX = config.locationLocal.x
    config.movementLocal.startY = config.locationLocal.y
    config.movementLocal.startYaw = config.locationLocal.yaw
    config.movementLocal.moveAngle = 0
    config.movementLocal.targetX = config.locationLocal.x
    config.movementLocal.targetY = config.locationLocal.y
    config.movementLocal.targetYaw = config.locationLocal.yaw + relativeAngle
    config.movementLocal.moveStartTime = time.time()
    config.movementLocal.maxDuration = 2500 + abs(relativeAngle) * 50;
    publishMovement()

    config.stateLocal.cartMoving = False
    config.stateLocal.cartRotating = True
    config.stateLocal.currentCommand = "ROTATE"
    publishState()

    arduinoSend.sendRotateCommand()


def terminateRotation(reason):
    config.stateLocal.cartMoving = False
    config.stateLocal.cartRotating = False
    config.stateLocal.currentCommand = "STOP"
    publishState()

    config.movementLocal.reasonStopped = reason
    publishMovement()

    config.log(f"!S3 cart rotation stopped, {reason}")


def updateCartLocation(yaw, distance):
    # only update when values have a significant change
    if abs(yaw - config.locationLocal.yaw) < 1 and abs(distance - config.movementLocal.distanceMoved) < 5:
        return
    #config.log(f"!P1, update cart location, yaw: {yaw}, distance: {distance}, move angle: {config.movementLocal.moveAngle}")
    config.locationLocal.x = config.movementLocal.startX + int(config.movementLocal.distanceMoved * np.cos(np.radians(config.movementLocal.moveAngle)))
    config.locationLocal.y = config.movementLocal.startY + int(config.movementLocal.distanceMoved * np.sin(np.radians(config.movementLocal.moveAngle)))
    config.share.updateSharedData((mg.SharedDataItem.CART_LOCATION, config.locationLocal))
    saveCartLocation()

    config.movementLocal.rotated = config.movementLocal.startYaw - yaw
    config.movementLocal.distanceMoved = distance
    config.share.updateSharedData((mg.SharedDataItem.CART_MOVEMENT, config.movementLocal))


def updateCartRotation(yaw):
    # only update when values have a significant change
    if abs(yaw - config.locationLocal.yaw) < 1:
        return
    config.log(f"!P2, update cart rotation, yaw: {yaw}")
    config.locationLocal.yaw = yaw
    config.share.updateSharedData((mg.SharedDataItem.CART_LOCATION, config.locationLocal))
    saveCartLocation()

    config.movementLocal.rotated = config.movementLocal.startYaw - yaw
    config.share.updateSharedData((mg.SharedDataItem.CART_MOVEMENT, config.movementLocal))


def saveCartLocation():
    # avoid too many saves
    if time.time() - config.locationLocal.lastLocationSaveTime < 2:
        return

    # Saving the location objects:
    cartData = {'posX': int(config.locationLocal.x), 'posY': int(config.locationLocal.y), 'yaw': int(config.locationLocal.yaw)}
    #if cart.location.x == 0 and cart.location.y == 0:
    #    config.log(f"CART LOCATION SAVED AS 0,0")
    filename = persistedLocation
    with open(filename, "w") as write_file:
        json.dump(cartData, write_file)
    config.locationLocal.lastLocationSaveTime = time.time()
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
    config.locationLocal = marvinglobal.cartClasses.Location()
    filename = persistedLocation
    if os.path.exists(filename):
        with open(filename, "r") as read_file:
            cartData = json.load(read_file)

        config.locationLocal.x = cartData['posX']
        config.locationLocal.y = cartData['posY']
        config.platformImuLocal.yawCorrection = (cartData['yaw'] - config.platformImuLocal.yaw) % 360
        config.locationLocal.yaw = (cartData['yaw'] + config.platformImuLocal.yawCorrection) % 360

    else:
        config.locationLocal.x = 0
        config.locationLocal.y = 0
        config.locationLocal.yaw = 0
        config.platformImuLocal.yawCorrection = 0

    publishPlatformImu()
    publishLocation()

    config.log(f"imuYaw: {config.platformImuLocal.yaw}," +
        f"cartYawCorrection: {config.platformImuLocal.yawCorrection}," +
        f"roll: {config.platformImuLocal.roll}," +
        f"pitch: {config.platformImuLocal.pitch}, " +
        f"cartX: {config.locationLocal.x}, " +
        f"cartY: {config.locationLocal.y}, " +
        f"lastYaw: {config.locationLocal.yaw}")
