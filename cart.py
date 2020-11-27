
import os
import time
from typing import Tuple, List
import numpy as np
import simplejson as json

import marvinglobal.marvinglobal as mg
import marvinglobal.cartClasses as cartCls
import config
import arduinoSend

persistedLocation = "cartLocation.json"

# the local reference of the cart information
# changes get sent to marvinData
# other processes in need of cart information get updated by the shareManager

def publishState():
    updStmt:Tuple[mg.SharedDataItem,cartCls.State] = (mg.SharedDataItem.CART_STATE, config.stateLocal)
    config.share.updateSharedData(updStmt)

def publishLocation():
    updStmt:Tuple[mg.SharedDataItem, cartCls.Location] = (mg.SharedDataItem.CART_LOCATION, config.locationLocal)
    config.share.updateSharedData(updStmt)

def publishMovement():
    updStmt:Tuple[mg.SharedDataItem, cartCls.Movement] = (mg.SharedDataItem.CART_MOVEMENT, config.movementLocal)
    config.share.updateSharedData(updStmt)

def publishPlatformImu():
    updStmt:Tuple[mg.SharedDataItem,cartCls.ImuData] = (mg.SharedDataItem.PLATFORM_IMU, config.platformImuLocal)
    config.share.updateSharedData(updStmt)

def publishHeadImu():
    updStmt:Tuple[mg.SharedDataItem,cartCls.ImuData] = (mg.SharedDataItem.HEAD_IMU, config.headImuLocal)
    config.share.updateSharedData(updStmt)


def publishObstacleDistances():
    updStmt:Tuple[mg.SharedDataItem,cartCls.ObstacleDistance] = (mg.SharedDataItem.OBSTACLE_DISTANCE, config.obstacleDistanceLocal)
    config.share.updateSharedData(updStmt)


#cartLastPublishTime = time.time()
#lastLocationSaveTime = time.time()

def initiateMove(moveDirection, speed, distanceMm, protected=True):

    # a protected move includes monitoring of obstacles during the movement
    # protection is only available for FORWARD moves
    #obstacleDistance = distanceMm
    if protected and moveDirection == mg.MoveDirection.FORWARD:
        # request image processing to monitor move
        config.share.imageProcessingQueue.put(mg.START_MONITOR_FORWARD_MOVE)
        protected = True
    else:
        protected = False

    config.movementLocal.moveDirection = moveDirection
    config.movementLocal.speed = speed
    config.movementLocal.distanceRequested = min(distanceMm, 2500)     # limit distance for single command
    config.movementLocal.protected = protected
    config.movementLocal.start.x = config.locationLocal.x
    config.movementLocal.start.y = config.locationLocal.y
    config.movementLocal.start.yaw = config.locationLocal.yaw
    config.movementLocal.moveAngle = config.movementLocal.evalMoveAngle(moveDirection)
    config.movementLocal.target.x = config.movementLocal.start.x + int(distanceMm * np.cos(np.radians(config.movementLocal.moveAngle)))
    config.movementLocal.target.y = config.movementLocal.start.y + int(distanceMm * np.sin(np.radians(config.movementLocal.moveAngle)))
    config.movementLocal.moveStartTime = time.time()
    config.movementLocal.maxDuration = min(((distanceMm / speed) * 1500) + 1500, 15000)
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
    config.log(f"!A6 cart move stopped, {reason}")

    # if sensorTest is active stop it
    #if distanceSensors.sensorInTest is not None:
    #    distanceSensors.sensorInTest = None


def initiateRotation(relativeAngle, speed=200):

    config.movementLocal.moveDirection = mg.MoveDirection.ROTATE_LEFT if relativeAngle > 0 else mg.MoveDirection.ROTATE_RIGHT
    config.movementLocal.rotationRequested = relativeAngle
    config.movementLocal.speed = speed
    config.movementLocal.startYaw = config.locationLocal.yaw
    config.moveStartTime = time.time()
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


def updateCartLocation(yaw, distance):
    # only update when values have changed
    if abs(yaw - config.locationLocal.yaw) < 1 and abs(distance - config.movementLocal.distanceMoved) < 5:
        return
    config.movementLocal.distanceMoved = distance
    config.locationLocal.x = config.movementLocal.start.x + int(config.movementLocal.distanceMoved * np.cos(np.radians(config.movementLocal.start.yaw)))
    config.locationLocal.y = config.movementLocal.start.y + int(config.movementLocal.distanceRequested * np.sin(np.radians(config.movementLocal.start.yaw)))
    config.share.updateSharedData((mg.SharedDataItem.CART_LOCATION, config.locationLocal))
    saveCartLocation()


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
    config.locationLocal = cartCls.Location()
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
