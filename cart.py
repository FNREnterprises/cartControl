
import time
from dataclasses import dataclass
from typing import Tuple, List
import numpy as np

import marvinglobal.marvinglobal as mg
import config
import arduinoSend

# the local reference of the cart information
# changes get sent to marvinData
# other processes in need of cart information get updated by the shareManager

def publishState():
    updStmt:Tuple[int,mg.State] = (mg.SharedDataUpdate.CART_STATE, config.state)
    config.share.updateSharedData(updStmt)

def publishLocation():
    updStmt:Tuple[int,mg.Location] = (mg.SharedDataUpdate.CART_LOCATION, config.location)
    config.share.updateSharedData(updStmt)

def publishMovement():
    updStmt:Tuple[int,mg.Movement] = (mg.SharedDataUpdate.CART_MOVEMENT, config.movement)
    config.share.updateSharedData(updStmt)

def publishPlatformImu():
    updStmt:Tuple[int,mg.ImuData] = (mg.SharedDataUpdate.PLATFORM_IMU, config.platformImu)
    config.share.updateSharedData(updStmt)

def publishHeadImu():
    updStmt:Tuple[int,mg.ImuData] = (mg.SharedDataUpdate.HEAD_IMU, config.headImu)
    config.share.updateSharedData(updStmt)

def publishFloorOffset(sensorId:int):
    updStmt:Tuple[int,int,...] = (mg.SharedDataUpdate.FLOOR_OFFSET, sensorId, config.floorOffset.offset[sensorId])
    config.share.updateSharedData(updStmt)

def publishSensorTestData():
    updStmt:Tuple[mg.SharedDataUpdate,mg.SensorTestData] = (mg.SharedDataUpdate.SENSOR_TEST_DATA, config.sensorTestData)
    config.share.updateSharedData(updStmt)

def publishObstacleDistances():
    updStmt:Tuple[int,mg.ObstacleDistance] = (mg.SharedDataUpdate.OBSTACLE_DISTANCE, config.obstacleDistance)
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

    config.movement.moveDirection = moveDirection
    config.movement.speed = speed
    config.movement.distanceRequested = min(distanceMm, 2500)     # limit distance for single command
    config.movement.protected = protected
    config.movement.start.x = config.location.x
    config.movement.start.y = config.location.y
    config.movement.start.yaw = config.location.yaw
    config.movement.moveAngle = config.movement.evalMoveAngle(moveDirection)
    config.movement.target.x = config.movement.start.x + int(distanceMm * np.cos(np.radians(config.movement.moveAngle)))
    config.movement.target.y = config.movement.start.y + int(distanceMm * np.sin(np.radians(config.movement.moveAngle)))
    config.movement.moveStartTime = time.time()
    config.movement.maxDuration = min(((distanceMm / speed) * 1500) + 1500, 15000)
    publishMovement()

    config.state.cartMoving = True
    config.state.cartRotating = False
    config.state.currentCommand = "MOVE"
    publishState()

    arduinoSend.sendMoveCommand()


def terminateMove(distanceMoved, yaw, reason):

    # stop move monitoring if it was requested
    if config.movement.protected:
        config.share.imageProcessingQueue.put(mg.STOP_MONITOR_FORWARD_MOVE)

    config.state.cartMoving = False
    config.state.cartRotating = False
    config.state.currentCommand = "STOP"
    publishState()

    config.movement.distanceMoved = distanceMoved
    config.movement.reasonStopped = reason
    publishMovement()

    config.location.yaw = yaw
    publishLocation()

    # config.log("<-A " + recv, publish=False)  # stop and reason
    config.log(f"!A6 cart move stopped, {reason}")

    # if sensorTest is active stop it
    #if distanceSensors.sensorInTest is not None:
    #    distanceSensors.sensorInTest = None


def initiateRotation(relativeAngle):

    config.movement.moveDirection = config.MoveDirection.ROTATE_LEFT if relativeAngle > 0 else config.MoveDirection.ROTATE_RIGHT
    config.movement.rotationRequested = relativeAngle
    config.movement.speed = 200
    config.movement.startYaw = config.location.yaw
    config.moveStartTime = time.time()
    publishMovement()

    config.state.cartMoving = False
    config.state.cartRotating = True
    config.state.currentCommand = "ROTATE"
    publishState()

    arduinoSend.sendRotateCommand()

def terminateRotation(reason):
    config.state.cartMoving = False
    config.state.cartRotating = False
    config.state.currentCommand = "STOP"
    publishState()

    config.movement.reasonStopped = reason
    publishMovement()

def updateCartLocation(distanceMoved, final=True):
    movement.distanceMoved = distanceMoved
    location.x = movement.start.x + int(movement.distanceMoved * np.cos(np.radians(movement.start.yaw)))
    location.y = movement.start.y + int(movement.distanceRequested * np.sin(np.radians(movement.start.yaw)))
    config.share.updateSharedData((mg.SharedDataUpdate.CART_LOCATION, location))
