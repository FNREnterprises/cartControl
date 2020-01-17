
import time
import numpy as np
import pyrealsense2 as rs
import pickle

import inmoovGlobal
import config
import findObstacles
import arduinoSend
import camImages
import handleRobot

def headPositioned(requestedYaw, requestedPitch):
    return abs(requestedYaw - config.headImuYaw) < 3 and \
           abs(requestedPitch - config.headImuPitch) < 3


def moveRequest(moveDirection: config.MoveDirection, speed: int, distance: int, protected: bool) -> bool:

    # check for obstacles with head cam for forward moves
    if moveDirection == config.MoveDirection.FORWARD and protected:

        # check for connection with servo control, used to position head
        if config.robotControlConnection is None:
            config.log(f"no connection with robotControl, protected forward move not possible")
            return False

        if not camImages.cams[inmoovGlobal.HEAD_CAM].streaming:
            camImages.cams[inmoovGlobal.HEAD_CAM].startStream()

        # try to move head into ground watch position
        config.headImuYaw, config.headImuPitch = handleRobot.positionHead(0, config.pitchGroundWatchDegrees)
        if config.headImuYaw is None:
            return False
        else:
            if not headPositioned(0, config.pitchGroundWatchDegrees):
                config.log(f"moveRequest, could not position head")
                return False

        # take depth image
        rawPoints = camImages.cams[inmoovGlobal.HEAD_CAM].takeDepth()
        if rawPoints is None:
            config.log(f"could not acquire the depth points")
            return False

        # check for ground obstacles
        rotatedPoints = camImages.alignPointsWithGround(rawPoints, config.headImuPitch)
        distClosestObstacle, obstacleDirection = findObstacles.distGroundObstacles(rotatedPoints)

        if distClosestObstacle is None:
            config.log(f"could not evaluate ground obstacles")
            return False

        if distClosestObstacle > 0.2:
            config.log(f"start driving, free move: {distClosestObstacle * 100:.0f} cm")
            arduinoSend.sendMoveCommand(moveDirection, speed, distance, protected)
            config.flagInForwardMove = True     # triggers action in threadMonitorForwardMove
            return True

        else:
            config.log(f"can not move forward because of an obstacle ahead in {distClosestObstacle * 100:.0f} cm at {obstacleDirection:.0f} degrees")
            return False

    else:
        # for non-forward or unprotected moves do not use the head cam
        arduinoSend.sendMoveCommand(moveDirection, speed, distance, protected)


