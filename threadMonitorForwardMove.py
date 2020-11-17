
import time

#import inmoovGlobal
import config
#import depthImage
import findObstacles
import arduinoSend
#import camImages
#import handleRobot

# threadMonitorForwardMove
# runs as thread
# activated by config.inForwardMove
# checks for ground and wall obstacles

def monitorLoop():

    checkGround = False
    checkAhead = False
    checkWall = False

    config.log(f"start monitoring for forward move")
    while True:

        if not config.flagInForwardMove:
            checkGround = False     # check Ground is done first time in move request
            checkAhead = True       # when cart starts to move check for ahead obstacles (0.5..2m)
            checkWall = False       # check wall follows after checkAhead
            if camImages.cams[inmoovGlobal.HEAD_CAM].streaming:
               camImages.cams[inmoovGlobal.HEAD_CAM].stopStream()
            time.sleep(1)
            continue

        else:   # if D415 is streaming stop it
            if not camImages.cams[inmoovGlobal.HEAD_CAM].streaming:
                camImages.cams[inmoovGlobal.HEAD_CAM].startStream()

        if checkGround:

            config.log(f"D415 ground check")

            yaw, pitch = handleRobot.positionHead(0, config.pitchGroundWatchDegrees)
            if yaw is None:
                config.log(f"moveRequest, could not position head")
                return False

            points = camImages.cams[inmoovGlobal.HEAD_CAM].getDepth()

            if points is None:
                config.log(f"could not acquire depth points")
                return False

            distClosestObstacle, obstacleDirection = findObstacles.distGroundObstacles(points)

            if distClosestObstacle > 0.2:
                config.log(f"continue driving, free move: {distClosestObstacle * 100:.0f} cm")
                checkAhead = True   # next check is for ahead obstacles
                checkGround = False

            else:
                config.log(
                    f"stop move forward because of a ground obstacle ahead in {distClosestObstacle * 100:.0f} cm at {obstacleDirection:.0f} degrees")
                arduinoSend.sendStopCommand("ground object detected")
                config.flagInForwardMove = False
                camImages.stopD415Stream()


        if checkAhead:   # check medium distance

            config.log(f"depth ahead check")

            yaw, pitch = handleRobot.positionHead(0, config.pitchAheadWatchDegrees)
            if yaw is None:
                config.log(f"moveRequest, could not position head")
                return False

            rawPoints = camImages.cams[inmoovGlobal.HEAD_CAM].takeDepth()
            if rawPoints is None:
                config.log(f"could not acquire the depth points")

            else:
                points = camImages.alignPointsWithGround(rawPoints, config.pitchAheadWatchDegrees)

                freeMoveDistance, obstacleDirection = findObstacles.lookForCartPathObstacle(points)
                config.log(f"obstacle in path: freeMoveDistance: {freeMoveDistance:.3f}, obstacleDirection: {obstacleDirection:.1f}")


            if freeMoveDistance > 0.4:
                config.log(f"continue driving, free move: {freeMoveDistance * 100:.0f} cm")
                checkAhead = False
                checkWall = True        # next check is for wall


            else:
                config.log(
                    f"stop move forward because of an obstacle ahead in {freeMoveDistance * 100:.0f} cm at {obstacleDirection:.0f} degrees")
                arduinoSend.sendStopCommand("object in cart path detected")
                config.flagInForwardMove = False


        if checkWall:   # check wall

            config.log(f"depth wall check")

            yaw, pitch = handleRobot.positionHead(0, inmoovGlobal.pitchWallWatchDegrees)
            if yaw is None:
                config.log(f"moveRequest, could not position head")
                return False

            points = camImages.cams[inmoovGlobal.HEAD_CAM].takeDepth()

            if points is None:
                config.log(f"could not acquire depth points")
                return None, None

            distClosestObstacle, obstacleDirection, _ = findObstacles.aboveGroundObstacles(points, inmoovGlobal.pitchWallWatchDegrees)

            if distClosestObstacle > 0.4:
                config.log(f"continue driving, free move: {distClosestObstacle * 100:.0f} cm")
                checkGround = True      # revert back to ground check
                checkWall = False

            else:
                config.log(
                    f"stop move forward because of a wall obstacle ahead in {distClosestObstacle * 100:.0f} cm at {obstacleDirection:.0f} degrees")
                arduinoSend.sendStopCommand("wall object detected")
                config.flagInForwardMove = False
                depthImage.stopD415Stream()
