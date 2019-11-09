
import time

import config
import depthImage
import findObstacles
import arduinoSend

# monitorForwardMove
# runs as thread
# activated by config.inForwardMove
# checks for ground and wall obstacles

def monitorLoop():

    checkGround = False

    config.log(f"start monitoring for forward move")
    while True:
        if not config.inForwardMove:
            checkGround = False
            time.sleep(1)
            continue

        if checkGround:

            distClosestObstacle, obstacleDirection = findObstacles.distGroundObstacles()

            if distClosestObstacle > 0.2:
                config.log(f"continue driving, free move: {distClosestObstacle * 100:.0f} cm")

            else:
                config.log(
                    f"stop move forward because of a ground obstacle ahead in {distClosestObstacle * 100:.0f} cm at {obstacleDirection:.0f} degrees")
                arduinoSend.sendStopCommand("ground object detected")
                config.inForwardMove = False

        else:   # check wall

            distClosestObstacle, obstacleDirection = findObstacles.distWall()
