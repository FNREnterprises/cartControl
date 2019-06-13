
import time

import config
import cartControl
import watchDog

########################################################################
########################################################################
def sendConfigValues():
    msg = 'a,' + str(config.FLOOR_MAX_OBSTACLE).zfill(2) + "," \
          + str(config.FLOOR_MAX_ABYSS).zfill(2) + "," \
          + str(config.delayBetweenDistanceMeasurements).zfill(2) + "," \
          + str(config.finalDockingMoveDistance).zfill(2)
    config.arduino.write(bytes(msg + '\n', 'ascii'))
    time.sleep(0.5)

    msg = f"b,{config.SPEED_FACTOR:07.4f},{config.SPEED_OFFSET:07.4f},{config.SPEED_FACTOR_SIDEWAY:05.2f},{config.SPEED_FACTOR_DIAGONAL:05.2f}"
    config.arduino.write(bytes(msg + '\n', 'ascii'))
    time.sleep(0.5)     # problems with overwriting message in Arduino

    # distance sensor corrections
    ''' try with auto-calibrate for the moment
    msg = f"c"
    for corr in config.distanceSensorCorrections:
        msg += f",{corr:+03d}"
    config.arduino.write(bytes(msg + '\n', 'ascii'))
    config.log(f"distanceSensorCorrections sent: {msg}")
    time.sleep(0.5)
    '''

    # motor speed unifyers, FRONT_RIGHT, FRONT_LEFT, BACK_RIGHT, BACK_LEFT
    msg = f"d,104,102,097,088"
    config.arduino.write(bytes(msg + '\n', 'ascii'))
    config.log(f"motor speedUnifyers sent: {msg}")
    time.sleep(0.5)

def powerKinect(newState):
    config.log(f"arduinoSend, send power command: {newState}")
    if newState == True:
        powerMsg = f"e,1"
    else:
        powerMsg = f"e,0"
    config.arduino.write(bytes(powerMsg + "\n", 'ascii'))
    config.log(f"kinect power command sent: {powerMsg}")


def sendMoveCommand(moveDirection: 'config.Direction', speed, distanceMm, protected=True):
    '''
    send move command to cart
    distance calculation is based on rotary encoder on one of the weels for forward/backward moves
    for other moves a time assumption for the move is used
    if final orientation <> initial orientation send a rotate command at the end to end up in same orientation
    '''
    # conmmand 1,<dir>,<speed>,<distance>,<max duration>
    # e.g. forward, speed=180, distance=100 mm, max duration=5 s: 1,0,180,0100,0005

    obstacleDistance = distanceMm
    if protected:
        if moveDirection == config.Direction.FORWARD and config.kinectConnection is not None:
            numTries = 1
            while numTries <= 2:
                try:
                    startTime = time.time()
                    config.kinectConnection.root.startMonitoring()
                    config.monitoringWithKinect = True
                    config.log(f"kinect monitoring for obstacles activated")
                    break

                except Exception as e:
                    if numTries == 1:
                        # retry to connect
                        numTries += 1
                        watchDog.connectWithKinect()
                    else:
                        config.log(f"sendMoveCommand, could not start kinect monitoring, {e}")

        else:
            config.log(f"move without kinect control, {config.Direction(moveDirection)}")

    distanceMmLimited = min(distanceMm, 2500)  # limit distance for single command
    config._moveDirection = moveDirection
    cartControl.setCartMoveDistance(distanceMmLimited)
    cartControl.calculateCartTargetLocation(cartControl.getCartYaw(), distanceMm, moveDirection)
    config._requestedCartSpeed = speed
    moveDuration = ((distanceMmLimited / speed) * 1500) + 1500
    config.log(f"moveDuration: {moveDuration:.0f}")

    durationLimited = min(moveDuration, 15000)  # do not move more than 15 seconds without verification of free space

    cartControl.setMovementBlocked(False)
    cartControl.setCartMoving(True, time.time() + durationLimited + 2000)
    flagProtected = 1 if protected else 0
    moveMsg = f"1,{moveDirection.value},{speed:03.0f},{distanceMmLimited:04.0f},{durationLimited:05.0f},{flagProtected}"
    config.arduino.write(bytes(moveMsg + "\n", 'ascii'))
    config.log(
        f"Send move {moveMsg}, speed: {speed:.0f}, distance: {distanceMmLimited:.0f}, duration: {durationLimited:.0f}")


def sendRotateCommand(relAngle, speed=200):

    config.cartRotating = relAngle != 0
    cartControl.setMovementBlocked(False)
    config.rotateStartDegrees = cartControl.getCartYaw()
    config._targetOrientation = (config.rotateStartDegrees + relAngle) % 360
    cartControl.setRequestedCartSpeed(speed)

    # command 2
    # msg = f"b,{cartGlobal.SPEED_FACTOR:07.4f},{cartGlobal.SPEED_EXPONENT:07.4f},{cartGlobal.SPEED_FACTOR_SIDEWAY:05.2f}"
    # arduinoReceive.arduino.write(bytes(msg + '\n', 'ascii'))

    if relAngle > 0:  # rotate counterclock
        msg = f"2,{abs(relAngle):03.0f},{speed:03.0f},1"
        config.log(f"Send rotate counterclock {msg}")
        config.arduino.write(bytes(msg + '\n', 'ascii'))

    # command 3
    if relAngle < 0:
        msg = f"3,{abs(relAngle):03.0f},{speed:03.0f},1"
        config.log(f"Send rotate clockwise {msg}")
        config.arduino.write(bytes(msg + '\n', 'ascii'))


def sendStopCommand(reason):
    # command 4
    msg = b'4' + b'\n'
    config.log(f"Send stop, reason: {reason}")
    config.arduino.write(msg)
    cartControl.setCartMoving(False)


def sendSpeedCommand(speed):
    # command 6
    msg = bytes(str(6000 + speed) + '\n', 'ascii')
    # cartGlobal.log("Send speed "+str(msg))
    config.arduino.write(msg)
    cartControl.setRequestedCartSpeed(speed)


def sendReadSensorCommand(sensorID):
    cartControl.setCartMoveDistance(0)
    msg = b'7' + bytes(str(sensorID), 'ascii') + b'\n'
    config.log("Send readSensor " + str(msg))
    config.arduino.write(msg)


def sendHeartbeat():
    try:
        msg = b'9' + b'\n'
        config.arduino.write(msg)
    except:
        pass


def requestCartOrientation():
    msg = b'5' + b'\n'
    # cartGlobal.log("get Orientation " + msg)
    config.arduino.write(msg)


def setVerbose(newState: bool):
    try:
        if newState:
            msg = b'v,1' + b'\n'
        else:
            msg = b'v,0' + b'\n'
        config.arduino.write(msg)
    except:
        pass