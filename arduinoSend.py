
import time

import config
import arduinoReceive

########################################################################
########################################################################
def sendConfigValues():
    msg = 'a' + str(config.SHORT_RANGE_MIN).zfill(2) + str(config.SHORT_RANGE_MAX).zfill(2) \
          + str(config.LONG_RANGE_MIN).zfill(2) + str(config.LONG_RANGE_MAX).zfill(2) + str(
        config.delayBetweenDistanceMeasurements).zfill(2)
    arduinoReceive.ser.write(bytes(msg + '\n', 'ascii'))

    msg = f"b,{config.SPEED_FACTOR:07.4f},{config.SPEED_OFFSET:07.4f},{config.SPEED_FACTOR_SIDEWAY:05.2f},{config.SPEED_FACTOR_DIAGONAL:05.2f}"
    arduinoReceive.ser.write(bytes(msg + '\n', 'ascii'))

    # cartGlobal.log(f"{msg}")


def sendMoveCommand(direction, speed, distanceMm):
    '''
    send move command to cart
    distance calculation is based on rotary encoder on one of the weels for forward/backward moves
    for other moves a time assumption for the move is used
    if final orientation <> initial orientation send a rotate command at the end to end up in same orientation
    '''
    # conmmand 1,<dir>,<speed>,<distance>,<max duration>
    # e.g. forward, speed=180, distance=100 mm, max duration=5 s: 1,0,180,0100,0005
    distanceMmLimited = min(distanceMm, 2000)  # limit distance for single command
    config.setMoveDirection(direction)
    config.setCartMoveDistance(distanceMmLimited)
    config.calculateCartTargetLocation(config.getCartOrientation(), distanceMm, direction)
    config.setRequestedCartSpeed(speed)
    moveDuration = ((distanceMmLimited / speed) * 850) + 1500
    if direction in [config.Direction.LEFT.value, config.Direction.RIGHT.value]:
        moveDuration = int(moveDuration / config.SPEED_FACTOR_SIDEWAY)
    if direction in [config.Direction.BACK_DIAG_LEFT, config.Direction.BACK_DIAG_RIGHT,
                     config.Direction.FOR_DIAG_LEFT, config.Direction.FOR_DIAG_RIGHT]:
        moveDuration = int(moveDuration / config.SPEED_FACTOR_DIAGONAL)
    durationLimited = min(moveDuration, 9999)  # do not move more than 10 seconds without verification of free space

    config.setMovementBlocked(False)
    config.setCartMoving(True, time.time() + durationLimited + 2000)

    moveMsg = f"1,{direction},{speed:03.0f},{distanceMmLimited:04.0f},{durationLimited:04.0f}"
    arduinoReceive.ser.write(bytes(moveMsg + "\n", 'ascii'))
    config.log(
        f"Send move {moveMsg}, speed: {speed:.0f}, distance: {distanceMmLimited:.0f}, duration: {durationLimited:.0f}")


def sendRotateCommand(relAngle, speed=200):

    config.setMovementBlocked(False)
    config.setTargetOrientation(relAngle)
    config.setRequestedCartSpeed(speed)

    # command 2
    # msg = f"b,{cartGlobal.SPEED_FACTOR:07.4f},{cartGlobal.SPEED_EXPONENT:07.4f},{cartGlobal.SPEED_FACTOR_SIDEWAY:05.2f}"
    # arduinoReceive.ser.write(bytes(msg + '\n', 'ascii'))

    if relAngle > 0:  # rotate counterclock
        msg = f"2,{abs(relAngle):03.0f},{speed:03.0f}"
        config.log(f"Send rotate counterclock {msg}")
        arduinoReceive.ser.write(bytes(msg + '\n', 'ascii'))

    # command 3
    if relAngle < 0:
        msg = f"3,{abs(relAngle):03.0f},{speed:.03f}"
        config.log(f"Send rotate clockwise {msg}")
        arduinoReceive.ser.write(bytes(msg + '\n', 'ascii'))


def sendStopCommand(reason):
    # command 4
    msg = b'4' + b'\n'
    config.log(f"Send stop, reason: {reason}")
    arduinoReceive.ser.write(msg)
    config.setCartMoving(False)


def sendSpeedCommand(speed):
    # command 6
    msg = bytes(str(6000 + speed) + '\n', 'ascii')
    # cartGlobal.log("Send speed "+str(msg))
    arduinoReceive.ser.write(msg)
    config.setRequestedCartSpeed(speed)


def sendReadSensorCommand(sensorID):
    msg = b'7' + bytes(str(sensorID), 'ascii') + b'\n'
    config.log("Send readSensor " + str(msg))
    arduinoReceive.ser.write(msg)


def sendHeartbeat():
    try:
        msg = b'9' + b'\n'
        arduinoReceive.ser.write(msg)
    except:
        pass


def requestCartOrientation():
    msg = b'5' + b'\n'
    # cartGlobal.log("get Orientation " + msg)
    arduinoReceive.ser.write(msg)

