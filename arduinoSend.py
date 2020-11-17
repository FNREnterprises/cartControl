
import time

import config
import cartControl
import cart
import distanceSensors

sendCommandsToArduino = True   # allow or prevent messages to arduino

########################################################################
########################################################################
def sendConfigValues():
    msg = 'a,' + str(config.FLOOR_MAX_OBSTACLE).zfill(2) + "," \
          + str(config.FLOOR_MAX_ABYSS).zfill(2) + "," \
          + str(config.NUM_REPEATED_MEASURES).zfill(2) + "," \
          + str(config.DELAY_BETWEEN_ANALOG_READS).zfill(2) + "," \
          + str(config.MIN_SCAN_CYCLE_DURATION).zfill(3) + "," \
          + str(config.finalDockingMoveDistance).zfill(2)
    config.arduino.write(bytes(msg + '\n', 'ascii'))
    config.log(f"configuration part a sent, {msg}")
    time.sleep(0.5)

    msg = f"b,{config.SPEED_FACTOR:07.4f},{config.SPEED_OFFSET:07.4f},{config.SPEED_FACTOR_SIDEWAY:05.2f},{config.SPEED_FACTOR_DIAGONAL:05.2f}"
    config.arduino.write(bytes(msg + '\n', 'ascii'))
    config.log(f"configuration part b sent, {msg}")
    time.sleep(0.5)     # problems with overwriting message in Arduino

    # motor speed unifyers, FRONT_RIGHT, FRONT_LEFT, BACK_RIGHT, BACK_LEFT
    msg = f"d,104,102,097,088"
    config.arduino.write(bytes(msg + '\n', 'ascii'))
    config.log(f"configuration part d sent, {msg}")
    time.sleep(0.5)

    msg = f"e,"
    config.arduino.write(bytes(msg + '\n', 'ascii'))
    config.log(f"arduino configuration complete sent: {msg}")
    time.sleep(0.5)



def sendMoveCommand():

    flagProtected = 1 if config.movment.protected else 0
    msg = f"1,{config.movement.moveDirection}," \
          f"{config.movement.speed:03.0f}," \
          f"{config.movement.distanceMm:04.0f}," \
          f"{config.movement.maxDuration:05.0f}," \
          f"{config.movement.flagProtected}"

    if config.simulateArduino:
        config.log(f"simulated move: {msg}")
    else:
        config.arduino.write(bytes(msg + "\n", 'ascii'))


def sendRotateCommand():

    if config.movement.moveDirection == mg.MoveDirection.ROTATE_LEFT:  # rotate counterclock
        msg = f"2,{abs(config.movement.rotationRequested):03.0f},{config.movement.speed:03.0f},1"
        config.log(f"Send rotate counterclock {msg}")

    else:
        msg = f"3,{abs(config.movement.rotationRequested):03.0f},{config.movement.speed:03.0f},1"
        config.log(f"Send rotate clockwise {msg}")

    if config.simulateArduino:
        config.log("simulated rotation: {msg}")
    else:
        config.arduino.write(bytes(msg + '\n', 'ascii')) if sendCommandsToArduino else config.log(f"->A, {msg}")



def sendStopCommand(reason):
    # command 4
    msg = b'4' + b'\n'
    config.log(f"Send stop, reason: {reason}")
    if config.simulateArduino:
        config.log(f"simulated cart stop: {msg=}")
    else:
        config.arduino.write(msg)


def sendSpeedCommand(speed):
    # command 6
    msg = bytes(str(6000 + speed) + '\n', 'ascii')
    # cartGlobal.log("Send speed "+str(msg))
    config.arduino.write(msg) if sendCommandsToArduino else config.log(f"->A, {msg}")
    #cartControl.setRequestedCartSpeed(speed)


def testSensorCommand(sensorID):

    msg = b'7,' + bytes(str(sensorID), 'ascii') + b'\n'
    config.log("test sensor " + str(msg))
    config.arduino.write(msg)



def sendHeartbeat():
    try:
        msg = b'9' + b'\n'
        config.arduino.write(msg)
    except:
        pass


def requestCartYaw():
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


def setFloorDistances(sensorId):
    msg = f"f,{sensorId:02.0f},"
    msg += ','.join([str(int(i)) for i in config.sensorTestData.average/config.sensorTestData.numMeasures])
    config.log(f"Send averaged floor distances {msg}")
    if config.simulateArduino:
        config.log(f"->A, {msg}")
    else:
        config.arduino.write(bytes(msg + '\n', 'ascii'))

