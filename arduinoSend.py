
import time

import config
import marvinglobal.marvinglobal as mg

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
    config.arduino.write(bytes(msg + ',\n', 'ascii'))
    config.log(f"configuration part a sent, {msg}")
    time.sleep(0.5)
    msg = f"b,{config.SPEED_FACTOR:07.4f},{config.SPEED_OFFSET:07.4f},{config.SPEED_FACTOR_SIDEWAY:05.2f},{config.SPEED_FACTOR_DIAGONAL:05.2f}"
    config.arduino.write(bytes(msg + ',\n', 'ascii'))
    config.log(f"configuration part b sent, {msg}")
    time.sleep(0.5)     # problems with overwriting message in Arduino

    # motor speed unifyers, FRONT_RIGHT, FRONT_LEFT, BACK_RIGHT, BACK_LEFT
    msg = f"d,104,102,097,088"
    config.arduino.write(bytes(msg + ',\n', 'ascii'))
    config.log(f"configuration part c sent, {msg}")
    time.sleep(0.5)

    msg = f"d,"
    config.arduino.write(bytes(msg + ',\n', 'ascii'))
    config.log(f"arduino configuration complete sent: {msg}")
    time.sleep(0.5)


def sendMoveCommand():

    flagProtected = 1 if config.movementLocal.protected else 0
    msg = f"1,{config.movementLocal.moveDirection.value}," \
          f"{config.movementLocal.speed:03.0f}," \
          f"{config.movementLocal.distanceRequested:04.0f}," \
          f"{config.movementLocal.maxDuration:05.0f}," \
          f"{flagProtected}"

    config.log(f"sendMoveCommand {msg=}")
    config.arduino.write(bytes(msg + ",\n", 'ascii'))


def sendRotateCommand():

    if config.movementLocal.moveDirection == mg.MoveDirection.ROTATE_LEFT:  # rotate counterclock
        msg = f"2,{abs(config.movementLocal.rotationRequested):03.0f},{config.movementLocal.speed:03.0f},{config.movementLocal.maxDuration:05.0f}"
        config.log(f"Send rotate counterclock {msg}")

    else:
        msg = f"3,{abs(config.movementLocal.rotationRequested):03.0f},{config.movementLocal.speed:03.0f},{config.movementLocal.maxDuration:05.0f}"
        config.log(f"Send rotate clockwise {msg}")

    config.log(f"sendMoveCommand {msg=}")
    config.arduino.write(bytes(msg + ',\n', 'ascii'))



def sendStopCommand(reason):
    # command 4
    msg = b'4' + b',\n'
    config.log(f"Send stop, reason: {reason}")
    config.arduino.write(msg)


def sendSpeedCommand(speed):
    # command 6
    msg = bytes(str(6000 + speed) + ',\n', 'ascii')
    # cartGlobal.log("Send speed "+str(msg))
    config.arduino.write(msg) if sendCommandsToArduino else config.log(f"->A, {msg}")
    #cartControl.setRequestedCartSpeed(speed)


def testSensorCommand(sensorId):

    msg = b'7,' + bytes(str(sensorId), 'ascii') + b',\n'
    config.log("test sensor " + str(msg))
    config.arduino.write(msg)


def sendHeartbeat():
    try:
        msg = b'9' + b',\n'
        config.arduino.write(msg)
    except:
        pass


def requestCartYaw():
    msg = b'5' + b',\n'
    config.log(f"requestCartYaw, {msg=}")
    config.arduino.write(msg)


def setVerbose(newState: bool):
    try:
        if newState:
            msg = b'v,1' + b',\n'
        else:
            msg = b'v,0' + b',\n'
        config.log(f"setVerbose, {msg=}")
        config.arduino.write(msg)
    except:
        pass


def setIrSensorReferenceDistances(sensorId, distances):
    msg = f"f,{sensorId:02.0f},"
    msg += ','.join([str(int(i)) for i in distances])

    config.log(f"ir sensor reference distances sent {msg}")
    config.arduino.write(bytes(msg + ',\n', 'ascii'))



