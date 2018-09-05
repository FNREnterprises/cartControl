import sys
import time
import serial

import gui
import cartGlobal

ser = None


def initSerial(comPort):
    global ser

    cartGlobal.arduinoStatus = 0

    while True:
        try:
            ser = serial.Serial(comPort)
            # try to reset the arduino
            ser.setDTR(False)
            time.sleep(0.1)
            ser.setDTR(True)
            ser.baudrate = 115200
            ser.writeTimeout = 0
            cartGlobal.log("Serial comm to arduino established")
            return

        except:
            cartGlobal.log(f"exception on serial connect with {comPort} {sys.exc_info()[0]}")
            time.sleep(3)


#####################################
# readMessages runs in its own thread
#####################################
def readMessages():
    import cartControl

    global ser

    while ser is None:
        time.sleep(0.1)

    while ser.is_open:

        if ser.inWaiting() > 0:
            # cartGlobal.log(f"inWaiting > 0")
            recvB = ser.readline(120)
            try:
                recv = recvB.decode()
            except:
                cartGlobal.log(f"problem with decoding cart msg '{recvB}'")

            # cartGlobal.log(f"line read {recv}")
            msgID = recvB[0:3].decode()

            # cartGlobal.log("in read message: ", msgID)

            if msgID == "!A0":  # "cart ready"
                cartGlobal.arduinoStatus = 1
                cartGlobal.log("cart ready")
                sendConfigValues()

            elif msgID == "!A1":  # distance values from obstacle sensors
                # !A1,<sensorID>,<ANZ_MESSUNGEN_PRO_SCAN>,[ANZ_MESSUNGEN_PRO_SCAN<value>,]
                messageItems = [int(e) if e.isdigit() else e for e in recv.split(',')]
                cartControl.updateDistances(messageItems[1:])
                # gui.controller.showDistances(messageItems[1])      test, could be responsible for delays in message read?


            elif msgID == "!A2":  # "obstacle:":
                # !A2,<shortest distance>,<farthest distance>,<first sensorId with exceeding range>
                try:
                    distanceShort = float(recv.split(",")[1])
                    distanceLong = float(recv.split(",")[2])
                    sensorID = int(recv.split(",")[3])
                except:
                    cartGlobal.log(f"exception with message: '{recv[:-2]}', error in message structure")
                    distanceShort = -1
                    distanceLong = -1
                cartGlobal.setMovementBlocked(True)
                if distanceShort < distanceLong:
                    distance = distanceShort
                else:
                    distance = distanceLong
                gui.controller.updateDistanceSensorObstacle(distance, sensorID)
                gui.controller.updateDistanceSensorAbyss("", -1)


            elif msgID == "!A3":  # "abyss:":
                # !A2,<farthest distance>,<farthest distance>,<first sensorId with exceeding range>
                try:
                    distanceShort = float(recv.split(",")[1])
                    distanceLong = float(recv.split(",")[2])
                    sensorID = int(recv.split(",")[3])
                except:
                    cartGlobal.log(f"exception with message: '{recv[:-2]}', error in message structure")
                    distanceShort = -1
                    distanceLong = -1

                if distanceShort > distanceLong:
                    distance = distanceShort
                else:
                    distance = distanceLong

                cartGlobal.setMovementBlocked(True)
                gui.controller.updateDistanceSensorAbyss(distance, sensorID)
                gui.controller.updateDistanceSensorObstacle("", -1)


            elif msgID == "!A4":  # free path after move blocked
                cartGlobal.setMovementBlocked(False)
                gui.controller.updateDistanceSensorAbyss("", -1)
                gui.controller.updateDistanceSensorObstacle("", -1)


            elif msgID == "!A5":  # "cart stopped":
                # !A5,<orientation>,<distance moved>
                orientation = round(float(recv.split(",")[1]))
                distance = round(float(recv.split(",")[2]))
                direction = int(recv.split(",")[3])

                cartGlobal.setOrientation(orientation)  # delays with A9 occurred
                cartGlobal.calculateNewCartPosition(orientation, distance, direction)
                cartGlobal.setMovementBlocked(False)
                cartGlobal.setCartRotating(False)
                cartGlobal.setCartMoving(False)
                cartGlobal.log("<-A " + recv[:-2])  # stop and reason
                cartGlobal.log("!A5 cart stopped received from Arduino")

                # persist position
                # do this on the cart as cart might be moved locally not through navManager
                cartGlobal.saveCartLocation()


            elif msgID == "!Aa":  # cart position update:
                # !Aa,<orientation>,<distance moved>
                orientation = round(float(recv.split(",")[1]))
                distance = round(float(recv.split(",")[2]))
                direction = int(recv.split(",")[3])
                cartGlobal.setOrientation(orientation)
                if distance > 0:
                    cartGlobal.calculateNewCartPosition(orientation, distance, direction)
                cartGlobal.log(f"!Aa, distance: {distance}")


            elif msgID == "!A6":  # cart docked
                # Arduino triggers the relais for loading batteries through the docking contacts
                # gui update by heartbeat logic
                cartGlobal.log("cart docked")
                if cartGlobal.navManager is not None:
                    cartGlobal.navManager.root.cartDocked(True)


            elif msgID == "!A7":  # cart docked
                # Arduino releases the relais for loading batteries through the docking contacts
                cartGlobal.log("cart undocked")
                if cartGlobal.navManager is not None:
                    cartGlobal.navManager.root.cartDocked(False)


            elif msgID == "!A8":  # 12 V supply, measured value
                cartGlobal.log(f"{recv[:-2]} received")
                try:
                    newVoltage12V = round(float(recv.split(",")[1]))
                except:
                    cartGlobal.log("!A8 not able to retrieve voltage value {recv[:-2]}")
                    newVoltage12V = cartGlobal.getVoltage12V()

                # test for change in Voltage
                if abs(newVoltage12V - cartGlobal.getVoltage12V()) > 500:
                    cartGlobal.setVoltage12V(newVoltage12V)
                    cartGlobal.log(f"new 12V voltage read [mV]: {newVoltage12V}")


            elif msgID == "!A9":  # orientation x:":
                # !A9,<orientation>
                try:
                    newOrientation = round(float(recv.split(",")[1]))
                except:
                    cartGlobal.log("!A9 not able to retrieve orientation {recv[:-2]}")
                    newOrientation = cartGlobal.getOrientation()

                # test for change of orientation
                if cartGlobal.getOrientation() != newOrientation:
                    cartGlobal.setOrientation(newOrientation)
                    # cartGlobal.log(f"new cart orientation: {newOrientation}")

                    # if cart is in rotation no blocking exists
                    if cartGlobal.getMovementBlocked()[0]:
                        cartGlobal.setMovementBlocked(False)

                # no new orientation, check for blocked movement and stop cart
                else:
                    blocked, startTime = cartGlobal.getMovementBlocked()
                    if blocked:
                        if time.time() - startTime > 3:
                            if cartGlobal.isCartMoving():
                                # cartGlobal.log("cart stopped by MovementBlocked")
                                sendStopCommand("blocked movement")
                                cartGlobal.setMovementBlocked(False)

            else:
                try:
                    cartGlobal.log("<-A " + recv[:-2])
                except:
                    cartGlobal.log(f"Unexpected error on reading messages: {sys.exc_info()[0]}")

            # cartGlobal.log("msg processed")
        time.sleep(0.001)  # give other threads a chance


########################################################################
########################################################################
def sendConfigValues():
    msg = 'a' + str(cartGlobal.SHORT_RANGE_MIN).zfill(2) + str(cartGlobal.SHORT_RANGE_MAX).zfill(2) \
          + str(cartGlobal.LONG_RANGE_MIN).zfill(2) + str(cartGlobal.LONG_RANGE_MAX).zfill(2) + str(
        cartGlobal.delayBetweenDistanceMeasurements).zfill(2)
    ser.write(bytes(msg + '\n', 'ascii'))

    msg = f"b,{cartGlobal.SPEED_FACTOR:07.4f},{cartGlobal.SPEED_OFFSET:07.4f},{cartGlobal.SPEED_FACTOR_SIDEWAY:05.2f},{cartGlobal.SPEED_FACTOR_DIAGONAL:05.2f}"
    ser.write(bytes(msg + '\n', 'ascii'))

    # cartGlobal.log(f"{msg}")


def sendMoveCommand(direction, speed, distanceMm):
    '''
    send move command to cart
    distance calculation is based on time travelled and requested speed
    if final orientation <> initial orientation send a rotate command at the end to end up in same orientation
    '''
    # conmmand 1,<dir>,<speed>,<distance>,<max duration>
    # e.g. forward, speed=180, distance=100 mm, max duration=5 s: 1,0,180,0100,0005
    distanceMmLimited = min(distanceMm, 2500)  # limit distance for single command
    cartGlobal.setMoveDirection(direction)

    cartGlobal.setCartMoveDistance(distanceMmLimited)
    cartGlobal.setRequestedCartSpeed(speed)
    moveDuration = ((distanceMmLimited / speed) * 800) + 1500
    if direction in [cartGlobal.Direction.LEFT.value, cartGlobal.Direction.RIGHT.value]:
        moveDuration = int(moveDuration / cartGlobal.SPEED_FACTOR_SIDEWAY)
    if direction in [cartGlobal.Direction.BACK_DIAG_LEFT, cartGlobal.Direction.BACK_DIAG_RIGHT,
                     cartGlobal.Direction.FOR_DIAG_LEFT, cartGlobal.Direction.FOR_DIAG_RIGHT]:
        moveDuration = int(moveDuration / cartGlobal.SPEED_FACTOR_DIAGONAL)
    durationLimited = min(moveDuration, 9999)  # do not move more than 10 seconds

    cartGlobal.setMovementBlocked(False)
    cartGlobal.setCartMoving(True)

    moveMsg = f"1,{direction},{speed:03.0f},{distanceMmLimited:04.0f},{durationLimited:04.0f}"
    ser.write(bytes(moveMsg + "\n", 'ascii'))
    cartGlobal.log(
        f"Send move {moveMsg}, speed: {speed:.0f}, distance: {distanceMmLimited:.0f}, duration: {durationLimited:.0f}")


def sendRotateCommand(relAngle, speed):
    ROTATION_SPEED = 200

    cartGlobal.setMovementBlocked(False)
    cartGlobal.setTargetOrientation(relAngle)
    cartGlobal.setRequestedCartSpeed(speed)

    # command 2
    # msg = f"b,{cartGlobal.SPEED_FACTOR:07.4f},{cartGlobal.SPEED_EXPONENT:07.4f},{cartGlobal.SPEED_FACTOR_SIDEWAY:05.2f}"
    # ser.write(bytes(msg + '\n', 'ascii'))

    if relAngle < 0:  # rotate counterclock
        msg = f"2,{abs(relAngle):03.0f},{speed:03.0f}"
        cartGlobal.log(f"Send rotate counterclock {msg}")
        ser.write(bytes(msg + '\n', 'ascii'))

    # command 3
    if relAngle > 0:
        msg = f"3,{abs(relAngle):03.0f},{speed:.03f}"
        cartGlobal.log(f"Send rotate clockwise {msg}")
        ser.write(bytes(msg + '\n', 'ascii'))


def sendStopCommand(reason):
    # command 4
    msg = b'4' + b'\n'
    cartGlobal.log(f"Send stop, reason: {reason}")
    ser.write(msg)
    cartGlobal.setCartMoving(False)


def sendSpeedCommand(speed):
    # command 6
    msg = bytes(str(6000 + speed) + '\n', 'ascii')
    # cartGlobal.log("Send speed "+str(msg))
    ser.write(msg)
    cartGlobal.setRequestedCartSpeed(speed)


def sendReadSensorCommand(sensorID):
    msg = b'7' + bytes(str(sensorID), 'ascii') + b'\n'
    cartGlobal.log("Send readSensor " + str(msg))
    ser.write(msg)


def sendHeartbeat():
    try:
        msg = b'9' + b'\n'
        ser.write(msg)
    except portNotOpenError as e:
        pass


def getCartOrientation():
    msg = b'5' + b'\n'
    # cartGlobal.log("get Orientation " + msg)
    ser.write(msg)

