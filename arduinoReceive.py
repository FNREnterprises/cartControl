import sys
import time
import serial

import gui
import config
import arduinoSend

ser = None

def initSerial(comPort):
    global ser

    config.arduinoStatus = 0

    while True:
        try:
            ser = serial.Serial(comPort)
            # try to reset the arduino
            ser.setDTR(False)
            time.sleep(0.1)
            ser.setDTR(True)
            ser.baudrate = 115200
            ser.writeTimeout = 0
            config.log("Serial comm to arduino established")
            return

        except Exception as e:
            config.log(f"exception on serial connect with {comPort} {e}")
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
            except Exception as e:
                config.log(f"problem with decoding cart msg '{recvB}' {e}")
                continue

            # cartGlobal.log(f"line read {recv}")
            msgID = recvB[0:3].decode()

            # cartGlobal.log("in read message: ", msgID)

            if msgID == "!A0":  # "cart ready"
                config.arduinoStatus = 1
                config.log("cart ready")
                arduinoSend.sendConfigValues()


            elif msgID == "!A1":  # distance values from obstacle sensors
                # !A1,<sensorID>,<ANZ_MESSUNGEN_PRO_SCAN>,[ANZ_MESSUNGEN_PRO_SCAN<value>,]
                messageItems = [int(e) if e.isdigit() else e for e in recv.split(',')]
                cartControl.updateDistances(messageItems[1:])
                # gui.controller.showDistances(messageItems[1])      test, could be responsible for delays in message read?


            elif msgID == "!A2":  # "obstacle:":
                # !A2,<shortest distance>,<farthest distance>,<first sensorId with exceeding range>
                sensorID = 0
                try:
                    distanceShort = float(recv.split(",")[1])
                    distanceLong = float(recv.split(",")[2])
                    sensorID = int(recv.split(",")[3])
                except Exception as e:
                    config.log(f"exception with message: '{recv[:-2]}', error in message structure {e}")
                    distanceShort = -1
                    distanceLong = -1
                config.setMovementBlocked(True)
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
                except Exception as e:
                    config.log(f"exception with message: '{recv[:-2]}', error in message structure {e}")
                    distanceShort = -1
                    distanceLong = -1

                if distanceShort > distanceLong:
                    distance = distanceShort
                else:
                    distance = distanceLong

                config.setMovementBlocked(True)
                gui.controller.updateDistanceSensorAbyss(distance, sensorID)
                gui.controller.updateDistanceSensorObstacle("", -1)


            elif msgID == "!A4":  # free path after move blocked
                config.setMovementBlocked(False)
                gui.controller.updateDistanceSensorAbyss("", -1)
                gui.controller.updateDistanceSensorObstacle("", -1)


            elif msgID == "!A5":  # "cart stopped":
                # !A5,<orientation>,<distance moved>
                orientation = round(float(recv.split(",")[1]))
                distance = round(float(recv.split(",")[2]))
                direction = int(recv.split(",")[3])

                config.setImuOrientation(orientation)  # delays with A9 occurred
                config.updateCartLocation(config.getCartOrientation(), distance, direction)
                config.setMovementBlocked(False)
                config.setCartRotating(False)
                config.setCartMoving(False)
                config.log("<-A " + recv[:-2])  # stop and reason
                config.log("!A5 cart stopped received from Arduino")

                # persist position
                # do this on the cart as cart might be moved locally not through navManager
                config.saveCartLocation()


            elif msgID == "!Aa":  # cart position update:
                # !Aa,<orientation>,<distance moved>, <moveDirection>
                orientation = round(float(recv.split(",")[1]))
                distance = round(float(recv.split(",")[2]))
                moveDirection = int(recv.split(",")[3])
                config.setImuOrientation(orientation)
                config.updateCartLocation(config.getCartOrientation(), distance, moveDirection)
                if config.navManager is not None:
                    locX, locY = config.getCartLocation()
                    config.navManager.root.updateCartInfo(locX, locY, orientation)
                #config.log(f"!Aa, distance: {distance}, orientation: {orientation}")


            elif msgID == "!A6":  # cart docked
                # Arduino triggers the relais for loading batteries through the docking contacts
                # gui update by heartbeat logic
                config.log("cart docked")
                if config.navManager is not None:
                    config.navManager.root.cartDocked(True)


            elif msgID == "!A7":  # cart docked
                # Arduino releases the relais for loading batteries through the docking contacts
                config.log("cart undocked")
                if config.navManager is not None:
                    config.navManager.root.cartDocked(False)


            elif msgID == "!A8":  # 12 V supply, measured value
                config.log(f"{recv[:-2]} received")
                try:
                    newVoltage12V = round(float(recv.split(",")[1]))
                except Exception as e:
                    config.log("!A8 not able to retrieve voltage value {recv[:-2]}, {e}")
                    newVoltage12V = config.getVoltage12V()

                # test for change in Voltage
                if abs(newVoltage12V - config.getVoltage12V()) > 500:
                    config.setVoltage12V(newVoltage12V)
                    config.log(f"new 12V voltage read [mV]: {newVoltage12V}")


            elif msgID == "!A9":  # orientation x:":
                # !A9,<orientation>
                try:
                    newOrientation = round(float(recv.split(",")[1]))
                except Exception as e:
                    config.log("!A9 not able to retrieve orientation {recv[:-2]}, {e}")
                    newOrientation = config.getCartOrientation()

                # test for change of orientation
                if config.getCartOrientation() != newOrientation:
                    config.setImuOrientation(newOrientation)
                    # cartGlobal.log(f"new cart orientation: {newOrientation}")

                    # if cart is in rotation no blocking exists
                    if config.getMovementBlocked()[0]:
                        config.setMovementBlocked(False)

                # no new orientation, check for blocked movement and stop cart
                else:
                    blocked, startTime = config.getMovementBlocked()
                    if blocked:
                        if time.time() - startTime > 3:
                            if config.isCartMoving():
                                # cartGlobal.log("cart stopped by MovementBlocked")
                                arduinoSend.sendStopCommand("blocked movement")
                                config.setMovementBlocked(False)

            else:
                try:
                    config.log("<-A " + recv[:-2])
                except:
                    config.log(f"Unexpected error on reading messages: {sys.exc_info()[0]}")

            # cartGlobal.log("msg processed")

        if config._cartMoving and time.time() > config._cartMoveTimeout:
            arduinoSend.sendStopCommand("timeout in move")

        time.sleep(0.001)  # give other threads a chance
