
import os
import sys
import time
import serial

import gui
import config
import arduinoSend
import cartControl
import rpcSend


def initSerial(comPort):

    config.arduinoStatus = 0

    while True:
        try:
            config.arduino = serial.Serial(comPort)
            # try to reset the arduino
            config.arduino.setDTR(False)
            time.sleep(0.1)
            config.arduino.setDTR(True)
            config.arduino.baudrate = 115200
            config.arduino.writeTimeout = 0
            config.log("Serial comm to arduino established")
            return

        except Exception as e:
            config.log(f"exception on serial connect with {comPort} {e}")
            time.sleep(10)
            os._exit(0)


#####################################
# readMessages runs in its own thread
#####################################
def readMessages():

    while config.arduino is None:
        time.sleep(0.1)

    while config.arduino.is_open:

        try:
            numBytesAvailable = config.arduino.inWaiting()
        except Exception as e:
            config.log(f"exception on arduino.inWaiting: {e}")
            time.sleep(10)
            os._exit(1)

        if numBytesAvailable > 0:
            # cartGlobal.log(f"inWaiting > 0")
            recvB = config.arduino.readline(120)
            try:
                recv = recvB.decode()
            except Exception as e:
                config.log(f"problem with decoding cart msg '{recvB}' {e}")
                continue

            if config.arduinoStatus == 0:
                config.arduinoStatus = 1        # first message from arduino received
                #config.log("initialize cart sensors ...")
                #arduinoSend.sendConfigValues()

            # cartGlobal.log(f"line read {recv}")
            msgID = recvB[0:3].decode()

            # cartGlobal.log("in read message: ", msgID)

            if msgID == "!A0":  # "cart ready"
                config.arduinoStatus = 2                # cart has started up
                config.log("!A0 received, cart ready")
                config.cartReady = True


            elif msgID == "!A1":  # distance values from obstacle sensors
                # !A1,<sensorID>,<ANZ_MESSUNGEN_PRO_SCAN>,[ANZ_MESSUNGEN_PRO_SCAN<value>,]
                messageItems = [int(e) if e.isdigit() else e for e in recv.split(',')]
                cartControl.updateDistances(messageItems[1:])
                #config.log(f"{recv[0:-2]}", publish=False)


            elif msgID == "!A2":  # "obstacle:":
                # !A2,<shortest distance>,<farthest distance>,<first sensorId with exceeding range>
                sensorID = 0
                try:
                    items = recv.split(",")
                    distanceShort = int(items[1])
                    limitShort = int(items[2])
                    distanceLong = int(items[3])
                    limitLong = int(items[4])
                    sensorID = int(recv.split(",")[5])

                except Exception as e:
                    config.log(f"exception with message: '{recv[:-2]}', error in message structure {e}")
                    continue

                cartControl.setMovementBlocked(True)
                if distanceShort < limitShort:
                    config.log(f"!A2 obstacle detected, measured dist: {distanceShort} < limit: {limitShort}, sensor: {config.getSensorName(sensorID)}")
                else:
                    config.log(f"!A2 obstacle detected, measured dist: {distanceLong} < limit: {limitLong}, sensor: {config.getSensorName(sensorID)}")
                gui.controller.updateDistanceSensorObstacle(distanceShort, sensorID)
                gui.controller.updateDistanceSensorAbyss("", -1)


            elif msgID == "!A3":  # "abyss:":
                # !A2,<farthest distance>,<farthest distance>,<first sensorId with exceeding range>
                try:
                    items = recv.split(",")
                    distanceShort = int(items[1])
                    limitShort = int(items[2])
                    distanceLong = int(items[3])
                    limitLong = int(items[4])
                    sensorID = int(recv.split(",")[5])
                except Exception as e:
                    config.log(f"exception with message: '{recv[:-2]}', error in message structure {e}")
                    continue

                cartControl.setMovementBlocked(True)
                if distanceShort > limitShort:
                    config.log(f"!A3 abyss detected, measured dist: {distanceShort} > limit: {limitShort}, sensor: {config.getSensorName(sensorID)}")
                else:
                    config.log(f"!A3 abyss detected, measured dist: {distanceLong} > limit: {limitLong}, sensor: {config.getSensorName(sensorID)}")
                gui.controller.updateDistanceSensorAbyss(distanceLong, sensorID)
                gui.controller.updateDistanceSensorObstacle("", -1)


            elif msgID == "!A4":  # free path after move blocked
                config.log(f"!A4 free path after blocked move")
                cartControl.setMovementBlocked(False)
                gui.controller.updateDistanceSensorAbyss("", -1)
                gui.controller.updateDistanceSensorObstacle("", -1)


            elif msgID == "!A5":  # "cart stopped":
                # !A5,<orientation>,<distance moved>,<moveDirection>,<reason>
                msgParts = recv[:-2].split(",")     # without terminating \n
                config.imuDegrees = round(float(msgParts[1]))
                distanceMoved = int(msgParts[2])
                moveDirection = config.Direction(int(msgParts[3]))
                reason = msgParts[4]

                cartControl.setMovementBlocked(False)
                cartControl.setCartRotating(False)
                cartControl.setCartMoving(False)

                if moveDirection not in [config.Direction.ROTATE_LEFT, config.Direction.ROTATE_RIGHT]:
                    magnitude = distanceMoved
                else:
                    magnitude = abs(config.signedAngleDifference(config.rotateStartDegrees, cartControl.getCartYaw()))
                cartControl.updateCartLocation(magnitude, moveDirection, final=True)

                #config.log("<-A " + recv[:-2], publish=False)  # stop and reason
                config.log(f"!A5 cart stopped, {reason}")

                # if kinect monitoring is enabled try to stop it
                if config.monitoringWithKinect:
                    config.monitoringWithKinect = False
                    try:
                        config.kinectConnection.root.stopMonitoring()
                    except Exception as e:
                        config.log(f"could not request stop monitoring from kinect, {e}")


            elif msgID == "!Aa":  # cart position/orientation update:
                # !Aa,<imuYaw>, <distance moved in mm>, <moveDirection>
                #config.log(f"!Aa, cartUpdate: {recv[:-2]}", publish=False)
                data = recv[:-2].split(",")
                config.imuDegrees = round(float(data[1]))
                distanceMoved = round(float(data[2]))
                moveDirection = config.Direction(int(data[3]))

                if moveDirection not in [config.Direction.ROTATE_LEFT, config.Direction.ROTATE_RIGHT]:
                    magnitude = distanceMoved
                else:
                    magnitude = abs(config.signedAngleDifference(config.rotateStartDegrees, cartControl.getCartYaw()))

                cartControl.updateCartLocation(magnitude, moveDirection, final=False)


            elif msgID == "!Ab":  # bno055 sensor update:
                # !Ab,<yaw>,<roll>, <pitch>
                data = recv.split(",")
                config.imuDegrees = round(float(data[1]))
                config._imuRoll = float(data[2])
                config._imuPitch = float(data[3])
                #config.log(f"!Ab, imuYaw: {config.imuDegrees}, imuRoll: {config._imuRoll}, imuPitch: {config._imuPitch}", publish=False)


            elif msgID == "!A6":  # cart docked
                # Arduino triggers the relais for loading batteries through the docking contacts
                # gui update by heartbeat logic
                config.log("cart docked")
                rpcSend.cartDocked(True)


            elif msgID == "!A7":  # cart undocked
                # Arduino releases the relais for loading batteries through the docking contacts
                config.log("cart undocked")
                rpcSend.cartDocked(False)


            elif msgID == "!A8":  # 12 V supply, measured value
                config.log(f"{recv[:-2]} received")
                try:
                    newVoltage12V = round(float(recv.split(",")[1]))
                except Exception as e:
                    config.log("!A8 not able to retrieve voltage value {recv[:-2]}, {e}")
                    newVoltage12V = cartControl.getVoltage12V()

                # test for change in Voltage
                if abs(newVoltage12V - cartControl.getVoltage12V()) > 500:
                    cartControl.setVoltage12V(newVoltage12V)
                    config.log(f"new 12V voltage read [mV]: {newVoltage12V}")


            elif msgID == "!A9":  # orientation x:":
                # !A9,<orientation>
                try:
                    data = recv.split(",")
                    newYaw = round(float(data[1]))
                    newRoll = float(data[2])
                    newPitch = float(data[3])
                except Exception as e:
                    config.log("!A9 not able to retrieve imu values {recv[:-2]}, {e}")
                    newOrientation = cartControl.getCartYaw()

                # test for change of orientation
                if config.imuDegrees != newYaw:
                    config.imuDegrees = newYaw
                    # cartGlobal.log(f"new cart orientation: {newOrientation}")

                    # if cart is in rotation no blocking exists
                    if cartControl.getMovementBlocked()[0]:
                        cartControl.setMovementBlocked(False)

                if config._imuRoll != newRoll:
                    config._imuRoll = newRoll
                    config.log(f"new roll value: {config._imuRoll}")

                if config._imuPitch != newPitch:
                    config._imuPitch = newPitch
                    config.log(f"new pitch value: {config._imuPitch}")

                # no new orientation, check for blocked movement and stop cart
                else:
                    blocked, startTime = cartControl.getMovementBlocked()
                    if blocked:
                        if time.time() - startTime > 3:
                            if config.cartMoving:
                                # cartGlobal.log("cart stopped by MovementBlocked")
                                arduinoSend.sendStopCommand("blocked movement")
                                cartControl.setMovementBlocked(False)

            else:
                try:
                    config.log("<-A " + recv[:-2], publish=False)
                except:
                    config.log(f"Unexpected error on reading messages: {sys.exc_info()[0]}")

            # cartGlobal.log("msg processed")

        if config.cartMoving and time.time() > config._cartMoveTimeout:
            arduinoSend.sendStopCommand("timeout in move")

        time.sleep(0.001)  # give other threads a chance
