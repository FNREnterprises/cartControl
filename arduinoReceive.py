
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
                recv = recvB.decode()[:-2]  # without cr/lf
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
                config.log("")
                config.cartReady = True


            elif msgID == "!A1":  # distance values from obstacle sensors
                # !A1,<sensorID>,<ANZ_MESSUNGEN_PRO_SCAN>,[ANZ_MESSUNGEN_PRO_SCAN<value>,]
                config.log(f"{recv}", publish=False)
                messageItems = [int(e) if e.isdigit() else e for e in recv.split(',')]
                cartControl.updateDistances(messageItems)


            elif msgID == "!A2":  # "obstacle:":
                # !A2,<distance>,<limit>,<sensorName>
                sensorID = 0
                try:
                    items = recv.split(",")
                    distance= int(items[1])
                    limit= int(items[2])
                    sensorName = items[3]

                except Exception as e:
                    config.log(f"exception with message: '{recv}', error in message structure {e}")
                    continue

                config.log(f"!A2 obstacle detected, height: {distance} > limit: {limit}, sensor: {sensorName}", publish=cartControl.getMovementBlocked())
                cartControl.setMovementBlocked(True)

                gui.controller.updateDistanceSensorObstacle(distance, sensorName)
                gui.controller.updateDistanceSensorAbyss("", -1)


            elif msgID == "!A3":  # "abyss:":
                # !A2,<farthest distance>,<farthest distance>,<first sensorId with exceeding range>
                try:
                    items = recv.split(",")
                    distance = int(items[1])
                    limit = int(items[2])
                    sensorName = items[3]
                except Exception as e:
                    config.log(f"exception with message: '{recv}', error in message structure {e}")
                    continue

                config.log(f"!A3 abyss detected, measured dist: {distance} > limit: {limit}, sensor: {sensorName}", publish=cartControl.getMovementBlocked())

                cartControl.setMovementBlocked(True)

                gui.controller.updateDistanceSensorAbyss(distance, sensorName)
                gui.controller.updateDistanceSensorObstacle("", -1)


            elif msgID == "!A4":  # free path after move blocked
                config.log(f"!A4 free path after blocked move")
                cartControl.setMovementBlocked(False)
                gui.controller.updateDistanceSensorAbyss("", -1)
                gui.controller.updateDistanceSensorObstacle("", -1)


            elif msgID == "!A5":  # "cart stopped":
                # !A5,<orientation>,<distance moved>,<moveDirection>,<reason>
                items = recv.split(",")
                config.platformImuYaw = round(float(items[1]))
                distanceMoved = int(items[2])
                moveDirection = config.Direction(int(items[3]))
                reason = items[4]

                cartControl.setMovementBlocked(False)
                cartControl.setCartRotating(False)
                cartControl.setCartMoving(False)

                if moveDirection not in [config.Direction.ROTATE_LEFT, config.Direction.ROTATE_RIGHT]:
                    magnitude = distanceMoved
                else:
                    magnitude = abs(config.signedAngleDifference(config.rotateStartDegrees, cartControl.getCartYaw()))
                cartControl.updateCartLocation(magnitude, moveDirection, final=True)

                #config.log("<-A " + recv, publish=False)  # stop and reason
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
                #config.log(f"!Aa, cartUpdate: {recv}", publish=False)
                items = recv.split(",")
                config.platformImuYaw = round(float(items[1]))
                distanceMoved = round(float(items[2]))
                moveDirection = config.Direction(int(items[3]))

                if moveDirection not in [config.Direction.ROTATE_LEFT, config.Direction.ROTATE_RIGHT]:
                    magnitude = distanceMoved
                else:
                    magnitude = abs(config.signedAngleDifference(config.rotateStartDegrees, cartControl.getCartYaw()))

                cartControl.updateCartLocation(magnitude, moveDirection, final=False)


            elif msgID == "!Ab":  # bno055 platform sensor update:
                # !Ab,<yaw>,<roll>, <pitch>
                items = recv.split(",")
                config.platformImuYaw = 360 - round(float(items[1]) / 1000.0)
                config.platformImuRoll = float(items[2]) / 1000.0
                config.platformImuPitch = float(items[3]) / 1000.0
                config.log(f"!Ab, platformImu yaw: {config.platformImuYaw}, roll: {config.platformImuRoll}, pitch: {config.platformImuPitch}", publish=False)

            elif msgID == "!Ac":  # bno055 head sensor update:
                # !Ab,<milliYaw>,<milliRoll>, <milliPitch>
                items = recv.split(",")
                config.headImuYaw = round(float(items[1]) / 1000.0)
                config.headImuRoll = float(items[2]) / 1000.0
                config.headImuPitch = float(items[3]) / 1000.0
                config.log(f"!Ac, headImu, yaw: {config.headImuYaw}, roll: {config.headImuRoll}, pitch: {config.headImuPitch}", publish=False)

            elif msgID == "!A6":  # cart docked
                # Arduino triggers the relais for loading batteries through the docking contacts
                # gui update by heartbeat logic
                config.log("cart docked")
                config.cartDocked = True
                rpcSend.cartDocked(True)


            elif msgID == "!A7":  # cart undocked
                # Arduino releases the relais for loading batteries through the docking contacts
                config.log("cart undocked")
                config.cartDocked = False
                rpcSend.cartDocked(False)


            elif msgID == "!A8":  # 12 V supply, measured value
                config.log(f"{recv} received")
                try:
                    newVoltage12V = round(float(recv.split(",")[1]))
                except Exception as e:
                    config.log("!A8 not able to retrieve voltage value {recv}, {e}")
                    newVoltage12V = cartControl.getVoltage12V()

                # test for change in Voltage
                if abs(newVoltage12V - cartControl.getVoltage12V()) > 500:
                    cartControl.setVoltage12V(newVoltage12V)
                    config.log(f"new 12V voltage read [mV]: {newVoltage12V}")


            elif msgID == "!A9":  # orientation x:":
                # !A9,<orientation>
                try:
                    items = recv.split(",")
                    newYaw = round(float(items[1]))
                    newRoll = float(items[2])
                    newPitch = float(items[3])
                except Exception as e:
                    config.log("!A9 not able to retrieve imu values {recv}, {e}")
                    newOrientation = cartControl.getCartYaw()

                # test for change of orientation
                if config.platformImuYaw != newYaw:
                    config.platformImuYaw = newYaw
                    # cartGlobal.log(f"new cart orientation: {newOrientation}")

                    # if cart is in rotation no blocking exists
                    if cartControl.getMovementBlocked():
                        cartControl.setMovementBlocked(False)

                if config.platformImuRoll != newRoll:
                    config.platformImuRoll = newRoll
                    config.log(f"new roll value: {config.platformImuRoll}")

                if config.platformImuPitch != newPitch:
                    config.platformImuPitch = newPitch
                    config.log(f"new pitch value: {config.platformImuPitch}")

                # no new orientation, check for blocked movement and stop cart
                else:
                    blocked = cartControl.getMovementBlocked()
                    if blocked:
                        if time.time() - cartControl.getMovementBlockedStartTime() > 3:
                            if config.cartMoving:
                                # cartGlobal.log("cart stopped by MovementBlocked")
                                arduinoSend.sendStopCommand("blocked movement")
                                cartControl.setMovementBlocked(False)

            else:
                try:
                    config.log("<-A " + recv, publish=False)
                except:
                    config.log(f"Unexpected error on reading messages: {sys.exc_info()[0]}")

            # cartGlobal.log("msg processed")

        # monitor move timeout
        if config.cartMoving and time.time() > config._cartMoveTimeout:
            arduinoSend.sendStopCommand("timeout in move")

        # check for docked and batteries loaded
        if config.cartDocked:
            # check battery status of laptop
            if time.time() - cartControl.getLastBatteryCheckTime() > 5:
                cartControl.updateBatteryStatus()
                if config._batteryStatus.percent == 100:

                    # undock the cart
                    arduinoSend.sendMoveCommand(config.Direction.BACKWARD, 100, 200)

                    # and go to sleep
                    arduinoSend.powerKinect(False)
                    os.system("rundll32.exe powrprof.dll,SetSuspendState 0,1,0")

        time.sleep(0.001)  # give other threads a chance
