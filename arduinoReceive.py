
import os
import sys
import time
import serial
import subprocess
from typing import Tuple
from PyQt5.QtCore import Qt

#import gui
import marvinglobal.marvinglobal as mg
import config
import arduinoSend
import cartControl
#import rpcSend
import distanceSensors
import cart

def initSerial(comPort):

    config.arduinoConnEstablished = False
    reconnectionTried = False

    while True:
        try:
            config.arduino = serial.Serial(comPort)

            config.arduino.setDTR(False)
            time.sleep(0.1)
            config.arduino.setDTR(True)
            config.arduino.baudrate = 115200
            config.arduino.writeTimeout = 0
            config.log("Serial comm to arduino established")
            return

        except Exception as e:
            if reconnectionTried:
                config.log(f"could not reconnect COM5, going down")
                os._exit(0)

            config.log(f"exception on serial connect with {comPort} {e}")
            config.log(f"trying to reattach COM5")
            subprocess.call("c:/utils/devcon.exe enable @USB\VID_2341&PID_0042\95432313338351718151")

            reconnectionTried = True


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

            if not config.arduinoConnEstablished:
                config.arduinoConnEstablished = True        # first message from arduino received
                #config.log("initialize cart sensors ...")
                #arduinoSend.sendConfigValues()

            # cartGlobal.log(f"line read {recv}")
            msgID = recvB[0:3].decode()

            # cartGlobal.log("in read message: ", msgID)

            if msgID == "!A0":  # "cart ready"
                config.log("!A0 received, cart ready")
                config.log("----------")
                config.cartReady = True
                config.state.cartStatus = mg.CartStatus.READY
                cart.publishState()

            elif msgID == "!A1":  # floor offsets from infrared sensors
                # !A1,<sensorID>,<ANZ_MESSUNGEN_PRO_SCAN>,[ANZ_MESSUNGEN_PRO_SCAN<value>,]
                #þconfig.log(f"{recv}", publish=False)
                messageItems = [int(e) if e.isdigit() else e for e in recv.split(',')]
                distanceSensors.updateFloorOffset(messageItems)


            elif msgID == "!A2":  # "obstacle:":
                # !A2,<t<pe>,<distance>,<limit>,<sensorName>
                sensorID = 0
                try:
                    items = recv.split(",")
                    type = int(items[1])
                    distance= int(items[2])
                    limit= int(items[3])
                    sensorId = items[4]

                except Exception as e:
                    config.log(f"exception with message: '{recv}', error in message structure {e}")
                    continue

                #config.log(f"!A2 obstacle detected, height: {distance} > limit: {limit}, sensor: {sensorId}")

                cart.movement.distanceSensorObstacle = str(distance)
                cart.movement.sensorIdObstacle = sensorId
                cart.publishMovement()


            elif msgID == "!A3":  # "abyss:":
                # !A2,<farthest distance>,<farthest distance>,<first sensorId with exceeding range>
                try:
                    items = recv.split(",")
                    distance = int(items[1])
                    limit = int(items[2])
                    sensorId = items[3]

                except Exception as e:
                    config.log(f"exception with message: '{recv}', error in message structure {e}")
                    continue

                config.log(f"!A3 abyss detected, measured dist: {distance} > limit: {limit}, sensor: {sensorId}")
                cart.movement.distanceSensorAbyss = str(distance)
                cart.movement.sensorIdAbyss = sensorId
                cart.publishMovement()


            elif msgID == "!A4":  # free path after move blocked
                config.log(f"!A4 free path after blocked move")
                cart.movement.distanceSensorObstacle = "-"
                cart.movement.distanceSensorAbyss = "-"
                cart.publishMovement()


            elif msgID == "!A5":  # "rotation stopped":
                # !A5,<currentYaw>,<reason>
                items = recv.split(",")
                yaw = round(float(items[1]))
                stopReason = items[2]
                cart.terminateRotation(stopReason)
                #config.log("<-A " + recv, publish=False)  # stop and reason
                config.log(f"!A5 cart rotation stopped, {stopReason}")


            elif msgID == "!A6":  # movement stopped
                # !A6,<orientation>,<distance moved>,<reason>
                items = recv.split(",")
                yaw = round(float(items[1]))
                distanceMoved = int(items[2])
                stopReason = items[3]

                cart.terminateMove(distanceMoved, yaw, stopReason)


            elif msgID == "!A7":  # measured distances of infrared sensor
                # !A7,<sensorID>,<ANZ_MESSUNGEN_PRO_SCAN>,[ANZ_MESSUNGEN_PRO_SCAN<value>,]
                config.log(f"{recv}", publish=False)
                messageItems = [int(e) if e.isdigit() else e for e in recv.split(',')]
                distanceSensors.updateSensorTestData(messageItems)


            elif msgID == "!A8":  # 12 V supply, measured value
                config.log(f"{recv} received")
                newVoltage12V = 0
                try:
                    newVoltage12V = round(float(recv.split(",")[1]))
                except Exception as e:
                    config.log("!A8 not able to retrieve voltage value {recv}, {e}")

                # test for significant change in Voltage
                if abs(config.state.Voltage12V - newVoltage12V) > 500:
                    config.state.Voltage12V = newVoltage12V
                    config.log(f"new 12V voltage read [mV]: {newVoltage12V}")
                    updStmt:Tuple[int,mg.State] = (mg.SharedDataUpdate.CART_STATE, config.state)
                    config.share.updateSharedData(updStmt)


            elif msgID == "!A9":  # orientation x:":
                config.log(f"legacy !A9 message received, should be !Ah or !Ai")

                """
                # !A9,<orientation>
                try:
                    items = recv.split(",")
                    newYaw = round(float(items[1]))
                    newRoll = float(items[2])
                    newPitch = float(items[3])
                except Exception as e:
                    config.log("!A9 not able to retrieve imu values {recv}, {e}")
                    newOrientation = config.location.yaw

                # test for change of orientation
                if config.platformImu.yaw != newYaw:
                    config.platformImu.yaw = newYaw
                    config.location.yaw = newYaw + config.platformImu.yawCorrection
                    # cartGlobal.log(f"new cart orientation: {newOrientation}")

                if config.platformImu.roll != newRoll:
                    config.platformImu.roll = newRoll
                    config.log(f"new roll value: {config.platformImu.roll}")

                if config.platformImu.pitch != newPitch:
                    config.platformImu.pitch = newPitch
                    config.log(f"new pitch value: {config.platformImu.pitch}")

                # no new orientation, check for blocked movement and stop cart
                else:
                    if config.movement.blocked:
                        if time.time() - config.movement.blockedStartTime() > 3:
                            if config.state.cartMoving:
                                # cartGlobal.log("cart stopped by MovementBlocked")
                                arduinoSend.sendStopCommand("blocked movement")
                """

            elif msgID == "!Aa":  # cart position update:
                # !Aa, <currentYaw>, <distance moved in mm>
                items = recv.split(",")
                cart.location.yaw = round(float(items[1]))
                cart.updateCartLocation(round(float(items[2])))

            elif msgID == "!Ab":  # cart rotation update:
                # !Aa, <currentYaw>
                items = recv.split(",")
                cart.location.yaw = round(float(items[1]))
                cart.movement.updateCartRotation()


            elif msgID == "!Ad":  # cart docked
                # Arduino triggers the relais for loading batteries through the docking contacts
                # gui update by heartbeat logic
                config.log("cart docked")
                config.state.cartDocked = True
                cart.publishState()


            elif msgID == "!Ae":  # cart undocked
                # Arduino releases the relais for loading batteries through the docking contacts
                config.log("cart undocked")
                config.state.cartDocked = False
                cart.publishState()


            elif msgID == "!Af":  # distances from ultrasonic sensors
                # The sensors see only obstacles, 0 means no echo received or obstacles too far away
                config.log("ultrasonic sensor results")
                messageItems = [int(e) if e.isdigit() else e for e in recv.split(',')]
                distanceSensors.updateFreeFrontRoom(messageItems)


            elif msgID == "!Ah":  # bno055 platform sensor update:
                # !Ab,<yaw>,<roll>, <pitch>
                items = recv.split(",")
                config.platformImu.yaw = 360 - round(float(items[1]) / 1000.0)
                config.platformImu.roll = float(items[2]) / 1000.0
                config.platformImu.pitch = float(items[3]) / 1000.0
                config.platformImu.updateTime = time.time()
                cart.publishPlatformImu()

                config.log(f"!Ah, platformImu yaw: {config.platformImu.yaw}, roll: {config.platformImu.roll}, pitch: {config.platformImu.pitch}", publish=False)
                config.location.yaw = (config.platformImu.yaw + config.platformImu.yawCorrection) % 360
                cart.publishState()

            elif msgID == "!Ai":  # bno055 head sensor update:
                # !Ab,<milliYaw>,<milliRoll>, <milliPitch>
                items = recv.split(",")
                config.headImu.yaw = round(float(items[1]) / 1000.0)
                config.headImu.roll = float(items[2]) / 1000.0
                config.headImu.pitch = float(items[3]) / 1000.0
                config.headImu.updateTime = time.time()
                cart.publishHeadImu()
                config.log(f"!Ai, headImu, yaw: {config.headImu.yaw}, roll: {config.headImu.roll}, pitch: {config.headImu.pitch}", publish=False)


            else:
                try:
                    config.log("<-A " + recv, publish=False)
                except:
                    config.log(f"Unexpected error on reading messages: {sys.exc_info()[0]}")

            # cartGlobal.log("msg processed")

        # monitor move timeout
        if config.state.cartMoving:
            if time.time() > config.movement.moveStartTime + config.movement.maxDuration:
                arduinoSend.sendStopCommand("timeout in move")

        # check for docked and batteries loaded
        if config.state.cartDocked:
            # check battery status of laptop
            if time.time() - cartControl.getLastBatteryCheckTime() > 5:
                cartControl.updateBatteryStatus()
                if config._batteryStatus.percent == 100:

                    # undock the cart
                    cart.initiateMove(mg.MoveDirection.BACKWARD, 100, 200)

                    # and go to sleep
                    os.system("rundll32.exe powrprof.dll,SetSuspendState 0,1,0")

        time.sleep(0.001)  # give other threads a chance
