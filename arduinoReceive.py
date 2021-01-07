
import os
import sys
import time
import serial
import subprocess
from typing import Tuple, List
import numpy as np

import marvinglobal.marvinglobal as mg
import marvinglobal.cartClasses as cartCls
import config
import arduinoSend
import cartControl
import distanceSensors
import cart

def initSerial(comPort):

    config.arduinoConnEstablished = False

    try:
        config.arduino = serial.Serial(comPort)

        config.arduino.setDTR(False)    # setting DTR should reset the arduino
        time.sleep(0.3)
        config.arduino.setDTR(True)
        time.sleep(0.3)
        config.arduino.baudrate = 115200
        config.arduino.writeTimeout = 0
        config.log("Serial comm to arduino established")
        return True

    except Exception as e:
        config.log(f"could not connect with COM5, going down")
        return False

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
            config.log(f"exception in arduinoReceive, readMessages: {e}")
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
                config.stateLocal.cartStatus = mg.CartStatus.READY
                cart.publishState()


            elif msgID == "!F0":
                # send sensor results to cartGui
                config.log(f"!F0: signal cartgui to update sensor data {recv}")
                config.share.cartGuiUpdateQueue.put({'msgType': mg.SharedDataItem.FLOOR_OFFSET})


            elif msgID == "!F1":  # floor offsets from infrared sensors
                # !F1,[<sensorId>,<step>,<obstacleHeight>,<abyssDepth] * numInvolvedSensors
                config.log(f"{recv=}, FLOOR_OFFSET", publish=False)
                distanceSensors.updateFloorOffset(recv)
                if distanceSensors.sensorInTest is not None:
                    config.share.cartGuiUpdateQueue.put({'msgType': mg.SharedDataItem.FLOOR_OFFSET})


            elif msgID == "!F2":  # reference distances of infrared sensor stored in arduino eeprom
                # !Ac,<sensorID>, value, * NUM_SCAN_STEPS
                config.log(f"irSensor reference distance: {recv}", publish=False)
                messageItems = [int(e) if e.isdigit() else e for e in recv.split(',')]

                sensorId = int(messageItems[1])
                distanceValues = [int(i) for i in messageItems[3:-1]]

                updStmt: Tuple[mg.SharedDataItem, int, List[int]] = (mg.SharedDataItem.IR_SENSOR_REFERENCE_DISTANCE, sensorId, distanceValues)
                config.share.updateSharedData(updStmt)


            elif msgID == "!F3":  # measured distances of infrared sensor
                # !F3,<sensorID>,<ANZ_MESSUNGEN_PRO_SCAN>,[ANZ_MESSUNGEN_PRO_SCAN<value>,]
                config.log(f"F3: irSensor distance: {recv}", publish=False)
                messageItems = [int(e) if e.isdigit() else e for e in recv.split(',')]

                sensorId = messageItems[1]
                distanceValues = [int(i) for i in messageItems[3:]]

                config.share.cartGuiUpdateQueue.put(
                {'msgType': mg.CartGuiUpdateRequest.SENSOR_TEST_DATA, 'sensorId': sensorId, 'distances': distanceValues})


            elif msgID == "!F4":  # distances from ultrasonic sensors
                # The sensors see only obstacles, 0 means no echo received or obstacles too far away
                config.log(f"ultrasonic sensor results: {recv}")
                messageItems = [int(e) if e.isdigit() else e for e in recv.split(',')]
                distanceSensors.updateFreeFrontRoom(messageItems)


            elif msgID == "!S0":  # free path after move blocked
                config.log(f"!S0 free path after blocked move")
                config.movementLocal.distanceSensorObstacle = 0
                config.movementLocal.distanceSensorAbyss = 0
                config.movementLocal.blocked = False
                config.movementLocal.blockedStartTime = 0
                cart.publishMovement()


            elif msgID == "!S1":  # "obstacle on floor:":
                # !S1,<height>,<limit>,<sensorId>
                sensorID = 0
                try:
                    items = recv.split(",")
                    height = int(items[1])
                    limit= int(items[2])
                    sensorId = int(items[3])

                except Exception as e:
                    config.log(f"exception with message: '{recv}', error in message structure {e}")
                    continue

                config.log(f"!S1 obstacle detected by infrared sensor, height: {height} > limit: {limit}, sensor: {config.getIrSensorName(sensorId)}")
                config.movementLocal.obstacleFromInfraredDistanceSensor = height
                config.movementLocal.sensorIdObstacle = sensorId

                config.movementLocal.blocked = True
                config.movementLocal.blockedStartTime = time.time()
                config.movementLocal.reasonStopped = "obstacle detected by ir sensor"

                cart.publishMovement()


            elif msgID == "!S2":  # "abyss:":
                # !A2,<farthest distance>,<farthest distance>,<first sensorId with exceeding range>
                try:
                    items = recv.split(",")
                    depth = int(items[1])
                    limit = int(items[2])
                    sensorId = int(items[3])

                except Exception as e:
                    config.log(f"exception with message: '{recv}', error in message structure {e}")
                    continue

                config.log(f"!S3 abyss detected, measured depth: {depth} > limit: {limit}, sensor: {config.getIrSensorName(sensorId)}")
                config.movementLocal.abyssFromInfraredDistanceSensor = depth
                config.movementLocal.sensorIdAbyss = sensorId

                config.movementLocal.blocked = True
                config.movementLocal.blockedStartTime = time.time()
                config.movementLocal.reasonStopped = "abyss detected by ir sensor"

                cart.publishMovement()


            elif msgID == "!S3":  # obstacle in front (ultrasonic sensors)
                # !S3,<distance>,<limit>,<sensorId>
                sensorID = 0
                try:
                    items = recv.split(",")
                    distance = int(items[1])
                    limit = int(items[2])
                    sensorId = int(items[3])

                except Exception as e:
                    config.log(f"exception with message: '{recv}', error in message structure {e}")
                    continue

                config.log(
                    f"!S3 obstacle detected by ultrasonic sensor, distance: {distance} > limit: {limit}, sensor: {config.getUsSensorName(sensorId)}")
                config.movementLocal.obstacleFromInfraredDistanceSensor = distance
                config.movementLocal.sensorIdObstacle = sensorId

                config.movementLocal.blocked = True
                config.movementLocal.blockedStartTime = time.time()
                config.movementLocal.reasonStopped = "obstacle detected by us sensor"

                cart.publishMovement()


            elif msgID == "!S4":  # "rotation stopped":
                # !A5,<currentYaw>,<reason>
                items = recv.split(",")
                yaw = round(float(items[1]))
                stopReason = items[2]
                cart.terminateRotation(stopReason)
                #config.log("<-A " + recv, publish=False)  # stop and reason
                config.log(f"!S3 cart rotation stopped, {stopReason}")


            elif msgID == "!S5":  # movement stopped
                # !A6,<orientation>,<distance moved>,<reason>
                items = recv.split(",")
                yaw = round(float(items[1]))
                distanceMoved = int(items[2])
                stopReason = items[3]
                cart.terminateMove(distanceMoved, yaw, stopReason)


            elif msgID == "!B0":  # 12 V supply, measured value
                config.log(f"{recv} received")
                newVoltage12V = 0
                try:
                    newVoltage12V = round(float(recv.split(",")[1]))
                except Exception as e:
                    config.log("!B0 not able to retrieve voltage value {recv}, {e}")

                # test for significant change in Voltage
                if abs(config.stateLocal.Voltage12V - newVoltage12V) > 500:
                    config.stateLocal.Voltage12V = newVoltage12V
                    config.log(f"!B0, new 12V voltage read [mV]: {newVoltage12V}")
                    updStmt:Tuple[int,cartCls.State] = (mg.SharedDataItem.CART_STATE, config.stateLocal)
                    config.share.updateSharedData(updStmt)


            elif msgID == "!P1": #"!P1":  # cart position update:
                # !P1, <currentYaw>, <distance moved in mm>,
                items = recv.split(",")
                yaw = round(float(items[1]))
                distance = round(float(items[2]))
                config.log(f"!P1 cart location update, yaw: {yaw}, distance: {distance}")
                cart.updateCartLocation(yaw, distance)


            elif msgID == "!P2":  #"!Ab":  # cart rotation update:
                # !Aa, <currentYaw>
                items = recv.split(",")
                yaw = round(float(items[1]))
                config.log(f"!P2 cart rotation update, yaw: {yaw}")
                cart.updateCartLocation(yaw, distance=0)


            elif msgID == "!D1":  #"!Ad":  # cart docked
                # Arduino triggers the relais for loading batteries through the docking contacts
                # gui update by heartbeat logic
                config.log("cart docked")
                config.stateLocal.cartDocked = True
                cart.publishState()


            elif msgID == "!D2":  #"!Ae":  # cart undocked
                # Arduino releases the relais for loading batteries through the docking contacts
                config.log("cart undocked")
                config.stateLocal.cartDocked = False
                cart.publishState()


            elif msgID == "!I1":  # bno055 platform sensor update:
                # !Ab,<yaw>,<roll>, <pitch>
                items = recv.split(",")
                config.platformImuLocal.yaw = 360 - round(float(items[1]) / 1000.0)
                config.platformImuLocal.roll = float(items[2]) / 1000.0
                config.platformImuLocal.pitch = float(items[3]) / 1000.0
                config.platformImuLocal.updateTime = time.time()
                cart.publishPlatformImu()

                config.log(f"!I1, platformImu yaw: {config.platformImuLocal.yaw}, roll: {config.platformImuLocal.roll}, pitch: {config.platformImuLocal.pitch}", publish=False)
                config.locationLocal.yaw = (config.platformImuLocal.yaw + config.platformImuLocal.yawCorrection) % 360
                cart.publishLocation()

            elif msgID == "!I2":  # bno055 head sensor update:
                # !Ab,<milliYaw>,<milliRoll>, <milliPitch>
                items = recv.split(",")
                config.headImuLocal.yaw = round(float(items[1]) / 1000.0)
                config.headImuLocal.roll = float(items[2]) / 1000.0
                config.headImuLocal.pitch = float(items[3]) / 1000.0
                config.headImuLocal.updateTime = time.time()
                cart.publishHeadImu()
                config.log(f"!I2, headImu, yaw: {config.headImuLocal.yaw}, roll: {config.headImuLocal.roll}, pitch: {config.headImuLocal.pitch}", publish=False)


            else:
                try:
                    config.log("<-A " + recv, publish=False)
                except:
                    config.log(f"Unexpected error on reading messages: {sys.exc_info()[0]}")

            # cartGlobal.log("msg processed")

        # monitor move timeout
        if config.stateLocal.cartMoving:
            if time.time() > config.movementLocal.moveStartTime + config.movementLocal.maxDuration:
                arduinoSend.sendStopCommand("timeout in move")

        # check for docked and batteries loaded
        if config.stateLocal.cartDocked:
            # check battery status of laptop
            if time.time() - cartControl.getLastBatteryCheckTime() > 5:
                cartControl.updateBatteryStatus()
                if config._batteryStatus.percent == 100:

                    # undock the cart
                    cart.initiateMove(mg.MoveDirection.BACKWARD, 100, 200)

                    # and go to sleep
                    os.system("rundll32.exe powrprof.dll,SetSuspendState 0,1,0")

        time.sleep(0.001)  # give other threads a chance
