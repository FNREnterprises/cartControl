
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


#####################################
# readMessages runs in its own thread
#####################################
def readMessages():

    timeout = time.time() + 5
    while config.arduino is None and time.time() < timeout:
        time.sleep(0.1)

    while config.arduino.is_open:

        numBytesAvailable = 0
        try:
            numBytesAvailable = config.arduino.inWaiting()
        except Exception as e:
            config.log(f"exception in arduinoReceive, readMessages: {e}")
            time.sleep(10)
            os._exit(1)

        if numBytesAvailable > 0:
            # cartGlobal.log(f"inWaiting > 0")
            recvB = config.arduino.readline()
            try:
                recv = recvB.decode()[:-2]  # without cr/lf
            except Exception as e:
                config.log(f"problem with decoding cart msg '{recvB}' {e}")
                continue

            if not config.arduinoConnEstablished:
                config.log(f"message received from arduino: {recv}")
                if "cartControlArduino" in recv:
                    config.arduinoConnEstablished = True        # first message from arduino received
                else:
                    config.log(f"not connected with cartControlArduino, going down")
                    os._exit(1)


            # cartGlobal.log(f"line read {recv}")
            msgID = recvB[0:3].decode()

            # cartGlobal.log("in read message: ", msgID)

            if msgID == "!A0":  # "cart ready"
                config.log("!A0 received, cart ready")
                config.log("----------")
                config.cartReady = True
                config.stateMaster.cartStatus = mg.CartStatus.READY
                cart.publishState()


            elif msgID == "!F0":
                # send request to update cartGui with FLOOR_OFFSET
                config.log(f"!F0: signal cartgui to update sensor data {recv}")
                config.marvinShares.cartGuiUpdateQueue.put({'msgType': mg.SharedDataItems.FLOOR_OFFSET})


            elif msgID == "!F1":  # floor offsets from infrared sensors (distance - reference)
                # !F1,[<irSensorId>,<step>,<obstacleHeight>,<abyssDepth] * numInvolvedSensors
                config.log(f"{recv=}, FLOOR_OFFSET", publish=False)
                distanceSensors.updateFloorOffset(recv)
                #if distanceSensors.sensorInTest is not None:
                config.marvinShares.cartGuiUpdateQueue.put({'msgType': mg.SharedDataItems.FLOOR_OFFSET})


            elif msgID == "!F2":  # reference distances of infrared sensor stored in arduino eeprom
                # !Ac,<sensorID>, value, * NUM_SCAN_STEPS
                config.log(f"irSensor reference distance: {recv}", publish=False)
                messageItems = [int(e) if e.isdigit() else e for e in recv.split(',')]

                irSensorId = int(messageItems[1])
                distanceValues = [int(i) for i in messageItems[3:-1]]

                #updStmt: Tuple[mg.SharedDataItems, int, List[int]] = (mg.SharedDataItems.IR_SENSOR_REFERENCE_DISTANCE, irSensorId, distanceValues)
                if irSensorId < mg.NUM_SWIPING_IR_DISTANCE_SENSORS:
                    config.swipingIrSensorsMaster[irSensorId].referenceDistances = distanceValues
                    updStmt = {'msgType': mg.SharedDataItems.SWIPING_IR_SENSORS, 'sender': config.processName,
                           'info': {'irSensorId': irSensorId, 'data': config.swipingIrSensorsMaster[irSensorId].__dict__}}
                    config.marvinShares.updateSharedData(updStmt)

                if irSensorId >= mg.NUM_SWIPING_IR_DISTANCE_SENSORS:
                    config.staticIrSensorsMaster[irSensorId].referenceDistance = distanceValues[0]
                    updStmt = {'msgType': mg.SharedDataItems.STATIC_IR_SENSORS, 'sender': config.processName,
                           'info': {'irSensorId': irSensorId, 'data': config.staticIrSensorsMaster[irSensorId].__dict__}}
                    config.marvinShares.updateSharedData(updStmt)


            elif msgID == "!F3":  # measured distances of infrared sensor (independent of reference value)
                # !F3,<sensorID>,<ANZ_MESSUNGEN_PRO_SCAN>,[ANZ_MESSUNGEN_PRO_SCAN<value>,]
                config.log(f"F3: irSensor distance: {recv}", publish=False)
                messageItems = [int(e) if e.isdigit() else e for e in recv.split(',')]

                irSensorId = messageItems[1]
                distanceValues = [int(i) for i in messageItems[3:]]

                if irSensorId < mg.NUM_SWIPING_IR_DISTANCE_SENSORS:
                    config.swipingIrSensorsMaster[irSensorId].measuredDistances = distanceValues
                    updStmt = {'msgType': mg.SharedDataItems.SWIPING_IR_SENSORS, 'sender': config.processName,
                           'info': {'irSensorId': irSensorId, 'data': config.swipingIrSensorsMaster[irSensorId].__dict__}}
                    config.marvinShares.updateSharedData(updStmt)

                if irSensorId >= mg.NUM_SWIPING_IR_DISTANCE_SENSORS:
                    config.staticIrSensorsMaster[irSensorId].measuredDistance = distanceValues[0]
                    updStmt = {'msgType': mg.SharedDataItems.STATIC_IR_SENSORS, 'sender': config.processName,
                           'info': {'irSensorId': irSensorId, 'data': config.staticIrSensorsMaster[irSensorId].__dict__}}
                    config.marvinShares.updateSharedData(updStmt)


            elif msgID == "!F4":  # distances from ultrasonic sensors
                # The sensors see only obstacles, 0 means no echo received or obstacles too far away
                config.log(f"ultrasonic sensor results: {recv}")
                messageItems = [int(e) if e.isdigit() else e for e in recv.split(',')]

                """
                the cart has 4 ultrasonic sensors in the front
                these sensors can only detect distances to obstacles, so without obstacles we get no distance
                :param messageItems:
                :return:
                """
                for usSensorId in range(mg.NUM_US_DISTANCE_SENSORS):
                    config.usSensorsMaster[usSensorId].offsetDistance = messageItems[usSensorId]

                    updStmt = {'msgType': mg.SharedDataItems.ULTRASONIC_SENSORS, 'sender': config.processName,
                               'info': {'usSensorId': usSensorId, 'data': config.usSensorsMaster[usSensorId].__dict__}}
                    config.marvinShares.updateSharedData(updStmt)


            elif msgID == "!S0":  # free path after move blocked
                config.log(f"!S0 free path after blocked move")
                config.movementMaster.clearBlocking()
                cart.publishMovement()


            elif msgID == "!S1":  # obstacle detected by irSensor
                # !S1,<height>,<limit>,<irSensorId>
                sensorID = 0
                try:
                    items = recv.split(",")
                    height = int(items[1])
                    limit= int(items[2])
                    irSensorId = int(items[3])

                except Exception as e:
                    config.log(f"exception with message: '{recv}', error in message structure {e}")
                    continue

                config.log(f"!S1 obstacle detected by infrared sensor, height: {height} > limit: {limit}, sensor: {config.getIrSensorName(irSensorId)}")
                config.movementMaster.obstacleHeightIrSensor = height
                config.movementMaster.irSensorIdObstacle = irSensorId

                config.movementMaster.blocked = True
                config.movementMaster.blockedStartTime = time.time()
                config.movementMaster.blockEvent = mg.CartMoveBlockEvents.CLOSE_RANGE_OBSTACLE
                config.movementMaster.reasonStopped = "obstacle detected by ir sensor"

                cart.publishMovement()


            elif msgID == "!S2":  # abyss detected by infrared floor monitoring sensors:
                # !S2,<deepest depth>,<limit>,<first irSensorId with exceeding range>
                try:
                    items = recv.split(",")
                    depth = int(items[1])
                    limit = int(items[2])
                    irSensorId = int(items[3])

                except Exception as e:
                    config.log(f"exception with message: '{recv}', error in message structure {e}")
                    continue

                config.log(f"!S3 abyss detected, measured depth: {depth} > limit: {limit}, sensor: {config.getIrSensorName(irSensorId)}")
                config.movementMaster.abyssDepthIrSensor = depth
                config.movementMaster.irSensorIdAbyss = irSensorId

                config.movementMaster.blocked = True
                config.movementMaster.blockedStartTime = time.time()
                config.movementMaster.blockEvent = mg.CartMoveBlockEvents.CLOSE_RANGE_ABYSS
                config.movementMaster.reasonStopped = "abyss detected by ir sensor"

                cart.publishMovement()


            elif msgID == "!S3":  # obstacle in front (ultrasonic sensors)
                # !S3,<distance>,<limit>,<irSensorId>
                sensorID = 0
                try:
                    items = recv.split(",")
                    distance = int(items[1])
                    limit = int(items[2])
                    usSensorId = int(items[3])

                except Exception as e:
                    config.log(f"exception with message: '{recv}', error in message structure {e}")
                    continue

                config.log(
                    f"!S3 obstacle detected by ultrasonic sensor, distance: {distance} > limit: {limit}, sensor: {config.getUsSensorName(usSensorId)}")
                config.movementMaster.obstacleDistanceUsSensor = distance
                config.movementMaster.usSensorIdObstacle = usSensorId

                # a short range distance needs to stop the cart
                if distance < 100:
                    config.movementMaster.blocked = True
                    config.movementMaster.blockedStartTime = time.time()
                    config.movementMaster.blockEvent = mg.CartMoveBlockEvents.CLOSE_RANGE_OBSTACLE
                    config.movementMaster.reasonStopped = "obstacle detected by us sensor"

                cart.publishMovement()


            elif msgID == "!S4":  # "rotation stopped":
                # !A5,<currentYaw>,<reason>
                items = recv.split(",")
                yaw = round(float(items[1]))
                stopReason = items[2]
                cart.terminateRotation(stopReason)
                #config.log("<-A " + recv, publish=False)  # stop and reason
                config.log(f"!S4 cart rotation stopped, {stopReason}")


            elif msgID == "!S5":  # movement stopped
                # !A6,<orientation>,<distance moved>,<reason>
                items = recv.split(",")
                yaw = round(float(items[1]))
                distanceMoved = int(items[2])
                stopReason = items[3]
                cart.terminateMove(distanceMoved, yaw, stopReason)
                config.log(f"!S5 cart movement stopped, {stopReason}")


            elif msgID == "!B0":  # 12 V supply, measured value
                config.log(f"{recv} received")
                newVoltage12V = 0
                try:
                    newVoltage12V = round(float(recv.split(",")[1]))
                except Exception as e:
                    config.log("!B0 not able to retrieve voltage value {recv}, {e}")

                # test for significant change in Voltage
                if abs(config.stateMaster.Voltage12V - newVoltage12V) > 500:
                    config.stateMaster.Voltage12V = newVoltage12V
                    config.log(f"!B0, new 12V voltage read [mV]: {newVoltage12V}")
                    #updStmt:Tuple[int,cartCls.State] = (mg.SharedDataItems.CART_STATE, config.stateMaster)
                    updStmt = {'msgType': mg.SharedDataItems.CART_STATE, 'sender': config.processName,
                               'info': config.stateMaster}
                    config.marvinShares.updateSharedData(updStmt)


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
                config.stateMaster.cartDocked = True
                cart.publishState()


            elif msgID == "!D2":  #"!Ae":  # cart undocked
                # Arduino releases the relais for loading batteries through the docking contacts
                config.log("cart undocked")
                config.stateMaster.cartDocked = False
                cart.publishState()


            elif msgID == "!I1":  # bno055 platform sensor update:
                # !Ab,<yaw>,<roll>, <pitch>
                items = recv.split(",")
                config.platformImuMaster.yaw = 360 - round(float(items[1]) / 1000.0)
                config.platformImuMaster.roll = float(items[2]) / 1000.0
                config.platformImuMaster.pitch = float(items[3]) / 1000.0
                config.platformImuMaster.updateTime = time.time()
                cart.publishPlatformImu()

                config.log(f"!I1, platformImu yaw: {config.platformImuMaster.yaw}, roll: {config.platformImuMaster.roll}, pitch: {config.platformImuMaster.pitch}", publish=False)
                config.locationMaster.yaw = (config.platformImuMaster.yaw + config.platformImuMaster.yawCorrection) % 360
                cart.publishLocation()

            elif msgID == "!I2":  # bno055 head sensor update:
                # !Ab,<milliYaw>,<milliRoll>, <milliPitch>
                items = recv.split(",")
                config.headImuMaster.yaw = round(float(items[1]) / 1000.0)
                config.headImuMaster.roll = float(items[2]) / 1000.0
                config.headImuMaster.pitch = float(items[3]) / 1000.0
                config.headImuMaster.updateTime = time.time()
                cart.publishHeadImu()
                config.log(f"!I2, headImu, yaw: {config.headImuMaster.yaw}, roll: {config.headImuMaster.roll}, pitch: {config.headImuMaster.pitch}", publish=False)


            else:
                try:
                    config.log("<-A " + recv, publish=False)
                except:
                    config.log(f"Unexpected error on reading messages: {sys.exc_info()[0]}")

            # cartGlobal.log("msg processed")

        # monitor move timeout
        if config.stateMaster.cartMoving:
            if time.time() > config.movementMaster.moveStartTime + config.movementMaster.maxDuration:
                arduinoSend.sendStopCommand("timeout in move")

        # check for docked and batteries loaded
        if config.stateMaster.cartDocked:
            # check battery status of laptop
            if time.time() - cartControl.getLastBatteryCheckTime() > 5:
                cartControl.updateBatteryStatus()
                if config._batteryStatus.percent == 100:

                    # undock the cart
                    cart.initiateMove(mg.MoveDirection.BACKWARD, 100, 200)

                    # and go to sleep
                    os.system("rundll32.exe powrprof.dll,SetSuspendState 0,1,0")

        time.sleep(0.001)  # give other threads a chance
