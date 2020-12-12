
import os
import time
from typing import Tuple

import config
import marvinglobal.marvinglobal as mg
import cart

sensorInTest = None

irDistanceSensorDefinitions = []
# front left
irDistanceSensorDefinitions.append(
    {'sensorID': 0, 'direction': 'forward', 'position': 'left', 'installed': True, 'servoIndex': 0})
# front center
irDistanceSensorDefinitions.append(
    {'sensorID': 1, 'direction': 'forward', 'position': 'center', 'installed': True, 'servoIndex': 1})
# front right
irDistanceSensorDefinitions.append(
    {'sensorID': 2, 'direction': 'forward', 'position': 'right', 'installed': True, 'servoIndex': 2})
# back left
irDistanceSensorDefinitions.append(
    {'sensorID': 3, 'direction': 'back', 'position': 'left', 'installed': True, 'servoIndex': 3})
# back center
irDistanceSensorDefinitions.append(
    {'sensorID': 4, 'direction': 'back', 'position': 'center', 'installed': True, 'servoIndex': 4})
# back right
irDistanceSensorDefinitions.append(
    {'sensorID': 5, 'direction': 'back', 'position': 'right', 'installed': True, 'servoIndex': 5})
# left side front
irDistanceSensorDefinitions.append(
    {'sensorID': 6, 'direction': 'left', 'position': 'front', 'installed': True, 'servoIndex': None})
# left side back
irDistanceSensorDefinitions.append(
    {'sensorID': 7, 'direction': 'left', 'position': 'back', 'installed': True, 'servoIndex': None})
# right side front
irDistanceSensorDefinitions.append(
    {'sensorID': 8, 'direction': 'right', 'position': 'front', 'installed': True, 'servoIndex': None})
# right side back
irDistanceSensorDefinitions.append(
    {'sensorID': 9, 'direction': 'right', 'position': 'back', 'installed': True, 'servoIndex': None,})


usDistanceSensorDefinitions = []
# front left
usDistanceSensorDefinitions.append(
    {'sensorID': 0, 'name': 'left wheel', 'installed': True})
usDistanceSensorDefinitions.append(
    {'sensorID': 0, 'name': 'left center', 'installed': True})
usDistanceSensorDefinitions.append(
    {'sensorID': 0, 'name': 'right center', 'installed': True})
usDistanceSensorDefinitions.append(
    {'sensorID': 0, 'name': 'right wheel', 'installed': True})

# the distance values
usDistanceSensor = [0] * mg.NUM_US_DISTANCE_SENSORS


def updateFloorOffset(recv):
    """
    cart arduino msg F1 sends floor offsets of involved ir sensors of movement direction
    # !F1,[<sensorID>,<obstacleheight>,<abyssDepth>]*involvedSensors(max 6)
    """
    msg = [int(e) if e.isdigit() else e for e in recv.split(',')]

    numSensors = int((len(msg)-1) / 4)

    config.log("!F1:")
    for sensor in range(numSensors):
        sensorId = msg[sensor * 4 + 1]
        logMsg = config.getIrSensorName(sensorId)

        step = msg[sensor * 4 + 2]
        logMsg += f", {step=}"

        obstacleHeight = msg[sensor * 4 + 3]
        config.floorOffsetLocal[sensorId].obstacleHeight[step] = obstacleHeight
        logMsg += f", {obstacleHeight=}"

        abyssDepth = msg[sensor * 4 + 4]
        config.floorOffsetLocal[sensorId].abyssDepth[step] = abyssDepth
        logMsg += f", {abyssDepth=}"
        config.log(logMsg)
        config.floorOffsetLocal[sensorId].lastUpdate = time.time()

        updStmt:Tuple[mg.SharedDataItem,int,...] = (mg.SharedDataItem.FLOOR_OFFSET, sensorId, step, obstacleHeight, abyssDepth)
        config.share.updateSharedData(updStmt)




def updateSensorTestData(msg):
    """
    cart arduino msg A7 contains measured distance values for each scan step
    the measured distances are only sent when a sensor test is requested
    # !F3,<sensorId>,<NUM_SCAN_STEPS>,<distances>]
    forward the distances to the gui
    """
    sensorId:int = msg[1]
    numValues:int = msg[2]
    distanceValues = [int(i) for i in msg[3:-1]]

    formattedList = "".join([f"{x:5d}" for x in distanceValues])
    config.log(f"floor distances, sensor: {config.getIrSensorName(sensorId)} {formattedList}", publish=False)

    config.share.cartGuiUpdateQueue.put({'msgType': mg.SharedDataItem.SENSOR_TEST_DATA, 'sensorId': sensorId, 'distances': distanceValues})


def updateFreeFrontRoom(messageItems):
    """
    the cart has 4 ultrasonic sensors in the front
    these sensors can only detect distances to obstacles, so without obstacles we get no distance
    :param messageItems:
    :return:
    """
    for usSensorId in range(mg.NUM_US_DISTANCE_SENSORS):
        usDistanceSensor[usSensorId] = messageItems[usSensorId]

