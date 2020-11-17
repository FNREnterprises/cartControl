
import time
import numpy as np

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


def updateFloorOffset(msg):
    """
    cart arduino msg A1 sends floor offsets (measured distance - calibration distance
    # !A1,<sensorID>,<NUM_SCAN_STEPS>,[NUM_SCAN_STEPS<value>,]
    :param msg:
    :return:
    """
    sensorId = msg[1]
    numValues = msg[2]
    offsetValues = [int(i) for i in msg[3:-1]]

    formattedList = "".join([f"{x:5d}" for x in offsetValues])
    config.log(f"floor offsets [mm], sensor: {config.getSensorName(sensorId)} {formattedList}", publish=False)

    cart.floor.offset[sensorId] = offsetValues

    cart.floor.timeStamp[sensorId] = time.time()
    cart.publishFloorOffset(sensorId)


def updateSensorTestData(msg):
    """
    cart arduino msg A7 sends measured distance values
    the measured distances are only sent when a sensor test is requested
    # !A7,<sensorId>,<NUM_SCAN_STEPS>,<distances>]
    :param Values:
    :return:
    """
    sensorId:int = msg[1]
    numValues:int = msg[2]
    distanceValues = [int(i) for i in msg[3:-1]]

    formattedList = "".join([f"{x:5d}" for x in distanceValues])
    config.log(f"floor distances, sensor: {config.getSensorName(sensorId)} {formattedList}", publish=False)

    config.sensorTestData.sensorId = sensorId
    config.sensorTestData.distance = np.asarray(distanceValues)     # measured distance in mm
    config.sensorTestData.timeStamp = time.time()
    config.sensorTestData.sumMeasures = np.add(config.sensorTestData.sumMeasures, config.sensorTestData.distance)
    config.sensorTestData.numMeasures += 1
    cart.publishSensorTestData()


def updateFreeFrontRoom(messageItems):
    """
    the cart has 4 ultrasonic sensors in the front
    these sensors can only detect distances to obstacles, so without obstacles we get no distance
    :param messageItems:
    :return:
    """
    for usSensorId in range(mg.NUM_US_DISTANCE_SENSORS):
        usDistanceSensor[usSensorId] = messageItems[usSensorId]

    config.distanceDataChanged = True