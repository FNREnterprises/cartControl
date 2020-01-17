
import time
import numpy as np

import config

NUM_DISTANCE_SENSORS = 10
NUM_MEASUREMENTS_PER_SCAN = 11

distanceList = np.zeros((NUM_DISTANCE_SENSORS, NUM_MEASUREMENTS_PER_SCAN), dtype=np.int16)

distanceData = []
# front left
distanceData.append(
    {'sensorID': 0, 'direction': 'forward', 'position': 'left', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (0, 0), 'installed': True, 'servoIndex': 0, 'color': 'red',
     'rotation': -180})
# front center
distanceData.append(
    {'sensorID': 1, 'direction': 'forward', 'position': 'center', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (0, 1), 'installed': True, 'servoIndex': 0, 'color': 'blue',
     'rotation': -180})
# front right
distanceData.append(
    {'sensorID': 2, 'direction': 'forward', 'position': 'right', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (1, 0), 'installed': True, 'servoIndex': 1, 'color': 'red',
     'rotation': -180})
# back left
distanceData.append(
    {'sensorID': 3, 'direction': 'back', 'position': 'left', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (1, 1), 'installed': True, 'servoIndex': 1, 'color': 'blue',
     'rotation': -180})
# back center
distanceData.append(
    {'sensorID': 4, 'direction': 'back', 'position': 'center', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (2, 0), 'installed': True, 'servoIndex': 2, 'color': 'red',
     'rotation': 90})
# back right
distanceData.append(
    {'sensorID': 5, 'direction': 'back', 'position': 'right', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (2, 1), 'installed': True, 'servoIndex': 2, 'color': 'red',
     'rotation': -90})
# left side front
distanceData.append(
    {'sensorID': 6, 'direction': 'left', 'position': 'front', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (3, 0), 'installed': True, 'servoIndex': 3, 'color': 'red', 'rotation': 0})
# right side front
distanceData.append(
    {'sensorID': 7, 'direction': 'right', 'position': 'front', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (3, 1), 'installed': True, 'servoIndex': 3, 'color': 'blue',
     'rotation': 0})
# left side back
distanceData.append(
    {'sensorID': 8, 'direction': 'left', 'position': 'back', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (3, 0), 'installed': True, 'servoIndex': 4, 'color': 'red', 'rotation': 0})
# right side back
distanceData.append(
    {'sensorID': 9, 'direction': 'right', 'position': 'back', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (3, 1), 'installed': True, 'servoIndex': 4, 'color': 'blue',
     'rotation': 0})


def updateDistances(Values):
    """
    cart arduino msg A1 sends measured distance values
    # !A1,<sensorID>,<ANZ_MESSUNGEN_PRO_SCAN>,<servoStep>,[ANZ_MESSUNGEN_PRO_SCAN<value>,]
    :param Values:
    :return:
    """
    sensorID = Values[1]
    numValues = Values[2]
    servoStep = Values[3]

    config.log(f"updateDistances, sensor: {config.getSensorName(sensorID)} {Values[4:-1]}", publish=False)

    for i in range(numValues):
        try:
            distanceList[sensorID][i] = round(float(Values[4 + i]))
        except ValueError:
            distanceList[sensorID][i] = 0
        except IndexError:
            distanceList[sensorID][i] = 0
            config.log(f"something is wrong with !A1 message from cart: {Values}")

    distanceData[sensorID]['timestamp'] = time.time()
    distanceData[sensorID]['newValuesShown'] = False


def setSensorDataShown(sensorID, new):
    distanceData[sensorID]['newValuesShown'] = new
