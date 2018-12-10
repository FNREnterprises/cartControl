

import time
import threading
import numpy as np

import config
import watchDog
import arduinoReceive
import arduinoSend
import rpcReceive
import gui


CART_PORT = 20003


NUM_DISTANCE_SENSORS = 10
NUM_MEASUREMENTS_PER_SCAN = 11
distanceSensors = []
# VL nah
distanceSensors.append(
    {'sensorID': 0, 'direction': 'forward', 'position': 'left', 'range': 'short', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (0, 0), 'installed': True, 'servoIndex': 0, 'color': 'red',
     'rotation': -180})
# VL fern
distanceSensors.append(
    {'sensorID': 1, 'direction': 'forward', 'position': 'left', 'range': 'long', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (0, 1), 'installed': True, 'servoIndex': 0, 'color': 'blue',
     'rotation': -180})
# VR nah
distanceSensors.append(
    {'sensorID': 2, 'direction': 'forward', 'position': 'right', 'range': 'short', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (1, 0), 'installed': True, 'servoIndex': 1, 'color': 'red',
     'rotation': -180})
# VR fern
distanceSensors.append(
    {'sensorID': 3, 'direction': 'forward', 'position': 'right', 'range': 'long', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (1, 1), 'installed': True, 'servoIndex': 1, 'color': 'blue',
     'rotation': -180})
# LM nah
distanceSensors.append(
    {'sensorID': 4, 'direction': 'left', 'position': 'middle', 'range': 'short', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (2, 0), 'installed': True, 'servoIndex': 2, 'color': 'red',
     'rotation': 90})
# RM nah
distanceSensors.append(
    {'sensorID': 5, 'direction': 'right', 'position': 'middle', 'range': 'short', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (2, 1), 'installed': True, 'servoIndex': 2, 'color': 'red',
     'rotation': -90})
# HL nah
distanceSensors.append(
    {'sensorID': 6, 'direction': 'backward', 'position': 'left', 'range': 'short', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (3, 0), 'installed': True, 'servoIndex': 3, 'color': 'red', 'rotation': 0})
# HL fern
distanceSensors.append(
    {'sensorID': 7, 'direction': 'backward', 'position': 'left', 'range': 'long', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (3, 1), 'installed': True, 'servoIndex': 3, 'color': 'blue',
     'rotation': 0})
# HR nah
distanceSensors.append(
    {'sensorID': 8, 'direction': 'backward', 'position': 'right', 'range': 'short', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (3, 0), 'installed': True, 'servoIndex': 4, 'color': 'red', 'rotation': 0})
# HR fern
distanceSensors.append(
    {'sensorID': 9, 'direction': 'backward', 'position': 'right', 'range': 'long', 'newValuesShown': False,
     'timestamp': time.time(), 'valueIndex': (3, 1), 'installed': True, 'servoIndex': 4, 'color': 'blue',
     'rotation': 0})

distanceList = np.zeros((NUM_DISTANCE_SENSORS, NUM_MEASUREMENTS_PER_SCAN))


def cartInit():
    config.setMovementBlocked(False)

    # start cart control gui and initialization
    guiThread = threading.Thread(target=gui.startGui, args={})
    guiThread.start()
    print("cart gui started")

    # start serial reader for arduino messages
    msgThread = threading.Thread(target=arduinoReceive.readMessages, args={})
    print("start msg reader cart-arduino")
    msgThread.start()

    # test using speed for distance evaluation as camOdometry did not work reliably
    '''
    if not cartGlobal.isOdometryRunning():
        # start odometry thread    
        camThread = threading.Thread(target=odometry.trackCartMovements, args={})
        print("start cart tracker")
        camThread.start()
        cartGlobal.setOdometryRunning(True)
    '''
    print("gui, message and odometry threads startet")

    # wait for cart startup finished
    print("cart - wait for cart to startup ...")
    timeout = time.time() + 10
    while config.arduinoStatus == 0 or time.time() < timeout:
        time.sleep(1)

    if config.arduinoStatus == 0:
        print("cart - timeout in startup, terminating")
        raise SystemExit()

    # get current orientation of cart (bno055)
    arduinoSend.requestCartOrientation()
    time.sleep(0.1)
    config.loadCartLocation()


def updateDistances(Values):
    global distanceList, distanceSensors

    sensorID = Values[0]
    numValues = Values[1]

    # cartGlobal.log(f"updateDistancies: {Values}")

    for i in range(numValues):
        try:
            distanceList[sensorID][i] = float(Values[2 + i])
        except ValueError:
            distanceList[sensorID][i] = 0.0

    distanceSensors[sensorID]['timestamp'] = time.time()
    distanceSensors[sensorID]['newValuesShown'] = False


def setSensorDataShown(sensorID, new):
    global distanceSensors

    distanceSensors[sensorID]['newValuesShown'] = new


if __name__ == '__main__':

    print("startup cart")
    cartInit()

    if not config.standAloneMode:

        from rpyc.utils.server import ThreadedServer

        # start the watchDog for clientConnections
        navThread = threading.Thread(target=watchDog.watchDog, args={})
        navThread.setName("connectionWatchDog")
        navThread.start()

        print(f"start listening on port {config.MY_RPC_PORT}")
        myConfig = {"allow_all_attrs": True, "allow_pickle": True}
        server = ThreadedServer(rpcReceive.rpcListener, port=config.MY_RPC_PORT, protocol_config=myConfig)

        server.start()



