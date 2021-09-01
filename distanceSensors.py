

import time
import numpy as np
from dataclasses import dataclass

import config
import marvinglobal.marvinglobal as mg

sensorInTest = None

def updateFloorOffset(recv):
    """
    cart arduino msg F1 sends floor offsets of involved ir sensors of movement direction
    # !F1,[<sensorID>,<step>,<obstacleHeight>,<abyssDepth>] * involvedSensors(max 6)
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
        config.floorOffsetMaster[sensorId].obstacleHeight[step] = obstacleHeight
        logMsg += f", {obstacleHeight=}"

        abyssDepth = msg[sensor * 4 + 4]
        config.floorOffsetMaster[sensorId].abyssDepth[step] = abyssDepth
        logMsg += f", {abyssDepth=}"
        config.log(logMsg)
        config.floorOffsetMaster[sensorId].lastUpdate = time.time()

        # in normal operation keep this local in cartControl as it causes many data updates
        # running into obstacle/abyss will create its own messages !S1/!S2 to update the gui
        # only when running a sensor test the gui is updated with each step measure result
        if sensorInTest is not None:
            updStmt = {'msgType': mg.SharedDataItems.FLOOR_OFFSET, 'sender': config.processName,
                       'info': {'irSensorId': sensorId, 'step': step, 'height': obstacleHeight, 'depth': abyssDepth}}
            config.marvinShares.updateSharedData(updStmt)




