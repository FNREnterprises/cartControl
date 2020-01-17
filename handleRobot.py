
import time

import config
import findObstacles


def headPosOK(yaw, pitch, neckDegrees, rotheadDegrees):
    return abs(neckDegrees - pitch) < 3 and \
           abs(rotheadDegrees - yaw) < 3


def positionHead(yaw, pitch):

    # having a connection verify head neck position
    neckDegrees, _, _ = config.robotControlConnection.root.exposed_getPosition("head.neck")
    rotheadDegrees, _, _ = config.robotControlConnection.root.exposed_getPosition("head.rothead")

    # if neck is not in ground observation position request neck position
    numTries = 0
    while not headPosOK(yaw, pitch, neckDegrees, rotheadDegrees) and numTries < 3:
        numTries += 1
        config.robotControlConnection.root.exposed_requestServoDegrees("head.neck", config.pitchGroundWatchDegrees, 1.5)
        config.robotControlConnection.root.exposed_requestServoDegrees("head.rothead", 0, 1)
        time.sleep(1)
        neckDegrees, _, _ = config.robotControlConnection.root.exposed_getPosition("head.neck")
        rotheadDegrees, _, _ = config.robotControlConnection.root.exposed_getPosition("head.rothead")

    if not headPosOK(yaw, pitch, neckDegrees, rotheadDegrees):
        config.log \
            (f"could not move head into ground watch position, neck: {neckDegrees:0f}, rothead: {rotheadDegrees:0f}")
        return None, None
