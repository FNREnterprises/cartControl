
import os
import time

from xmlrpc.server import SimpleXMLRPCServer
import xmlrpc.client

import config
import arduinoSend
import gui


# xmlrpc
xmlrpcServer  = None

def xmlrpcListener():

    global xmlrpcServer

    xmlrpcServer = SimpleXMLRPCServer((config.MY_IP, config.MY_XMLRPC_PORT))

    xmlrpcServer.register_function(exposed_register, "exposed_register")
    xmlrpcServer.register_function(exposed_getLifeSignal, "exposed_getLifeSignal")
    xmlrpcServer.register_function(exposed_move, "exposed_move")
    xmlrpcServer.register_function(exposed_rotateRelative, "exposed_rotateRelative")
    xmlrpcServer.register_function(exposed_stop, "exposed_stop")
    xmlrpcServer.register_function(exposed_requestCartOrientation,"exposed_requestCartOrientation")
    xmlrpcServer.register_function(exposed_heartBeat, "exposed_heartBeat")
    xmlrpcServer.register_function(exposed_getObstacleInfo, "exposed_getObstacleInfo")
    xmlrpcServer.register_function(exposed_getCartInfo, "exposed_getCartInfo")
    xmlrpcServer.register_function(exposed_getBatteryStatus, "exposed_getBatteryStatus")
    xmlrpcServer.register_function(exposed_isCartMoving, "exposed_isCartMoving")
    xmlrpcServer.register_function(exposed_isCartRotating, "exposed_isCartRotating")
    xmlrpcServer.register_function(exposed_terminate, "exposed_terminate")

    xmlrpcServer.serve_forever()


def exposed_register(ip, port):
    config.log(f"exposed_register request received from {ip}:{port}")
    clientAddr = f"http://{ip}:{port}"
    print(f"clientAddr: {clientAddr}")
    try:
        config.setXmlrpcClient(xmlrpc.client.ServerProxy(clientAddr))
    except Exception as e:
        print(f"failure to add client {ip}:{port} - {e}")
        os._exit(1)
    return True


def exposed_getLifeSignal():
    return True


def exposed_move(direction, speed, distance):
    config.log(f"exposed_move (xmlrpc), direction: {direction}, speed: {speed}, distance: {distance}")
    arduinoSend.sendMoveCommand(direction, speed, distance)
    return True


def exposed_rotateRelative(angle, speed):
    if abs(angle) > 0:
        config.log(f"exposed_rotateRelative, angle: {angle:.1f}")
        arduinoSend.sendRotateCommand(int(angle), int(speed))
        gui.controller.updateTargetRotation(config.getCartOrientation() + int(angle))
    return True


def exposed_stop():
    config.log("exposed_stop")
    gui.controller.stopCart()
    return True


def exposed_requestCartOrientation():
    return config.getCartOrientation()


def exposed_heartBeat():
    global lastMessage

    # watchdog for incoming commands from navManager
    # stop cart if we do not get regular messages
    currTime = int(round(time.time() * 1000))
    config.log(currTime - lastMessage)
    if currTime - lastMessage > config.TIMEOUT:
        if config.isCartMoving:
            arduinoSend.sendStopCommand("missing heartbeat from navManager")
    else:
        arduinoSend.sendHeartbeat()
        config.log("arduino Heartbeat sent")

    lastMessage = int(round(time.time() * 1000))
    return True


def exposed_getObstacleInfo():
    return config.obstacleInfo


def exposed_getCartInfo():
    posX, posY = config.getCartLocation()
    return config.getCartOrientation(), posX, posY, config.isCartMoving(), config.isCartRotating()


def exposed_getBatteryStatus():
    return config.getBatteryStatus()


def exposed_isCartMoving():
    return config.isCartMoving()


def exposed_isCartRotating():
    return config.isCartRotating()


def exposed_terminate():
    config.log(f"cartControl - terminate command received")
    os._exit(0)
    return True



