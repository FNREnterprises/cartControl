
import os
import time
import rpyc

import config
import arduinoSend
import gui

clientList = []
watchInterval = 5

server = 'cartControl'


def addClient(c, i):

    global clientList, watchInterval

    clientList.append(c.copy())
    watchInterval = i
    print(f"client added, {c}")


def updateMessageTime(pid):

    global clientList

    for i, c in enumerate(clientList):
        if c['pid'] == pid:
            #print(f"{time.time():.0f} update message time, pid {pid}, clientIndex: {i}")
            clientList[i]['lastMessageReceivedTime'] = time.time()
            #config.log(f"getLifeSignal time update, pid: {pid}, clientIndex: {i}, time: {time.time()}")


def removeClient(i):

    global clientList

    print(f"remove client from clientList, index: {i}, client: {clientList[i]}")
    del clientList[i]




class rpcListener(rpyc.Service):

    ############################## common routines for clients
    watchInterval = 5

    def on_connect(self, conn):
        print(f"on_connect seen {conn}")
        callerName = conn._channel.stream.sock.getpeername()
        print(f"caller: {callerName}")


    def exposed_requestForReplyConnection(self, ip, port, interval=5):

        print(f"request for reply connection {ip}:{port}")
        replyConn = rpyc.connect(ip, port)

        clientId = (ip, port)
        connectionUpdated = False
        for c in clientList:
            if c['clientId'] == clientId:
                print(f"update client connection")
                c['replyConn'] = replyConn
                connectionUpdated = True

        if not connectionUpdated:
            print(f"append client connection {clientId}")
            clientList.append({'clientId': clientId, 'replyConn': replyConn, 'lastMessageReceivedTime': time.time(), 'interval': interval})


    def exposed_getLifeSignal(self, ip, port):

        for c in clientList:
            if c['clientId'] == (ip, port):
                #print(f"life signal received from  {ip}, {port}, {time.time()}")
                c['lastMessageReceivedTime'] = time.time()

        return True


    def exposed_terminate(self):
        print(f"{server} task - terminate request received")
        os._exit(0)
        return True

    ############################## common routines for clients


    def exposed_move(self, direction, speed, distance):
        config.log(f"exposed_move (xmlrpc), direction: {direction}, speed: {speed}, distance: {distance}")
        arduinoSend.sendMoveCommand(direction, speed, distance)
        return True


    def exposed_rotateRelative(self, angle, speed):
        if abs(angle) > 0:
            config.log(f"exposed_rotateRelative, angle: {angle:.1f}")
            arduinoSend.sendRotateCommand(int(angle), int(speed))
            gui.controller.updateTargetRotation(config.getCartOrientation() + int(angle))
        return True


    def exposed_stop(self):
        config.log("exposed_stop")
        gui.controller.stopCart()
        return True


    def exposed_requestCartOrientation(self):
        return config.getCartOrientation()


    def exposed_getObstacleInfo(self):
        return config.obstacleInfo


    def exposed_getCartInfo(self):
        cartX, cartY = config.getCartLocation()
        return config.getCartOrientation(), cartX, cartY, config.isCartMoving(), config.isCartRotating()


    def exposed_getBatteryStatus(self):
        return config.getBatteryStatus()


    def exposed_isCartMoving(self):
        return config.isCartMoving()


    def exposed_isCartRotating(self):
        return config.isCartRotating()




