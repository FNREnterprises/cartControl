
import os
import time
import datetime
import rpyc

import config
import arduinoSend
import gui
import cartControl
import rpcSend

watchInterval = 5

server = 'cartControl'


class rpcListener(rpyc.Service):

    ############################## common routines for clients
    watchInterval = 5

    def on_connect(self, conn):
        print(f"on_connect seen {conn}")
        callerName = conn._channel.stream.sock.getpeername()
        print(f"caller: {callerName}")


    def exposed_requestForReplyConnection(self, ip, port, messages=[], interval=5):

        messageList = list(messages)
        config.log(f"request for reply connection received from {ip}:{port}, messageList: {messageList}")
        myConfig = {"allow_all_attrs": True, "allow_pickle": True}
        try:
            replyConn = rpyc.connect(ip, port)
            config.log(f"reply connection established")
        except Exception as e:
            config.log(f"failed to open a reply connection, {e}")
            return

        clientId = (ip, port)
        connectionUpdated = False
        for c in config.clientList:
            if c['clientId'] == clientId:
                config.log(f"update client connection")
                c['replyConn'] = replyConn
                connectionUpdated = True

        if not connectionUpdated:
            config.log(f"append client connection {clientId}")
            config.clientList.append({'clientId': clientId,
                                      'replyConn': replyConn,
                                      'lastMessageReceivedTime': time.time(),
                                      'messageList': messageList,
                                      'interval': interval})

        # if cart is already running send basic data and a ready message
        if config.cartReady:
            rpcSend.publishCartInfo()
            rpcSend.publishServerReady()
        else:
            rpcSend.publishLifeSignal(clientId)


    def exposed_requestLifeSignal(self, ip, port):

        for c in config.clientList:
            if c['clientId'] == (ip, port):
                #config.log(f"request for life signal received from  {ip}, {port}, {str(datetime.datetime.now())[11:22]}")
                c['lastMessageReceivedTime'] = time.time()
        rpcSend.publishLifeSignal((ip, port))
        return True


    def exposed_terminate(self):
        print(f"{server} task - terminate request received")
        time.sleep(10)
        os._exit(0)
        return True

    ############################## common routines for clients


    def exposed_move(self, direction: int, speed, distance):
        moveDirection = config.Direction(direction)
        config.log(f"exposed_move , direction: {moveDirection}, speed: {speed}, distance: {distance}", publish=False)
        arduinoSend.sendMoveCommand(moveDirection, speed, distance)
        return True


    def exposed_rotateRelative(self, angle, speed):
        if abs(angle) > 0:
            config.log(f"exposed_rotateRelative, angle: {angle}", publish=False)
            arduinoSend.sendRotateCommand(int(angle), int(speed))
            gui.controller.updateTargetRotation(cartControl.getCartYaw() + int(angle))
        return True


    def exposed_stop(self):
        config.log("stop received", publish=False)
        gui.controller.stopCart()
        return True


    def exposed_setCartLocation(self, x, y):
        config.log(f"set cart location received, x: {x}, y: {y}", publish=False)
        config.cartLocationX = x
        config.cartLocationY = y
        return True


    def exposed_adjustCartPosition(self, dX, dY, dYaw):
        config.log(f"adjust cart position received, x: {dX}, y: {dY}, degrees: {dYaw}", publish=False)
        config.cartLocationX += dX
        config.cartLocationY += dY
        config.imuDegreesCorrection += dYaw
        return True


    def exposed_getCartInfo(self):
        #cartX, cartY = cartControl.getCartLocation()
        config.log(f"getCartInfo, x: {config.cartLocationX}, y: {config.cartLocationY}, o: {cartControl.getCartYaw()}, m: {config.cartMoving}, r: {config.cartRotating}", publish=False)
        return config.cartLocationX, config.cartLocationY, cartControl.getCartYaw(), config.cartMoving, config.cartRotating


    def exposed_requestCartOrientation(self):
        return cartControl.getCartYaw()


    def exposed_getObstacleInfo(self):
        return config.obstacleInfo


    def exposed_getBatteryStatus(self):
        if config._batteryStatus is None:
            return None
        else:
            msg = {'plugged': config._batteryStatus.power_plugged,
                   'percent': config._batteryStatus.percent,
                   'v12': config._mVolts12V,
                   'v6': config._mVolts6V}
            #config.log(f"batteries: {msg}", publish=False)
            return msg


    def exposed_isCartMoving(self):
        return config.cartMoving

    def exposed_isCartRotating(self):
        return config.cartRotating


    def exposed_obstacleUpdate(self, data):

        distance = int(float(data.split(',')[0]))
        angle = int(float(data.split(',')[1]))
        config.log(f"obstacle detected, distance: {distance}, angle: {angle}")
        if distance < 850:
            arduinoSend.sendStopCommand("kinect detected obstacle")
        return True


    def exposed_lifeSignalUpdate(self, server):
        pass


    def exposed_powerKinect(self, newState):
        config.log(f"request for kinect power {newState} received")
        arduinoSend.powerKinect(newState)
        return True

