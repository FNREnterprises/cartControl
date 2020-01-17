
import os
import time
import datetime
import rpyc

import inmoovGlobal
import config
import arduinoSend
import gui
import cartControl
import rpcSend
import move
import camImages
import findObstacles

server = 'cartControl'

class rpcListener(rpyc.Service):

    ############################## common routines for clients

    def on_connect(self, conn):
        """
        This gets called when a client opens a connection
        :param conn:
        :return:
        """
        print(f"on_connect seen {conn}")
        callerName = conn._channel.stream.sock.getpeername()
        print(f"caller: {callerName}")


    def exposed_requestForReplyConnection(self, ip, port, messages=[], interval=5):
        """
        To allow the server to send data to registred clients on it own timing
        the server needs to have its own sending connection
        :param ip:      the clients ip
        :param port:    the clients port
        :param messages: None or list of messages the client is interrested in
        :param interval:
        :return:
        """
        messageList = list(messages)
        config.log(f"request for reply connection received from {ip}:{port}, messageList: {messageList}")

        # types of message content allowed with this connection
        myConfig = {"allow_all_attrs": True, "allow_pickle": True}
        try:
            replyConn = rpyc.connect(ip, port)
            config.log(f"reply connection established")
        except Exception as e:
            config.log(f"failed to open a reply connection, {e}")
            return

        # add the reply connection handle into the client list
        clientId = (ip, port)
        connectionUpdated = False
        for c in config.clientList:
            if c['clientId'] == clientId:
                config.log(f"update client connection")
                c['replyConn'] = replyConn
                connectionUpdated = True

        # add the client to the client list if we don't have it in the list yet
        if not connectionUpdated:
            config.log(f"append client connection {clientId}")
            config.clientList.append({'clientId': clientId,
                                      'replyConn': replyConn,
                                      'lifeSignalReceived': time.time(),
                                      'messageList': messageList,
                                      'interval': interval})

        ############## server special behaviour with available reply connection ###########
        # if cart task is already running send basic data
        if config.cartReady:
            rpcSend.publishCartInfo()
            rpcSend.publishServerReady()

        # .. else send only a connection ready message
        else:
            rpcSend.publishLifeSignal()
        #############


    def exposed_clientLifeSignal(self, ip, port):
        clientId = (ip,port)
        for c in config.clientList:
            if c['clientId'] == clientId:
                c['lifeSignalReceived'] = time.time()


    def exposed_terminate(self):
        """
        client request to terminate this server task
        :return:
        """
        print(f"{server} task - terminate request received")
        time.sleep(10)
        os._exit(0)
        return True


    def exposed_log(self, msg):
        """
        ignore requests from clients for logging
        :param msg:
        :return:
        """
        # ignore
        pass

    ############################## end common routines for clients

    #############################################################
    ### SERVER SPECIFIC PUBLIC FUNCTIONS AVAILABLE FOR CLIENTS
    #############################################################

    def exposed_move(self, direction: int, speed, distance, protected:bool=True):
        """
        request to move the cart
        :param direction: config.direction class
        :param speed: theoretically 0..255, practically values below 50 do not move the motors
        :param distance:
        :param protected: docking moves need to be unprotected as they contact the dock
        :return: request accepted (True) or ignored(False)
        """
        moveDirection = config.MoveDirection(direction)
        config.log(f"exposed_move , direction: {moveDirection}, speed: {speed}, distance: {distance}", publish=False)
        success = move.moveRequest(moveDirection, speed, distance, protected)
        return success


    def exposed_rotateRelative(self, angle, speed):
        """
        request for relative cart rotation
        :param angle: >0 counterclock, <0 clockwise rotation
        :param speed: theoretically 0..255, practically values below 50 do not move the motors
        :return:
        """
        if abs(angle) > 0:
            config.log(f"exposed_rotateRelative, angle: {angle}", publish=False)
            arduinoSend.sendRotateCommand(int(angle), int(speed))
            #gui.controller.updateTargetRotation(cartControl.getCartYaw() + int(angle))
            cartGui.updateTargetRotation(cartControl.getCartYaw() + int(angle))
        return True


    def exposed_stop(self):
        """
        stops the cart
        :return:
        """
        gui.controller.stopCart()
        config.log("stop received", publish=False)


    def exposed_setCartLocation(self, x, y):
        """
        The cart takes care of its location and persists the location
        A client can change this location
        :param x:
        :param y:
        :return:
        """
        config.log(f"set cart location received, x: {x}, y: {y}", publish=False)
        config.cartLocationX = x
        config.cartLocationY = y
        return True


    def exposed_adjustCartPosition(self, dX, dY, dYaw):
        """
        The cart takes care of its location and orientation
        A client can request adjustments of the location and orientation
        :param dX:
        :param dY:
        :param dYaw:
        :return:
        """
        config.log(f"adjust cart position received, x: {dX}, y: {dY}, degrees: {dYaw}", publish=False)
        config.cartLocationX += dX
        config.cartLocationY += dY
        config.platformImuYawCorrection += dYaw
        return True


    def exposed_getCartInfo(self):
        """
        a client can request the current cart position, orientation and state
        :return:
        """
        #cartX, cartY = cartControl.getCartLocation()
        config.log(f"getCartInfo, x: {config.cartLocationX}, y: {config.cartLocationY}, o: {cartControl.getCartYaw()}, m: {config.cartMoving}, r: {config.cartRotating}", publish=False)
        return config.cartLocationX, config.cartLocationY, cartControl.getCartYaw(), config.cartMoving, config.cartRotating


    def exposed_requestCartOrientation(self):
        """
        a client can request the current cart orientation
        :return:
        """
        return cartControl.getCartYaw()


    def exposed_getObstacleDistances(self):
        _, _, obstacleDist = findObstacles.aboveGroundObstacles(inmoovGlobal.pitchWallWatchDegrees)
        return obstacleDist


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
            arduinoSend.sendStopCommand("obstacle detected with depth cam")
        return True


    def exposed_lifeSignalUpdate(self, server):
        pass


    def exposed_requestHeadOrientation(self):
        return config.headImuYaw, config.headImuRoll, config.headImuPitch


    def exposed_getCamProperties(self, cam):
        config.log(f"getCamProperties, cam: {cam}")
        return camImages.cams[cam].__dict__


    def exposed_takeImage(self, cam):       # the cams ID as defined in inmoovGlobal
        config.log(f"takeImage received, cam: {cam}")
        return camImages.cams[cam].takeImage()


    def exposed_findObstacles(self, cam):       # the cams ID as defined in inmoovGlobal
        config.log(f"take depth received, cam: {cam}")
        if cam == inmoovGlobal.HEAD_CAM:
            points = camImages.cams[cam].takeDepth()
            _, _, obstacleDistances = findObstacles.aboveGroundObstacles(points,inmoovGlobal.pitchWallWatchDegrees)
            return obstacleDistances

        else:
            config.log(f"wrong camera for depth request received")
            return None

