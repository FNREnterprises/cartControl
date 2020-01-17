
import time
import rpyc

import inmoovGlobal
import config
import rpcSend


def connectWithRobotControl():

    ####################################
    #  try to connect with robot control
    ####################################
    #config.log(f"try to connect with 'robotControl' at {config.robotControlIp}, {config.robotControlPort}")
    try:
        config.robotControlConnection = rpyc.connect(config.robotControlIp, config.robotControlPort)
        config.log(f"try to connect with robotControl")
    except Exception as e:
        if config.robotControlConnectionFirstTry:
            config.log(f"could not connect with 'robotControl', exception: {e}")
            config.robotControlConnectionFirstTry = False
            config.robotControlConnection = None

    # try to open a response connection in the server
    if config.robotControlConnection is not None:
        messageList = ['lifeSignalUpdate','obstacleUpdate']
        try:
            config.robotControlConnection.root.requestForReplyConnection(config.pcIP, config.MY_RPC_PORT, messageList)
            config.log(f"request for reply connection sent to robotControl")
        except Exception as e:
            config.log(f"could not request reply connection with 'robotControl', exception: {e}")
            config.robotControlConnection = None

    return config.robotControlConnection is not None


# check for life signal from client while having a connection
def watchDog():

    while True:

        # check for connection with robotControl
        if config.robotControlConnection is None:
            connectWithRobotControl()

        # watchdog for incoming commands from clients
        # remove client if we do not get regular messages
        for i, c in enumerate(config.clientList):

            timeSinceLastMessage = time.time() - c['lifeSignalReceived']

            if timeSinceLastMessage > inmoovGlobal.RPYC_HEARTBEAT_INTERVAL * 1.5:
                config.log(f"no message from {c['clientId']} for {timeSinceLastMessage} s")
                config.log(f"remove client: {c['clientId']}")
                del config.clientList[i]

        # send life signal to clients
        rpcSend.publishLifeSignal()

        time.sleep(inmoovGlobal.RPYC_HEARTBEAT_INTERVAL)


