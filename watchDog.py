
import time
import rpyc

import config
import rpcReceive

watchInterval = 3   #seconds

def connectWithKinect():

    #############################
    #  try to connect with kinect
    #############################
    success = True

    if config.firstKinectConnectionAttempt:
        config.log(f"try to connect with 'kinect' at {config.kinectIp}, {config.kinectPort}")
    try:
        config.kinectConnection = rpyc.connect(config.kinectIp, config.kinectPort)
    except Exception as e:
        if config.firstKinectConnectionAttempt:
            config.log(f"could not connect with 'kinect', exception: {e}")
        success = False
        config.firstKinectConnectionAttempt = False

    # try to open a response connection in the server
    if success:
        config.firstKinectConnectionAttempt = True
        messageList = ['lifeSignalUpdate','obstacleUpdate']
        try:
            config.kinectConnection.root.requestForReplyConnection(config.MY_IP, config.MY_RPC_PORT, messageList)
        except Exception as e:
            config.log(f"could not request reply connection with 'kinect', exception: {e}")
            success = False

    return success


def connectWithServoControl():

    ####################################
    #  try to connect with servo control
    ####################################
    #config.log(f"try to connect with 'servoControl' at {config.servoControlIp}, {config.servoControlPort}")
    try:
        config.servoControlConnection = rpyc.connect(config.servoControlIp, config.servoControlPort)
    except Exception as e:
        if config.servoControlConnectionFirstTry:
            config.log(f"could not connect with 'servoControl', exception: {e}")
            config.servoControlConnectionFirstTry = False
            config.servoControlConnection = None

    # try to open a response connection in the server
    if config.servoControlConnection is not None:
        messageList = ['lifeSignalUpdate','obstacleUpdate']
        try:
            config.servoControlConnection.root.requestForReplyConnection(config.MY_IP, config.MY_RPC_PORT, messageList)
            config.log(f"connection with servoControl established")
        except Exception as e:
            config.log(f"could not request reply connection with 'servoControl', exception: {e}")
            config.servoControlConnection = None

    return config.servoControlConnection is not None


# check for life signal from client while having a connection
def watchDog():

    while True:

        # watchdog for incoming commands from clients
        # stop cart if we do not get regular messages
        for i, c in enumerate(config.clientList):

            if c['replyConn'] is not None:

                span = (time.time() - c['lastMessageReceivedTime'])
                if (span) > (c['interval'] * 1.5):
                    config.log(
                        f"{time.time():.0f} heartbeat interval with {c['clientId']} exceeded, span: {span:.0f}, interval: {c['interval']}")
                    c['replyConn'] = None

        '''
        # try to open a connection with the kinect server
        if config.kinectConnection is None:

            connectWithKinect()
        '''
        # try to connect with servo control
        if config.servoControlConnection is None:
            connectWithServoControl()

        # time.sleep(rpcReceive.watchInterval)
        time.sleep(watchInterval)
