
import time
import config
import cartControl

def publishLog(msg):

    for i, c in enumerate(config.clientList):

        if c['replyConn'] is not None:

            try:
                c['replyConn'].root.exposed_log(msg)

            except Exception as e:
                c['replyConn'] = None
                config.log(f"exception in publishLog to {c['clientId']}: {e}")


def publishCartProgress(direction, magnitude, final):

    # limit cart position update while moving/rotating to 5 per second
    if not final and (config.cartRotating or config.cartMoving):
        if time.time() - config.cartLastPublishTime < 0.2:
            return

    config.log(f"publishCartProgress, direction: {direction}, mag: {magnitude}, x: {config.cartLocationX}, y: {config.cartLocationY}, yaw: {cartControl.getCartYaw()}, "
               f"moving: {config.cartMoving}, rotating: {config.cartRotating}", publish=final)

    config.cartLastPublishTime = time.time()

    for i, c in enumerate(config.clientList):

        if c['replyConn'] is not None:

            try:
                c['replyConn'].root.exposed_cartProgress(
                    magnitude,
                    final,
                    config.cartLocationX,
                    config.cartLocationY,
                    cartControl.getCartYaw(),
                    config.cartMoving,
                    config.cartRotating)

            except Exception as e:
                c['replyConn'] = None
                config.log(f"exception in publishCartProgress to {c['clientId']}: {e}")


def publishLifeSignal(clientId):

    for c in config.clientList:

        if c['clientId'] == clientId:
            if c['replyConn'] is not None:
                #config.log(f"publishing life signal to {c['clientId']}")
                try:
                    c['replyConn'].root.exposed_lifeSignalUpdate(config.MY_NAME)
                except Exception as e:
                    c['replyConn'] = None
                    config.log(f"exception in publishLifeSignal to {c['clientId']}: {e}")


def publishCartInfo():

    config.log(f"publish cart info to clients")

    for i, c in enumerate(config.clientList):

        if c['replyConn'] is not None:

            try:
                c['replyConn'].root.exposed_updateCartInfo(
                    config.cartLocationX,
                    config.cartLocationY,
                    cartControl.getCartYaw(),
                    config.cartMoving,
                    config.cartRotating)

            except Exception as e:
                c['replyConn'] = None
                config.log(f"exception in publishBasicData to {c['clientId']}: {e}")


def publishServerReady():

    config.log(f"publishing ready={config.cartReady} message to clients")

    for i, c in enumerate(config.clientList):

        if c['replyConn'] is not None:
            if len(c['messageList']) == 0 or 'serverReady' in c['messageList']:
                try:
                    c['replyConn'].root.exposed_serverReady(f"cartControl", config.cartReady)
                except Exception as e:
                    c['replyConn'] = None
                    config.log(f"exception in publishServerReady with {c['clientId']}: {e}")


def cartDocked(state: bool):

    for i, c in enumerate(config.clientList):

        if c['replyConn'] is not None:
            if len(c['messageList']) == 0 or 'serverReady' in c['messageList']:
                try:
                    c['replyConn'].root.exposed_cartDocked(state)
                except Exception as e:
                    c['replyConn'] = None
                    config.log(f"exception in publishing cartDocked with {c['clientId']}: {e}")

