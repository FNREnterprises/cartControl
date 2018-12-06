
import config
import rpcReceive


def publishLog(msg):

    for i, c in enumerate(rpcReceive.clientList):

        try:
            c['conn'].root.exposed_log(msg)

        except Exception as e:
            rpcReceive.removeClient(i)
            config.log(f"exception in publishLog: {str(e)}, client removed from clientList")
