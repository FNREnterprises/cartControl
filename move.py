
import time
import numpy as np
import pyrealsense2 as rs
import pickle

import config
import depthImage
import findObstacles
import arduinoSend


def getDepth(camAngle):
    if not config.D415streaming:
        config.log(f"request streaming from D415")
        try:
            config.streamD415.start(config.D415config)
            config.D415streaming = True
        except Exception as e:
            config.log(f"start stream failed: {e}")
            config.D415streaming = False
            return None

    decimate = rs.decimation_filter()
    decimate.set_option(rs.option.filter_magnitude, 3)

    frames = config.streamD415.wait_for_frames()

    depth_frame = frames.get_depth_frame()
    depth_frame = decimate.process(depth_frame)

    # Grab new intrinsics (may be changed by decimation)
    depth_intrinsics = rs.video_stream_profile(depth_frame.profile).get_intrinsics()
    # w, h = depth_intrinsics.width, depth_intrinsics.height
    config.pc = rs.pointcloud()
    points = config.pc.calculate(depth_frame)

    # Pointcloud data to arrays
    v = points.get_vertices()
    # verts = np.asanyarray(v).view(np.float32).reshape(240,428, 3)  # xyz
    verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz

    # rotate over x-axis
    angle = camAngle
    cosa = np.cos(np.radians(angle))
    sina = np.sin(np.radians(angle))

    xyz = np.zeros_like(verts)
    xyz[:, 0] = verts[:, 0]
    xyz[:, 1] = -(verts[:, 1] * cosa - verts[:, 2] * sina)
    xyz[:, 2] = verts[:, 1] * sina + verts[:, 2] * cosa
    xyz = xyz.reshape(240, 428, 3)

    # save as pickle file for testing
    '''
    afile = open(r'C:\d.pkl', 'wb')
    pickle.dump(d, afile)
    afile.close()

    #reload object from file
    file2 = open(r'C:\d.pkl', 'rb')
    new_d = pickle.load(file2)
    file2.close()
    '''

    file = "realsense.pickle"
    myFile = open(r'C:\Projekte\InMoov\realsense\pc.pickle', 'wb')
    pickle.dump(xyz, myFile)
    myFile.close()

    return xyz  # rotated point cloud giving vertical distances


def moveRequest(moveDirection: config.Direction, speed: int, distance: int, protected: bool) -> bool:

    # check for obstacles with head cam for forward moves
    if moveDirection == config.Direction.FORWARD and protected:

        # check for connection with servo control
        if config.servoControlConnection is None:
            config.log(f"no connection with servoControl, protected forward move not possible")
            return False


        distClosestObstacle, obstacleDirection = findObstacles.distGroundObstacles()

        if distClosestObstacle > 0.2:
            config.log(f"start driving, free move: {distClosestObstacle * 100:.0f} cm")
            arduinoSend.sendMoveCommand(moveDirection, speed, distance, protected)
            config.inForwardMove = True     # triggers thread monitorForwardMove
            return True

        else:
            config.log(
                f"do not move forward because of an obstacle ahead in {distClosestObstacle * 100:.0f} cm at {obstacleDirection:.0f} degrees")
            return False


    else:
        # for non-forward or unprotected moves do not use the head cam
        arduinoSend.sendMoveCommand(moveDirection, speed, distance, protected)


