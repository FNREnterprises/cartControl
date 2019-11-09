
import numpy as np
import pyrealsense2 as rs
import pickle

import config


def initD415():

    # Configure depth and color streams
    try:
        config.streamD415 = rs.pipeline()
        config.D415config = rs.config()
        config.D415config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        config.log(f"D415 ready to stream")

    except Exception as e:
        config.log(f"could not initialize D415 cam, {e}")
    #config.D415config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)



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
    #depth_intrinsics = rs.video_stream_profile(depth_frame.profile).get_intrinsics()
    # w, h = depth_intrinsics.width, depth_intrinsics.height
    config.pc = rs.pointcloud()
    points = config.pc.calculate(depth_frame)

    # Pointcloud data to arrays
    return points.get_vertices()


