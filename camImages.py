
import time
import numpy as np
import cv2
import imutils
import pyrealsense2 as rs

import inmoovGlobal
import config
import findObstacles

cams = {}


def initCameras():
    '''
    def __init__(self, name, deviceId, cols, rows, fovH, fovV, rotate, numReads):
    :return:
    '''
    global cams
    cams.update({inmoovGlobal.CART_CAM: UsbCamera("cartcam", 2, 640, 480, 60, 60, 0, 2)})
    #cams[inmoovGlobal.CART_CAM].takeImage(True)

    cams.update({inmoovGlobal.EYE_CAM: UsbCamera("eyecam", 3, 640, 480, 21, 40, 90, 2)})
    #cams[inmoovGlobal.EYE_CAM].takeImage(True)

    cams.update({inmoovGlobal.HEAD_CAM: D415Camera("headcam", None, 1920, 1080, 69.4, 42.5, 0, 10)})
    #cams[inmoovGlobal.HEAD_CAM].takeImage(True)
    #cams[inmoovGlobal.HEAD_CAM].takeDepth(True)

    return D415Camera.handle is not None


class Camera(object):

    def __init__(self, name, deviceId, cols, rows, fovH, fovV, rotate, numReads):
        self.name = name
        self.deviceId = deviceId
        self.cols = cols
        self.rows = rows
        self.fovH = fovH
        self.fovV = fovV
        self.rotate = rotate
        self.numReads = numReads

        self.handle = None
#        self.img = None


class UsbCamera(Camera):
    def __init__(self, name, deviceId, cols, rows, fovH, fovV, rotate, numReads):
        super().__init__(name, deviceId, cols, rows, fovH, fovV, rotate, numReads)

        try:
            self.handle = cv2.VideoCapture(self.deviceId, cv2.CAP_DSHOW)
        except Exception as e:
            config.log(f"could not connect with cam {self.name}")
            self.handle = None


    def takeImage(self, show=False):

        if self.handle is None:
            config.log(f"{self.name} not available", publish=False)
            return None

        _, _ = self.handle.read()
        success, image = self.handle.read()

        if success:
            config.log(f"{self.name} image successfully taken")
            #self.handle.release()
            if self.rotate != 0:
                image = imutils.rotate_bound(image, self.rotate)

            if show:
                cv2.imshow(f"usb image device {self.deviceId}", image)
                cv2.waitKey(0)
                cv2.destroyAllWindows()

            return image
        else:
            config.log(f"{self.name} image could not be taken")
            self.handle.release()
            self.handle = None


class D415Camera(Camera):

    # class variables, access as D415Camera.<variable>
    handle = None
    D415config = None
    streaming = False

    def __init__(self, name, deviceId, cols, rows, fovH, fovV, rotate, numReads):
        super().__init__(name, deviceId, cols, rows, fovH, fovV, rotate, numReads)

        # Configure depth and color streams
        if D415Camera.handle is None:
            try:
                D415Camera.handle = rs.pipeline()
                D415Camera.D415config = rs.config()
                D415Camera.D415config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
                D415Camera.D415config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
                config.log(f"{name} ready to stream")

            except Exception as e:
                config.log(f"could not initialize D415 cam, {e}")
                return

        if not D415Camera.streaming:
            self.startStream()

        if D415Camera.streaming:
            frames = D415Camera.handle.wait_for_frames()

            for i in range(self.numReads):
                rgb_frame = frames.get_color_frame()

            self.stopStream()


    def startStream(self):
        if not D415Camera.streaming:
            try:
                D415Camera.handle.start(D415Camera.D415config)
                D415Camera.streaming = True
                config.log(f"start D415 stream successful")

            except Exception as e:
                config.log(f"not able to start D415 stream: {e}")
                D415Camera.handle = None


    def stopStream(self):
        if D415Camera.streaming:
            D415Camera.handle.stop()
        D415Camera.streaming = False


    def takeImage(self, show=False):

        if not D415Camera.streaming:
            D415Camera.startStream()

        if not D415Camera.streaming:
            return None

        frames = D415Camera.handle.wait_for_frames()

        for i in range(self.numReads):
            colorFrame = frames.get_color_frame()

        if colorFrame is None:
            config.log(f"could not acquire rgb image from D415")
            colorImage = None
        else:
            colorImage = np.asanyarray(colorFrame.get_data())
            if show:
                cv2.imshow(f"D415 rgb", colorImage)
                cv2.waitKey(0)
                cv2.destroyAllWindows()

        return colorImage


    def takeDepth(self, show=False):

        if not D415Camera.streaming:
            D415Camera.startStream()

        if not D415Camera.streaming:
            return None

        decimate = rs.decimation_filter()
        decimate.set_option(rs.option.filter_magnitude, 3)

        frames = D415Camera.handle.wait_for_frames()

        depthFrame = frames.get_depth_frame()
        if depthFrame is None:
            config.log(f"could not acquire depth from D415")
            return None

        else:
            decimated = decimate.process(depthFrame)    # make 428x240x3 from 1280x720x3

            # Grab new intrinsics (may be changed by decimation)
            # depth_intrinsics = rs.video_stream_profile(depth_frame.profile).get_intrinsics()
            # w, h = depth_intrinsics.width, depth_intrinsics.height
            config.pc = rs.pointcloud()
            pointCloud = config.pc.calculate(decimated)

            # Pointcloud data to arrays
            verts = pointCloud.get_vertices()
            points = np.asanyarray(verts).view(np.float32).reshape(-1, 3)  # xzy [102720 <428x240>,3]

            if show:
                config.log(f"points available")

        return points


def savePointCloud(pc, filename):
    # save as pickle file for testing
    myFile = open(filename, 'wb')
    np.save(myFile, pc)
    myFile.close()


def loadPointCloud(filename):
    myFile = open(filename, 'rb')
    pc = np.load(myFile)
    myFile.close()
    return pc


def rotatePointCloud(verts, angle):

    # rotate over x-axis
    cosa = np.cos(np.radians(angle))
    sina = np.sin(np.radians(angle))

    # return rotated point cloud array
    return np.dot(verts, np.array([
        [1., 0, 0],
        [0, cosa, -sina],
        [0, sina, cosa]]))


def calculateCamXYZ(useHeadImu=True):
    '''
    The camera is mounted on the head with a downward pointing angle
    Y and Z of the cam depend on the neck setting (headImuPitch, headImuYaw)
    headYaw is assumed to be 0!
    :return:
    '''
    if not useHeadImu:
        # imageAngle: -80.38, camYXZ: (0, 0.26752425602438334, 1.5068158952609565)
        #return -68, (0, 0.267, 1.5)  # ground watch values
        return -45, (0,0.25,1.56) # wall watch values with headNeck -15

    while not config.headImuSet:
        config.log(f"wait for headImu")
        time.sleep(1)

    neckToCamDist = np.hypot(config.D415_Y, config.D415_Z)
    camPosAngle = np.degrees(np.arctan(config.D415_Z / config.D415_Y)) \
                  + config.headImuPitch

    camY = config.robotNeckY + np.cos(np.radians(camPosAngle)) * neckToCamDist
    camZ = config.robotBaseZ + config.robotNeckZ + np.sin(np.radians(camPosAngle)) * neckToCamDist

    camAngle = config.headImuPitch + config.D415_MountingPitch

    config.log(
        f"headcamOrientation: head pitch: {config.headImuPitch}, D415 angle: {camAngle:.0f}, camZ: {camZ:.3f}")

    return camAngle, (0, camY, camZ)


def alignPointsWithGround(rawPoints, headPitch, useHeadImu = True):
    '''
    raw points are taken in an angle
    rotate the point cloud to be aligned with the ground
    remove the ground and convert point height to above ground height
    raise below ground points to be an obstacle
    :return: aligned point cloud
    '''
    config.log(f"start align points with ground, headPitch: {headPitch:.0f}")

    # replace out of scope distance values with NaN
    with np.errstate(invalid='ignore'):
        rawPoints[:, 2][(rawPoints[:,2] < 0.2) |
                     (rawPoints[:,2] > 4)] = np.NaN

    #findObstacles.showSliceRaw(rawPoints, 208)

    # get image angle and cam xyz position
    imageAngle, camXYZ = calculateCamXYZ(useHeadImu)

    #showPoints(points,     # rotate points for a horizontal representation of the points
    rotatedPoints = rotatePointCloud(rawPoints, -90-imageAngle)
    config.log(f"rotation angle: {-90-imageAngle:.1f}")

    ####################################################
    # after rotation:
    # points[0] = left/right, center=0
    # points[1] = horizontal distance to point, row 0 farthest
    # points[2] = height of point above ground, in relation to cam height
    ####################################################

    #showPoints(rotatedPoints, 'wall image rotated')

    # subtract the cam height from the points
    rotatedPoints[:,2] = rotatedPoints[:,2] - camXYZ[2]

    # set distance positive
    rotatedPoints[:,1] = -rotatedPoints[:,1]

    # show a rotated slice as line
    #findObstacles.showSlice(rotatedPoints, 208)

    # create obstacles for below ground points, suppress runtime warnings caused by nan values
    with np.errstate(invalid='ignore'):
        rotatedPoints[:,2] = np.where(rotatedPoints[:,2] < - 0.1, 1, rotatedPoints[:,2])

    return rotatedPoints        # for each col/row point the xyz values



def stopD415Stream():
    config.log(f"request stop streaming from D415")
    try:
        config.streamD415.stop()
        config.D415streaming = False
        return True
    except Exception as e:
        config.log(f"stop stream failed: {e}")
        config.D415streaming = False
        return False

'''
def getDepth():

    if not config.D415streaming:
        if not startD415Stream():
            return None

    decimate = rs.decimation_filter()
    decimate.set_option(rs.option.filter_magnitude, 3)

    frames = config.streamD415.wait_for_frames()

    depth_frame = frames.get_depth_frame()
    depth_frame = decimate.process(depth_frame)

    # Grab new intrinsics (may be changed by decimation)
    # depth_intrinsics = rs.video_stream_profile(depth_frame.profile).get_intrinsics()
    # w, h = depth_intrinsics.width, depth_intrinsics.height
    config.pc = rs.pointcloud()
    points = config.pc.calculate(depth_frame)

    # Pointcloud data to arrays
    v = points.get_vertices()
    verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz [102720,3]

    return verts
'''

