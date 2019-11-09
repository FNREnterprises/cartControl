


import numpy as np
import pickle
import cv2
import time

import config
import depthImage

# x = left/right
# y = front/back
# z = depth

def initObstacleDetection():

    # try to load the registred cart front image
    try:
        pickleFile = open(r'cartFront.pickle', 'rb')
        config.cartFrontImage = pickle.load(pickleFile)     # boolean array
    except Exception as e:
        config.log(f'no cart front image found, current image is used to create a cart front image, {e}')


def camPosGroundOK(neckDegrees, rotheadDegrees):
    return abs(neckDegrees - config.pitchGroundWatchDegrees) < 3 and \
           abs(rotheadDegrees - config.yawGroundWatchDegrees) < 3


def camPosWallOK(neckDegrees, rotheadDegrees):
    return abs(neckDegrees - config.pitchWallWatchDegrees) < 3 and \
           abs(rotheadDegrees - config.yawWallWatchDegrees) < 3


def showObstacles(mat, title):
    '''
    shows an 2 dim boolean array as white for true, black for false
    :param mat:
    :param title:
    :return:
    '''
    img = np.zeros(mat.shape, np.uint8)     # create array
    img[mat] = 255                          # fill with white
    img[mat == False] = 0                   # set all points with False to black

    config.log(f"showObstacles: {title}")
    cv2.imshow(title, img)
    cv2.waitKey(1)
    #cv2.destroyAllWindows()


def numpy_ffill(arr):
    '''
    replace nan values from left to right with the preceeding value on the left
    (here row-wise) left to right
    50, nan, nan, 49 -> 50, 50, 50, 49
    '''
    mask = np.isnan(arr)
    idx = np.where(~mask,np.arange(mask.shape[1]),0)
    np.maximum.accumulate(idx,axis=1, out=idx)
    out = arr[np.arange(idx.shape[0])[:,None], idx]
    return out


def numpy_bfill(arr):
    '''
    replace  nan values right to left with the preceeding value on the right
    (here row-wise) right to left
    50, nan, nan, 49 -> 50, 49, 49, 49
    '''
    mask = np.isnan(arr)
    idx = np.where(~mask, np.arange(mask.shape[1]), mask.shape[0] + 1)
    idx = np.minimum.accumulate(idx[:, ::-1], axis=1)[:, ::-1]
    out = arr[np.arange(idx.shape[0])[:,None], idx]
    return out


def translateImage(img, translateX, translateY):
    '''
    shift image by x and y pix
    :param img:
    :param translateX:
    :param translateY:
    :return:
    '''
    M = np.float32([[1,0,translateX],[0,1,translateY]])
    return cv2.warpAffine(img, M, img.shape[:2])


def removeCartFront(image):
    '''
    When the cam points down to check for ground obstacles it sees the cart front too
    Use a registred image with the cart front to find a match with current picture and
    remove the cart front from the obstacle image
    This is needed because the cam is mounted on the inMoov head and the pitch angle
    is not guaranteed to be repeatable.
    '''

    # use the cartFront as mask and shift it over the current image
    # find minimum pixel count in combined image
    cfRows, cfCols = config.cartFrontImage.shape
    imgRows, imgCols= image.shape
    lowestSum = imgCols * imgRows          # initialize with max sum of pixels in image
    bestShift = None
    showObstacles(image, 'image')  # verification only

    # use only lower part of image for cart front search
    img1 = image[config.cartFrontMinRow:, :]

    # add the registered cartFront to the image at different locations
    for x in range(2 * config.cartFrontBorderX):         # left/right shift
        for y in range(imgRows - config.cartFrontMinRow - cfRows):   # top/down shift

            # add shifted cart front to img2
            img2 = np.zeros_like(img1, dtype=np.bool)
            img2[y:y+cfRows, x:x+cfCols] = np.bitwise_or(img2[y:y+cfRows, x:x+cfCols], config.cartFrontImage)
            #showObstacles(img2, 'shifted cartFront')  # verification only

            img3 = np.bitwise_or(img1, img2)
            #showObstacles(img3, 'overlay')  # verification only

            newSum = np.sum(img3)
            if newSum < lowestSum:
                lowestSum = newSum
                bestShift = img2
                config.cartFrontRowShift = y
                config.cartFrontColShift = x
                #config.log(f"lowestSum: {lowestSum}, shift: {y}")

    showObstacles(bestShift, 'bestShift')  # for verification only

    # for each col find the cart front
    rowCartFront = np.argmax(bestShift, axis=0) + config.cartFrontMinRow - config.cartFrontRowShift

    # create an image mask without the cart
    groundMask = np.ones_like(image, dtype=np.bool)

    # take out leftmost cols ...
    for col in range(0, config.cartFrontColShift):
        groundMask[:, col] = False

    # ... and pixels within the cart front range
    for col in range(config.cartFrontColShift, imgCols):
        groundMask[rowCartFront[col]:imgRows, col] = False

    showObstacles(groundMask, 'cart mask')  # for verification only

    ground = np.bitwise_and(image, groundMask)

    showObstacles(ground, 'ground without cart')  # for verification only

    # as argmax returns the index of the first True value flip the image
    np.flipud(ground)

    # for each column the closest y point with z(height) change
    rowObstacles = imgRows - np.argmax(ground, axis=0)

    return rowCartFront, rowObstacles


def obstacleDistances(xyz, camZ: float):
    """
    here in xyz
    x = left-right
    y = depth (drive direction)
    z = height
    :param xyz:     points
    :param camY:    height of cam over ground
    :return:        for each image col the closest obstacle
    """

    config.log(f"eval closest point per column", publish=False)

    # replace too close and too far points with NaN

    # TODO without knowing current arm/finger positions get these out of the way
    lowestArmZ = 0.92  #lowest reachable position with finger in relation to cam
    np.warnings.filterwarnings('ignore')

    # remove depth values above lowest finger Z
    # and below max depth
    xyz[:, :, 2][(xyz[:, :, 2] < lowestArmZ) | (xyz[:, :, 2] > 2.5)] = np.NaN
    #xyz[:, :, 2][(xyz[:, :, 2] > camY - 0.050) & (xyz[:, :, 2] < camY + 0.050)] = np.NaN

    # limit area in drive-direction
    # replace z(height) values outside y-range(drive direction) 0..1m with NaN
    xyz[:, :, 2][(xyz[:, :, 1] == 0.0) | (xyz[:,:,1] > 1)] = np.NaN

    # limit area to cart's width
    # replace z(height) values outside x-range(width) -0.3..0.3m (cart width) with NaN
    xyz[:, :, 2][(xyz[:, :, 0] < -0.3) | (xyz[:, :, 0] > 0.3)] = np.NaN

    heights = xyz[:,:,2]
    colsWithDepth = np.nanmax(heights, axis=0)
    firstDepthCol = np.where(~np.isnan(colsWithDepth))[0][0]
    lastDepthCol =  np.where(~np.isnan(colsWithDepth))[0][-1]

    # get row indices with non-nan-values
    rowsWithDepth = np.nanmax(heights, axis=1)
    # get the first row with a value
    firstDepthRow = np.where(~np.isnan(rowsWithDepth))[0][0] + 2

    reducedHeights = heights[firstDepthRow:,firstDepthCol:lastDepthCol]

    # We might have nan values in left and right area
    # replace nan heights with the left/right preceeding value
    forwardFilledHeights = numpy_ffill(reducedHeights)
    # do the reverse too (replace nan with the right preceeding value
    filledHeights = numpy_bfill(forwardFilledHeights)

    #filledHeights = np.concatenate((backwardFilledHeights[:,:100], forwardFilledHeights[:,101:]), axis=1)

    # leave out far away zone
    #filledHeights = filledHeights[firstDepthRow:,:]

    # calc the height changes in drive direction row wise
    depthChangeForward = np.diff(filledHeights[:, :], axis=1, append=np.NaN)

    # ... and set depthChanges < 0.020 m to False (not an obstacle)
    depthObstacleForward = np.full((depthChangeForward.shape), True, dtype=bool)
    depthObstacleForward[(abs(depthChangeForward) < 0.020) | (depthChangeForward == np.NaN)] = False

    #] calc the changes in left-right-direction column-wise
    depthChangeLeftRight = np.diff(filledHeights[:, :], axis=0, append=np.NaN)

    # ... and set depthChanges < 0.02 m to nan
    depthObstacleLeftRight = np.full((depthChangeForward.shape), True, dtype=bool)
    depthObstacleLeftRight[(abs(depthChangeLeftRight) < 0.020) | (depthChangeForward == np.NaN)] = False

    # for tests, add an obstacle
    #depthObstacleForward[100, 50:65] = True

    depthObstaclesAll = np.logical_or(depthObstacleLeftRight, depthObstacleForward)


    # check for existing cart front image

    if config.cartFrontImage is None:

        # if it does not exist create it from current image
        # in this case the current image is assumed to be free of obstacles (beside the cart itself)
        config.log(f"create new cart front")
        showObstacles(depthObstaclesAll, 'create new cartFront')        # for verification

        # use reduced range of image to allow for shifting of cart front over realtime image
        h, w = depthObstaclesAll.shape  # the height and width of the reference image

        refImg = depthObstaclesAll[config.cartFrontMinRow:h, config.cartFrontBorderX:w-config.cartFrontBorderX]
        h, w = refImg.shape           # the height and width of the reference image

        # for each column the first row showing the cart
        frontLine = np.nanargmax(refImg, axis=0)

        firstPixRow = h     # initialize to max
        for x in range(w):
            if frontLine[x] > 0 and frontLine[x] < firstPixRow:       # find min range of cart front
                firstPixRow = frontLine[x]                            # the first row with ground diff
            for y in range(h):                                        # clear pixels below front line
                if y > frontLine[x]:
                    refImg[y,x] = False

        # fatten up the line
        refImg = np.bitwise_or(refImg[:-1], refImg[1:])
        refImg = np.bitwise_or(refImg[1:], refImg[:-1])
        refImg = np.bitwise_or(refImg[:,:-1], refImg[:,1:])
        refImg = np.bitwise_or(refImg[:,1:], refImg[:,:-1])
        h, w = refImg.shape     # fatting the line reduced the img size

        showObstacles(refImg, 'front shape of refImg')

        # include only rows that represent the cart front
        lastPixRow = 0
        for testRow in range(firstPixRow, h-1):
            if np.sum(refImg[testRow]) > 0:
                lastPixRow = testRow

        firstPixRow = max(firstPixRow - 4, 0)   # add some head space
        refImg = refImg[firstPixRow:lastPixRow]
        h, w = refImg.shape

        config.log(f"firstPixRow: {firstPixRow}, lastPixRow: {lastPixRow}")

        if lastPixRow > firstPixRow:

            # create a filled cart front as mask
            # find first pix in each col
            cfStartRows = np.argmax(refImg, axis = 0)

            # and fill area below
            for col in range(w):
                if cfStartRows[col] > 0:
                    refImg[cfStartRows[col]:h, col] = True

            config.cartFrontImage = refImg
            showObstacles(config.cartFrontImage, 'cartFront for pickle')  # for verification only

            pickleFile = open(r'cartFront.pickle', 'wb')
            pickle.dump(config.cartFrontImage, pickleFile)
            pickleFile.close()


    # as the cam angle is dependent on the neck servo it is not over-accurate
    # try to find the cart front in the image by comparing it with the registred cart front shape
    # cartFront has the highest image row per column of the cart
    obstacles = removeCartFront(depthObstaclesAll)

    config.log(f"return obstacle array (closest obstacle in each col)", publish=False)
    return obstacles   # for each image column the closest obstacle


def rotatePointCloud(points, angle):

    verts = np.asanyarray(points).view(np.float32).reshape(-1, 3)  # xyz

    # rotate over x-axis
    cosa = np.cos(np.radians(angle))
    sina = np.sin(np.radians(angle))

    xyz = np.zeros_like(verts)
    xyz[:, 0] = verts[:, 0]
    xyz[:, 1] = -(verts[:, 1] * cosa - verts[:, 2] * sina)
    xyz[:, 2] = verts[:, 1] * sina + verts[:, 2] * cosa
    xyz = xyz.reshape(240, 428, 3)

    # save as pickle file for testing
    file = "xyzGround.pickle"
    myFile = open(r'C:\Projekte\InMoov\realsense\pc.pickle', 'wb')
    pickle.dump(xyz, myFile)
    myFile.close()

    return xyz  # rotated point cloud giving vertical distances


def distGroundObstacles():

    # having a connection verify head neck position
    neckDegrees, _, _ = config.servoControlConnection.root.exposed_getPosition("head.neck")
    rotheadDegrees, _, _ = config.servoControlConnection.root.exposed_getPosition("head.rothead")

    # if neck is not in ground observation position request neck position
    numTries = 0
    while not camPosGroundOK(neckDegrees, rotheadDegrees) and numTries < 3:
        numTries += 1
        config.servoControlConnection.root.exposed_requestServoDegrees("head.neck", config.pitchGroundWatchDegrees, 1)
        config.servoControlConnection.root.exposed_requestServoDegrees("head.rothead", config.yawGroundWatchDegrees, 1)
        time.sleep(1)
        neckDegrees, _, _ = config.servoControlConnection.root.exposed_getPosition("head.neck")
        rotheadDegrees, _, _ = config.servoControlConnection.root.exposed_getPosition("head.rothead")

    if not camPosGroundOK(neckDegrees, rotheadDegrees):
        config.log(f"could not move head into ground watch position, neck: {neckDegrees:0f}, rothead: {rotheadDegrees:0f}")
        return False

    else:

        points = depthImage.getDepth()

        if points is None:
            config.log(f"could not get image from D415")
            return False

        depth = rotatePointCloud(points, camAngle=16)

        camZ = 1.48
        rowCartFront, rowObstacle = obstacleDistances(depth, camZ)

        # the start column in the total image of the cart front
        xMin = int((depth.shape[0] - rowCartFront.shape[0]) / 2)

        # get the y difference of cartFront and obstacle
        distAndPos = \
            [(depth[rowObstacle[d], d + xMin, 1] - depth[rowCartFront[d], d + xMin, 1],
              depth[rowObstacle[d], d + xMin, 0],
              depth[rowObstacle[d], d + xMin, 1])
             for d in range(rowCartFront.shape[0])]

        posDist = [d[0] if d[0] > 0 else 1 for d in distAndPos]

        distClosestObstacle = np.amin(posDist)
        xClosestObstacle = distAndPos[np.argmin(posDist)][1]
        yClosestObstacle = distAndPos[np.argmin(posDist)][2]

        config.log(f"distClosestObstacle: {distClosestObstacle:.3f}, x: {xClosestObstacle:.3f}, y: {yClosestObstacle:.3f}")
        obstacleDirection = np.degrees(np.arctan2(xClosestObstacle, yClosestObstacle))

        return distClosestObstacle, obstacleDirection


def distWall():
    '''
    raise head and take a depth image
    find closest point above ground in path and shape of robot
    to simplify use a rectangle for the robot shape
    For a start use a "move pose" of the robot to avoid collisions with an outstreched hand with the wall.
    :return:
    distanceArray robot/wall
    '''

    # having a connection verify head neck position
    neckDegrees, _, _ = config.servoControlConnection.root.exposed_getPosition("head.neck")
    rotheadDegrees, _, _ = config.servoControlConnection.root.exposed_getPosition("head.rothead")

    # if neck is not in wall observation position request neck position
    numTries = 0
    while not camPosWallOK(neckDegrees, rotheadDegrees) and numTries < 3:
        numTries += 1
        config.servoControlConnection.root.exposed_requestServoDegrees("head.neck", config.pitchGroundWatchDegrees, 1)
        config.servoControlConnection.root.exposed_requestServoDegrees("head.rothead", config.yawGroundWatchDegrees, 1)
        time.sleep(1)
        neckDegrees, _, _ = config.servoControlConnection.root.exposed_getPosition("head.neck")
        rotheadDegrees, _, _ = config.servoControlConnection.root.exposed_getPosition("head.rothead")

    if not camPosWallOK(neckDegrees, rotheadDegrees):
        config.log(f"could not move head into wall watch position, neck: {neckDegrees:0f}, rothead: {rotheadDegrees:0f}")
        return False

    else:

        depth = depthImage.getDepth()

        if depth is None:
            config.log(f"could not get image from D415")
            return False

        camZ = 1.48
