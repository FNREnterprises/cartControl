


import numpy as np
import cv2

import inmoovGlobal
import config
import camImages

# x = left/right
# y = front/back
# z = depth


def crop_boolMat(boolMat):
    # img is 2D array with true/false
    # remove false only rows and cols from array
    coords = np.argwhere(boolMat)
    x_min, y_min = coords.min(axis=0)
    x_max, y_max = coords.max(axis=0)
    return boolMat[x_min:x_max + 1, y_min:y_max + 1]


def initObstacleDetection():

    # try to load the registred cart front image
    try:
        with open(r'cartFrontLine.npy', 'rb') as npFile:
            config.cartFrontLine = np.load(npFile)     # boolean array
            createCartFrontShape()
    except Exception as e:
        config.log(f'no cart front line found, current image is used to create a cart front line, {e}')
        return


def createCartFrontShape():

    # create the cart front shape from the cart front line data
    rows = np.amax(config.cartFrontLine)
    cols = config.cartFrontLine.shape[0]
    config.cartFrontShape = np.zeros((rows+1, cols+1), dtype=np.bool)
    for c in range(cols):
        if config.cartFrontLine[c] > 0:
            config.cartFrontShape[rows - config.cartFrontLine[c], c] = True

    #showBoolMat(config.cartFrontShape, "cart front shape", (100,0))


def showBoolMat(mat: np.ndarray, title: str, pos: tuple = (100, 100)):
    '''
    shows an 2 dim boolean array as white for true, black for false
    :param mat:
    :param title:
    :return:
    '''
    img = np.zeros(mat.shape, np.uint8)     # create array
    img[mat] = 255                          # fill with white
    img[mat == False] = 0                   # set all points with False to black

    config.log(f"showObstacles: {title} at {pos}")
    cv2.imshow(title, img)
    cv2.moveWindow(title, pos[0], pos[1])
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
    idx = np.where(~mask, np.arange(mask.shape[1]), mask.shape[1] - 1)
    idx = np.minimum.accumulate(idx[:, ::-1], axis=1)[:, ::-1]      # ::-1 reverses array
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


def removeCartFront(groundImage):
    """
    use only upper area of image to find first obstacle
    this should avoid the cart front to be seen as obstacle

    this is a simpler version of the more detailed cart front removal removeCartFrontDetailed
    (which never worked so far ...)
    """
    rows, cols = groundImage.shape
    upperArea = groundImage[0:rows-30]

    # as argmax returns the index of the first True value flip the image
    np.flipud(upperArea)
    #showBoolMat(upperArea, "flipped upper area")

    # for each column the closest y point with z(height) change
    rowObstacles = upperArea.shape[0] - np.argmax(upperArea, axis=0) + 30
    rowCartFront = np.full(cols, 30)

    return rowCartFront, rowObstacles

'''
def removeCartFrontDetailed(image, firstDepthRow):
    """
    When the cam points down to check for ground obstacles it sees the cart front too
    Use a registred image with the cart front to find a match with current picture and
    remove the cart front from the obstacle image
    This is needed because the cam is mounted on the inMoov head and the pitch angle
    is not guaranteed to be repeatable.
    """

    # use the cartFront as mask and shift it over the current image
    # find minimum pixel count in combined image
    cfRows, cfCols = config.cartFrontShape.shape
    imgRows, imgCols= image.shape

    config.log(f"cf: {config.cartFrontShape.shape}, img: {image.shape}")

    lowestSum = imgCols * imgRows          # initialize with max sum of pixels in image
    bestShiftImg = None
    #showObstacles(image, 'ground image')  # verification only

    # use only lower part of image for cart front search
    img1 = image[(imgRows - config.cartFrontSearchRows):, :]
    #showObstacles(img1, 'front search area')  # verification only

    # add the registered cartFront to the image at different locations
    for x in range(2 * config.cartFrontBorderX):         # left/right shift
        for y in range(config.cartFrontRows - cfRows):   # top/down shift

            # add shifted cart front to img2
            img2 = np.zeros_like(img1, dtype=np.bool)
            #config.log(f"y: {y}, y+cfRows: {y+cfRows}, x: {x}, x+cfCols: {x+cfCols}, img2.shape: {img2.shape}")
            img2[y:y+cfRows, x:x+cfCols] = np.bitwise_or(img2[y:y+cfRows, x:x+cfCols], config.cartFrontShape)
            #showObstacles(img2, 'shifted cartFront')  # verification only

            img3 = np.bitwise_or(img1, img2)
            showBoolMat(img3, 'overlay')  # verification only

            newSum = np.sum(img3)
            if newSum < lowestSum:
                # save the best fitting cart front offset
                lowestSum = newSum
                bestShiftImg = img2
                config.cartFrontRowShift = y
                config.cartFrontColShift = x
                #config.log(f"lowestSum: {lowestSum}, shift: {y}")

    #showObstacles(bestShiftImg, 'bestShiftImg', (50,200))  # for verification only

    # for each col set the cart front start row
    #rowCartFront = np.argmax(bestShiftImg, axis=0) + imgRows - config.cartFrontRows + config.cartFrontRowShift
    startCol = config.cartFrontColShift
    endRow = config.cartFrontRows + config.cartFrontRowShift + config.cartFrontShape.shape[0]
    for col in range(config.cartFrontLine.shape[0]):
        rowCartFront[col+startCol] = -config.cartFrontLine[col] + endRow  # the distance per col of the cart front

    # create a ground image mask without the cart
    groundMask = np.ones_like(image, dtype=np.bool)

    # take out leftmost cols ...
    for col in range(0, config.cartFrontColShift):
        groundMask[:, col] = False

    # ... and pixels within the cart front range
    for col in range(config.cartFrontColShift, imgCols):
        groundMask[rowCartFront[col]:imgRows, col] = False

    #showBoolMat(groundMask, 'cart mask', (100, 100))  # for verification only

    ground = np.bitwise_and(image, groundMask)

    showBoolMat(ground, 'ground without cart', (100, 300))  # for verification only

    # as argmax returns the index of the first True value flip the image
    np.flipud(ground)

    # for each column the closest y point with z(height) change
    rowObstacles = imgRows - np.argmax(ground, axis=0)

    return rowCartFront, rowObstacles
'''


def groundObstacles(xyz):
    """
    here in xyz
    x = left-right
    y = dist (drive direction)
    z = vertical height above ground
    find changes in height in x and y direction and mark changes > 20mm as obstacles
    :param xyz:     points
    :return:        for each image col the first obstacle row
    """

    HEIGHT_THRESHOLD = 0.025

    config.log(f"eval closest obstacle per column", publish=False)

    # the points in the bird view image (rotated points)
    rows, cols, _ = xyz.shape

    # we are interested in the height values of the ground points only
    origHeights = xyz[:,:,2]

    # the cam delivers the first row as the one farthest away from the cart
    # flip the rows
    heights = np.flipud(origHeights)

    # skip the cart front from the heights
    firstDepthRow = 30

    # from height 1.50 m one row represents around 5 mm ground space
    # check for obstacles in the range of 60 cm ahead of the cart
    numDepthRows = int(0.6 / 0.005)
    lastDepthRow = firstDepthRow + numDepthRows

    # limit cols to the robots width
    # from height 1.50 m one col represents around 4.5 mm ground space
    robotCols = int(config.robotWidth / 0.0045)
    firstDepthCol = int((cols + robotCols) / 2)
    lastDepthCol = firstDepthCol + robotCols


    limitedHeightsArea = heights[firstDepthRow:lastDepthRow,firstDepthCol:lastDepthCol]
    rows, cols = limitedHeightsArea.shape

    # We might have nan values as height
    # replace nan heights from left to right, then right to left with the preceeding value
    limitedHeightsArea[:,0] = np.NaN             # set leftmost col to nan
    limitedHeightsArea[:,cols-1] = np.NaN        # set rightmost col to nan

    leftToRightFilledHeights = numpy_ffill(limitedHeightsArea)
    # do the reverse too (replace nan with the right preceeding value
    filledHeights = numpy_bfill(leftToRightFilledHeights)

    # calc the height changes in drive direction row wise
    groundChangeForward = np.diff(filledHeights, axis=0, append=np.NaN)

    # ... and set depthChanges < 0.020 m to False (not an obstacle)
    with np.errstate(invalid='ignore'):
        groundObstacleForward = np.full((groundChangeForward.shape), True, dtype=bool)
        groundObstacleForward[(abs(groundChangeForward) < HEIGHT_THRESHOLD)] = False

    # set bottom most row to False
    groundObstacleForward[-1] = False

    #showBoolMat(groundObstacleForward,"height change forward")

    # calc the changes in left-right-direction column-wise
    groundChangeLeftRight = np.diff(filledHeights, axis=1, append=np.NaN)

    # ... and set depthChanges < 0.02 m to nan
    with np.errstate(invalid='ignore'):
        groundObstacleLeftRight = np.full((groundChangeLeftRight.shape), True, dtype=bool)
        groundObstacleLeftRight[(abs(groundChangeLeftRight) < 0.020)] = False

    # set right most col to False
    groundObstacleLeftRight[:,-1] = False

    #showBoolMat(groundObstacleLeftRight,"height change left/right")

    # combine forward/side bottom changes into an obstacle map
    groundObstaclesAll = np.logical_or(groundObstacleLeftRight, groundObstacleForward)

    #showBoolMat(groundObstaclesAll, 'all height changes')  # for verification

    groundObstaclesAll[50, 100] = True  # simulate a single pixle obstacle

    # filter out single pixels
    filteredObstacles = cv2.blur(groundObstaclesAll.astype(np.uint8), (3,3))

    #showBoolMat(filteredObstacles.astype(bool), 'filtered obstacles')  # for verification
    '''
    # check for existing cart front definition
    if config.cartFrontLine is None:

        # if it does not exist create it from current image
        # --> in this case the current image is assumed to be free of obstacles (beside the cart itself)
        config.log(f"create new cart front")

        showBoolMat(groundObstaclesAll, 'create new cartFrontShape', (100,400))        # for verification

        # use a limited image size for the cart front to allow shifting it over later images
        # use only area in front of cart
        rows, cols = groundObstaclesAll.shape
        frontShape = groundObstaclesAll[rows-config.cartFrontRows:,config.cartFrontBorderX:-config.cartFrontBorderX]

        # fatten up the line in left/right direction (closing gaps)
        frontShape = np.bitwise_or(frontShape[:,:-1], frontShape[:,1:])
        frontShape = np.bitwise_or(frontShape[:,1:], frontShape[:,:-1])

        # remove all false outside rows and cols, drop last row
        cropped = crop_boolMat(frontShape[:-1])

        # for each column the number of rows at the image bottom (highest row) to remove the cart front
        config.cartFrontLine = cropped.shape[0] - np.nanargmax(cropped, axis=0) - 1


        with open(r'cartFrontLine.npy', 'wb') as npFile:
            np.save(npFile, config.cartFrontLine)

        # create the cartFrontShape for shifting it over this image
        createCartFrontShape()

        showBoolMat(config.cartFrontShape, 'new cart front shape')      # for verification only

    # as the cam angle is dependent on the neck servo it is not over-accurate
    # try to find the cart front in the image by comparing it with the registred cart front shape
    # cartFrontShape has the highest image row per column of the cart

    #obstacles = removeCartFrontDetailed(groundObstaclesAll)        # TODO
    '''

    config.log(f"return obstacle array (closest obstacle in each col)", publish=False)
    w, h = filteredObstacles.shape
    obstacleRows = np.zeros(w)
    filteredObstacles[:,h-1] = 255        # set top row of each col as obstacle
    for col in range(w):
        obstacleRows[col] = np.argmax(filteredObstacles[col,:])

    return obstacleRows   # for each image column the first row with an obstacle


def showSliceRaw(pc, slice):

    # show a slice as line
    p = pc.reshape(240,428,3)

    h,w = 200,300
    img = np.zeros((h,w))
    for row in range(240):
        try:
            x = int(p[row, 208, 2] / 0.02)   #dist
            y = h+int(p[row, 208, 1] / 0.02) - 50 #height, use 0,150 as zero point
        except:
            y=0
            x=0
        if 0 < x < 300 and 0 < y < 200:
            img[y,x] = 255
    cv2.imshow(f"slice {slice}", img)
    cv2.waitKey(500)
    cv2.destroyAllWindows()

def showSlice(pc, slice):

    # show a slice as line
    p = pc.reshape(240,428,3)

    img = np.zeros((200,300))
    for row in range(240):
        if not np.isnan(p[row, 208, 1]):
            x = int(p[row, 208, 1] / 0.02)   #dist
            y = 150 + int(p[row, 208, 2] / 0.02) #height, use 0,150 as zero point
        else:
            y=0
            x=0
        if 0 < x < 200 and 0 < y < 300:
            img[y,x] = 255
    cv2.imshow(f"slice {slice}", img)
    cv2.waitKey(500)
    cv2.destroyAllWindows()


def showPoints(p, title):

    import matplotlib.pyplot as plt
    import mpl_toolkits.mplot3d.axes3d as p3    # needed for projection='3d'

    ax=plt.axes(projection='3d')
    xList = [x[0] for x in p]
    yList = [y[1] for y in p]
    zList = [z[2] for z in p]
    ax.scatter3D(xList, yList, zList, c=zList, cmap='Greens')

    ax.set_xlabel('X-0')
    ax.set_ylabel('Y-1')
    ax.set_zlabel('Z-2')
    ax.set_title(title)

    plt.show()


def distGroundObstacles(points):
    '''
    for ground obstacles use the height differences between points and not the absolute distance to cam
    :param points:
    :return:
    '''
    # replace out of scope points with NaN
    # limit to robot's width
    with np.errstate(invalid='ignore'):
        points[:,0] [(points[:,0] < -config.robotWidth2) | (points[:,0] > config.robotWidth2)] = np.NaN
    #showPoints(groundPoints, 'orig')

    # TODO without knowing current arm/finger positions get these out of the way
    # remove depth values above lowest arm Z
    lowestArmZ = 0.5  # lowest reachable position with finger in relation to ground
    with np.errstate(invalid='ignore'):
        points[:,2][(points[:,2] > lowestArmZ)] = np.NaN

    #showPoints(xyz, 'above ground')

    # limit area in drive-direction
    # replace z(height) values outside y-range(drive direction) 0.5m with NaN
    #xyz[:,2][(xyz[:,1] > 0.6)] = np.NaN

    #showPoints(points, 'cartFront')

    points = points.reshape(240, 428, 3)

    obstacleRows = groundObstacles(points)


    # at height 1.6, FOV Vertical = 42.5, 240 Rows one row accounts for about 5 mm
    # at height 1.6, FOV Horizontal = 69.4, 428 Cols one col accounts for about 4.5 mm

    distClosestObstacle = np.amin(obstacleRows) * 0.005
    xClosestObstacle = np.argmin(obstacleRows)  * 0.0045 - config.robotWidth2

    config.log(f"distClosestObstacle: {distClosestObstacle:.3f}, x: {xClosestObstacle:.3f}")
    obstacleDirection = np.degrees(np.arctan2(xClosestObstacle, distClosestObstacle))

    return distClosestObstacle, obstacleDirection


def lookForCartPathObstacle(points):
    """
    this uses the cart's width to find closest obstacle in path
    it looks at the points using the xyz position
    :param points:
    :return:
    """
    ####################################################
    # points[0] = left/right, center=0
    # points[1] = horizontal distance to point
    # points[2] = height of point above ground
    ####################################################
    # reduce the points to the carts path
    with np.errstate(invalid='ignore'):     # suppress nan warnings
        points[:, 2][(points[:,0] < -config.robotWidth2) |
                     (points[:,0] > config.robotWidth2) |
                     (points[:,1] < 0.2) |
                     (points[:,1] > 2)] = np.NaN

    # remove the floor (reduces the points, col+row order of cam image not valid anymore)
    with np.errstate(invalid='ignore'):     # suppress nan warnings
        obstaclePoints = points[(points[:, 2] > 0.1)]

    #showPoints(obstaclePoints, "without floor")

    # create x-slices based on points x-position [:0] find closest point [:1] in each slice
    robotWidthCm = int(config.robotWidth * 100)
    robotWidthCm2 = int(robotWidthCm / 2)
    obstacleDist = np.zeros(robotWidthCm)
    obstacleMap = np.zeros((robotWidthCm, 201), dtype=np.bool)
    for col in range(robotWidthCm):
        xStart = (col - robotWidthCm2) / 100

        # for each x-cm the distances to the obstacles
        xSlice = obstaclePoints[(obstaclePoints[:, 0] >= xStart) & (obstaclePoints[:, 0] < xStart + 0.01)]

        # from the cm slice the closest distance
        obstacleDist[col] = np.amin(xSlice[:,1]) if len(xSlice) > 0 else 2

        # set the obstacle points in the map
        obstacleMap[col, int(obstacleDist[col]*100)] = True

    # finally find closest obstacle in robot path to decide for stopping or continuing drive
    freeMoveDist = np.amin(obstacleDist) - config.distOffsetCamFromCartFront
    xPos = (np.argmin(obstacleDist) - robotWidthCm2) / 100
    obstacleDirection = 90 - np.degrees(np.arctan2(freeMoveDist, xPos))

    return freeMoveDist, obstacleDirection


def findObstacleLine(points, show=True):
    """
    for each col from the cam depth image eval the closest obstacle ignoring floor and above head obstacles
    :return:
    """
    ####################################################
    # points[0] = left/right, center=0
    # points[1] = horizontal distance to point
    # points[2] = height of point above ground
    ####################################################

    # ultra fast!! inPathPoints = pc[(pc[:, 0] > -0.5) & (pc[:, 0] < 0.5) & (pc[:, 2] > 0.2) & (pc[:, 2] < 2)]
    # set points outside scope to nan
    with np.errstate(invalid='ignore'):
        points[:, 2][(points[:,1] < 0.2) |          # point too close
                     (points[:,1] > 4) |            # point too far away
                     (points[:,2] < 0.1) |          # point in ground area
                     (points[:,2] > 1.8)] = np.NaN  # point above robots head

        #map = points.reshape(240,428,3)

        # we are interested in the first 4 meters distance
        # calc width with fovH depthCam at 4 meters
        fovH = camImages.cams[inmoovGlobal.HEAD_CAM].fovH
        xRange = np.tan(np.radians(fovH/2)) * 4 * 2

        # use 2cm slices on x-axis
        xSlices = int(xRange/0.02)

        # array for first obstacle in x-slice
        obstacleLine = np.full(xSlices, np.NaN)

        for slice in range(xSlices):

            # get all points with x-position in x-slice
            xStart = (slice * 0.02) - (xSlices/2 * 0.02)
            xEnd = xStart + 0.02
            sliceObstacleDistances = points[:,1][(points[:,0] >= xStart) & (points[:,0] < xEnd) & (points[:,2] > 0.1)]

            # check for found points and eval the closest one
            if len(sliceObstacleDistances) > 0:
                obstacleLine[slice] = np.nanmin(sliceObstacleDistances)

        if show:
            # draw a map in 2cm resolution
            depthRows = int(4/0.02)
            img = np.zeros((depthRows, xSlices))
            for col, dist in enumerate(obstacleLine):
                if not np.isnan(dist):
                    img[depthRows - int(dist/0.02),col] = 255

            cv2.imshow("obstacle line", img)
            cv2.waitKey()
            cv2.destroyAllWindows()

    return obstacleLine





