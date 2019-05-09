import time
import collections
import tkinter as tk
import numpy as np

import arduinoSend
import cartControl
import config

CANV_WIDTH = 300
CANV_HEIGHT = 500
''' from fahren.cpp in MotorizedBase
todo: pass these values from cartControl to the Arduino on startup
#define BODEN_NAH_MIN  9
#define BODEN_NAH_MAX  20
#define BODEN_FERN_MIN  17
#define BODEN_FERN_MAX  28
'''

LINE_SCALE_SHORT = 2.0
LINE_SCALE_LONG = 1.5
SHORT_ARC_BOX = (config.SHORT_RANGE_MAX + config.SHORT_RANGE_MIN) / 2 * LINE_SCALE_SHORT
LONG_ARC_BOX = (config.LONG_RANGE_MAX + config.LONG_RANGE_MIN) / 2 * LINE_SCALE_LONG

# use named tupels to make the base positions available
# e.g.  reference by: servos[0].id
location = collections.namedtuple('Location', 'id x y x1 y1 x2 y2 arcFrom arcLength arcWidth')
sensors = []

# front left
X = CANV_WIDTH / 4
Y = CANV_HEIGHT / 4
sensors.append(location(id=0, x=X, y=Y,
                        x1=X - SHORT_ARC_BOX, y1=Y - SHORT_ARC_BOX,
                        x2=X + SHORT_ARC_BOX, y2=Y + SHORT_ARC_BOX,
                        arcFrom=0, arcLength=180,
                        arcWidth=(config.SHORT_RANGE_MAX - config.SHORT_RANGE_MIN) * LINE_SCALE_SHORT))
Y = CANV_HEIGHT / 4 - 40
sensors.append(location(id=1, x=X, y=Y,
                        x1=X - LONG_ARC_BOX, y1=Y - LONG_ARC_BOX,
                        x2=X + LONG_ARC_BOX, y2=Y + LONG_ARC_BOX,
                        arcFrom=0, arcLength=180,
                        arcWidth=(config.LONG_RANGE_MAX - config.LONG_RANGE_MIN) * LINE_SCALE_LONG))

# front right
X = CANV_WIDTH / 4 * 3
Y = CANV_HEIGHT / 4
sensors.append(location(id=2, x=X, y=Y,
                        x1=X - SHORT_ARC_BOX, y1=Y - SHORT_ARC_BOX,
                        x2=X + SHORT_ARC_BOX, y2=Y + SHORT_ARC_BOX,
                        arcFrom=0, arcLength=180,
                        arcWidth=(config.SHORT_RANGE_MAX - config.SHORT_RANGE_MIN) * LINE_SCALE_SHORT))
Y = CANV_HEIGHT / 4 - 40
sensors.append(location(id=3, x=X, y=Y,
                        x1=X - LONG_ARC_BOX, y1=Y - LONG_ARC_BOX,
                        x2=X + LONG_ARC_BOX, y2=Y + LONG_ARC_BOX,
                        arcFrom=0, arcLength=180,
                        arcWidth=(config.LONG_RANGE_MAX - config.LONG_RANGE_MIN) * LINE_SCALE_LONG))

# left
X = CANV_WIDTH / 4
Y = CANV_HEIGHT / 2
sensors.append(location(id=4, x=X, y=Y,
                        x1=X - SHORT_ARC_BOX, y1=Y - SHORT_ARC_BOX,
                        x2=X + SHORT_ARC_BOX, y2=Y + SHORT_ARC_BOX,
                        arcFrom=90, arcLength=180,
                        arcWidth=(config.SHORT_RANGE_MAX - config.SHORT_RANGE_MIN) * LINE_SCALE_SHORT))

# right
X = CANV_WIDTH / 4 * 3
Y = CANV_HEIGHT / 2
sensors.append(location(id=5, x=X, y=Y,
                        x1=X - SHORT_ARC_BOX, y1=Y - SHORT_ARC_BOX,
                        x2=X + SHORT_ARC_BOX, y2=Y + SHORT_ARC_BOX,
                        arcFrom=270, arcLength=180,
                        arcWidth=(config.SHORT_RANGE_MAX - config.SHORT_RANGE_MIN) * LINE_SCALE_SHORT))

# back left
X = CANV_WIDTH / 4
Y = CANV_HEIGHT / 4 * 3
sensors.append(location(id=6, x=X, y=Y,
                        x1=X - SHORT_ARC_BOX, y1=Y - SHORT_ARC_BOX,
                        x2=X + SHORT_ARC_BOX, y2=Y + SHORT_ARC_BOX,
                        arcFrom=180, arcLength=180,
                        arcWidth=(config.SHORT_RANGE_MAX - config.SHORT_RANGE_MIN) * LINE_SCALE_SHORT))
Y = CANV_HEIGHT / 4 * 3 + 40
sensors.append(location(id=7, x=X, y=Y,
                        x1=X - LONG_ARC_BOX, y1=Y - LONG_ARC_BOX,
                        x2=X + LONG_ARC_BOX, y2=Y + LONG_ARC_BOX,
                        arcFrom=180, arcLength=180,
                        arcWidth=(config.LONG_RANGE_MAX - config.LONG_RANGE_MIN) * LINE_SCALE_LONG))

# back right
X = CANV_WIDTH / 4 * 3
Y = CANV_HEIGHT / 4 * 3
sensors.append(location(id=8, x=X, y=Y,
                        x1=X - SHORT_ARC_BOX, y1=Y - SHORT_ARC_BOX,
                        x2=X + SHORT_ARC_BOX, y2=Y + SHORT_ARC_BOX,
                        arcFrom=180, arcLength=180,
                        arcWidth=(config.SHORT_RANGE_MAX - config.SHORT_RANGE_MIN) * LINE_SCALE_SHORT))

Y = CANV_HEIGHT / 4 * 3 + 40
sensors.append(location(id=9, x=X, y=Y,
                        x1=X - LONG_ARC_BOX, y1=Y - LONG_ARC_BOX,
                        x2=X + LONG_ARC_BOX, y2=Y + LONG_ARC_BOX,
                        arcFrom=180, arcLength=180,
                        arcWidth=(config.LONG_RANGE_MAX - config.LONG_RANGE_MIN) * LINE_SCALE_LONG))

# global variables
gui = None
controller = None


class manualControl:
    def __init__(self, mainWindow):

        self.w = mainWindow

        self.canvas = tk.Canvas(mainWindow, width=CANV_WIDTH, height=CANV_HEIGHT, background='white')
        self.canvas.grid(row=0, column=1)
        self.canvas.create_text(sensors[0].x, sensors[0].y + 25, text="Front Left")
        self.canvas.create_text(sensors[2].x, sensors[2].y + 25, text="Front Right")
        self.canvas.create_text(sensors[4].x + 40, sensors[4].y, text="Left")
        self.canvas.create_text(sensors[5].x - 40, sensors[5].y, text="Right")
        self.canvas.create_text(sensors[6].x, sensors[6].y - 25, text="Back Left")
        self.canvas.create_text(sensors[8].x, sensors[8].y - 25, text="Back Right")

        frame = tk.Frame(self.w)
        frame.grid(row=0, column=0, sticky='n')

        self.lblInfo = tk.Label(frame, text="wait for Arduino button pressed", fg="white", bg="red")
        self.lblInfo.grid(row=1, columnspan=2)
        self.lblInfo.configure(text="waiting for cart ready message", bg="white smoke", fg="orange")
        #self.btnArduino.configure(state="disabled")

        # informational elements
        ##########################
        self.lblDistanceObstacle = tk.Label(frame, text="distance to obstacle: ")
        self.lblDistanceObstacle.grid(row=20, column=0, sticky=tk.E)

        self.lblDistanceObstacleValue = tk.Label(frame, text=" ", fg="white", bg="red")
        self.lblDistanceObstacleValue.grid(row=20, column=1)

        self.lblDistanceAbyss = tk.Label(frame, text="max distance value: ")
        self.lblDistanceAbyss.grid(row=25, column=0, sticky=tk.E)

        self.lblDistanceAbyssValue = tk.Label(frame, text=" ", fg="white", bg="red")
        self.lblDistanceAbyssValue.grid(row=25, column=1)

        self.lblCommand = tk.Label(frame, text="command: ")
        self.lblCommand.grid(row=30, column=0, sticky=tk.E)

        self.lblCommandValue = tk.Label(frame, text=" ")
        self.lblCommandValue.grid(row=30, column=1)

        self.lblRotationTarget = tk.Label(frame, text="target yaw: ")
        self.lblRotationTarget.grid(row=35, column=0, sticky=tk.E)

        self.lblRotationTargetValue = tk.Label(frame, text=" ")
        self.lblRotationTargetValue.grid(row=35, column=1)

        self.lblRotationCurrent = tk.Label(frame, text="current yaw/x/y: ")
        self.lblRotationCurrent.grid(row=40, column=0, sticky=tk.E)

        self.lblRotationCurrentValue = tk.Label(frame, text=" ")
        self.lblRotationCurrentValue.grid(row=40, column=1)

        # rotate
        ######################################
        self.separator1 = tk.Frame(frame, height=3, bd=2, relief=tk.SUNKEN)
        self.separator1.grid(row=45, column=0, columnspan=2, pady=10, sticky="we")

        self.lblRotationHelp1 = tk.Label(frame, text="+ counterclock")
        self.lblRotationHelp1.grid(row=50, column=0, sticky=tk.E)

        self.sbRotation = tk.Spinbox(frame, from_=-90, to=90, width=3)
        self.sbRotation.grid(row=50, column=1, padx=10, pady=0, sticky=tk.W)
        self.sbRotation.delete(0, "end")
        self.sbRotation.insert(0, 30)

        self.lblRotationHelp2 = tk.Label(frame, text="- clockwise")
        self.lblRotationHelp2.grid(row=52, column=0, sticky=tk.E)

        self.btnRotate = tk.Button(frame, text="Rotate", state="disabled", command=self.rotateCart)
        self.btnRotate.grid(row=52, column=1, padx=10, sticky=tk.W)

        # move
        #########################################
        self.separator2 = tk.Frame(frame, height=3, bd=2, relief=tk.SUNKEN)
        self.separator2.grid(row=59, column=0, columnspan=2, pady=10, sticky="we")

        self.lblDistValue = tk.Label(frame, text="Distance [mm]: ")
        self.lblDistValue.grid(row=60, column=0, pady=0, sticky=tk.E)

        self.sbDist = tk.Spinbox(frame, from_=0, to=4000, width=4)
        self.sbDist.grid(row=60, column=1, padx=10, sticky=tk.W)
        self.sbDist.insert(0, 20)

        self.lblSpeedValue = tk.Label(frame, text="Speed: ")
        self.lblSpeedValue.grid(row=62, column=0, pady=0, sticky=tk.E)

        self.sbSpeed = tk.Spinbox(frame, from_=0, to=250, width=3)
        self.sbSpeed.grid(row=62, column=1, padx=10, sticky=tk.W)
        self.sbSpeed.insert(0, 18)

        self.choices = ["stop", "forward", "for_diag_right", "for_diag_left", "left", "right", "backward",
                        "back_diag_right", "back_diag_left"]
        defaultDirection = "forward"
        self.direction = self.choices.index(defaultDirection)

        move = tk.StringVar(gui)
        move.set(defaultDirection)
        self.ddMove = tk.OptionMenu(frame, move, *self.choices, command=self.selectedDirection)
        self.ddMove.grid(row=64, column=0, sticky=tk.E)

        self.btnMove = tk.Button(frame, text="Move", state="disabled", command=self.moveCart)
        self.btnMove.grid(row=64, column=1, padx=10, sticky=tk.W)

        # sensor test
        ##############
        self.separator4 = tk.Frame(frame, height=3, bd=2, relief=tk.SUNKEN)
        self.separator4.grid(row=90, column=0, columnspan=2, pady=10, sticky="we")

        self.sensorTestChoices = ["FLnear", "FLfar", "FRnear", "FRfar", "Lnear", "Rnear", "BLnear","BLfar","BRnear", "BRfar"]
        defaultSensorTest = "FLnear"
        self.sensorTest = self.sensorTestChoices.index(defaultSensorTest)

        testSensor = tk.StringVar(gui)
        testSensor.set(defaultSensorTest)
        self.ddSensor = tk.OptionMenu(frame, testSensor, *self.sensorTestChoices, command=self.selectedSensorTest)
        self.ddSensor.grid(row=95, column=0, sticky=tk.E)

        self.btnSensorTest = tk.Button(frame, text="Test Sensors", state="normal", command=self.testSensor)
        self.btnSensorTest.grid(row=95, column=1, padx=10, sticky=tk.W)

        # heartbeat blinker
        self.lblHeartBeat = tk.Label(frame, text="Heart Beat", fg="red")
        self.lblHeartBeat.grid(row=100, column=0, columnspan=2, pady=10)

        self.btnStop = tk.Button(frame, text="STOP CART", state="normal", command=self.stopCart, bg="red", fg="white")
        self.btnStop.grid(row=200, column=0, columnspan=2, pady=0)

        self.checkArduinoReady()


    def showNewDistances(self):

        # cartGlobal.log("showNewDistancies")

        config.obstacleInfo = []

        r = config.NUM_DISTANCE_SENSORS

        for i in range(r):
            s = config.distanceSensors[i]
            d = config.distanceList[i]
            sensor = sensors[i]

            # check for new data
            if s['newValuesShown']:
                continue

            cartControl.setSensorDataShown(i, True)
            # cartGlobal.log(f"show new sensor data {i}")

            # clear area
            col = "white"
            if sensor.id < 4:
                self.canvas.create_rectangle(sensor.x1 - 20, sensor.y1 - 10, sensor.x2 + 20, sensor.y, fill=col,
                                             outline=col)
            if sensor.id > 5:
                self.canvas.create_rectangle(sensor.x1 - 20, sensor.y, sensor.x2 + 20, sensor.y + 40, fill=col,
                                             outline=col)
            if sensor.id == 4:
                self.canvas.create_rectangle(sensor.x1 - 20, sensor.y - 40, sensor.x, sensor.y + 40, fill=col,
                                             outline=col)
            if sensor.id == 5:
                self.canvas.create_rectangle(sensor.x, sensor.y - 40, sensor.x + 20, sensor.y + 40, fill=col,
                                             outline=col)

            # show valid range of distance measures
            self.canvas.create_arc(sensor.x1, sensor.y1, sensor.x2, sensor.y2,
                                   start=sensor.arcFrom, extent=sensor.arcLength, style=tk.ARC, width=sensor.arcWidth,
                                   outline="snow3")

            if s.get('range') == 'long':
                lengthFactor = LINE_SCALE_LONG
                minRange = config.LONG_RANGE_MIN
                maxRange = config.LONG_RANGE_MAX
            else:
                lengthFactor = LINE_SCALE_SHORT
                minRange = config.SHORT_RANGE_MIN
                maxRange = config.SHORT_RANGE_MAX

            for k in range(config.NUM_MEASUREMENTS_PER_SCAN):

                # line color depends on distance value

                age = time.time() - s.get('timestamp')

                # limit line length
                if d[k] < 40:
                    lineLength = d[k]
                else:
                    lineLength = 40

                if lineLength < minRange or lineLength > maxRange:
                    config.obstacleInfo.append({'direction': s.get('direction'), 'position': s.get('position')})

                    if age > 2:
                        col = "coral1"
                        lineWidth = 2
                    else:
                        col = "red"
                        lineWidth = 2
                else:
                    if age > 2:
                        col = "paleGreen"
                        lineWidth = 1
                    else:
                        col = "green"
                        lineWidth = 1

                # Angle = value index * 15 + const + start rotation of arc
                xOffset = np.cos(np.radians(k * 15 + 15 + s.get('rotation'))) * lineLength * lengthFactor
                yOffset = np.sin(np.radians(k * 15 + 15 + s.get('rotation'))) * lineLength * lengthFactor

                try:
                    self.canvas.create_line(sensor.x, sensor.y, sensor.x + xOffset, sensor.y + yOffset, fill=col,
                                            width=lineWidth)
                except:
                    config.log("ERROR: sensor.x,.y: " + str(sensor.x) + ", " + str(sensor.y))

    def selectedDirection(self, value):
        self.direction = self.choices.index(value)

    def selectedSensorTest(self, value):
        self.sensorTest = self.sensorTestChoices.index(value)


    def checkArduinoReady(self):
        '''
        this is only called when arduino is not ready yet
        '''
        if config.arduinoStatus == 2:
            self.lblInfo.configure(text="cart ready", bg="lawn green", fg="black")
            self.btnRotate.configure(state="normal")
            self.btnMove.configure(state="normal")
            #self.btnNav.configure(state="normal")
            self.w.update_idletasks()
            self.w.after(100, self.heartBeat)

        else:
            self.w.after(400, self.checkArduinoReady)


    def navigateTo(self):
        start = time.time()
        # PathFinder.analyze((int(self.sbX.get()),int(self.sbY.get())))
        config.log(f"analyzed in: {time.time() - start} seconds")
        self.w.update_idletasks()


    def heartBeat(self):

        #config.log("heartBeat")

        # toggle heartBeat display color
        if self.lblHeartBeat.cget("fg") == "red":
            self.lblHeartBeat.configure(fg="green")
        else:
            self.lblHeartBeat.configure(fg="red")

        arduinoSend.sendHeartbeat()

        locX, locY = cartControl.getCartLocation()
        self.lblRotationCurrentValue.configure(text=f"{cartControl.getCartYaw()} / {locX:.0f} / {locY:.0f}")

        # check for updating new sensor data in gui
        self.showNewDistances()

        # update battery level every x seconds
        if time.time() - cartControl.getLastBatteryCheckTime() > 5:
            cartControl.updateBatteryStatus()
            battery = cartControl.getBatteryStatus()
            plugged = battery.power_plugged
            if not plugged:
                plugged = "on battery"
            else:
                plugged = "docked"
            batteryInfo = f", power: {plugged}, percent: {battery.percent:.0f}"
            self.lblInfo.configure(text="cart ready" + batteryInfo, bg="lawn green", fg="black")
            cartControl.setLastBatteryCheckTime(time.time())

        self.w.update_idletasks()
        self.w.after(500, self.heartBeat)  # heart beat loop


    def stopCart(self):

        arduinoSend.sendStopCommand("manual request")
        arduinoSend.requestCartOrientation()
        self.lblRotationCurrentValue.configure(text=str(cartControl.getCartYaw()))
        self.lblCommandValue.configure(text="Stop")
        self.w.update_idletasks()

    def moveCart(self):

        cartSpeed = int(self.sbSpeed.get())
        distance = int(self.sbDist.get())
        config.log(f"moveCart requested from cart gui, speed: {cartSpeed}, distance: {distance}")
        arduinoSend.sendMoveCommand(config.Direction(self.direction), cartSpeed, distance)
        self.lblCommandValue.configure(text="Move")
        # self.lblMove.configure(text=str(speed))
        self.w.update_idletasks()


    def testSensor(self):

        arduinoSend.sendReadSensorCommand(self.sensorTest)
        self.lblCommandValue.configure(text="testSensors")
        self.w.update_idletasks()


    def rotateCart(self):

        relAngle = int(self.sbRotation.get())

        self.lblCommandValue.configure(text="Rotate")
        targetOrientation = (cartControl.getCartYaw() + relAngle) % 360
        self.lblRotationTargetValue.configure(text=str(targetOrientation))
        arduinoSend.sendRotateCommand(relAngle, 150)
        self.w.update_idletasks()

    def updateDistanceSensorObstacle(self, distance, sensorID):
        info = f"{distance}, {config.getSensorName(sensorID)}"
        self.lblDistanceObstacleValue.configure(text=info)
        self.w.update_idletasks()

    def updateDistanceSensorAbyss(self, distance, sensorID):
        info = f"{distance}, {config.getSensorName(sensorID)}"
        self.lblDistanceAbyssValue.configure(text=info)
        self.w.update_idletasks()

    def updateTargetRotation(self, degrees):
        self.lblRotationTargetValue.configure(text=str(degrees))
        self.w.update_idletasks()


def startGui():
    global gui, controller

    gui = tk.Tk()
    gui.geometry('600x600+1100+100')  # window size and position

    controller = manualControl(gui)
    # cartGlobal.log "gui initialized in: ", time.time()-start, " seconds"
    try:
        gui.mainloop()
    except:
        print("exception in tk")
