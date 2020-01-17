
import sys
import time
import numpy as np
from enum import Enum
from collections import deque

from PyQt5 import QtGui, uic, QtWidgets, QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow, QGroupBox, QPushButton
from PyQt5.QtCore import pyqtSlot, Qt, QThread
from PyQt5.QtGui import QPainter, QBrush, QPen, QColor, QFont
from collections import namedtuple

import config
import guiUpdate
import cartControl
import arduinoSend
import move

class updType(Enum):
    SENSOR = 1
    CART_STATE = 2

# queue for gui updates
guiUpdateQueue = deque(maxlen=100)


NUM_DISTANCE_SENSORS = 10
NUM_MEASUREMENTS_PER_SCAN = 11
SCAN_RANGE = 140

#distanceList = np.zeros((NUM_DISTANCE_SENSORS, NUM_MEASUREMENTS_PER_SCAN), dtype=np.int16)
distanceList = np.random.random_sample((NUM_DISTANCE_SENSORS, NUM_MEASUREMENTS_PER_SCAN)) * 25

SensorDef = namedtuple('SensorDef', ['scanning', 'x', 'y', 'w', 'h', 'arcStart', 'arcSpan'])

class Window(QMainWindow):
    def __init__(self):
        super().__init__()
        self.title = "PyQt5 Cart Gui"
        self.top= 150
        self.left= 150
        self.width = 500
        self.height = 500
        self.InitWindow()


    def InitWindow(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.top, self.left, self.width, self.height)
        self.show()


#class gui(QtWidgets.QMainWindow, gui.Ui_MainWindow):
class cartGui(QMainWindow):

    def __init__(self, *args, **kwargs):
        super(cartGui, self).__init__()
        uic.loadUi('gui.ui', self)

        self.direction="STOP"
        self.sensorsDrawn=False
        self.sensorPositions = []

        # add buttons over the move graph icons
        self.addButtons()

        # add the sensor types and positions
        self.addSensorDefinitions()


        # start thread for checking update queue
        guiUpdateThread = GuiUpdateThread()
        guiUpdateThread.updateGui.connect(self.updateGui)
        #self.threads.append(guiUpdateThread)
        guiUpdateThread.start()

        config.cartStateChanged = True

        self.show()


    def addButtons(self):
        # add buttons over the graph labels
        btnRotateRight = QPushButton('', self)
        btnRotateRight.setStyleSheet("QPushButton{background: transparent;}");
        groupPos = self.groupRotate.geometry()
        btnPos = self.rotateRight.geometry()
        btnRotateRight.setGeometry(groupPos.x()+btnPos.x(), groupPos.y()+btnPos.y(), btnPos.width(), btnPos.height())
        btnRotateRight.setToolTip("clockwise rotation")
        btnRotateRight.clicked.connect(self.doRotateRight)

        btnRotateLeft = QPushButton('', self)
        btnRotateLeft.setStyleSheet("QPushButton{background: transparent;}");
        groupPos = self.groupRotate.geometry()
        btnPos = self.rotateLeft.geometry()
        btnRotateLeft.setGeometry(groupPos.x()+btnPos.x(), groupPos.y()+btnPos.y(), btnPos.width(), btnPos.height())
        btnRotateLeft.setToolTip("anticlockwise rotation")
        btnRotateLeft.clicked.connect(self.doRotateLeft)

        btnForward = QPushButton('', self)
        btnForward.setStyleSheet("QPushButton{background: transparent;}");
        groupPos = self.groupMove.geometry()
        btnPos = self.moveForward.geometry()
        btnForward.setGeometry(groupPos.x()+btnPos.x(), groupPos.y()+btnPos.y(), btnPos.width(), btnPos.height())
        btnForward.setToolTip("move forward")
        btnForward.clicked.connect(self.doForwardMove)

        btnForDiagRight = QPushButton('', self)
        btnForDiagRight.setStyleSheet("QPushButton{background: transparent;}");
        groupPos = self.groupMove.geometry()
        btnPos = self.moveDiagonalRightForward.geometry()
        btnForDiagRight.setGeometry(groupPos.x()+btnPos.x(), groupPos.y()+btnPos.y(), btnPos.width(), btnPos.height())
        btnForDiagRight.setToolTip("move forward diagonal right")
        btnForDiagRight.clicked.connect(self.doForwardDiagonalRightMove)

        btnRight = QPushButton('', self)
        btnRight.setStyleSheet("QPushButton{background: transparent;}");
        groupPos = self.groupMove.geometry()
        btnPos = self.moveRight.geometry()
        btnRight.setGeometry(groupPos.x()+btnPos.x(), groupPos.y()+btnPos.y(), btnPos.width(), btnPos.height())
        btnRight.setToolTip("move right")
        btnRight.clicked.connect(self.doRightMove)

        btnBackDiagRight = QPushButton('', self)
        btnBackDiagRight.setStyleSheet("QPushButton{background: transparent;}");
        groupPos = self.groupMove.geometry()
        btnPos = self.moveDiagonalRightBackward.geometry()
        btnBackDiagRight.setGeometry(groupPos.x()+btnPos.x(), groupPos.y()+btnPos.y(), btnPos.width(), btnPos.height())
        btnBackDiagRight.setToolTip("move backward diagonal right")
        btnBackDiagRight.clicked.connect(self.doBackwardDiagonalRightMove)

        btnBackward = QPushButton('', self)
        btnBackward.setStyleSheet("QPushButton{background: transparent;}");
        groupPos = self.groupMove.geometry()
        btnPos = self.moveBackward.geometry()
        btnBackward.setGeometry(groupPos.x()+btnPos.x(), groupPos.y()+btnPos.y(), btnPos.width(), btnPos.height())
        btnBackward.setToolTip("move backward")
        btnBackward.clicked.connect(self.doBackwardMove)

        btnBackDiagLeft = QPushButton('', self)
        btnBackDiagLeft.setStyleSheet("QPushButton{background: transparent;}");
        groupPos = self.groupMove.geometry()
        btnPos = self.moveDiagonalLeftBackward.geometry()
        btnBackDiagLeft.setGeometry(groupPos.x()+btnPos.x(), groupPos.y()+btnPos.y(), btnPos.width(), btnPos.height())
        btnBackDiagLeft.setToolTip("move backward diagonal left")
        btnBackDiagLeft.clicked.connect(self.doBackwardDiagonalLeftMove)

        btnLeft = QPushButton('', self)
        btnLeft.setStyleSheet("QPushButton{background: transparent;}");
        groupPos = self.groupMove.geometry()
        btnPos = self.moveLeft.geometry()
        btnLeft.setGeometry(groupPos.x()+btnPos.x(), groupPos.y()+btnPos.y(), btnPos.width(), btnPos.height())
        btnLeft.setToolTip("move Left")
        btnLeft.clicked.connect(self.doLeftMove)

        btnForDiagLeft = QPushButton('', self)
        btnForDiagLeft.setStyleSheet("QPushButton{background: transparent;}");
        groupPos = self.groupMove.geometry()
        btnPos = self.moveDiagonalLeftForward.geometry()
        btnForDiagLeft.setGeometry(groupPos.x()+btnPos.x(), groupPos.y()+btnPos.y(), btnPos.width(), btnPos.height())
        btnForDiagLeft.setToolTip("move forward diagonal Left")
        btnForDiagLeft.clicked.connect(self.doForwardDiagonalLeftMove)


    def addSensorDefinitions(self):

        # arc position is upper left corner of arc
        qrec = self.sensorArea.geometry()
        x,y,w,h = qrec.getRect()
        cx = qrec.center().x()
        wSensorArc = 40
        wSensorArc2 = int((wSensorArc)/2)
        hSensorArc = wSensorArc

        # the 3 front scanning sensors
        self.sensorPositions.append(SensorDef(scanning=True,
            x=cx - 3*wSensorArc2-20, y=y+25, w=wSensorArc, h=wSensorArc,
            arcStart=20, arcSpan=140))
        self.sensorPositions.append(SensorDef(scanning=True,
            x=cx - wSensorArc2, y=y+25, w=wSensorArc, h=wSensorArc,
            arcStart=20, arcSpan=140))
        self.sensorPositions.append(SensorDef(scanning=True,
            x=cx + wSensorArc2+20, y=y+25, w=wSensorArc, h=wSensorArc,
            arcStart=20, arcSpan=140))

        # the 3 back scanning snsors
        self.sensorPositions.append(SensorDef(scanning=True,
            x=cx - 3*wSensorArc2-20, y=y+h-70, w=wSensorArc, h=wSensorArc,
            arcStart=200, arcSpan=140))
        self.sensorPositions.append(SensorDef(scanning=True,
            x=cx - wSensorArc2, y=y+h-70, w=wSensorArc, h=wSensorArc,
            arcStart=200, arcSpan=140))
        self.sensorPositions.append(SensorDef(scanning=True,
            x=cx + wSensorArc2+20, y=y+h-70, w=wSensorArc, h=wSensorArc,
            arcStart=200, arcSpan=140))

        # the fixed sensors above the wheels
        sWidth = 20
        #front left
        self.sensorPositions.append(SensorDef(scanning=False,
            x=x+10, y=y+70, w=sWidth, h=sWidth, arcStart=0, arcSpan=0))
        #front right
        self.sensorPositions.append(SensorDef(scanning=False,
            x=x+w-sWidth-20, y=y+70, w=sWidth, h=sWidth, arcStart=0, arcSpan=0))
        #back left
        self.sensorPositions.append(SensorDef(scanning=False,
            x=x+10, y=y+h-100, w=sWidth, h=sWidth, arcStart=0, arcSpan=0))
        #back right
        self.sensorPositions.append(SensorDef(scanning=False,
            x=x+w-sWidth-20, y=y+h-100, w=sWidth, h=sWidth, arcStart=0, arcSpan=0))


    def paintEvent(self, event):

        #config.log(f"paintEvent: {event}")
        painter = QPainter(self)

        qrec = self.sensorArea.geometry()
        x,y,w,h = qrec.getRect()
        cx, cy = qrec.center().x(), qrec.center().y()

        # erase sensor area
        painter.setPen(QColor(225,225,225))
        painter.setBrush(QColor(220,220,220))
        painter.drawRect(self.sensorArea.geometry())

        # draw cart rectangle within sensor area
        painter.setBrush(QColor(200,200,200))
        xInset = 35
        yInset = 60
        painter.drawRect(x+xInset, y+yInset, w-2*xInset, h-2*yInset)

        painter.setPen(Qt.black)
        font = QFont()
        font.setPixelSize(15)
        painter.setFont(font)
        painter.drawText(cx-20,y+90,"Front")
        painter.drawText(cx-20,y+250,"Back")

        for i1, s in enumerate(self.sensorPositions):
            if s.scanning:
                painter.setPen(QPen(Qt.gray, 12, Qt.SolidLine))
                painter.drawArc(s.x, s.y, s.w, s.h, s.arcStart*16, s.arcSpan*16)

                for i2 in range(NUM_MEASUREMENTS_PER_SCAN):
                    d = distanceList[i1][i2]
                    dLine = d * 2
                    cx = 21     # looks better with arcWitdh/2 + 1
                    if i1 < 3:
                        angle = np.radians(i2 * SCAN_RANGE / NUM_MEASUREMENTS_PER_SCAN +110)
                        cy=25
                    else:
                        angle = np.radians(i2 * SCAN_RANGE / NUM_MEASUREMENTS_PER_SCAN -70)
                        cy=20
                    tx = np.sin(angle) * dLine
                    ty = np.cos(angle) * dLine

                    painter.setPen(Qt.green)
                    if d>16:
                        painter.setPen(QPen(Qt.blue,2,Qt.SolidLine))
                        d = min(d,18)
                    if d<10:
                        painter.setPen(QPen(Qt.red,2,Qt.SolidLine))

                    painter.drawLine(s.x+cx, s.y+cy, s.x+cx+tx, s.y+cy+ty)
                    #print(f"i1: {i1}, i2: {i2}, d: {d}")

            else:
                d= distanceList[i1][NUM_MEASUREMENTS_PER_SCAN-1]
                painter.setPen(Qt.green)
                painter.setBrush(Qt.green)
                if d>16:
                    painter.setPen(Qt.blue)
                    painter.setBrush(Qt.blue)
                if d<10:
                    painter.setPen(Qt.red)
                    painter.setBrush(Qt.red)
                painter.drawEllipse(s.x, s.y, 30, 30)


    def updateTargetRotation(self, degrees):
        self.targetYaw.setValue(degrees)

    def updateCommand(self, cmd):
        config.currentCommand = cmd
        config.cartStateChanged = True


    @pyqtSlot()
    def doRotateRight(self):
        config.log("rotateRight clicked")
        relAngle = self.rotationDegrees.value()
        if relAngle == 0:
            config.log(f"no rotation degrees set")
        else:
            config.cartTargetOrientation = (cartControl.getCartYaw() + self.rotationDegrees.value()) % 360
            arduinoSend.sendRotateCommand(relAngle, 150)
            self.updateCommand("ROTATE")


    @pyqtSlot()
    def doRotateLeft(self):
        print("rotateLeft clicked")
        relAngle = -self.rotationDegrees.value()
        if relAngle == 0:
            config.log(f"no rotation degrees set")
        else:
            config.cartTargetOrientation = (cartControl.getCartYaw() + self.rotationDegrees.value()) % 360
            arduinoSend.sendRotateCommand(relAngle, 150)
            self.updateCommand("ROTATE")

    @pyqtSlot()
    def doForwardMove(self):
        config.log(f"move FORWARD clicked")
        cartSpeed = int(self.cartSpeed.value())
        distance = int(self.moveDistanceCm.value())
        if distance > 0:
            config.log(f"moveCart requested from cart gui, speed: {cartSpeed}, distance: {distance}, direction: {config.MoveDirection.FORWARD}")
            move.moveRequest(config.MoveDirection.FORWARD, cartSpeed, distance, protected=True)
            self.updateCommand("MOVE")

    @pyqtSlot()
    def doForwardDiagonalRightMove(self):
        config.log(f"move FOR_DIAG_RIGHT clicked")
        cartSpeed = int(self.cartSpeed.value())
        distance = int(self.moveDistanceCm.value())
        if distance > 0:
            config.log(f"moveCart requested from cart gui, speed: {cartSpeed}, distance: {distance}, direction: {config.MoveDirection.FOR_DIAG_RIGHT}")
            move.moveRequest(config.MoveDirection.FOR_DIAG_RIGHT, cartSpeed, distance, protected=True)
            self.updateCommand("MOVE")

    @pyqtSlot()
    def doForwardDiagonalLeftMove(self):
        config.log(f"move FOR_DIAG_LEFT clicked")
        cartSpeed = int(self.cartSpeed.value())
        distance = int(self.moveDistanceCm.value())
        if distance > 0:
            config.log(f"moveCart requested from cart gui, speed: {cartSpeed}, distance: {distance}, direction: {config.MoveDirection.FOR_DIAG_LEFT}")
            move.moveRequest(config.MoveDirection.FOR_DIAG_LEFT, cartSpeed, distance, protected=True)
            self.updateCommand("MOVE")

    @pyqtSlot()
    def doBackwardDiagonalRightMove(self):
        config.log(f"move BACK_DIAG_RIGHT clicked")
        cartSpeed = int(self.cartSpeed.value())
        distance = int(self.moveDistanceCm.value())
        if distance > 0:
            config.log(f"moveCart requested from cart gui, speed: {cartSpeed}, distance: {distance}, direction: {config.MoveDirection.BACK_DIAG_RIGHT}")
            move.moveRequest(config.MoveDirection.BACK_DIAG_RIGHT, cartSpeed, distance, protected=True)
            self.updateCommand("MOVE")

    @pyqtSlot()
    def doBackwardDiagonalLeftMove(self):
        config.log(f"move BACK_DIAG_LEFT clicked")
        cartSpeed = int(self.cartSpeed.value())
        distance = int(self.moveDistanceCm.value())
        if distance > 0:
            config.log(f"moveCart requested from cart gui, speed: {cartSpeed}, distance: {distance}, direction: {config.MoveDirection.BACK_DIAG_LEFT}")
            move.moveRequest(config.MoveDirection.BACK_DIAG_LEFT, cartSpeed, distance, protected=True)
            self.updateCommand("MOVE")

    @pyqtSlot()
    def doBackwardMove(self):
        config.log(f"move BACKWARD clicked")
        cartSpeed = int(self.cartSpeed.value())
        distance = int(self.moveDistanceCm.value())
        if distance > 0:
            config.log(f"moveCart requested from cart gui, speed: {cartSpeed}, distance: {distance}, direction: {config.MoveDirection.BACKWARD}")
            move.moveRequest(config.MoveDirection.BACKWARD, cartSpeed, distance, protected=True)
            self.updateCommand("MOVE")

    @pyqtSlot()
    def doLeftMove(self):
        config.log(f"move LEFT clicked")
        cartSpeed = int(self.cartSpeed.value())
        distance = int(self.moveDistanceCm.value())
        if distance > 0:
            config.log(f"moveCart requested from cart gui, speed: {cartSpeed}, distance: {distance}, direction: {config.MoveDirection.LEFT}")
            move.moveRequest(config.MoveDirection.LEFT, cartSpeed, distance, protected=True)
            self.updateCommand("MOVE")

    @pyqtSlot()
    def doRightMove(self):
        config.log(f"move RIGHT clicked")
        cartSpeed = int(self.cartSpeed.value())
        distance = int(self.moveDistanceCm.value())
        if distance > 0:
            config.log(f"moveCart requested from cart gui, speed: {cartSpeed}, distance: {distance}, direction: {config.MoveDirection.RIGHT}")
            move.moveRequest(config.MoveDirection.RIGHT, cartSpeed, distance, protected=True)
            self.updateCommand("MOVE")


    def updateGui(self, data):

        if data['type'] == updType.CART_STATE:

            style = "QLabel{" + f"color: {config.cartStatusColor[0]}; background-color: {config.cartStatusColor[1]};" + "}"
            config.log(f"style: {style}")
            self.cartStatus.setStyleSheet(style)
            self.cartStatus.setText(config.cartStatus)
            self.currentCommand.setText(config.currentCommand)

            # obstacle and abyss
            self.obstacleSensors.setText(config.distanceSensorObstacle)
            self.abyssSensors.setText(config.distanceSensorAbyss)

            # current location
            self.currentX.setText(str(config.cartLocationX))
            self.currentY.setText(str(config.cartLocationY))
            self.currentYaw.setText(str(config.cartOrientation))

            # target location
            self.targetX.setText(str(config.cartTargetLocationX))
            self.targetY.setText(str(config.cartTargetLocationY))
            self.targetYaw.setText(str(config.cartTargetOrientation))


        if data['type'] == updType.SENSOR:
            pass




class GuiUpdateThread(QtCore.QThread):
    """
    This checks for new data in the guiUpdateQueue
    the queue can have different types of data based on the type attribute
    """

    # signal for gui update
    # raises guiLogic.updateGui
    updateGui = QtCore.pyqtSignal(object)

    def __init__(self):
        QThread.__init__(self)


    def run(self):

        time.sleep(2)       # wait for gui to startup

        while True:

            if config.cartStateChanged:
                config.cartStateChanged = False
                self.updateGui.emit({'type': updType.CART_STATE}) # triggers cartGui.updateGui

            if config.cartSensorUpdate:
                config.sensorStateChanged = False
                self.updateGui.emit({'type': updType.SENSOR}) # triggers cartGui.updateGui

            time.sleep(0.1)     # do max 10 updates per second of gui



def startGui():
    # qt gui
    app = QApplication(sys.argv)
    _ = cartGui()
    sys.exit(app.exec())

