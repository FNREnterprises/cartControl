# cartControl

python task accepting rpyc commands for controlling a mechanical wheel cart and a depth camera

The python task communicates with an Arduino to move the wheels and servos for obstacle scanning.

The cart is part of my InMoov robot setup.

Author: Juerg Maier, maier.juerg@gmail.com


## Cart Parts
* 4 mecanum wheels  
  purchased from china  
  

* 4 wheel motors
  
* 1 wheel motor controller  
  Adafruit V2 motor shield
* 2 table elevator motors
* 1 table motor controller  
  H-Bridge DC dual motor driver pwm module IRF3205, 3..36V, 10A, Peak 30A
* 6 scanning servos  
  3 in cart front  
  3 at cart back  
  each carries an infrared sensor and makes 10 distance checks per scan
* 10 infrared sensors  
  GP2Y0A21YK0F 
* 2 imu
* 1 rgb cart front cam
* 1 intel realsense D415 depth cam

# Arduino
The arduino accepts commands and reports the carts status back
This in a separate project
#####Move Directions
* forward    
  forward moves will use the depth cam to check for obstacles
* backward
* left
* right
* diagonal forward left
* diagonal forward right
* diagonal backward left
* diagonal backward right
* rotation clockwise
* rotation counterclockwise 
###Exposed functions
####rpc Basics
* on_connect
* exposed_requestForReplyConnection
* exposed_requestLifeSignal
* exposed_terminate

#### Cart Control
* exposed_move
* exposed_rotateRelative
* exposed_stop
* exposed_setCartLocation
* exposed_adjustCartPosition
* exposed_getCartInfo(self)
* exposed_requestCartOrientation(self):
* exposed_getObstacleInfo(self):
* exposed_getBatteryStatus(self):
* exposed_isCartMoving(self):
* exposed_isCartRotating(self):
* exposed_obstacleUpdate(self, data):
* exposed_lifeSignalUpdate(self, server):
* exposed_requestHeadOrientation(self):
* exposed_requestD415Depth
