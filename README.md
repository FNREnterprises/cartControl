# cartControl

python task accepting commands put in a queue for controlling a mechanical wheel cart and a depth camera

The python task communicates with an Arduino to move the wheels and servos for obstacle scanning.

The cart is part of the InMoov robot setup.

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
This in a separate project and is written in Arduino C++ code. See documentation of the Interface



## Dataflow Arduino Startup
```puml
@startuml
    hide footbox
    participant "Arduino\nsetup" as setup
    
    note over setup
    read ir reference
    distances from
    eeprom
    end note
    
    setup -> cartControl : reference distances ir sensors\n<font color=red><b>!F2
        
    cartControl -> marvinData : reference distances ir sensors\n<font color=red><b>IR_SENSOR_REFERENCE_DISTANCE
    
@enduml
````

## IR Sensor Measure Results

- reference distances:
  - reference distances of the IR sensors are stored in the eeprom of the arduino.
  - during setup the reference distances are sent to the cartControl (!F2)
    and the cartControl updates the values in the shared dict for displaying them in the gui
  - through the cart gui a user can request a IR sensor test.
    The measured distances are sent back to the gui and added up to build an average within the cartGui process.
  - These averages can then be set by user action (button) as the new reference distances for the sensor

- sensor test requested:
  - the measured distances are reported with Arduino message !A7

- normal operation:
  - the arduino subtracts the reference distance from the measured distance and reports floorOffsets (+ for obstacles, - for abysses). Arduino message !A1

## User Action: IR Test Sensor
````puml
@startuml
    actor cartGui
    cartGui -> cartControl : test sensor (sensorId)\n<font color=red><b>mg.CartCommands.TEST_SENSOR
    cartControl -> cartArduino : test sensor\n<font color=red><b>7,<sensorId>
    loop 5 seconds
        cartArduino -> cartControl : sensor distances\n<font color=blue><b>!A7
        cartControl -> cartGui : measured distances
        cartGui -> cartGui : display values\ncalculate and display average
    end
@enduml
````

## Move Cart

Cart movements can be requested through the cart GUI or by any other process using the CartCommandsQueue

````puml
@startuml
class Move{
- move direction
- distance
- speed
- protected
}
@enduml
````


The command for a move is "1"

Move Directions
* forward  
  forward moves will use the depth cam and ultrasonic sensors to check for obstacles
* backward
* left
* right
* diagonal forward left
* diagonal forward right
* diagonal backward left
* diagonal backward right
* rotation clockwise
* rotation counterclockwise

| Field         | Explanation                 |
| ------------- | ----------------------------|
| **Distance**  | distance in mm              |
| **Speed**     | a value between 50 and 250  |
| **Protected** |for normal moves in forward direction with protection activated it will make use of the head camera and the ultrasonic distance sensors in addition to the infrared sensors<br>for special moves (e.g. docking) protection needs to be disabled as it  would prevent the cart to get in close encounter with objects.|

{{CommandsToArduio.puml}}


