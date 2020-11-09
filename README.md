# Indoor-Positioning-and-Navigation-using-Reinforcement-Learning

## Basics
This project is used to solve the problem of indoor positioning. As gps does not work indoor, This project was tested for accuracy upto 15cm for indoor 2d positioning.
The positioning this passed to a AI Model which navigates the vehicle to required position using Reinforcement Learning



## Hardware
Three anchors are used in which UWB is connected to arduino pro mini
The tag is connected to software serial of nodemcu i.e (D8,D7)
Vehicle is controlled by Arduino Uno connected to NodeMCU Tx,Rx

The vehicle has gyro sensor and adafruit motor shield



## Libraries
Add dwm1000 Custom Library :      (Added custom channels and changed Tx Power)
  https://github.com/wahajmurtaza/arduino-dw1000

Add ESP Library:
  https://github.com/wahajmurtaza/NodeMCU-Python-Wifi

Python:
  pygame
  Pytorch
  Pyserial
  
 
## Block Diagram
<img src="Block Diagram.png">
  
