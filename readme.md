# ESP32 codes for exoskeleton project

## Setting up the esp32 in windows.
---
### Installing from windows installer.
- provide video links here

### setting up it in vs code for easy use
-


## ESP32 files 
---

### <ins>**Troubleshooting and getting started with actuating motor**</ins>

- **Hall sensor check.**
    - hall sensor value should be in right order for smooth rotation.
    - build and flash the hall sensor test file in the esp32.
    - Rotate the motor very slowly.
    - The hall sensor value order should be as following.
        - **Anti-clockwise** - 5,1,3,2,6,4
        - **Clockwise** - 4,6,2,3,1,5

- **Actuating the motors**
    - Please ensure that the hallsenosr values are in order before this step.
    - set the duty value to 70 in the motor_control.c file.
    - Build and flash the motor_control.c file.
    - if the windings are connected in correct order the motor should start rotating smoothly.

- **Check the encoder readings**
    - build and upload the encoder_test.c file to check
    - rotate the motor by hand.
    - check if the values are increasing when rotated anticlockwise. and decreasing when rotated clockwise.
    - If this works then encoder is working properly.

- **Controling the motor speed**
    - Make sure that the motor is rotating smoothly.
    - Change the duty value in the motor_test.c file to set the motor at different speeds.

### <ins> **Implementing position control code** </ins>

    
    - get the encoder value
    - convert it to the final output angle after applying gear reduction ratio of 80.
    - Apply the control algorihm 

## MQTT

- **Setting up the MQTT network** 

    - Install mosquitto mqtt server
    - Connect all the devices which should exchange data to the same wifi network.
    - start the MQTT server through command line use the config file provided on the PC connected to the same network as your esp32's.
        - read the MQTT documentation to more costumize the config file.
    - get the ip address of the PC on which the mosquitto is running.
    - use this ip address as the server address for the MQTT clients to connect.



# MQTT files




## Communicating using python and paho
---

Setting 
    - Start the mosquitto mqtt server on the wifi network, it may be on any pc connecte with the network
    - 


## Reference materials

- Working BLDC motor.
- To understand the quadrature Encoder.
- Position and velocity control.