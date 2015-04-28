# Overview #
MCBMini is a complete and open-source motor control solution.
It includes the hardware design files for PCBs, firmware, partslists and suppliers, firmware for micro controllers and java host software.

### Table of contents ###
|  | ![http://media.mit.edu/~siggi/mcbmini_images/mcbmini44small.jpg](http://media.mit.edu/~siggi/mcbmini_images/mcbmini44small.jpg) | ![http://media.mit.edu/~siggi/mcbmini_images/oshw-logo-200-px.png](http://media.mit.edu/~siggi/mcbmini_images/oshw-logo-200-px.png) |
|:-|:--------------------------------------------------------------------------------------------------------------------------------|:------------------------------------------------------------------------------------------------------------------------------------|


# Features #
Each board has two motor control channels, each spec is "per channel" but eacg board can also be run in "joint" mode where two channels can be used to control one (beefy) motor.

  * Separate logic and motor power sources (can be use single power source with jumper setting)
  * Logic power voltage range: 6-24V
  * Motor power voltage range: 5.5-24V
  * Maximum channel output current: 30A (max current of bridge, heat considerations need to be made)
  * PWM frequencies of 20kHz (inaudible)
  * Velocity and position PID closed-loop control
    * Maximum velocity and acceleration parameters
  * Position feedback: Quadrature encoder or potentiometer (or both)
  * Current sensing
  * Communication to MCBCom board supports:
    * TTL level serial (RX/TX)
    * USB bia USB-mini connector (using FTDI technology)
    * Bluetooth via socket for the SparkFun BlueSMIRF board
  * Electrical protection:
    * Output short to GND or Power shutdown (with software notification)
    * Thermal shutdown (with software notification)
    * Reverse battery protection (for motor power)
  * 2 extra pins per board which can each be configured to:
    * Read an external analog voltage
    * Monitor Logic/Motor batteries (each has dedicated extra pin)
    * Read switch state (possibly a limit switch) and notify software of event
    * Control servo motor (create pulse signal needed by a servo)



# Images #

| ![http://media.mit.edu/~siggi/mcbmini_images/mcbmini44.jpg](http://media.mit.edu/~siggi/mcbmini_images/mcbmini44.jpg) |
|:----------------------------------------------------------------------------------------------------------------------|
| The MCBMini v4.4 motor controller board |

| ![http://media.mit.edu/~siggi/mcbmini_images/mcbcom13.jpg](http://media.mit.edu/~siggi/mcbmini_images/mcbcom13.jpg) |
|:--------------------------------------------------------------------------------------------------------------------|
| The MCBCom v1.3 communications board (USB, serial, Bluetooth) |


| ![http://media.mit.edu/~siggi/mcbmini_images/mcbmini_v43.jpg](http://media.mit.edu/~siggi/mcbmini_images/mcbmini_v43.jpg) |
|:--------------------------------------------------------------------------------------------------------------------------|
| The MCBMini v4.3 motor controller board |


| ![http://media.mit.edu/~siggi/mcbmini_images/overview.png](http://media.mit.edu/~siggi/mcbmini_images/overview.png) |
|:--------------------------------------------------------------------------------------------------------------------|
| An overview schematic of a stack of boards (com board is optional) |

| ![http://media.mit.edu/~siggi/mcbmini_images/screenshot1.png](http://media.mit.edu/~siggi/mcbmini_images/screenshot1.png) |
|:--------------------------------------------------------------------------------------------------------------------------|
| This is the main motor target window where a user can specify motor target positions or velocities and observe instantaneous feedback values. |

| ![http://media.mit.edu/~siggi/mcbmini_images/screenshot2.png](http://media.mit.edu/~siggi/mcbmini_images/screenshot2.png) |
|:--------------------------------------------------------------------------------------------------------------------------|
| In this window the user can specify various parameters for the boards and change them on the fly. These parameters are read from a configuration XML file. |

| ![http://media.mit.edu/~siggi/mcbmini_images/screenshot3.png](http://media.mit.edu/~siggi/mcbmini_images/screenshot3.png) |
|:--------------------------------------------------------------------------------------------------------------------------|
| In this window the user can observe target and actual values for all the controllers. This is often crucial to tune the controllers for different setups. |


# Licenses #
This project contains three different kinds of licenses:
  * MCBMiniServer (the java host software): **LGPLv2**
  * PCB designs (both for motor controllers and communication board): **CC w/Attribution**
  * Motor control firmware: **GPLv2**


# Documentation #
Manuals can be found in the Downloads section of this site

# How to get them #
You can of course use the hardware design files and have the PCBs made (Silver Circuits, Advanced Circuits etc.) and purchase the parts from the partslists from Digikey and stuff them yourself.

But this is a lot of work and usually the cost of making PCBs is expensive when buying less than about 50 boards so I am currently trying to set up a good automatic pipeline for anybody acquiring them.

For now just contact me (owner of project) if you are interested in acquiring some of these boards.