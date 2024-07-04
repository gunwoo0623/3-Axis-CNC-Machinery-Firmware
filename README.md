# 3-Axis CNC Machinery Firmware for Digital Twin

This project was completed under the supervision of Professor *Quan Zhang* during a three-month undergraduate research period in XJTLU. I, *Geonwoo Cho*, developed the CNC machine firmware, while *Jennifer Hengky Liandy* worked on inmplementing the digital twin with MATLAB and Unity. Lastly, *Hang Yuan* assisted with the final documentation. I extend my sincere gratitude to everyone involved.

※ If the MATLAB and Unity files are uploaded to her GitHub, the link will be updated here.

## Demonstration Video

### X, Y, and Z Axis Movement

The following video demonstrates homing, calibration, and moving to a specific point using the Unity 3D user interface:

![Video1](https://github.com/gunwoo0623/3-Axis-CNC-Machinery-Firmware/assets/52570227/37511b81-a4c6-4ac2-8926-fe690e20c81b)

### Drawing a Star

The following video demonstrates the process of optimizing handwritten lines into linear shapes and sequentially following the order in which they were written:

![Video2](https://github.com/gunwoo0623/3-Axis-CNC-Machinery-Firmware/assets/52570227/f63c7529-cfa7-4a10-903b-ceb3b68e39a0)

## Hardware and Electronic Circuit

For more detailed configurations and diagrams, please refer to the "Guidance.pdf."

<div class="form-group">
        <div style="height: 200px; width: 500px;">
                <img style="height: 100%; width: 55%; float:left;" src="https://github.com/gunwoo0623/3-Axis-CNC-Machinery/assets/52570227/d958c287-57fc-4a85-a1a3-cf04541632b9">
                <img style="height: 100%; width: 44%; float:right;" src="https://github.com/gunwoo0623/3-Axis-CNC-Machinery/assets/52570227/b1111b31-0fd8-455d-8fa2-a6b7da4fc737">
        </div>
</div>

## Development of Custom Firmware

### AccelStepper Library

The use of the [AccelStepper library](https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html#a5dce13ab2a1b02b8f443318886bf6fc5) significantly reduces processing time compared to manual methods. This library simplifies motion control through functions such as 'setMaxSpeed()', 'setSpeed()', and 'setAcceleration()'. However, it creates a trapezoidal profile that introduces sudden changes in jerk due to instantaneous acceleration, putting stress on motors and mechanical components. Alternatively, S-curve acceleration cannot be implemented with the AccelStepper library. Due to time limitations, instead of creating a new library specifically for direct S-curve motion profiles, the decision was made to develop firmware using the AccelStepper library.

#### Primary Functions for Stepper Motor Movement:
* **move():** Enables relative movement by setting the target position relative to the current one but was excluded due to potential direction confusion for enhanced code readability.
* **moveTo():** Facilitates absolute movement by defining the absolute target position and proves versatile in establishing relative movement by combining variables for position and direction.
* **run():** Essential for implementing accelerations and decelerations to achieve precise positioning, ensuring dynamic motor response aligned with designated speed adjustments until reaching the desired position.
* **runSpeed():** Designed for constant speed implementation, poses the risk of infinite jerk during the initial movement from the initial point, as indicated in the concept design, and therefore, this function will be omitted.

In addition, the AccelStepper class reference specifies that the Arduino Mega, operating at a 16 MHz clock frequency, supports a maximum motor speed of approximately 4000 steps per second.

### Steps  to  Millimeters Conversion 

The conversion of steps to millimeters in linear motion depends on the lead screw's pitch length. For a CNC machine with a 2mm pitch length on the X-axis driven by a single motor, 1 mm of linear motion corresponds to the pitch length. A stepper motor with a 1.8° step angle typically has 200 steps per revolution. When configured with a micro-step setting of 4, the step count for 1 mm is calculated as 800 (1 pitch length × 200 steps per revolution × 4 micro-steps). Detailed step values for 1 mm at various micro-step settings are summarized in following table:

| Microstep                 | Steps for 1mm                       |
|---------------------------|-------------------------------------|
| 1                         | 200                                 |
| 2                         | 400                                 |
| 4                         | 800                                 |
| 8                         | 1600                                |
| 16                        | 3200                                |
| 32                        | 6400                                |

### 2-D Line Bresenham Algorithm

Bresenham's line algorithm is exclusively used for G01, as G00 is solely for positioning and does not require precise work. The plotline function plays a primary role in classifying situations. Approximately 90 lines of code were dedicated to this algorithm, totaling 126 lines for full functionality.

<div class="form-group">
        <div style="height: 200px; width: 500px;">
                <img style="height: 100%; width: 64%; float:left;" src="https://github.com/gunwoo0623/3-Axis-CNC-Machinery/assets/52570227/ac3c192f-1490-42ad-ae17-d26765b1b2e3">
                <img style="height: 100%; width: 35%; float:right;" src="https://github.com/gunwoo0623/3-Axis-CNC-Machinery/assets/52570227/438131ef-d3ad-4743-8692-197d5c282376">
        </div>
</div>

For further understanding of the importance of the line algorithm in CNC machines, [Dan](https://www.marginallyclever.com/2020/07/moving-your-cnc-with-bresenhams-algorithm/) provides a clear explanation on why the line algorithm is crucial for CNC machines. [Abrash](https://www.ercankoclar.com/wp-content/uploads/2016/12/gpbb35.pdf) further supports the Bresenham line algorithm as the most practical among notable alternatives, as detailed in the methodology by [Bresenham](https://ohiostate.pressbooks.pub/app/uploads/sites/45/2017/09/bresenham.pdf). For additional insights, [Flanagan](https://cs.helsinki.fi/group/goa/mallinnus/lines/bresenh.html) discusses the slope ranges and characteristics for each octant, facilitating easier coding.


### G-code and M-code

This table lists commands that can be executed, excluding G00, G01, G02, and G03 which will be addressed by communication between Arduino and Unity:

| Code Number               | Description                         |
|---------------------------|-------------------------------------|
| G20                       | Inch Units                          |
| G21                       | Millimeter Units                    |
| G28                       | Return to Home Position             |
| G33                       | Calibration                         |
| G60                       | Save Current Position               |
| G61                       | Return to Saved Position            |
| M17                       | Enable Motors                       |
| M18                       | Disable Motors                      |
| M114                      | Manually Set Position               |
| M115                      | Check Maximum Distance              |
| M205                      | Set Advanced Settings               |

For more details on G-code commands, refer to [Dejan's explanation](https://howtomechatronics.com/tutorials/g-code-explained-list-of-most-important-g-code-commands/).  
To explore all G-code and M-code commands, visit the [Marlin firmware documentation](https://marlinfw.org/meta/gcode/).


As a result, the entire flowchart is illustrated below:
![Figure 37. Flowchart illustrating the firmware communicating with Unity for simulation. ](https://github.com/gunwoo0623/3-Axis-CNC-Machinery/assets/52570227/19e5d9d5-d1da-4b16-9bca-07a8001c4f86)

### Future Work: Drawing Curves

The implementation of G02 and G03 for circular interpolation remains incomplete. Future efforts should focus on a thorough review of resources, particularly those referenced by [Alois](https://zingl.github.io/Bresenham.pdf) for plotting ellipses and Bézier curves.

## Conclusion of Digital Twin

Both MATLAB and Unity were used for the digital twin, but Unity outperformed MATLAB due to reduced communication delays. Thus, future improvements should focus on enhancing data processing to improve real-time performance.

![Result of Digital Twin with MATLAB and Unity](https://github.com/gunwoo0623/3-Axis-CNC-Machinery/assets/52570227/dfb5de02-4196-4d1b-a47e-fed64252988e)
