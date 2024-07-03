# 3-Axis CNC Machine Firmware for Digital Twin

This project was completed under the supervision of Professor *Quan Zhang* during a three-month undergraduate research period in XJTLU. I, *Geonwoo Cho*, developed the CNC machine firmware, while *Jennifer Hengky Liandy* worked on integrating the digital twin with MATLAB and Unity. Lastly, *Hang Yuan* assisted with the final documentation. I extend my sincere gratitude to everyone involved.

※ If Jennifer Hengky Liadny uploads the MATLAB and Unity files to her GitHub, the link will be updated here.
## Development of Custom Firmware

### AccelStepper Library

이 프로젝트는 와 함꼐 아두이노와 유니티 간의 디지털 트윈을 적용시키기이다. 

기본적인 

### Steps  to  Millimeters Conversion 

### 2-D Line Bresenham Algorithm

[Dan](https://www.marginallyclever.com/2020/07/moving-your-cnc-with-bresenhams-algorithm/) provides a clear explanation on why the line algorithm is crucial for CNC machines. [Abrash](https://www.ercankoclar.com/wp-content/uploads/2016/12/gpbb35.pdf) further supports the Bresenham line algorithm as the most practical among notable alternatives, as detailed in the methodology by [Bresenham](https://ohiostate.pressbooks.pub/app/uploads/sites/45/2017/09/bresenham.pdf). For additional insights, [Flanagan](https://cs.helsinki.fi/group/goa/mallinnus/lines/bresenh.html) discusses the slope ranges and characteristics for each octant, facilitating easier coding.

### G-code and M-code

This table lists commands that can be executed, excluding G00, G01, G02, and G03 which will be addressed by communication between Arduino and Unity:

| Code Number               | Description                         |
|-------------------------|-------------------------------------|
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

### Future Work: Drawing Curves

## Conclusion of Digital Twin

Both MATLAB and Unity were used for the digital twin, but Unity outperformed MATLAB due to reduced communication delays. Thus, future improvements should focus on enhancing data processing to improve real-time performance.

![Result of Digital Twin with MATLAB and Unity](https://github.com/gunwoo0623/3-Axis-CNC-Machinery/assets/52570227/dfb5de02-4196-4d1b-a47e-fed64252988e)
