# 3-Axis CNC Machine Firmware for Digital Twin

This project was completed under the supervision of Professor *Quan Zhang* during a three-month undergraduate research period in XJTLU. I, *Geonwoo Cho*, developed the CNC machine firmware, while *Jennifer Hengky Liandy* worked on integrating the digital twin with MATLAB and Unity. Lastly, *Hang Yuan* assisted with the final documentation. I extend my sincere gratitude to everyone involved.


## Development of Custom Firmware

### AccelStepper Library



이 프로젝트는 와 함꼐 아두이노와 유니티 간의 디지털 트윈을 적용시키기이다. 

기본적인 

### Steps  to  Millimeters Conversion 

### 2D Line Bresenham Algorithm

### G-code and M-code
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


## Digital Twin
Both MATLAB and Unity were used for the digital twin, but Unity outperformed MATLAB due to reduced communication delays. Future improvements should focus on enhancing data processing to improve real-time performance.
![7](https://github.com/gunwoo0623/3-Axis-CNC-Machinery/assets/52570227/dfb5de02-4196-4d1b-a47e-fed64252988e)
