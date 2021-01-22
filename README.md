# Segway and Paperbot Simulation
## Description
---
The objective of this lab was to create a mathematical formulation and simulate two-wheeled robots, more specifically a Segway and a Paperbot, each with a wheel diameter *d* and width *w*. The computational simulation was made to simulate either Segway or Paperbot in a rectangular environment with 4 walls and boundaries specified by *xW*, *xE*, *yN*, *yS*, representing the west, east, north, and south wall limits respecively.

**Mathematical Formulations**
* [Math Formulation]() //Include link to math formulation here


## Usage 
---
The simulation is initialized with the user-specified values:
* West *xW*, East *xE*, North *yN*, South *yS* coordinate values for the fixed x or y 
positions  of the walls, respectively
* Segway Dimenstions *d* = 502mm, *w* = 530mm; or Paperbot Dimensions *d* = 50mm, *w* = 90mm
* *Dr*, the right distance sensor offset
* *Df*, the front distance sensor offset  
* *H*, the actuator/state transition covariance
* *V*, the sensor/output covariance
* *dt*, the sample period  

```python
# Installation Packages
pip install keyboard
pip install pygame

# Run Simulation, Using User-Input Arrow Keys
python ./paperbot.py
```

### Installation
The simulation requires packages keyboard and pygame. 

## Hardware
---
* Segway
  * Continuous Rotation Servo: Unspecified
  * Laser Range Sensor: [Lidar Lite v3](https://cdn.sparkfun.com/assets/f/e/6/3/7/PM-14032.pdf)
  * IMU: [3DM-GX5-10, LORD MicroStrain](https://www.microstrain.com/sites/default/files/3dm-gx5-25_datasheet_8400-0093_rev_o.pdf)
* Paperbot
  * Continuous Rotation Servo: [FS90R](https://cdn-shop.adafruit.com/product-files/2442/FS90R-V2.0_specs.pdf)
  * Laser Range Sensor: [GYVL53L0X](https://www.st.com/resource/en/datasheet/vl53l0x.pdf)
  * IMU: [MPU9250](https://invensense.tdk.com/products/motion-tracking/9-axis/mpu-9250/?fbclid=IwAR3oB_TRBq6vESyrCZcruXoCS__1Q0_s-4oi9rOmKmrxwR31XUUP7iYXZNA)

## Simulation
---
[Moving Robot](simulation.png)(https://drive.google.com/file/d/15Ngqlc0fzkjXBNw1BdblkG5iG976Lxk5/view?usp=sharing)


