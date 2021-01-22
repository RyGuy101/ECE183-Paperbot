#Segway and Paperbot
##Description
**Mathematical Formulations**
[Math Formulation]() //Include link to math formulation here

For Joint Lab 1, we created mathematical formulations of two-wheeled robots 
according to the specifications [here](https://ccle.ucla.edu/pluginfile.php/4059849/mod_resource/content/0/ME_ECE_Joint_Lab_1.pdf). Using these mathematical models, we then simulated the two-wheeled robot

##Usage 
The simulation is initialized with the user-specified values:
* West xW, East xE, North yN, South yS coordinate values for the fixed x or y 
positions  of the walls, respectively
* Segway Dimenstions d = 502mm, w = 530mm, or Paperbot Dimensions d = 50mm, w = 90mm
* Dr, the right distance sensor offset
* Df, the front distance sensor offset  
* H, the actuator/state transition covariance
* V, the sensor/output covariance
* dt, the sample period  

```python
<insert python code here to run simulation>
```



