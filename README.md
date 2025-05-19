# Romi32u4_Robot
## About
This was a term long project that I worked on with my partner Saeed Hage. Attached are some of the files necessary to the robot. We were Given a framework to build from that included a general template for event driven programming. 

## Robot Alignment Using Camera
![giphy (2)](https://github.com/user-attachments/assets/0e416bc5-1e19-4ad1-bb14-6e7857c9ddda)
![giphy (1)](https://github.com/user-attachments/assets/1df4eb0c-df65-4fd4-9e91-22534bda6871)

## Sensors
On this robot we implemented several sensors using the C++ language. 
### Motor encoders 
We use interrupts to convert encoder ticks to a perceived position of the robot in 2d space. Using this sensor we were able to implement inverse kinematics allowing the robot to drive to any arbitrary point and direction. 
### Ultrasonic sensor 
We use interrupts to detect the pulse duration. This is combined with a median filter to remove noise and accurately determine distance. Prior to the introduction of a camera we used this sensor to locate boxes by scanning its environment. 
### Sharp IR sensor
This sensor presented a unique challenge due to its high nonlinearity. However by experimentally measuring output signals we were able to fit a power regression to determine distance. We further enhanced readings by using a running average to reduce effects of noise. 
### Line sensor
This sensor was made up of multiple lightness sensors. We used an array in C++ to store the response from each individual sensor. This allowed us to precisely determine position on a line and adjust robot motion accordingly. We implemented a PID control system to follow a line using a weighted of the sensor data. 
### Load cell
We compared two processors for the load cell sensor. We first used an HX711 chip which is configured to produce an adc reading for the sensor. We then built our own circuit using an AD620 chip. By using different resistors we explored the nonlinearity of the sensor output. We found that a cubic fit most effectively modeled the sensor output. By programming this conversion ourselves allowed us to outperform the HX711 chip. 

![image](https://github.com/user-attachments/assets/e771f976-4426-4d62-b101-f4c1508bc445)
### IMU
To get the most accurate reading from the IMU we implemented a complementary filter that takes the stable but drifting gyroscope readings and adjusts them based on the noisy accelerometer readings to remove drift. This gave us precise readings for the incline of the robot. We were also able to use the gravitational acceleration reading of the three axes to determine the angle of the robot relative to the incline it is on. 
### Open MV camera
This camera is equipped with its own processing chip. This meant we could program the camera specifically using python to get relevant data. This data is then read by the main control board via UART protocol. In our case we tracked the corners of visible april tags. This information would be converted to a position and a rotation relative to the robot and sent via UART signal to the main board. The main board would then use the relative position of the tag to calculate a point and orientation to navigate in the environment.

Math on the camera side.

![image](https://github.com/user-attachments/assets/e3774bbd-19de-4ae9-8a40-eaa0d5796b9f)
![image](https://github.com/user-attachments/assets/ef0bf3a6-9f4c-4ac2-9ebd-c9d8dd9231db)

Math on the robot side.

![image](https://github.com/user-attachments/assets/31f9a8d2-5879-4103-9016-3cd2993424e0)

 
