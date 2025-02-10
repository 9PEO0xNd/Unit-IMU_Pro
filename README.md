# UNIT-IMU PRO mini

## Do not use this unit in situations involving safety to life.

![9DoF Display](image/DSC_0032.JPG)
![HSI Display](image/DSC_0034.JPG)
![orientation](image/IMU-Pro-Mini.jpg)
BMI270 and BMM150 Orentation and BMI270 I2C address = 0x69

## PlatformIO only (not tested Arduino-IDE)
Copy example code to src folder and rename main.cpp

- Modifed bmm150_defs.h using floating point
~~~
/******************************************************************************/
/*! @name        Compiler switch macros Definitions                */
/******************************************************************************/
#define BMM150_USE_FLOATING_POINT //OK

#ifndef BMM150_USE_FLOATING_POINT /*< Check if floating point (using BMM150_USE_FLOATING_POINT) is enabled */
#ifndef BMM150_USE_FIXED_POINT /*< If floating point is not enabled then enable BMM150_USE_FIXED_POINT */
#define BMM150_USE_FIXED_POINT
#endif
#endif
~~~

## Serial Monitor
To keep the display speed, use M5.Log.Printf instead of Serial or USBSerial

## References
https://github.com/boschsensortec/BMI270_SensorDriver  
https://github.com/boschsensortec/BMM150_SensorDriver  
https://github.com/m5stack/M5Unified  
https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/  

## License
- 9PEO0xNd - MIT
- M5Unified - MIT by M5Stack
- BMI270 BSD-3-Clause license by BoschSensortec
- BMM150 BSD-3-Clause license by BoschSensortec

## History
2024/Apr/06 Initial release
