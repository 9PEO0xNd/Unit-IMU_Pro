/*
*******************************************************************************
* IMU-Pro-mini
* Modified by 9PEO0xNd
* based on: BMI270 and BMM150 example.  IMU (c) 2023 by M5Stack
*                  Equipped with M5CoreS3 sample source code
*                          配套  M5CoreS3 示例源代码
* Visit for more information: https://docs.m5stack.com/en/unit/IMU%20Pro%20Mini%20Unit
*******************************************************************************
*/

#include <M5Unified.h>
#include "imupro.h"
//#include <M5StackUpdater.h>

imupro bmi2aux; //
auto &lcd = M5.Display; // 同一インスタンスを指し示す方法

void setup() {
    auto cfg = M5.config();

#if defined ( ARDUINO )
    cfg.serial_baudrate = 115200;   // default=115200. if "Serial" is not needed, set it to 0.
#endif

    M5.begin(cfg);
    lcd.setTextSize(2);        // Set text size. 设置文字大小
    lcd.setCursor(0, 0);       // Set the cursor. 设置光标位置
    delay(200);                   // Delay 200ms.  延迟200ms
/*
    checkSDUpdater(
        SD,           // filesystem (default=SD)
        MENU_BIN,     // path to binary (default=/menu.bin, empty string=rollback only)
        2000,         // wait delay, (default=0, will be forced to 2000 upon ESP.restart() )
        TFCARD_CS_PIN // (usually default=4 but your mileage may vary)
    );
*/
    Wire.begin(M5.Ex_I2C.getSDA(), M5.Ex_I2C.getSCL()); //GNSS or IMU pro

    bmi2aux.Init();                // Init IMU. with I2C
}

void loop() {
    M5.update();
    bmi2aux.Update();  // Update data from IMU. 更新IMU数据

    // gyroscope output related.  陀螺仪输出相关
    lcd.setCursor(20, 0); // x=30 y=70
    lcd.printf("  gyroX   gyroY   gyroZ");  // Screen printingformatted string.
    lcd.setCursor(20, 22); // y=92
    lcd.fillRect(20, 22, 290, 20, BLACK);
    lcd.printf("%7.2f %7.2f %7.2f",
                     bmi2aux.gyro_data.x, bmi2aux.gyro_data.y,
                     bmi2aux.gyro_data.z);

    // Accelerometer output is related
    lcd.setCursor(20, 46); // y=120
    lcd.printf("   accX    accY    accZ");
    lcd.setCursor(20, 68); // y=142
    lcd.fillRect(20, 68, 290, 20, BLACK);
    lcd.printf(" %6.2f  %6.2f  %6.2f",
                     bmi2aux.accel_data.x, bmi2aux.accel_data.y,
                     bmi2aux.accel_data.z);

    // mag Raw
    lcd.setCursor(20, 92); // y=170
    lcd.printf(" Raw mX  Raw mY  Raw mZ");
    lcd.setCursor(20, 114); // y=192
    lcd.fillRect(20, 114, 290, 20, BLACK);
    lcd.printf("%7.2f %7.2f %7.2f",
                     bmi2aux.mag_data.raw.x, bmi2aux.mag_data.raw.y,
                     bmi2aux.mag_data.raw.z);

    // mag offset compensated
    lcd.setCursor(20, 138); //
    lcd.printf("   magX    magY    magZ");
    lcd.setCursor(20, 160); //
    lcd.fillRect(20, 160, 290, 20, BLACK);
    lcd.printf("%7.2f %7.2f %7.2f",
                     bmi2aux.mag_data.x, bmi2aux.mag_data.y,
                     bmi2aux.mag_data.z);

    // Atan2 X,Y
    lcd.setCursor(20, 184); //
    lcd.printf("Heading: %5.1f deg",
                     bmi2aux.CalcuDir());

// float_compensated
/*  M5.Log.printf is used instead of Serial or USBSerial
    M5.Log.printf("\nAcc_X = %4.2f\t Acc_Y = %4.2f\t Acc_Z = %4.2f\r\n",
                     bmi2aux.accel_data.x, bmi2aux.accel_data.y,
                     bmi2aux.accel_data.z);
    M5.Log.printf("\nGyro_X = %4.2f\t Gyro_Y = %4.2f\t Gyro_Z = %4.2f\r\n",
                     bmi2aux.gyro_data.x, bmi2aux.gyro_data.y,
                     bmi2aux.gyro_data.z);

// float data
    M5.Log.printf("\nMag_X = %5.2f\t Mag_Y = %5.2f\t Mag_Z = %5.2f\r\n",
*/

    M5.Log.printf("%7.2f, %7.2f, %7.2f\r\n",
                     bmi2aux.mag_data.x, bmi2aux.mag_data.y,
                     bmi2aux.mag_data.z);


    lcd.setCursor(40, 224);
    lcd.printf("CAL ");

    if(M5.BtnA.wasPressed())
    {
        lcd.clear();        // 黒で塗り潰し
        lcd.setCursor(0, 0);
        lcd.print("begin calibration in 3 seconds");
        delay(3000);
        lcd.setCursor(0, 10);
        lcd.print("Flip + rotate core calibration");
      //bmm150.bmm150_calibrate(10000);
        bmi2aux.MagCalibration();
        delay(100);

      //bmm150.getMagnetOffset(&magoffsetX, &magoffsetY, &magoffsetZ);
        lcd.clear();        // 黒で塗り潰し
    }
    delay(20);
}