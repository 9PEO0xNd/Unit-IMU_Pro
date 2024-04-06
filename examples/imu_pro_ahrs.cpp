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
#include "utility/myMahonyAHRS.h"
//#include <M5StackUpdater.h>

imupro bmi2aux; //
auto &lcd = M5.Display; // 同一インスタンスを指し示す方法
static M5Canvas compass(&lcd);     // オフスクリーン描画用バッファ
static M5Canvas base(&compass);    // オフスクリーン描画用バッファ

float pitch = 0.0F;
float roll = 0.0F;
float yaw = 0.0F;

uint32_t Now = 0;
uint32_t lastUpdate = 0;
float deltat = 0.0f;
static uint32_t fsec, psec;
static size_t fps = 0, frame_count = 0;

void compassplot(float a);

void setup() {
    auto cfg = M5.config();

    M5.begin(cfg);
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

    bmi2aux.Init();                // Init IMU. 初始化IMU
    myKp=8.0f; myKi=0.0f;

// 必要に応じてカラーモードを設定します。（初期値は16）
// 16の方がSPI通信量が少なく高速に動作しますが、赤と青の諧調が5bitになります。
    lcd.setColorDepth(16);
// clearまたはfillScreenで画面全体を塗り潰します。
    lcd.clear();        // 黒で塗り潰し
// スプライト（オフスクリーン）への描画も同様の描画関数が使えます。
// 最初にスプライトの色深度をsetColorDepthで指定します。（省略した場合は16として扱われます。）
    compass.setColorDepth(16);
    base.setColorDepth(16);
// createSpriteで幅と高さを指定してメモリを確保します。
// 消費するメモリは色深度と面積に比例します。大きすぎるとメモリ確保に失敗しますので注意してください。
    compass.createSprite(180,180);
    base.createSprite(120,120);

// base airplane
    base.drawLine( 52, 36, 52, 14, TFT_WHITE); //front L
    base.drawLine(  2, 62, 52, 36, TFT_WHITE); //wing L
    base.drawLine(  4, 76,  2, 62, TFT_WHITE); //wing L
    base.drawLine( 52, 64,  4, 76, TFT_WHITE); //wing L
    base.drawLine( 52, 82, 52, 64, TFT_WHITE); //body L
    base.drawLine( 36,104, 54, 96, TFT_WHITE); //
    base.drawLine( 36,114, 36,104, TFT_WHITE);
    base.drawLine( 58,110, 36,114, TFT_WHITE);
    base.drawLine( 60,116, 58,110, TFT_WHITE);
    base.drawLine( 62,110, 60,116, TFT_WHITE);
    base.drawLine( 84,114, 62,110, TFT_WHITE);
    base.drawLine( 84,104, 84,114, TFT_WHITE);
    base.drawLine( 66, 96, 84,104, TFT_WHITE);
    base.drawLine( 68, 64, 68, 82, TFT_WHITE); //body R
    base.drawLine(116, 76, 68, 64, TFT_WHITE); //wing R
    base.drawLine(118, 62,116, 76, TFT_WHITE); //wing R
    base.drawLine( 68, 36,118, 62, TFT_WHITE); //wing R
    base.drawLine( 68, 14, 68, 36, TFT_WHITE);
    base.drawLine( 54, 96, 52, 82, TFT_WHITE);
    base.drawLine( 66, 96, 68, 82, TFT_WHITE);
    base.drawLine( 52, 14, 54,  4, TFT_WHITE);
    base.drawLine( 68, 14, 66,  4, TFT_WHITE);
    base.drawLine( 66,  4, 60,  0, TFT_WHITE); //head
    base.drawLine( 54,  4, 60,  0, TFT_WHITE); //head
  
// 作成したスプライトはpushSpriteで任意の座標に出力できます。
//  base.pushSprite(30,30,0); // (x,y)=((180-120)/2,(180-120)/2)  compassに出力する。透過色あり
}

void compassplot(float a) {
    int ang;
    a = 360.0 - a;
//  compass.startWrite(); //no effects
    compass.setTextDatum(middle_center);
    compass.setFont(&fonts::Font2);
    compass.fillScreen(0); // fill black

    for (ang=0 ; ang<36 ; ang++) {
        compass.drawLine(90+80*sin((a+ang*10)*DEG_TO_RAD),90-80*cos((a+ang*10)*DEG_TO_RAD),
  90+90*sin((a+ang*10)*DEG_TO_RAD), 90-90*cos((a+ang*10)*DEG_TO_RAD), TFT_WHITE); //0
        compass.setTextSize(0.72);
        compass.setFont(&fonts::Font4);
        if (ang==0) {
            compass.drawString("N", 90+72*sin((a+  0)*DEG_TO_RAD), 90-72*cos((a+  0)*DEG_TO_RAD)); //0
        } else if (ang==9) {
            compass.drawString("E", 90+72*sin((a+ 90)*DEG_TO_RAD), 90-72*cos((a+ 90)*DEG_TO_RAD)); //90
        } else if (ang==18) {
            compass.drawString("S", 90+72*sin((a+180)*DEG_TO_RAD), 90-72*cos((a+180)*DEG_TO_RAD)); //180
        } else if (ang==27) {
            compass.drawString("W", 90+72*sin((a+270)*DEG_TO_RAD), 90-72*cos((a+270)*DEG_TO_RAD)); //270
        } else if ((ang%3)==0) {
            compass.setTextSize(1);
            compass.setFont(&fonts::Font2);
            compass.drawNumber(ang, 90+72*sin((a+ang*10)*DEG_TO_RAD), 90-72*cos((a+ang*10)*DEG_TO_RAD));
        }
    }

    compass.fillTriangle(224-70,  56-30, 228-70,  47-30, 233-70,  52-30, TFT_WHITE);// 45
    compass.fillTriangle(224-70, 184-30, 233-70, 188-30, 228-70, 193-30, TFT_WHITE);//135
    compass.fillTriangle( 96-70, 184-30,  92-70, 193-30,  87-70, 188-30, TFT_WHITE);//225
    compass.fillTriangle( 96-70,  56-30,  87-70,  52-30,  92-70,  47-30, TFT_WHITE);//315
  
// 作成したスプライトはpushSpriteで任意の座標に出力できます。
    base.pushSprite(30,30,0); // (x,y)=((180-120)/2,(180-120)/2)  compassに出力する。透過色あり
    compass.pushSprite(70,30); // (x,y)=((320-180)/2,(240-180)/2) lcdに出力する。透過色なし
//  compass.endWrite();
}

void loop() {
    M5.update();
    bmi2aux.Update();  // Update data from IMU. 更新IMU数据

    float head_dir = bmi2aux.CalcuDir();

    Now = micros();
    deltat = ((Now - lastUpdate) / 1000000.0f);//0.09
    lastUpdate = Now;

    myIMU::MahonyAHRSupdate(
                bmi2aux.gyro_data.x * DEG_TO_RAD, bmi2aux.gyro_data.y * DEG_TO_RAD, bmi2aux.gyro_data.z * DEG_TO_RAD,
                bmi2aux.accel_data.x, bmi2aux.accel_data.y, bmi2aux.accel_data.z,
                bmi2aux.mag_data.x, -bmi2aux.mag_data.y, -bmi2aux.mag_data.z, deltat);

    pitch = asin(-2 * q[1] * q[3] + 2 * q[0]* q[2]); // pitch
    roll  = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2]* q[2] + 1); // roll
    yaw   = atan2(2*(q[1]*q[2] + q[0]*q[3]),q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3]);  //yaw
    yaw = -yaw;
    if(yaw < 0)
        yaw += 2*PI;
    if(yaw > 2*PI)
        yaw -= 2*PI;

    pitch *= RAD_TO_DEG;
    yaw *= RAD_TO_DEG;
    roll *= RAD_TO_DEG;

    compassplot(yaw);
  // for processing display
    Serial.printf(" %5.2f,  %5.2f,  %5.2f  \r\n", roll, pitch, yaw); // to processing

    lcd.startWrite(); //39fps-> 40fps
    lcd.fillTriangle(160,  29, 157,  20, 163,  20, TFT_WHITE);//  0
    lcd.fillTriangle(251, 120, 260, 117, 260, 123, TFT_WHITE);// 90
    lcd.fillTriangle(160, 211, 163, 220, 157, 220, TFT_WHITE);//180
    lcd.fillTriangle( 69, 120,  60, 123,  60, 117, TFT_WHITE);//270

    char text_string[100];
    lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    lcd.drawString("MAG X-Y", 20, 35);
    sprintf(text_string, "%5.1f   ", head_dir);
    lcd.drawString(text_string, 30, 50);
    lcd.drawString("Heading", 20, 65);
    sprintf(text_string, "%5.1f   ", yaw);
    lcd.drawString(text_string, 30, 80);
    sprintf(text_string, "fps:%d   ", fps);
    lcd.drawString(text_string, 270, 215);

    ++frame_count;
    uint32_t fsec = m5gfx::millis() / 1000;
    if (psec != fsec)
    {
        psec = fsec;
        fps = frame_count;
        frame_count = 0;
    }

    lcd.setCursor(40, 230);
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

        lcd.clear();        // 黒で塗り潰し
    }
}