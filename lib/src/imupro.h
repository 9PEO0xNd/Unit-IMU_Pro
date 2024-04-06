
#ifndef _IMUPRO_H_
#define _IMUPRO_H_
#include <Arduino.h>
#include <M5Unified.h>
//#include <nvs.h>
#include "Preferences.h"
#include "Wire.h"
#include "BMI270.h"
#include "BMM150.h"
#define ACCEL UINT8_C(0x00)
#define GYRO  UINT8_C(0x01)
#define AUX   UINT8_C(0x02)

#define GRAVITY_EARTH (9.80665f)
//#define BMM150_USE_FLOATING_POINT //no effects

class imupro {
   public:
    struct Accel {
        float x;
        float y;
        float z;
    } accel_data;
    struct Gyro {
        float x;
        float y;
        float z;
    } gyro_data;

    struct Mag {
        float x;
        float y;
        float z;
        struct Raw {
            float x;
            float y;
            float z;
        } raw;
        struct Offset {
            float x;
            float y;
            float z;
        } offset;
    } mag_data;

   public:
    Preferences prefs;

    imupro();
    ~imupro();

    void Init();
    void Update();
    float CalcuDir();
    void MagCalibration();

    void ReadCalibration();
    void SaveCalibration();

   private:
    const uint8_t BMI270_ADDR = BMI2_I2C_SEC_ADDR;
    const uint8_t BMM150_ADDR = BMM150_DEFAULT_I2C_ADDRESS;
    uint8_t sensor_list[3]    = {BMI2_ACCEL, BMI2_GYRO, BMI2_AUX};
    uint16_t bmi_bmm_init_status;

    bmi2_sens_config config[3];
    bmi2_sens_data bmi_sensor_data = {{0}};

    bmm150_settings bmm_settings;
    bmm150_mag_data bmm_mag_data;
    bmm150_raw_mag_data bmm_raw_mag_data;
    struct bmm150_dev dev;

    int _deviceAddress;

    static int8_t bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data,
                                uint32_t len, void *intf_ptr);
    static int8_t bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
                                 uint32_t len, void *intf_ptr);
    static void bmi2_delay_us(uint32_t period, void *intf_ptr);

    static int8_t aux_i2c_read(uint8_t reg_addr, uint8_t *reg_data,
                               uint32_t length, void *intf_ptr);
    static int8_t aux_i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
                                uint32_t length, void *intf_ptr);

    float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width);

    float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);

// debug
    void bmm150_error_codes_print_result(const char api_name[], int8_t rslt);
    void bmi2_error_codes_print_result(int8_t rslt); // common.c and common.h file

};

#endif