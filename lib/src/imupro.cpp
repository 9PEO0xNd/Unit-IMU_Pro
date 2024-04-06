#include "M5Unified.hpp"
#include "imupro.h"

bmi2_dev aux_bmi2_dev;
bmm150_dev aux_bmm150_dev;

bool bmm150_init_flag = false;  // FIXME: dirty code

imupro::imupro() {
}

imupro::~imupro() {
}

void imupro::Init() {
    config[ACCEL].type = BMI2_ACCEL;
    config[GYRO].type  = BMI2_GYRO;
    config[AUX].type   = BMI2_AUX;

    /* To enable the i2c interface settings for bmi270. */
    aux_bmi2_dev.intf            = BMI2_I2C_INTF;
    aux_bmi2_dev.read            = bmi2_i2c_read;
    aux_bmi2_dev.write           = bmi2_i2c_write;
    aux_bmi2_dev.delay_us        = bmi2_delay_us;
    aux_bmi2_dev.read_write_len  = 30;
    aux_bmi2_dev.config_file_ptr = NULL;
    aux_bmi2_dev.intf_ptr        = (void *)&BMI270_ADDR;

    /* To enable the i2c interface settings for bmm150. */
    aux_bmm150_dev.intf_ptr = (void *)&BMM150_ADDR;
    aux_bmm150_dev.read     = aux_i2c_read;
    aux_bmm150_dev.write    = aux_i2c_write;
    aux_bmm150_dev.delay_us = bmi2_delay_us;
    /* As per datasheet, aux interface with bmi270 will support only for I2C */
    aux_bmm150_dev.intf = BMM150_I2C_INTF;

    /* Initialize bmi270. */
    int8_t ret = bmi270_init(&aux_bmi2_dev);
    bmi2_error_codes_print_result(ret); // extraced common.c

    /* Pull-up resistor 2k is set to the trim regiter */
    uint8_t regdata = BMI2_ASDA_PUPSEL_2K;
    ret = bmi2_set_regs(BMI2_AUX_IF_TRIM, &regdata, 1, &aux_bmi2_dev);
    bmi2_error_codes_print_result(ret); // extraced common.c

    /* Get default configurations for the type of feature selected. */
    ret = bmi270_get_sensor_config(config, 3, &aux_bmi2_dev);
    bmi2_error_codes_print_result(ret); // extraced common.c

    /* Configurations for accel. */
    config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    config[ACCEL].cfg.acc.bwp         = BMI2_ACC_OSR2_AVG2;
    config[ACCEL].cfg.acc.odr         = BMI2_ACC_ODR_100HZ;
    config[ACCEL].cfg.acc.range       = BMI2_ACC_RANGE_2G;

    /* Configurations for gyro. */
    config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    config[GYRO].cfg.gyr.noise_perf  = BMI2_GYR_RANGE_2000;
    config[GYRO].cfg.gyr.bwp         = BMI2_GYR_OSR2_MODE;
    config[GYRO].cfg.gyr.odr         = BMI2_GYR_ODR_100HZ;
    config[GYRO].cfg.gyr.range       = BMI2_GYR_RANGE_2000;
    config[GYRO].cfg.gyr.ois_range   = BMI2_GYR_OIS_2000;

    /* Configurations for aux. */
    config[AUX].cfg.aux.odr             = BMI2_AUX_ODR_100HZ;
    config[AUX].cfg.aux.aux_en          = BMI2_ENABLE;
    config[AUX].cfg.aux.i2c_device_addr = BMM150_DEFAULT_I2C_ADDRESS;
    config[AUX].cfg.aux.fcu_write_en    = BMI2_ENABLE;
    config[AUX].cfg.aux.aux_rd_burst = BMI2_AUX_READ_LEN_3; // added
    config[AUX].cfg.aux.man_rd_burst    = BMI2_AUX_READ_LEN_3;
    config[AUX].cfg.aux.read_addr       = BMM150_REG_DATA_X_LSB;

    /* Set new configurations for accel, gyro and aux. */
    ret = bmi270_set_sensor_config(config, 3, &aux_bmi2_dev);
    bmi2_error_codes_print_result(ret); // extraced common.c

    /* NOTE:
     * Accel and gyro enable must be done after setting configurations
     */
    ret = bmi270_sensor_enable(sensor_list, 3, &aux_bmi2_dev);
    bmi2_error_codes_print_result(ret); // extraced common.c

    /* Initialize bmm150. */
    ret = bmm150_init(&aux_bmm150_dev);
//    bmm150_error_codes_print_result("bmm_sensor_enable",ret); // added

    /* Set the power mode to normal mode. */
    bmm_settings.pwr_mode = BMM150_POWERMODE_NORMAL;
    ret                   = bmm150_set_op_mode(&bmm_settings, &aux_bmm150_dev);
    bmm150_error_codes_print_result("bmm_set_op_mode",ret); // added

    ret = bmi270_get_sensor_config(config, 3, &aux_bmi2_dev);
    bmi2_error_codes_print_result(ret); // extraced common.c

    /* Disable manual mode so that the data mode is enabled. */
    config[AUX].cfg.aux.manual_en = BMI2_DISABLE;

    /* Set the aux configurations. */
    ret = bmi270_set_sensor_config(config, 3, &aux_bmi2_dev);
    bmi2_error_codes_print_result(ret); // extraced common.c

    /* Map data ready interrupt to interrupt pin. */
    ret = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, &aux_bmi2_dev);
    bmi2_error_codes_print_result(ret); // extraced common.c


    if (aux_bmm150_dev.chip_id == BMM150_CHIP_ID) {
        bmm150_init_flag = true;
    }
    ReadCalibration();
}

void imupro::Update() {
    aux_bmi2_dev.delay_us(1000, aux_bmi2_dev.intf_ptr);

    /* To get the data ready interrupt status of accel, gyro and aux */
    bmi2_get_int_status(&bmi_bmm_init_status, &aux_bmi2_dev);

    /* To check the data ready interrupt status and print the status for 20
     * samples. */
    if ((bmi_bmm_init_status & BMI2_AUX_DRDY_INT_MASK)) {
        int8_t ret = bmi2_get_sensor_data(&bmi_sensor_data, &aux_bmi2_dev);

        /* Converting lsb to meter per second squared for 16 bit
         * accelerometer at 2G range. */
        accel_data.x =
            lsb_to_mps2(bmi_sensor_data.acc.x, 2, aux_bmi2_dev.resolution);
        accel_data.y =
            lsb_to_mps2(bmi_sensor_data.acc.y, 2, aux_bmi2_dev.resolution);
        accel_data.z =
            lsb_to_mps2(bmi_sensor_data.acc.z, 2, aux_bmi2_dev.resolution);

        /* Converting lsb to degree per second for 16 bit gyro at 2000dps range.
         */
        gyro_data.x =
            lsb_to_dps(bmi_sensor_data.gyr.x, 2000, aux_bmi2_dev.resolution);
        gyro_data.y =
            lsb_to_dps(bmi_sensor_data.gyr.y, 2000, aux_bmi2_dev.resolution);
        gyro_data.z =
            lsb_to_dps(bmi_sensor_data.gyr.z, 2000, aux_bmi2_dev.resolution);

        if (ret == BMI2_OK) {

            /* To read the compensated mag data */
            ret = bmm150_aux_mag_data(bmi_sensor_data.aux_data, &bmm_mag_data, &aux_bmm150_dev);
            bmm150_error_codes_print_result("bmm150_aux_mag_data", ret);

            mag_data.raw.x = bmm_mag_data.x;
            mag_data.raw.y = bmm_mag_data.y;
            mag_data.raw.z = bmm_mag_data.z;

            mag_data.x = mag_data.raw.x - mag_data.offset.x;
            mag_data.y = mag_data.raw.y - mag_data.offset.y;
            mag_data.z = mag_data.raw.z - mag_data.offset.z;
        }
    }
}

float imupro::CalcuDir(void) {
    float xyHeading = atan2(mag_data.x, mag_data.y);
    float zxHeading = atan2(mag_data.z, mag_data.x);
    float heading   = xyHeading - PI/2;

    if (heading < 0) {
        heading += 2 * PI;
    }
    if (heading > 2 * PI) {
        heading -= 2 * PI;
    }
    return (heading * 180 / M_PI);
}

void imupro::MagCalibration() {
    float value_x_min =  2000;
    float value_x_max = -2000;
    float value_y_min =  2000;
    float value_y_max = -2000;
    float value_z_min =  2000;
    float value_z_max = -2000;

    time_t start = millis();
    while ((millis() - start) < 10000) {
        Update();
        value_x_min = (mag_data.raw.x < value_x_min) ? mag_data.raw.x : value_x_min;
        value_x_max = (mag_data.raw.x > value_x_max) ? mag_data.raw.x : value_x_max;

        value_y_min = (mag_data.raw.y < value_y_min) ? mag_data.raw.y : value_y_min;
        value_y_max = (mag_data.raw.y > value_y_max) ? mag_data.raw.y : value_y_max;

        value_z_min = (mag_data.raw.z < value_z_min) ? mag_data.raw.z : value_z_min;
        value_z_max = (mag_data.raw.z > value_z_max) ? mag_data.raw.z : value_z_max;

        delay(100);
    }

    mag_data.offset.x = value_x_min + (value_x_max - value_x_min) / 2;
    mag_data.offset.y = value_y_min + (value_y_max - value_y_min) / 2;
    mag_data.offset.z = value_z_min + (value_z_max - value_z_min) / 2;

// offset save
    SaveCalibration();
}

void imupro::SaveCalibration()
{
    prefs.begin("bmi2aux", false);
    prefs.putBytes("offset", (uint8_t *)&mag_data.offset, sizeof(Mag));
    // added soft-iron scale
    //prefs.putBytes("scale", (uint8_t *)&mag_scale, sizeof(bmm150_mag_data));
    prefs.end();
}

void imupro::ReadCalibration()
{
    if (prefs.begin("bmi2aux", true))
    {
        prefs.getBytes("offset", (uint8_t *)&mag_data.offset, sizeof(Mag));
        // added soft iron distortion scaling
        //prefs.getBytes("scale", (uint8_t *)&mag_scale, sizeof(bmm150_mag_data));
        prefs.end();
        Serial.printf("bmm150 load offset finish.... \r\n");
    }
    else
    {
        Serial.printf("bmm150 load offset failed.... \r\n");
    }
}

/******************************************************************************/
int8_t imupro::bmi2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                              void *intf_ptr) {
    if ((reg_data == NULL) || (len == 0) || (len > 32)) {
        return -1;
    }
    uint8_t bytes_received;

    uint8_t dev_addr = *(uint8_t *)intf_ptr;

    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    if (Wire.endTransmission() == 0) {
        bytes_received = Wire.requestFrom(dev_addr, len);
        // Optionally, throw an error if bytes_received != len
        for (uint16_t i = 0; i < bytes_received; i++) {
            reg_data[i] = Wire.read();
        }
    } else {
        return -1;
    }
    return 0;
}

int8_t imupro::bmi2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
                               uint32_t len, void *intf_ptr) {
    if ((reg_data == NULL) || (len == 0) || (len > 32)) {
        return -1;
    }

    uint8_t dev_addr = *(uint8_t *)intf_ptr;

    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    for (uint16_t i = 0; i < len; i++) {
        Wire.write(reg_data[i]);
    }
    if (Wire.endTransmission() != 0) {
        return -1;
    }
    return 0;
}

void imupro::bmi2_delay_us(uint32_t period, void *intf_ptr) {
    delayMicroseconds(period);
}

int8_t imupro::aux_i2c_read(uint8_t reg_addr, uint8_t *reg_data,
                             uint32_t length, void *intf_ptr) {
    int8_t ret;
    ret = bmi2_read_aux_man_mode(reg_addr, reg_data, (uint16_t)length, &aux_bmi2_dev);
    return ret;
}
int8_t imupro::aux_i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
                              uint32_t length, void *intf_ptr) {
    int8_t ret;
    ret = bmi2_write_aux_man_mode(reg_addr, reg_data, (uint16_t)length, &aux_bmi2_dev);
    return ret;
}

float imupro::lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width) {
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

float imupro::lsb_to_dps(int16_t val, float dps, uint8_t bit_width) {
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (dps / ((half_scale))) * (val);
}

// bmi, bmm error log
void imupro::bmm150_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BMM150_OK)
    {
        M5_LOGE("%s\t", api_name);

        switch (rslt)
        {
            case BMM150_E_NULL_PTR:
                M5_LOGE("Error [%d] : Null pointer error.", rslt);
                M5_LOGE(
                    "It occurs when the user tries to assign value (not address) to a pointer, which has been initialized to NULL.\r\n");
                break;

            case BMM150_E_COM_FAIL:
                M5_LOGE("Error [%d] : Communication failure error.", rslt);
                M5_LOGE(
                    "It occurs due to read/write operation failure and also due to power failure during communication\r\n");
                break;

            case BMM150_E_DEV_NOT_FOUND:
                M5_LOGE("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
                       rslt);
                break;

            case BMM150_E_INVALID_CONFIG:
                M5_LOGE("Error [%d] : Invalid sensor configuration.", rslt);
                M5_LOGE(" It occurs when there is a mismatch in the requested feature with the available one\r\n");
                break;

            default:
                M5_LOGE("Error [%d] : Unknown error code\r\n", rslt);
                break;
        }
    }
}

void imupro::bmi2_error_codes_print_result(int8_t rslt)
{
    switch (rslt)
    {
        case BMI2_OK:

            ///* Do nothing

            break;

        case BMI2_W_FIFO_EMPTY:
            M5_LOGE("Warning [%d] : FIFO empty\r\n", rslt);
            break;
        case BMI2_W_PARTIAL_READ:
            M5_LOGE("Warning [%d] : FIFO partial read\r\n", rslt);
            break;
        case BMI2_E_NULL_PTR:
            M5_LOGE(
                "Error [%d] : Null pointer error. It occurs when the user tries to assign value (not address) to a pointer," " which has been initialized to NULL.\r\n",
                rslt);
            break;

        case BMI2_E_COM_FAIL:
            M5_LOGE(
                "Error [%d] : Communication failure error. It occurs due to read/write operation failure and also due " "to power failure during communication\r\n",
                rslt);
            break;

        case BMI2_E_DEV_NOT_FOUND:
            M5_LOGE("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
                   rslt);
            break;

        case BMI2_E_INVALID_SENSOR:
            M5_LOGE(
                "Error [%d] : Invalid sensor error. It occurs when there is a mismatch in the requested feature with the " "available one\r\n",
                rslt);
            break;

        case BMI2_E_SELF_TEST_FAIL:
            M5_LOGE(
                "Error [%d] : Self-test failed error. It occurs when the validation of accel self-test data is " "not satisfied\r\n",
                rslt);
            break;

        case BMI2_E_INVALID_INT_PIN:
            M5_LOGE(
                "Error [%d] : Invalid interrupt pin error. It occurs when the user tries to configure interrupt pins " "apart from INT1 and INT2\r\n",
                rslt);
            break;

        case BMI2_E_OUT_OF_RANGE:
            M5_LOGE(
                "Error [%d] : Out of range error. It occurs when the data exceeds from filtered or unfiltered data from " "fifo and also when the range exceeds the maximum range for accel and gyro while performing FOC\r\n",
                rslt);
            break;

        case BMI2_E_ACC_INVALID_CFG:
            M5_LOGE(
                "Error [%d] : Invalid Accel configuration error. It occurs when there is an error in accel configuration" " register which could be one among range, BW or filter performance in reg address 0x40\r\n",
                rslt);
            break;

        case BMI2_E_GYRO_INVALID_CFG:
            M5_LOGE(
                "Error [%d] : Invalid Gyro configuration error. It occurs when there is a error in gyro configuration" "register which could be one among range, BW or filter performance in reg address 0x42\r\n",
                rslt);
            break;

        case BMI2_E_ACC_GYR_INVALID_CFG:
            M5_LOGE(
                "Error [%d] : Invalid Accel-Gyro configuration error. It occurs when there is a error in accel and gyro" " configuration registers which could be one among range, BW or filter performance in reg address 0x40 " "and 0x42\r\n",
                rslt);
            break;

        case BMI2_E_CONFIG_LOAD:
            M5_LOGE(
                "Error [%d] : Configuration load error. It occurs when failure observed while loading the configuration " "into the sensor\r\n",
                rslt);
            break;

        case BMI2_E_INVALID_PAGE:
            M5_LOGE(
                "Error [%d] : Invalid page error. It occurs due to failure in writing the correct feature configuration " "from selected page\r\n",
                rslt);
            break;

        case BMI2_E_SET_APS_FAIL:
            M5_LOGE(
                "Error [%d] : APS failure error. It occurs due to failure in write of advance power mode configuration " "register\r\n",
                rslt);
            break;

        case BMI2_E_AUX_INVALID_CFG:
            M5_LOGE(
                "Error [%d] : Invalid AUX configuration error. It occurs when the auxiliary interface settings are not " "enabled properly\r\n",
                rslt);
            break;

        case BMI2_E_AUX_BUSY:
            M5_LOGE(
                "Error [%d] : AUX busy error. It occurs when the auxiliary interface buses are engaged while configuring" " the AUX\r\n",
                rslt);
            break;

        case BMI2_E_REMAP_ERROR:
            M5_LOGE(
                "Error [%d] : Remap error. It occurs due to failure in assigning the remap axes data for all the axes " "after change in axis position\r\n",
                rslt);
            break;

        case BMI2_E_GYR_USER_GAIN_UPD_FAIL:
            M5_LOGE(
                "Error [%d] : Gyro user gain update fail error. It occurs when the reading of user gain update status " "fails\r\n",
                rslt);
            break;

        case BMI2_E_SELF_TEST_NOT_DONE:
            M5_LOGE(
                "Error [%d] : Self-test not done error. It occurs when the self-test process is ongoing or not " "completed\r\n",
                rslt);
            break;

        case BMI2_E_INVALID_INPUT:
            M5_LOGE("Error [%d] : Invalid input error. It occurs when the sensor input validity fails\r\n", rslt);
            break;

        case BMI2_E_INVALID_STATUS:
            M5_LOGE("Error [%d] : Invalid status error. It occurs when the feature/sensor validity fails\r\n", rslt);
            break;

        case BMI2_E_CRT_ERROR:
            M5_LOGE("Error [%d] : CRT error. It occurs when the CRT test has failed\r\n", rslt);
            break;

        case BMI2_E_ST_ALREADY_RUNNING:
            M5_LOGE(
                "Error [%d] : Self-test already running error. It occurs when the self-test is already running and " "another has been initiated\r\n",
                rslt);
            break;

        case BMI2_E_CRT_READY_FOR_DL_FAIL_ABORT:
            M5_LOGE(
                "Error [%d] : CRT ready for download fail abort error. It occurs when download in CRT fails due to wrong " "address location\r\n",
                rslt);
            break;

        case BMI2_E_DL_ERROR:
            M5_LOGE(
                "Error [%d] : Download error. It occurs when write length exceeds that of the maximum burst length\r\n",
                rslt);
            break;

        case BMI2_E_PRECON_ERROR:
            M5_LOGE(
                "Error [%d] : Pre-conditional error. It occurs when precondition to start the feature was not " "completed\r\n",
                rslt);
            break;

        case BMI2_E_ABORT_ERROR:
            M5_LOGE("Error [%d] : Abort error. It occurs when the device was shaken during CRT test\r\n", rslt);
            break;

        case BMI2_E_WRITE_CYCLE_ONGOING:
            M5_LOGE(
                "Error [%d] : Write cycle ongoing error. It occurs when the write cycle is already running and another " "has been initiated\r\n",
                rslt);
            break;

        case BMI2_E_ST_NOT_RUNING:
            M5_LOGE(
                "Error [%d] : Self-test is not running error. It occurs when self-test running is disabled while it's " "running\r\n",
                rslt);
            break;

        case BMI2_E_DATA_RDY_INT_FAILED:
            M5_LOGE(
                "Error [%d] : Data ready interrupt error. It occurs when the sample count exceeds the FOC sample limit " "and data ready status is not updated\r\n",
                rslt);
            break;

        case BMI2_E_INVALID_FOC_POSITION:
            M5_LOGE(
                "Error [%d] : Invalid FOC position error. It occurs when average FOC data is obtained for the wrong" " axes\r\n",
                rslt);
            break;

        default:
            M5_LOGE("Error [%d] : Unknown error code\r\n", rslt);
            break;
    }
}
