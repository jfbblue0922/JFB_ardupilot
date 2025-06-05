/*
 *  This program is free software
*/
#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor_LSM6DSO.h"
#include "AP_InertialSensor_LSM6DSO_registers.h"

#include <utility>

#include <AP_HAL/GPIO.h>

extern const AP_HAL::HAL& hal;

#define GYRO_SCALE      (G_SCALE_2000DPS)
#define ACCEL_SCALE     (A_SCALE_16G)

AP_InertialSensor_LSM6DSO::AP_InertialSensor_LSM6DSO(AP_InertialSensor &imu,
                                                     AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                                     enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
    , _rotation(rotation)
{
}

AP_InertialSensor_Backend *AP_InertialSensor_LSM6DSO::probe(AP_InertialSensor &_imu,
                                                            AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                                            enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    AP_InertialSensor_LSM6DSO *sensor =
        NEW_NOTHROW AP_InertialSensor_LSM6DSO(_imu,std::move(dev),
                                              rotation);
    if (!sensor || !sensor->_init_sensor()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

bool AP_InertialSensor_LSM6DSO::_init_sensor()
{
    _spi_sem = _dev->get_semaphore();

    bool success = _hardware_init();

#if LSM6DSO_DEBUG
    _dump_registers();
#endif
    return success;
}

bool AP_InertialSensor_LSM6DSO::_hardware_init()
{
    _spi_sem->take_blocking();

    uint8_t tries;
    uint8_t whoami;

    // set flag for reading registers
    _dev->set_read_flag(0x80);

    whoami = _register_read(LSM6DSO_REG_WHO_AM_I);
    if (whoami != WHO_AM_I) {
        DEV_PRINTF("LSM6DSO: unexpected acc/gyro WHOAMI 0x%x\n", whoami);
        goto fail_whoami;
    }

    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    for (tries = 0; tries < 5; tries++) {
        if (_chip_reset()) {
            _common_init();
            _fifo_init();
            _gyro_init(GYRO_SCALE);
            _accel_init(ACCEL_SCALE);

            _set_gyro_scale(GYRO_SCALE);
            _set_accel_scale(ACCEL_SCALE);

            hal.scheduler->delay(50);

            // if samples == 0 -> FIFO empty
            if (_get_count_fifo_unread_data() > 0) {
                break;
            }

#if LSM6DSO_DEBUG
            _dump_registers();
#endif
        }
    }

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    if (tries == 5) {
        DEV_PRINTF("Failed to boot LSM6DSO 5 times\n\n");
        goto fail_tries;
    }

    _spi_sem->give();
    return true;

fail_tries:
fail_whoami:
    _spi_sem->give();
    return false;
}

/*
  start the sensor going
 */
void AP_InertialSensor_LSM6DSO::start(void)
{
    if (!_imu.register_gyro(gyro_instance, 833, _dev->get_bus_id_devtype(DEVTYPE_INS_LSM6DSO)) ||
        !_imu.register_accel(accel_instance, 833, _dev->get_bus_id_devtype(DEVTYPE_INS_LSM6DSO))) {
        return;
    }

    set_accel_orientation(accel_instance, _rotation);
    set_gyro_orientation(gyro_instance, _rotation);

    // ■ 加速度キャリブレーション時に使用する最大絶対オフセットらしい
    // ■ 後で考える。使用箇所不明・・・。
    // _set_accel_max_abs_offset(accel_instance, 5.0f);

    _fifo_reset();

    // start the timer process to read samples
    _dev->register_periodic_callback(1000, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_LSM6DSO::_poll_data, void));
}

uint8_t AP_InertialSensor_LSM6DSO::_register_read(uint8_t reg)
{
    uint8_t val = 0;
    _dev->read_registers(reg, &val, 1);
    return val;
}

void AP_InertialSensor_LSM6DSO::_register_write(uint8_t reg, uint8_t val, bool checked)
{
    _dev->write_register(reg, val, checked);
}

bool AP_InertialSensor_LSM6DSO::_chip_reset()
{
    // CTRL3_C(12h) : 01h
    //      SW_RESET = 1b (reset device)
    _register_write(LSM6DSO_REG_CTRL3_C, LSM6DSO_REG_CTRL3_C_SW_RESET_RESET);
    hal.scheduler->delay(1);

    for (int tries = 0; tries < 5; tries++) {
        uint8_t ctrl3_c = _register_read(LSM6DSO_REG_CTRL3_C);
        if ((ctrl3_c & 0x01) == LSM6DSO_REG_CTRL3_C_SW_RESET_NORMAL) {
            return true;
        }

        hal.scheduler->delay(2);
    }

    return false;
}

void AP_InertialSensor_LSM6DSO::_fifo_reset()
{
    uint8_t fifo_ctrl4 = _register_read(LSM6DSO_REG_FIFO_CTRL4);

    // FIFO_MODE is Bypass mode
    _register_write(LSM6DSO_REG_FIFO_CTRL4, fifo_ctrl4 |
                                            LSM6DSO_REG_FIFO_CTRL4_FIFO_MODE_BYPASS);
    hal.scheduler->delay(1);

    // Revert FIFO_MODE
    _register_write(LSM6DSO_REG_FIFO_CTRL4, fifo_ctrl4);
    hal.scheduler->delay(1);

    notify_accel_fifo_reset(accel_instance);
    notify_gyro_fifo_reset(gyro_instance);
}

void AP_InertialSensor_LSM6DSO::_common_init()
{
    // CTRL3_C(12h) : 44h
    //      BOOT      = 0b (Reboots memory content is normal mode)
    //      BDU       = 1b (Block data update Enable)
    //      H_LACTIVE = 0b (interrupt output pins active high)
    //      PP_OD     = 0b (INT1 and INT2 pins push-pull mode)
    //      SIM       = 0b (SPI is 4-wire interface)
    //      IF_INC    = 1b (Register address automatically incremented Enable)
    //      SW_RESET  = 0b (Software reset is normal mode)
    _register_write(LSM6DSO_REG_CTRL3_C, LSM6DSO_REG_CTRL3_C_BOOT_NORMAL |
                                         LSM6DSO_REG_CTRL3_C_BDU_ENABLE |
                                         LSM6DSO_REG_CTRL3_C_H_LACTIVE_ACTIVE_HIGH |
                                         LSM6DSO_REG_CTRL3_C_PP_OD_PP |
                                         LSM6DSO_REG_CTRL3_C_SIM_4_WIRE |
                                         LSM6DSO_REG_CTRL3_C_IF_INC_ENABLE |
                                         LSM6DSO_REG_CTRL3_C_SW_RESET_NORMAL);
    hal.scheduler->delay(1);

    // CTRL4_C(13h) : 00h
    _register_write(LSM6DSO_REG_CTRL4_C, 0x00);
    hal.scheduler->delay(1);

    // CTRL5_C(14h) : 00h
    _register_write(LSM6DSO_REG_CTRL5_C, 0x00);
    hal.scheduler->delay(1);

    // CTRL6_C(15h) : 00h
    _register_write(LSM6DSO_REG_CTRL6_C, 0x00);
    hal.scheduler->delay(1);

    // CTRL10_C(19h) : 00h
    _register_write(LSM6DSO_REG_CTRL10_C, 0x00);
    hal.scheduler->delay(1);
}

void AP_InertialSensor_LSM6DSO::_fifo_init()
{
    // FIFO_CTRL1(07h) : 00h
    _register_write(LSM6DSO_REG_FIFO_CTRL1, 0x00);
    hal.scheduler->delay(1);

    // FIFO_CTRL2(08h) : 00h
    _register_write(LSM6DSO_REG_FIFO_CTRL2, 0x00);
    hal.scheduler->delay(1);

    // FIFO_CTRL3(09h) : 77h
    //      BDR_GY = 0111b (833Hz)
    //      BDR_XL = 0111b (833Hz)
    _register_write(LSM6DSO_REG_FIFO_CTRL3, LSM6DSO_REG_FIFO_CTRL3_BDR_GY_833Hz |
                                            LSM6DSO_REG_FIFO_CTRL3_BDR_XL_833Hz);
    hal.scheduler->delay(1);

    // FIFO_CTRL4(0Ah) : 06h
    //      DEC_TS_BATCH = 00b (timestamp not batched)
    //      ODR_T_BATCH  = 00b (temperature not batched)
    //      FIFO_MODE    = 110b (Continuous mode)
    _register_write(LSM6DSO_REG_FIFO_CTRL4, LSM6DSO_REG_FIFO_CTRL4_DEC_TS_BATCH_NOT_BATCH |
                                            LSM6DSO_REG_FIFO_CTRL4_DEC_ODR_T_BATCH_NOT_BATCH |
                                            LSM6DSO_REG_FIFO_CTRL4_FIFO_MODE_CONT);
    hal.scheduler->delay(1);
}

void AP_InertialSensor_LSM6DSO::_gyro_init(gyro_scale scale)
{
    // CTRL7_G(16h) : 00h
    //      G_HM_MODE      = 0b (high-performance operating mode)
    //      HP_EN_G        = 0b (HPF disabled)
    //      HPM_G          = 00b (HPF cutoff 16mHz)
    //      OIS_ON_EN      = 0b (with SPI2 interface)
    //      USR_OFF_ON_OUT = 0b (accelerometer user offset correction block bypassed)
    //      OIS_ON         = 0b (OIS disabled)
    _register_write(LSM6DSO_REG_CTRL7_G, LSM6DSO_REG_CTRL7_G_HM_MODE_ENABLE |
                                         LSM6DSO_REG_CTRL7_G_HP_EN_G_DISABLE |
                                         LSM6DSO_REG_CTRL7_G_HPM_G_16mHz |
                                         LSM6DSO_REG_CTRL7_G_OIS_ON_EN_SPI |
                                         LSM6DSO_REG_CTRL7_G_USR_OFF_ON_OUT_BYPASS |
                                         LSM6DSO_REG_CTRL7_G_OIS_ON_DISABLE);
    hal.scheduler->delay(1);

    // CTRL2_G(11h) : 0111XXX0b
    //      ODR  = 0111b (833Hz (high performance))
    uint8_t fs_g;
    switch (scale) {
    case G_SCALE_125DPS:
        fs_g = LSM6DSO_REG_CTRL2_G_FS_G_125PS;
        break;
    case G_SCALE_250DPS:
        fs_g = LSM6DSO_REG_CTRL2_G_FS_G_250DPS;
        break;
    case G_SCALE_500DPS:
        fs_g = LSM6DSO_REG_CTRL2_G_FS_G_500DPS;
        break;
    case G_SCALE_1000DPS:
        fs_g = LSM6DSO_REG_CTRL2_G_FS_G_1000DPS;
        break;
    case G_SCALE_2000DPS:
        fs_g = LSM6DSO_REG_CTRL2_G_FS_G_2000DPS;
        break;
    default:
        fs_g = LSM6DSO_REG_CTRL2_G_FS_G_2000DPS;
        break;
    }

    _register_write(LSM6DSO_REG_CTRL2_G, LSM6DSO_REG_CTRL2_G_ODR_G_833Hz |
                                         fs_g);
    hal.scheduler->delay(1);
}

void AP_InertialSensor_LSM6DSO::_accel_init(accel_scale scale)
{
    // CTRL8_XL(17h) : 02h
    //      HPCF_XL           = 000b (ODR / 4)
    //      HP_REF_MODE_XL    = 0b (HPF Reference Mode Disable)
    //      FASTSETTL_MODE_XL = 0b (LPF2 and HPF fast-setting Mode Disable)
    //      HP_SLOPE_XL_EN    = 0b (Slope filter / HPF Disable)
    //      XL_FS_MODE        = 0b (old full-scale mode)
    //      LOW_PASS_ON_6D    = 0b (ODR/2 low-pass filtered data sent to 6D interrupt function)
    _register_write(LSM6DSO_REG_CTRL8_XL, LSM6DSO_REG_CTRL8_XL_HPCF_XL_ODR_PER_4 |
                                          LSM6DSO_REG_CTRL8_XL_HP_REF_MODE_XL_DISABLE |
                                          LSM6DSO_REG_CTRL8_XL_FASTSETTL_MODE_XL_DISABLE |
                                          LSM6DSO_REG_CTRL8_XL_HP_SLOPE_XL_EN_DISABLE |
                                          LSM6DSO_REG_CTRL8_XL_XL_FS_MODE_OLD |
                                          LSM6DSO_REG_CTRL8_XL_LOW_PASS_ON_6D_LPF1);
    hal.scheduler->delay(1);

    // CTRL9_XL(18h) : E0h
    //      DEN_X       = 1b (DEN stored in X-axis LSB)
    //      DEN_Y       = 1b (DEN stored in Y-axis LSB)
    //      DEN_Z       = 1b (DEN stored in Z-axis LSB)
    //      DEN_XL_G    = 0b (DEN pin info stamped in the gyroscope axis)
    //      DEN_XL_EN   = 0b (Extends DEN functionality to accelerometer sensor Disabled)
    //      DEN_LH      = 0b (active low)
    //      I3C_disable = 0b (MIPI I3C interfaces enabled)
    _register_write(LSM6DSO_REG_CTRL9_XL, LSM6DSO_REG_CTRL8_XL_DEN_X_ENABLE |
                                          LSM6DSO_REG_CTRL8_XL_DEN_Y_ENABLE |
                                          LSM6DSO_REG_CTRL8_XL_DEN_Z_ENABLE |
                                          LSM6DSO_REG_CTRL9_XL_DEN_XL_G_GYRO |
                                          LSM6DSO_REG_CTRL9_XL_DEN_XL_EN_EXT_DISABLE |
                                          LSM6DSO_REG_CTRL9_XL_DEN_LH_ACTIVE_LOW |
                                          LSM6DSO_REG_CTRL8_XL_DEN_I3C_DISABLE );
    hal.scheduler->delay(1);

    // CTRL1_XL(10h) :  0111XX00b
    //      ODR        = 0111b (833Hz (high performance))
    //      LPF2_XL_EN = 0b (LPF2 Disable)
    uint8_t fs_xl;
    switch (scale) {
    case A_SCALE_2G:
        fs_xl = LSM6DSO_REG_CTRL1_XL_FS_XL_2G;
        break;
    case A_SCALE_4G:
        fs_xl = LSM6DSO_REG_CTRL1_XL_FS_XL_4G;
        break;
    case A_SCALE_8G:
        fs_xl = LSM6DSO_REG_CTRL1_XL_FS_XL_8G;
        break;
    case A_SCALE_16G:
        fs_xl = LSM6DSO_REG_CTRL1_XL_FS_XL_16G;
        break;
    default:
        fs_xl = LSM6DSO_REG_CTRL1_XL_FS_XL_16G;
        break;
    }

    _register_write(LSM6DSO_REG_CTRL1_XL, LSM6DSO_REG_CTRL1_XL_ODR_XL_HM_833Hz |
                                          fs_xl |
                                          LSM6DSO_REG_CTRL1_XL_LPF2_XL_EN_DISABLE);
    hal.scheduler->delay(1);
}

uint16_t AP_InertialSensor_LSM6DSO::_get_count_fifo_unread_data()
{
    const uint8_t _reg = LSM6DSO_REG_FIFO_STATUS1 | 0x80;
    uint16_t tmp;
    uint16_t samples;

    if (_dev->transfer(&_reg, 1, (uint8_t *)&tmp, sizeof(tmp))) {
        samples = (uint16_t)(tmp & 0x03FF);
    } else {
        DEV_PRINTF("LSM6DSO: error reading fifo status\n");
        samples = 0;
    }

    return samples;
}

void AP_InertialSensor_LSM6DSO::_set_gyro_scale(gyro_scale scale)
{
    float scale_val = 0.0f;

    /* scales values from datasheet in mdps/digit */
    switch (scale) {
    case G_SCALE_125DPS:
        scale_val = 4.375f;
        break;
    case G_SCALE_250DPS:
        scale_val = 8.75f;
        break;
    case G_SCALE_500DPS:
        scale_val = 17.5f;
        break;
    case G_SCALE_1000DPS:
        scale_val = 35.0f;
        break;
    case G_SCALE_2000DPS:
        scale_val = 70.0f;
        break;
    default:
        scale_val = 70.0f;
        break;
    }

    /* convert mdps/digit to dps/digit */
    _gyro_scale = scale_val / 1000.0f;

    /* convert dps/digit to (rad/s)/digit */
    _gyro_scale *= DEG_TO_RAD;
}

void AP_InertialSensor_LSM6DSO::_set_accel_scale(accel_scale scale)
{
    float scale_val = 0.0f;

    switch (scale) {
    case A_SCALE_2G:
        scale_val = 2.0f;
        break;
    case A_SCALE_4G:
        scale_val = 4.0f;
        break;
    case A_SCALE_8G:
        scale_val = 8.0f;
        break;
    case A_SCALE_16G:
        scale_val = 16.0f;
        break;
    default:
        scale_val = 16.0f;
        break;
    }

    _accel_scale = scale_val / 32768.0f;

    /* convert to G/LSB to (m/s/s)/LSB */
    _accel_scale *= GRAVITY_MSS;
}

/**
 * Timer process to poll for new data from the LSM6DSO.
 */
void AP_InertialSensor_LSM6DSO::_poll_data()
{
    const uint16_t samples = _get_count_fifo_unread_data();
    for (uint16_t i = 0; i < samples; i++) {
        const uint8_t _reg = LSM6DSO_REG_FIFO_DATA_OUT_TAG | 0x80;
        uint8_t fifo_tmp[7] = {0, 0, 0, 0, 0, 0, 0};

        if (!_dev->transfer(&_reg, 1, (uint8_t *)&fifo_tmp, sizeof(fifo_tmp))) {
            DEV_PRINTF("LSM6DSO: error reading fifo data\n");
            return;
        }

        uint8_t tag;
        struct sensor_raw_data raw_data;

        tag = (uint8_t)((fifo_tmp[0] & 0xF8) >> 3);
        raw_data.x = (int16_t)(fifo_tmp[1] + (fifo_tmp[2] << 8));
        raw_data.y = (int16_t)(fifo_tmp[3] + (fifo_tmp[4] << 8));
        raw_data.z = (int16_t)(fifo_tmp[5] + (fifo_tmp[6] << 8));

        switch (tag) {
        case 0x01:
            _update_transaction_g(raw_data);
            break;
        case 0x02:
            _update_transaction_x(raw_data);
            break;
        default:
            // unused fifo data
            ;
            break;
        }
    }

    // check next register value for correctness
    AP_HAL::Device::checkreg reg;
    if (!_dev->check_next_register(reg)) {
        log_register_change(_dev->get_bus_id(), reg);
        _inc_accel_error_count(accel_instance);
    }
}

/*
 *  update raw data
 */
void AP_InertialSensor_LSM6DSO::_update_transaction_g(struct sensor_raw_data raw_data)
{
    Vector3f gyro_data(raw_data.x, -raw_data.y, -raw_data.z);
    gyro_data *= _gyro_scale;

    _rotate_and_correct_gyro(gyro_instance, gyro_data);
    _notify_new_gyro_raw_sample(gyro_instance, gyro_data);
}

void AP_InertialSensor_LSM6DSO::_update_transaction_x(struct sensor_raw_data raw_data)
{
    Vector3f accel_data(raw_data.x, -raw_data.y, -raw_data.z);
    accel_data *= _accel_scale;

    _rotate_and_correct_accel(accel_instance, accel_data);
    _notify_new_accel_raw_sample(accel_instance, accel_data);
}

bool AP_InertialSensor_LSM6DSO::update()
{
    update_gyro(gyro_instance);
    update_accel(accel_instance);

    return true;
}

#if LSM6DSO_DEBUG
/*
 *  dump all config registers - used for debug
*/
void AP_InertialSensor_LSM6DSO::_dump_registers(void)
{
    hal.console->println("LSM6DSO registers:");

    const uint8_t first = LSM6DSO_REG_FUNC_CFG_ACCESS;
    const uint8_t last = LSM6DSO_REG_FIFO_DATA_OUT_Z_H;
    for (uint8_t reg=first; reg<=last; reg++) {
        uint8_t v = _register_read(reg);
        hal.console->printf("%02x:%02x ", reg, v);
        if ((reg - (first-1)) % 16 == 0) {
            hal.console->println("");
        }
    }
    hal.console->println("");
}
#endif
