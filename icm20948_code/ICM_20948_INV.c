#include <ICM_20948_INV.h>
#include <stdint.h>
#include <stdbool.h>

I2C_HandleTypeDef i2c_bus;

// select user bank 
void icm20948_set_user_bank(uint8_t bank)
{
    bank = (bank << 4) & 0x30; // bits 1:0 of bank -> 5:4 of icm bank select register
    HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR, REG_BANK_SEL, I2C_MEMADD_SIZE_8BIT, &bank, 1, 1000);
}

const uint8_t dmp3_image[] = {
#include "icm20948_img.dmp3a.h"
};

// ICM-20948 data is big-endian. We need to make it little-endian when writing into icm_20948_DMP_data_t
const int DMP_Quat9_Byte_Ordering[icm_20948_DMP_Quat9_Bytes] =
    {
        3, 2, 1, 0, 7, 6, 5, 4, 11, 10, 9, 8, 13, 12 // Also used for Geomag
};
const int DMP_Quat6_Byte_Ordering[icm_20948_DMP_Quat6_Bytes] =
    {
        3, 2, 1, 0, 7, 6, 5, 4, 11, 10, 9, 8 // Also used for Gyro_Calibr, Compass_Calibr
};
const int DMP_PQuat6_Byte_Ordering[icm_20948_DMP_PQuat6_Bytes] =
    {
        1, 0, 3, 2, 5, 4 // Also used for Raw_Accel, Compass
};
const int DMP_Raw_Gyro_Byte_Ordering[icm_20948_DMP_Raw_Gyro_Bytes + icm_20948_DMP_Gyro_Bias_Bytes] =
    {
        1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10};
const int DMP_Activity_Recognition_Byte_Ordering[icm_20948_DMP_Activity_Recognition_Bytes] =
    {
        0, 1, 5, 4, 3, 2};
const int DMP_Secondary_On_Off_Byte_Ordering[icm_20948_DMP_Secondary_On_Off_Bytes] =
    {
        1, 0};


int8_t sensor_type_2_android_sensor(enum inv_icm20948_sensor sensor)
{
  switch (sensor)
  {
  case INV_ICM20948_SENSOR_ACCELEROMETER:
    return ANDROID_SENSOR_ACCELEROMETER; // 1
  case INV_ICM20948_SENSOR_GYROSCOPE:
    return ANDROID_SENSOR_GYROSCOPE; // 4
  case INV_ICM20948_SENSOR_RAW_ACCELEROMETER:
    return ANDROID_SENSOR_RAW_ACCELEROMETER; // 42
  case INV_ICM20948_SENSOR_RAW_GYROSCOPE:
    return ANDROID_SENSOR_RAW_GYROSCOPE; // 43
  case INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
    return ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED; // 14
  case INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED:
    return ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED; // 16
  case INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON:
    return ANDROID_SENSOR_ACTIVITY_CLASSIFICATON; // 47
  case INV_ICM20948_SENSOR_STEP_DETECTOR:
    return ANDROID_SENSOR_STEP_DETECTOR; // 18
  case INV_ICM20948_SENSOR_STEP_COUNTER:
    return ANDROID_SENSOR_STEP_COUNTER; // 19
  case INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR:
    return ANDROID_SENSOR_GAME_ROTATION_VECTOR; // 15
  case INV_ICM20948_SENSOR_ROTATION_VECTOR:
    return ANDROID_SENSOR_ROTATION_VECTOR; // 11
  case INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
    return ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR; // 20
  case INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD:
    return ANDROID_SENSOR_GEOMAGNETIC_FIELD; // 2
  case INV_ICM20948_SENSOR_WAKEUP_SIGNIFICANT_MOTION:
    return ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION; // 17
  case INV_ICM20948_SENSOR_FLIP_PICKUP:
    return ANDROID_SENSOR_FLIP_PICKUP; // 46
  case INV_ICM20948_SENSOR_WAKEUP_TILT_DETECTOR:
    return ANDROID_SENSOR_WAKEUP_TILT_DETECTOR; // 41
  case INV_ICM20948_SENSOR_GRAVITY:
    return ANDROID_SENSOR_GRAVITY; // 9
  case INV_ICM20948_SENSOR_LINEAR_ACCELERATION:
    return ANDROID_SENSOR_LINEAR_ACCELERATION; // 10
  case INV_ICM20948_SENSOR_ORIENTATION:
    return ANDROID_SENSOR_ORIENTATION; // 3
  case INV_ICM20948_SENSOR_B2S:
    return ANDROID_SENSOR_B2S; // 45
  default:
    return ANDROID_SENSOR_NUM_MAX;
  }
}


const uint16_t inv_androidSensor_to_control_bits[ANDROID_SENSOR_NUM_MAX] =
    {
        // Data output control 1 register bit definition
        // 16-bit accel                                0x8000
        // 16-bit gyro                                 0x4000
        // 16-bit compass                              0x2000
        // 16-bit ALS                                  0x1000
        // 32-bit 6-axis quaternion                    0x0800
        // 32-bit 9-axis quaternion + heading accuracy 0x0400
        // 16-bit pedometer quaternion                 0x0200
        // 32-bit Geomag rv + heading accuracy         0x0100
        // 16-bit Pressure                             0x0080
        // 32-bit calibrated gyro                      0x0040
        // 32-bit calibrated compass                   0x0020
        // Pedometer Step Detector                     0x0010
        // Header 2                                    0x0008
        // Pedometer Step Indicator Bit 2              0x0004
        // Pedometer Step Indicator Bit 1              0x0002
        // Pedometer Step Indicator Bit 0              0x0001
        // Unsupported Sensors are 0xFFFF

        0xFFFF, // 0  Meta Data
        0x8008, // 1  Accelerometer
        0x0028, // 2  Magnetic Field
        0x0408, // 3  Orientation
        0x4048, // 4  Gyroscope
        0x1008, // 5  Light
        0x0088, // 6  Pressure
        0xFFFF, // 7  Temperature
        0xFFFF, // 8  Proximity <----------- fixme
        0x0808, // 9  Gravity
        0x8808, // 10 Linear Acceleration
        0x0408, // 11 Rotation Vector
        0xFFFF, // 12 Humidity
        0xFFFF, // 13 Ambient Temperature
        0x2008, // 14 Magnetic Field Uncalibrated
        0x0808, // 15 Game Rotation Vector
        0x4008, // 16 Gyroscope Uncalibrated
        0x0000, // 17 Significant Motion
        0x0018, // 18 Step Detector
        0x0010, // 19 Step Counter <----------- fixme
        0x0108, // 20 Geomagnetic Rotation Vector
        0xFFFF, // 21 ANDROID_SENSOR_HEART_RATE,
        0xFFFF, // 22 ANDROID_SENSOR_PROXIMITY,

        0x8008, // 23 ANDROID_SENSOR_WAKEUP_ACCELEROMETER,
        0x0028, // 24 ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD,
        0x0408, // 25 ANDROID_SENSOR_WAKEUP_ORIENTATION,
        0x4048, // 26 ANDROID_SENSOR_WAKEUP_GYROSCOPE,
        0x1008, // 27 ANDROID_SENSOR_WAKEUP_LIGHT,
        0x0088, // 28 ANDROID_SENSOR_WAKEUP_PRESSURE,
        0x0808, // 29 ANDROID_SENSOR_WAKEUP_GRAVITY,
        0x8808, // 30 ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION,
        0x0408, // 31 ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR,
        0xFFFF, // 32 ANDROID_SENSOR_WAKEUP_RELATIVE_HUMIDITY,
        0xFFFF, // 33 ANDROID_SENSOR_WAKEUP_AMBIENT_TEMPERATURE,
        0x2008, // 34 ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED,
        0x0808, // 35 ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR,
        0x4008, // 36 ANDROID_SENSOR_WAKEUP_GYROSCOPE_UNCALIBRATED,
        0x0018, // 37 ANDROID_SENSOR_WAKEUP_STEP_DETECTOR,
        0x0010, // 38 ANDROID_SENSOR_WAKEUP_STEP_COUNTER,
        0x0108, // 39 ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR
        0xFFFF, // 40 ANDROID_SENSOR_WAKEUP_HEART_RATE,
        0x0000, // 41 ANDROID_SENSOR_WAKEUP_TILT_DETECTOR,
        0x8008, // 42 Raw Acc
        0x4048, // 43 Raw Gyr
};


int8_t icm20948_i2c_controller_configure_peripheral(uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap, uint8_t dataOut)
{
  uint8_t periph_addr_reg;
  uint8_t periph_reg_reg;
  uint8_t periph_ctrl_reg;
  uint8_t periph_do_reg;

  switch (peripheral)
  {
  case 0:
    periph_addr_reg = AGB3_REG_I2C_PERIPH0_ADDR;
    periph_reg_reg = AGB3_REG_I2C_PERIPH0_REG;
    periph_ctrl_reg = AGB3_REG_I2C_PERIPH0_CTRL;
    periph_do_reg = AGB3_REG_I2C_PERIPH0_DO;
    break;
  case 1:
    periph_addr_reg = AGB3_REG_I2C_PERIPH1_ADDR;
    periph_reg_reg = AGB3_REG_I2C_PERIPH1_REG;
    periph_ctrl_reg = AGB3_REG_I2C_PERIPH1_CTRL;
    periph_do_reg = AGB3_REG_I2C_PERIPH1_DO;
    break;
  case 2:
    periph_addr_reg = AGB3_REG_I2C_PERIPH2_ADDR;
    periph_reg_reg = AGB3_REG_I2C_PERIPH2_REG;
    periph_ctrl_reg = AGB3_REG_I2C_PERIPH2_CTRL;
    periph_do_reg = AGB3_REG_I2C_PERIPH2_DO;
    break;
  case 3:
    periph_addr_reg = AGB3_REG_I2C_PERIPH3_ADDR;
    periph_reg_reg = AGB3_REG_I2C_PERIPH3_REG;
    periph_ctrl_reg = AGB3_REG_I2C_PERIPH3_CTRL;
    periph_do_reg = AGB3_REG_I2C_PERIPH3_DO;
    break;
  default:
    return -1;
  }

  icm20948_set_user_bank(3);


  // Set the peripheral address and the Rw flag
  ICM_20948_I2C_PERIPHX_ADDR_t address;
  address.ID = addr;
  if (Rw)
  {
    address.RNW = 1;
  }
  else
  {
    address.RNW = 0; // Make sure bit is clear (just in case there is any garbage in that RAM location)
  }
  HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR, periph_addr_reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&address, sizeof(ICM_20948_I2C_PERIPHX_ADDR_t), 1000);
  //ICM_20948_execute_w(pdev, periph_addr_reg, (uint8_t *)&address, sizeof(ICM_20948_I2C_PERIPHX_ADDR_t));


  // If we are setting up a write, configure the Data Out register too
  if (!Rw)
  {
    ICM_20948_I2C_PERIPHX_DO_t dataOutByte;
    dataOutByte.DO = dataOut;
    HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR, periph_do_reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&dataOutByte, sizeof(ICM_20948_I2C_PERIPHX_DO_t), 1000);
    //ICM_20948_execute_w(pdev, periph_do_reg, (uint8_t *)&dataOutByte, sizeof(ICM_20948_I2C_PERIPHX_DO_t));

  }

  // Set the peripheral sub-address (register address)
  ICM_20948_I2C_PERIPHX_REG_t subaddress;
  subaddress.REG = reg;
  HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR, periph_reg_reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&subaddress, sizeof(ICM_20948_I2C_PERIPHX_REG_t), 1000);
  //ICM_20948_execute_w(pdev, periph_reg_reg, (uint8_t *)&subaddress, sizeof(ICM_20948_I2C_PERIPHX_REG_t));


  // Set up the control info
  ICM_20948_I2C_PERIPHX_CTRL_t ctrl;
  ctrl.LENG = len;
  ctrl.EN = enable;
  ctrl.REG_DIS = data_only;
  ctrl.GRP = grp;
  ctrl.BYTE_SW = swap;
  HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR, periph_ctrl_reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl, sizeof(ICM_20948_I2C_PERIPHX_CTRL_t), 1000);
  //ICM_20948_execute_w(pdev, periph_ctrl_reg, (uint8_t *)&ctrl, sizeof(ICM_20948_I2C_PERIPHX_CTRL_t));

  return 0;
}


int8_t icm20948_set_clock_source(ICM_20948_PWR_MGMT_1_CLKSEL_e source)
{
  ICM_20948_PWR_MGMT_1_t reg;
  icm20948_set_user_bank(0); // Must be in the right bank

  HAL_I2C_Mem_Read(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg, sizeof(ICM_20948_PWR_MGMT_1_t), 1000);

  reg.CLKSEL = source;

  HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg, sizeof(ICM_20948_PWR_MGMT_1_t), 1000);

  return 0;
}


int8_t icm20948_set_sample_mode(ICM_20948_InternalSensorID_bm sensors, ICM_20948_LP_CONFIG_CYCLE_e mode)
{
  ICM_20948_LP_CONFIG_t reg;

  /*if (!(sensors & (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr | ICM_20948_Internal_Mst)))
  {
    return -1;
  }*/

  icm20948_set_user_bank(0); // Must be in the right bank

  HAL_I2C_Mem_Read(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_LP_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg, sizeof(ICM_20948_LP_CONFIG_t), 1000);


  if (sensors & ICM_20948_Internal_Acc)
  {
    reg.ACCEL_CYCLE = mode;
  } // Set all desired sensors to this setting
  if (sensors & ICM_20948_Internal_Gyr)
  {
    reg.GYRO_CYCLE = mode;
  }
  if (sensors & ICM_20948_Internal_Mst)
  {
    reg.I2C_MST_CYCLE = mode;
  }

  HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_LP_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg, sizeof(ICM_20948_LP_CONFIG_t), 1000);

  // Check the data was written correctly
  //HAL_I2C_Mem_Read(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_LP_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg, sizeof(ICM_20948_LP_CONFIG_t), 1000);
  return 0;
}


int8_t icm20948_enable_FIFO(bool enable)
{
  ICM_20948_USER_CTRL_t ctrl;
  icm20948_set_user_bank(0);


  HAL_I2C_Mem_Read(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_USER_CTRL, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl, sizeof(ICM_20948_USER_CTRL_t), 1000);

  if (enable)
    ctrl.FIFO_EN = 1;
  else
    ctrl.FIFO_EN = 0;

  HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_USER_CTRL, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl, sizeof(ICM_20948_USER_CTRL_t), 1000);

  return 0;
}


int8_t icm20948_reset_FIFO()
{
  ICM_20948_FIFO_RST_t ctrl;
  icm20948_set_user_bank(0);

  HAL_I2C_Mem_Read(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_FIFO_RST, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl, sizeof(ICM_20948_FIFO_RST_t), 1000);

  ctrl.FIFO_RESET = 0x1F; // Datasheet says "FIFO_RESET[4:0]"

  HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_FIFO_RST, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl, sizeof(ICM_20948_FIFO_RST_t), 1000);

  //delay ???

  ctrl.FIFO_RESET = 0x1E; // The InvenSense Nucleo examples write 0x1F followed by 0x1E
  HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_FIFO_RST, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl, sizeof(ICM_20948_FIFO_RST_t), 1000);

  return 0;
}


int8_t icm20948_enable_DMP(bool enable)
{
  ICM_20948_USER_CTRL_t ctrl;
  
  icm20948_set_user_bank(0);

  HAL_I2C_Mem_Read(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_USER_CTRL,  I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl, sizeof(ICM_20948_USER_CTRL_t), 1000);


  if (enable)
    ctrl.DMP_EN = 1;
  else
    ctrl.DMP_EN = 0;

  HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_USER_CTRL, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl, sizeof(ICM_20948_USER_CTRL_t), 1000);
  
  return 0;
}


int8_t icm20948_reset_DMP()
{
  ICM_20948_USER_CTRL_t ctrl;
  icm20948_set_user_bank(0);

  HAL_I2C_Mem_Read(&i2c_bus, ICM_20948_I2C_ADDR,  AGB0_REG_USER_CTRL, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl, sizeof(ICM_20948_USER_CTRL_t), 1000);

  ctrl.DMP_RST = 1;

  HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_USER_CTRL, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl, sizeof(ICM_20948_USER_CTRL_t), 1000);

  return 0;
}


int8_t icm20948_set_full_scale(ICM_20948_InternalSensorID_bm sensors, ICM_20948_fss_t fss)
{
  if (!(sensors & (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr)))
  {
    return -1;
  }

  if (sensors & ICM_20948_Internal_Acc)
  {
    ICM_20948_ACCEL_CONFIG_t reg;
    icm20948_set_user_bank(2); // Must be in the right bank
    HAL_I2C_Mem_Read(&i2c_bus, ICM_20948_I2C_ADDR, AGB2_REG_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t), 1000);
    reg.ACCEL_FS_SEL = fss.a;
    HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR,  AGB2_REG_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t), 1000);
    // Check the data was written correctly
    HAL_I2C_Mem_Read(&i2c_bus, ICM_20948_I2C_ADDR, AGB2_REG_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t), 1000);
    
  }
  if (sensors & ICM_20948_Internal_Gyr)
  {
    ICM_20948_GYRO_CONFIG_1_t reg;
    icm20948_set_user_bank(2); // Must be in the right bank
    HAL_I2C_Mem_Read(&i2c_bus, ICM_20948_I2C_ADDR,AGB2_REG_GYRO_CONFIG_1, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t), 1000);
    reg.GYRO_FS_SEL = fss.g;
    HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR,  AGB2_REG_GYRO_CONFIG_1, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t), 1000);
    // Check the data was written correctly
    HAL_I2C_Mem_Read(&i2c_bus, ICM_20948_I2C_ADDR, AGB2_REG_GYRO_CONFIG_1, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t), 1000);
  }
  return 0;
}


int8_t icm20948_enable_dlpf(ICM_20948_InternalSensorID_bm sensors, bool enable)
{
  if (!(sensors & (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr)))
  {
    return -1;
  }

  if (sensors & ICM_20948_Internal_Acc)
  {
    ICM_20948_ACCEL_CONFIG_t reg;
    icm20948_set_user_bank(2); // Must be in the right bank
    HAL_I2C_Mem_Read(&i2c_bus, ICM_20948_I2C_ADDR, AGB2_REG_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t), 1000);
    if (enable)
    {
      reg.ACCEL_FCHOICE = 1;
    }
    else
    {
      reg.ACCEL_FCHOICE = 0;
    }
    HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR,  AGB2_REG_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t), 1000);
    // Check the data was written correctly
    HAL_I2C_Mem_Read(&i2c_bus, ICM_20948_I2C_ADDR, AGB2_REG_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t), 1000);

  }
  if (sensors & ICM_20948_Internal_Gyr)
  {
    ICM_20948_GYRO_CONFIG_1_t reg;
    icm20948_set_user_bank(2); // Must be in the right bank
    HAL_I2C_Mem_Read(&i2c_bus, ICM_20948_I2C_ADDR, AGB2_REG_GYRO_CONFIG_1, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t), 1000);
    if (enable)
    {
      reg.GYRO_FCHOICE = 1;
    }
    else
    {
      reg.GYRO_FCHOICE = 0;
    }
    HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR,  AGB2_REG_GYRO_CONFIG_1, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t), 1000);
    // Check the data was written correctly
    HAL_I2C_Mem_Read(&i2c_bus, ICM_20948_I2C_ADDR, AGB2_REG_GYRO_CONFIG_1, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t), 1000);
  }
  return 0;
}


int8_t icm20948_set_sample_rate(ICM_20948_InternalSensorID_bm sensors, ICM_20948_smplrt_t smplrt)
{
  if (!(sensors & (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr)))
  {
    return -1;
  }

  if (sensors & ICM_20948_Internal_Acc)
  {
    icm20948_set_user_bank(2); // Must be in the right bank
    uint8_t div1 = (smplrt.a >> 8); // Thank you @yanivamichy #109
    uint8_t div2 = (smplrt.a & 0xFF);
    HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR,  AGB2_REG_ACCEL_SMPLRT_DIV_1, I2C_MEMADD_SIZE_8BIT, &div1, 1, 1000);
    HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR,  AGB2_REG_ACCEL_SMPLRT_DIV_2, I2C_MEMADD_SIZE_8BIT, &div2, 1, 1000);
  }
  if (sensors & ICM_20948_Internal_Gyr)
  {
    icm20948_set_user_bank(2); // Must be in the right bank
    uint8_t div = (smplrt.g);
    HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR,  AGB2_REG_GYRO_SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, &div, 1, 1000);
  }
  return 0;
}


int8_t icm20948_int_enable(ICM_20948_INT_enable_t *write, ICM_20948_INT_enable_t *read)
{
  ICM_20948_INT_ENABLE_t en_0;
  ICM_20948_INT_ENABLE_1_t en_1;
  ICM_20948_INT_ENABLE_2_t en_2;
  ICM_20948_INT_ENABLE_3_t en_3;

  icm20948_set_user_bank(0); // Must be in the right bank

  if (write != NULL)
  { // If the write pointer is not NULL then write to the registers BEFORE reading
    en_0.I2C_MST_INT_EN = write->I2C_MST_INT_EN;
    en_0.DMP_INT1_EN = write->DMP_INT1_EN;
    en_0.PLL_READY_EN = write->PLL_RDY_EN;
    en_0.WOM_INT_EN = write->WOM_INT_EN;
    en_0.reserved_0 = 0; // Clear RAM garbage
    en_0.REG_WOF_EN = write->REG_WOF_EN;
    en_1.RAW_DATA_0_RDY_EN = write->RAW_DATA_0_RDY_EN;
    en_1.reserved_0 = 0; // Clear RAM garbage
    en_2.individual.FIFO_OVERFLOW_EN_4 = write->FIFO_OVERFLOW_EN_4;
    en_2.individual.FIFO_OVERFLOW_EN_3 = write->FIFO_OVERFLOW_EN_3;
    en_2.individual.FIFO_OVERFLOW_EN_2 = write->FIFO_OVERFLOW_EN_2;
    en_2.individual.FIFO_OVERFLOW_EN_1 = write->FIFO_OVERFLOW_EN_1;
    en_2.individual.FIFO_OVERFLOW_EN_0 = write->FIFO_OVERFLOW_EN_0;
    en_2.individual.reserved_0 = 0; // Clear RAM garbage
    en_3.individual.FIFO_WM_EN_4 = write->FIFO_WM_EN_4;
    en_3.individual.FIFO_WM_EN_3 = write->FIFO_WM_EN_3;
    en_3.individual.FIFO_WM_EN_2 = write->FIFO_WM_EN_2;
    en_3.individual.FIFO_WM_EN_1 = write->FIFO_WM_EN_1;
    en_3.individual.FIFO_WM_EN_0 = write->FIFO_WM_EN_0;
    en_3.individual.reserved_0 = 0; // Clear RAM garbage

    HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_INT_ENABLE, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&en_0, sizeof(ICM_20948_INT_ENABLE_t), 1000);

    HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_INT_ENABLE_1, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&en_1, sizeof(ICM_20948_INT_ENABLE_1_t), 1000);

    HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_INT_ENABLE_2, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&en_2, sizeof(ICM_20948_INT_ENABLE_2_t), 1000);

    HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_INT_ENABLE_3, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&en_3, sizeof(ICM_20948_INT_ENABLE_3_t), 1000);

  }

  if (read != NULL)
  { // If read pointer is not NULL then read the registers (if write is not NULL then this should read back the results of write into read)
    HAL_I2C_Mem_Read(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_INT_ENABLE, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&en_0, sizeof(ICM_20948_INT_ENABLE_t), 1000);

    HAL_I2C_Mem_Read(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_INT_ENABLE_1, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&en_1, sizeof(ICM_20948_INT_ENABLE_1_t), 1000);

    HAL_I2C_Mem_Read(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_INT_ENABLE_2, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&en_2, sizeof(ICM_20948_INT_ENABLE_2_t), 1000);

    HAL_I2C_Mem_Read(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_INT_ENABLE_3, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&en_3, sizeof(ICM_20948_INT_ENABLE_3_t), 1000);


    read->I2C_MST_INT_EN = en_0.I2C_MST_INT_EN;
    read->DMP_INT1_EN = en_0.DMP_INT1_EN;
    read->PLL_RDY_EN = en_0.PLL_READY_EN;
    read->WOM_INT_EN = en_0.WOM_INT_EN;
    read->REG_WOF_EN = en_0.REG_WOF_EN;
    read->RAW_DATA_0_RDY_EN = en_1.RAW_DATA_0_RDY_EN;
    read->FIFO_OVERFLOW_EN_4 = en_2.individual.FIFO_OVERFLOW_EN_4;
    read->FIFO_OVERFLOW_EN_3 = en_2.individual.FIFO_OVERFLOW_EN_3;
    read->FIFO_OVERFLOW_EN_2 = en_2.individual.FIFO_OVERFLOW_EN_2;
    read->FIFO_OVERFLOW_EN_1 = en_2.individual.FIFO_OVERFLOW_EN_1;
    read->FIFO_OVERFLOW_EN_0 = en_2.individual.FIFO_OVERFLOW_EN_0;
    read->FIFO_WM_EN_4 = en_3.individual.FIFO_WM_EN_4;
    read->FIFO_WM_EN_3 = en_3.individual.FIFO_WM_EN_3;
    read->FIFO_WM_EN_2 = en_3.individual.FIFO_WM_EN_2;
    read->FIFO_WM_EN_1 = en_3.individual.FIFO_WM_EN_1;
    read->FIFO_WM_EN_0 = en_3.individual.FIFO_WM_EN_0;
  }

  return 0;
}


int8_t icm20948_int_enable_raw_data_ready(bool enable)
{
  ICM_20948_INT_enable_t en;                          // storage
  icm20948_int_enable(NULL, &en); // read phase
  en.RAW_DATA_0_RDY_EN = enable;                     // change the setting
  icm20948_int_enable(&en, &en); // write phase w/ readback

  return 0;
}


int8_t icm20948_set_DMP_start_address(unsigned short address)
{
  unsigned char start_address[2];

  start_address[0] = (unsigned char)(address >> 8);
  start_address[1] = (unsigned char)(address & 0xff);

  icm20948_set_user_bank(2); // Set bank 2

  // Write the sensor control bits into memory address AGB2_REG_PRGM_START_ADDRH
  HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR, AGB2_REG_PRGM_START_ADDRH, I2C_MEMADD_SIZE_8BIT, (uint8_t *)start_address, 2, 1000);

  return 0;
}


int8_t icm20948_write_mem(unsigned short reg, unsigned int length, const unsigned char *data)
{
  unsigned int bytesWritten = 0;
  unsigned int thisLen;
  unsigned char lBankSelected;
  unsigned char lStartAddrSelected;

  if (!data)
  {
    return 1; // nodata
  }

  icm20948_set_user_bank(0); // set user bank (0 through 3) to 0

    //set mem bank
  lBankSelected = (reg >> 8);
  HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_MEM_BANK_SEL, I2C_MEMADD_SIZE_8BIT, &lBankSelected, 1, 1000);
  //ICM_20948_execute_w(pdev, AGB0_REG_MEM_BANK_SEL, &lBankSelected, 1);

  while (bytesWritten < length)
  {
    lStartAddrSelected = (reg & 0xff);

    /* Sets the starting read or write address for the selected memory, inside of the selected page (see MEM_SEL Register).
           Contents are changed after read or write of the selected memory.
           This register must be written prior to each access to initialize the register to the proper starting address.
           The address will auto increment during burst transactions.  Two consecutive bursts without re-initializing the start address would skip one address. */

    HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_MEM_START_ADDR, I2C_MEMADD_SIZE_8BIT, &lStartAddrSelected, 1, 1000);
    //ICM_20948_execute_w(pdev, AGB0_REG_MEM_START_ADDR, &lStartAddrSelected, 1);


    if (length - bytesWritten <= MAX_SERIAL_R_W)
      thisLen = length - bytesWritten;
    else
      thisLen = MAX_SERIAL_R_W;

    /* Write data */

    HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_MEM_R_W, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&data[bytesWritten], thisLen, 1000);
    //ICM_20948_execute_w(pdev, AGB0_REG_MEM_R_W, (uint8_t *)&data[bytesWritten], thisLen);

    bytesWritten += thisLen;
    reg += thisLen;
  }

  return 0;
}


int8_t icm20948_read_mem(unsigned short reg, unsigned int length, unsigned char *data)
{
  unsigned int bytesRead = 0;
  unsigned int thisLen;
  unsigned char lBankSelected;
  unsigned char lStartAddrSelected;

  icm20948_set_user_bank(0); // set user bank (0 through 3) to 0

    //set mem bank
  lBankSelected = (reg >> 8);
  HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_MEM_BANK_SEL, I2C_MEMADD_SIZE_8BIT, &lBankSelected, 1, 1000);
  //ICM_20948_execute_w(pdev, AGB0_REG_MEM_BANK_SEL, &lBankSelected, 1);


  while (bytesRead < length)
  {
    lStartAddrSelected = (reg & 0xff);
    /* Sets the starting read or write address for the selected memory, inside of the selected page (see MEM_SEL Register).
           Contents are changed after read or write of the selected memory.
           This register must be written prior to each access to initialize the register to the proper starting address.
           The address will auto increment during burst transactions.  Two consecutive bursts without re-initializing the start address would skip one address. */

    HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_MEM_START_ADDR, I2C_MEMADD_SIZE_8BIT, &lStartAddrSelected, 1, 1000);
    //ICM_20948_execute_w(pdev, AGB0_REG_MEM_START_ADDR, &lStartAddrSelected, 1);

    if (length - bytesRead <= MAX_SERIAL_R_W)
      thisLen = length - bytesRead;
    else
      thisLen = MAX_SERIAL_R_W;

    /* Read data */
    HAL_I2C_Mem_Read(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_MEM_R_W, I2C_MEMADD_SIZE_8BIT, &data[bytesRead], thisLen, 1000);
    
    bytesRead += thisLen;
    reg += thisLen;
  }

  return 0;
}


int8_t icm20948_set_gyro_sf(unsigned char div, int gyro_level)
{
  // gyro_level should be set to 4 regardless of fullscale, due to the addition of API dmp_icm20648_set_gyro_fsr()
  gyro_level = 4;

  // First read the TIMEBASE_CORRECTION_PLL register from Bank 1
  int8_t pll; // Signed. Typical value is 0x18
  icm20948_set_user_bank(1);
  HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR, AGB1_REG_TIMEBASE_CORRECTION_PLL, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&pll, 1, 1000);


  // Now calculate the Gyro SF using code taken from the InvenSense example (inv_icm20948_set_gyro_sf)
  long gyro_sf;

  unsigned long long const MagicConstant = 264446880937391LL;
  unsigned long long const MagicConstantScale = 100000LL;
  unsigned long long ResultLL;

  if (pll & 0x80)
  {
    ResultLL = (MagicConstant * (long long)(1ULL << gyro_level) * (1 + div) / (1270 - (pll & 0x7F)) / MagicConstantScale);
  }
  else
  {
    ResultLL = (MagicConstant * (long long)(1ULL << gyro_level) * (1 + div) / (1270 + pll) / MagicConstantScale);
  }
  /*
	    In above deprecated FP version, worst case arguments can produce a result that overflows a signed long.
	    Here, for such cases, we emulate the FP behavior of setting the result to the maximum positive value, as
	    the compiler's conversion of a u64 to an s32 is simple truncation of the u64's high half, sadly....
	*/
  if (ResultLL > 0x7FFFFFFF)
    gyro_sf = 0x7FFFFFFF;
  else
    gyro_sf = (long)ResultLL;

  // Finally, write the value to the DMP GYRO_SF register
  unsigned char gyro_sf_reg[4];
  gyro_sf_reg[0] = (unsigned char)(gyro_sf >> 24);
  gyro_sf_reg[1] = (unsigned char)(gyro_sf >> 16);
  gyro_sf_reg[2] = (unsigned char)(gyro_sf >> 8);
  gyro_sf_reg[3] = (unsigned char)(gyro_sf & 0xff);
  icm20948_write_mem(GYRO_SF, 4, (const unsigned char*)&gyro_sf_reg);

  return 0;
}


int8_t icm20948_enable_dmp_sensor(enum inv_icm20948_sensor sensor, int state)
{
  uint32_t _enabled_Android_0 = 0;
  uint32_t _enabled_Android_1 = 0;
  uint16_t inv_event_control = 0; // Use this to store the value for MOTION_EVENT_CTL
  uint16_t data_rdy_status = 0;   // Use this to store the value for DATA_RDY_STATUS

  uint8_t androidSensor = sensor_type_2_android_sensor(sensor); // Convert sensor from enum inv_icm20948_sensor to Android numbering

  if (androidSensor >= ANDROID_SENSOR_NUM_MAX)
    return -1; // Bail if the sensor is not supported (TO DO: Support B2S etc)

  // Convert the Android sensor into a bit mask for DATA_OUT_CTL1
  uint16_t delta = inv_androidSensor_to_control_bits[androidSensor];
  if (delta == 0xFFFF)
    return -1; // Bail if the sensor is not supported

  // Convert the Android sensor number into a bitmask and set or clear that bit in _enabled_Android_0 / _enabled_Android_1
  unsigned long androidSensorAsBitMask;
  if (androidSensor < 32) // Sensors 0-31
  {
    androidSensorAsBitMask = 1L << androidSensor;
    if (state == 0) // Should we disable the sensor?
    {
      _enabled_Android_0 &= ~androidSensorAsBitMask; // Clear the bit to disable the sensor
    }
    else
    {
      _enabled_Android_0 |= androidSensorAsBitMask; // Set the bit to enable the sensor
    }
  }
  else // Sensors 32-
  {
    androidSensorAsBitMask = 1L << (androidSensor - 32);
    if (state == 0) // Should we disable the sensor?
    {
      _enabled_Android_1 &= ~androidSensorAsBitMask; // Clear the bit to disable the sensor
    }
    else
    {
      _enabled_Android_1 |= androidSensorAsBitMask; // Set the bit to enable the sensor
    }
  }

  // Now we know androidSensor is valid, reconstruct the value for DATA_OUT_CTL1 from _enabled_Android_0 and _enabled_Android_0
  delta = 0; // Clear delta
  for (int i = 0; i < 32; i++)
  {
    androidSensorAsBitMask = 1L << i;
    if ((_enabled_Android_0 & androidSensorAsBitMask) > 0) // Check if the Android sensor (0-31) is enabled
    {
      delta |= inv_androidSensor_to_control_bits[i]; // If it is, or the required bits into delta
    }
    if ((_enabled_Android_1 & androidSensorAsBitMask) > 0) // Check if the Android sensor (32-) is enabled
    {
      delta |= inv_androidSensor_to_control_bits[i + 32]; // If it is, or the required bits into delta
    }
    // Also check which bits need to be set in the Data Ready Status and Motion Event Control registers
    // Compare to INV_NEEDS_ACCEL_MASK, INV_NEEDS_GYRO_MASK and INV_NEEDS_COMPASS_MASK
    if (((androidSensorAsBitMask & INV_NEEDS_ACCEL_MASK) > 0) || ((androidSensorAsBitMask & INV_NEEDS_ACCEL_MASK1) > 0))
    {
      data_rdy_status |= DMP_Data_ready_Accel;
      inv_event_control |= DMP_Motion_Event_Control_Accel_Calibr;
    }
    if (((androidSensorAsBitMask & INV_NEEDS_GYRO_MASK) > 0) || ((androidSensorAsBitMask & INV_NEEDS_GYRO_MASK1) > 0))
    {
      data_rdy_status |= DMP_Data_ready_Gyro;
      inv_event_control |= DMP_Motion_Event_Control_Gyro_Calibr;
    }
    if (((androidSensorAsBitMask & INV_NEEDS_COMPASS_MASK) > 0) || ((androidSensorAsBitMask & INV_NEEDS_COMPASS_MASK1) > 0))
    {
      data_rdy_status |= DMP_Data_ready_Secondary_Compass;
      inv_event_control |= DMP_Motion_Event_Control_Compass_Calibr;
    }
  }

  /*
  ICM_20948_sleep(pdev, false); // Make sure chip is awake
  if (result != ICM_20948_Stat_Ok)
  {
    return result;
  }

  ICM_20948_low_power(pdev, false); // Make sure chip is not in low power state
  if (result != ICM_20948_Stat_Ok)
  {
    return result;
  }*/

  // Check if Accel, Gyro/Gyro_Calibr or Compass_Calibr/Quat9/GeoMag/Compass are to be enabled. If they are then we need to request the accuracy data via header2.
  uint16_t delta2 = 0;
  if ((delta & DMP_Data_Output_Control_1_Accel) > 0)
  {
    delta2 |= DMP_Data_Output_Control_2_Accel_Accuracy;
  }
  if (((delta & DMP_Data_Output_Control_1_Gyro_Calibr) > 0) || ((delta & DMP_Data_Output_Control_1_Gyro) > 0))
  {
    delta2 |= DMP_Data_Output_Control_2_Gyro_Accuracy;
  }
  if (((delta & DMP_Data_Output_Control_1_Compass_Calibr) > 0) || ((delta & DMP_Data_Output_Control_1_Compass) > 0) || ((delta & DMP_Data_Output_Control_1_Quat9) > 0) || ((delta & DMP_Data_Output_Control_1_Geomag) > 0))
  {
    delta2 |= DMP_Data_Output_Control_2_Compass_Accuracy;
  }
  // TO DO: Add DMP_Data_Output_Control_2_Pickup etc. if required

  // Write the sensor control bits into memory address DATA_OUT_CTL1
  unsigned char data_output_control_reg[2];
  data_output_control_reg[0] = (unsigned char)(delta >> 8);
  data_output_control_reg[1] = (unsigned char)(delta & 0xff);
  
  icm20948_write_mem(DATA_OUT_CTL1, 2, (const unsigned char *)&data_output_control_reg);


  // Write the 'header2' sensor control bits into memory address DATA_OUT_CTL2
  data_output_control_reg[0] = (unsigned char)(delta2 >> 8);
  data_output_control_reg[1] = (unsigned char)(delta2 & 0xff);
  
  icm20948_write_mem(DATA_OUT_CTL2, 2, (const unsigned char *)&data_output_control_reg);


  // Set the DATA_RDY_STATUS register
  data_output_control_reg[0] = (unsigned char)(data_rdy_status >> 8);
  data_output_control_reg[1] = (unsigned char)(data_rdy_status & 0xff);
  
  icm20948_write_mem(DATA_RDY_STATUS, 2, (const unsigned char *)&data_output_control_reg);


  // Check which extra bits need to be set in the Motion Event Control register
  if ((delta & DMP_Data_Output_Control_1_Quat9) > 0)
  {
    inv_event_control |= DMP_Motion_Event_Control_9axis;
  }
  if (((delta & DMP_Data_Output_Control_1_Step_Detector) > 0) || ((delta & DMP_Data_Output_Control_1_Step_Ind_0) > 0) || ((delta & DMP_Data_Output_Control_1_Step_Ind_1) > 0) || ((delta & DMP_Data_Output_Control_1_Step_Ind_2) > 0))
  {
    inv_event_control |= DMP_Motion_Event_Control_Pedometer_Interrupt;
  }
  if ((delta & DMP_Data_Output_Control_1_Geomag) > 0)
  {
    inv_event_control |= DMP_Motion_Event_Control_Geomag;
  }

  // Set the MOTION_EVENT_CTL register
  data_output_control_reg[0] = (unsigned char)(inv_event_control >> 8);
  data_output_control_reg[1] = (unsigned char)(inv_event_control & 0xff);
  
  icm20948_write_mem(MOTION_EVENT_CTL, 2, (const unsigned char *)&data_output_control_reg);
  /*if (result != ICM_20948_Stat_Ok)
  {
    return result;
  }

  ICM_20948_low_power(pdev, true); // Put chip into low power state

  */

  return 0;
}


int8_t icm20948_set_dmp_sensor_period(enum DMP_ODR_Registers odr_reg, uint16_t interval)
{
  // Set the ODR registers and clear the ODR counter

  // In order to set an ODR for a given sensor data, write 2-byte value to DMP using key defined above for a particular sensor.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate (225Hz) / ODR ) - 1
  // E.g. For a 25Hz ODR rate, value= (225/25) -1 = 8.

  // During run-time, if an ODR is changed, the corresponding rate counter must be reset.
  // To reset, write 2-byte {0,0} to DMP using keys below for a particular sensor:

  unsigned char odr_reg_val[2];
  odr_reg_val[0] = (unsigned char)(interval >> 8);
  odr_reg_val[1] = (unsigned char)(interval & 0xff);

  unsigned char odr_count_zero[2] = {0x00, 0x00};

  //ICM_20948_sleep(false); // Make sure chip is awake

  //ICM_20948_low_power(false); // Make sure chip is not in low power state


  switch (odr_reg)
  {
  case DMP_ODR_Reg_Cpass_Calibr:
  {
    icm20948_write_mem(ODR_CPASS_CALIBR, 2, (const unsigned char *)&odr_reg_val);
    icm20948_write_mem(ODR_CNTR_CPASS_CALIBR, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_Gyro_Calibr:
  {
    icm20948_write_mem(ODR_GYRO_CALIBR, 2, (const unsigned char *)&odr_reg_val);
    icm20948_write_mem(ODR_CNTR_GYRO_CALIBR, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_Pressure:
  {
    icm20948_write_mem(ODR_PRESSURE, 2, (const unsigned char *)&odr_reg_val);
    icm20948_write_mem(ODR_CNTR_PRESSURE, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_Geomag:
  {
    icm20948_write_mem(ODR_GEOMAG, 2, (const unsigned char *)&odr_reg_val);
    icm20948_write_mem(ODR_CNTR_GEOMAG, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_PQuat6:
  {
    icm20948_write_mem(ODR_PQUAT6, 2, (const unsigned char *)&odr_reg_val);
    icm20948_write_mem(ODR_CNTR_PQUAT6, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_Quat9:
  {
    icm20948_write_mem(ODR_QUAT9, 2, (const unsigned char *)&odr_reg_val);
    icm20948_write_mem(ODR_CNTR_QUAT9, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_Quat6:
  {
    icm20948_write_mem(ODR_QUAT6, 2, (const unsigned char *)&odr_reg_val);
    icm20948_write_mem(ODR_CNTR_QUAT6, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_ALS:
  {
    icm20948_write_mem(ODR_ALS, 2, (const unsigned char *)&odr_reg_val);
    icm20948_write_mem(ODR_CNTR_ALS, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_Cpass:
  {
    icm20948_write_mem(ODR_CPASS, 2, (const unsigned char *)&odr_reg_val);
    icm20948_write_mem(ODR_CNTR_CPASS, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_Gyro:
  {
    icm20948_write_mem(ODR_GYRO, 2, (const unsigned char *)&odr_reg_val);
    icm20948_write_mem(ODR_CNTR_GYRO, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  case DMP_ODR_Reg_Accel:
  {
    icm20948_write_mem(ODR_ACCEL, 2, (const unsigned char *)&odr_reg_val);
    icm20948_write_mem(ODR_CNTR_ACCEL, 2, (const unsigned char *)&odr_count_zero);
  }
  break;
  default:
    ICM_20948_Stat_InvalDMPRegister;
    break;
  }

  //ICM_20948_low_power(true); // Put chip into low power state

  return 0;
}


int8_t icm20948_get_FIFO_count(uint16_t *count)
{
  ICM_20948_FIFO_COUNTH_t ctrlh;
  ICM_20948_FIFO_COUNTL_t ctrll;
  
  icm20948_set_user_bank(0);

  HAL_I2C_Mem_Read(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_FIFO_COUNT_H, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrlh, sizeof(ICM_20948_FIFO_COUNTH_t), 1000);
  ctrlh.FIFO_COUNTH &= 0x1F; // Datasheet says "FIFO_CNT[12:8]"
  HAL_I2C_Mem_Read(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_FIFO_COUNT_L, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrll, sizeof(ICM_20948_FIFO_COUNTL_t), 1000);


  *count = (((uint16_t)ctrlh.FIFO_COUNTH) << 8) | (uint16_t)ctrll.FIFO_COUNTL;

  return 0;
}


int8_t icm20948_read_FIFO(uint8_t *data, uint8_t len)
{
  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  icm20948_set_user_bank(0);

  HAL_I2C_Mem_Read(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_FIFO_R_W, I2C_MEMADD_SIZE_8BIT, data, len, 1000);

  return 0;
}


int8_t icm20948_read_dmp_data(icm_20948_DMP_data_t *data)
{
  uint8_t fifoBytes[icm_20948_DMP_Maximum_Bytes]; // Interim storage for the FIFO data

  // Check how much data is in the FIFO
  uint16_t fifo_count;
  icm20948_get_FIFO_count(&fifo_count);


  if (fifo_count < icm_20948_DMP_Header_Bytes) // Has a 2-byte header arrived?
    return -1;     // Bail if no header is available

  // Read the header (2 bytes)
  data->header = 0; // Clear the existing header
  uint16_t aShort = 0;
  icm20948_read_FIFO(&fifoBytes[0], icm_20948_DMP_Header_Bytes);

  for (int i = 0; i < icm_20948_DMP_Header_Bytes; i++)
  {
    aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8)); // MSB first
  }
  data->header = aShort;                    // Store the header in data->header
  fifo_count -= icm_20948_DMP_Header_Bytes; // Decrement the count

  // If the header indicates a header2 is present then read that now
  data->header2 = 0;                                  // Clear the existing header2
  if ((data->header & DMP_header_bitmap_Header2) > 0) // If the header2 bit is set
  {
    if (fifo_count < icm_20948_DMP_Header2_Bytes) // Check if we need to read the FIFO count again
      icm20948_get_FIFO_count(&fifo_count);
    
    if (fifo_count < icm_20948_DMP_Header2_Bytes)
      return -1; // Bail if no header2 is available
    // Read the header (2 bytes)
    aShort = 0;
    icm20948_read_FIFO(&fifoBytes[0], icm_20948_DMP_Header2_Bytes);

    for (int i = 0; i < icm_20948_DMP_Header2_Bytes; i++)
    {
      aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8));
    }
    data->header2 = aShort;                    // Store the header2 in data->header2
    fifo_count -= icm_20948_DMP_Header2_Bytes; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Accel) > 0) // case DMP_header_bitmap_Accel:
  {
    if (fifo_count < icm_20948_DMP_Raw_Accel_Bytes) // Check if we need to read the FIFO count again
      icm20948_get_FIFO_count(&fifo_count);
    
    if (fifo_count < icm_20948_DMP_Raw_Accel_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    icm20948_read_FIFO(&fifoBytes[0], icm_20948_DMP_Raw_Accel_Bytes);

    for (int i = 0; i < icm_20948_DMP_Raw_Accel_Bytes; i++)
    {
      data->Raw_Accel.Bytes[DMP_PQuat6_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= icm_20948_DMP_Raw_Accel_Bytes; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Gyro) > 0) // case DMP_header_bitmap_Gyro:
  {
    if (fifo_count < (icm_20948_DMP_Raw_Gyro_Bytes + icm_20948_DMP_Gyro_Bias_Bytes)) // Check if we need to read the FIFO count again
      icm20948_get_FIFO_count(&fifo_count);

    if (fifo_count < (icm_20948_DMP_Raw_Gyro_Bytes + icm_20948_DMP_Gyro_Bias_Bytes))
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    icm20948_read_FIFO(&fifoBytes[0], (icm_20948_DMP_Raw_Gyro_Bytes + icm_20948_DMP_Gyro_Bias_Bytes));

    for (int i = 0; i < (icm_20948_DMP_Raw_Gyro_Bytes + icm_20948_DMP_Gyro_Bias_Bytes); i++)
    {
      data->Raw_Gyro.Bytes[DMP_Raw_Gyro_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= (icm_20948_DMP_Raw_Gyro_Bytes + icm_20948_DMP_Gyro_Bias_Bytes); // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Compass) > 0) // case DMP_header_bitmap_Compass:
  {
    if (fifo_count < icm_20948_DMP_Compass_Bytes) // Check if we need to read the FIFO count again
      icm20948_get_FIFO_count(&fifo_count);
    
    if (fifo_count < icm_20948_DMP_Compass_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    icm20948_read_FIFO(&fifoBytes[0], icm_20948_DMP_Compass_Bytes);

    for (int i = 0; i < icm_20948_DMP_Compass_Bytes; i++)
    {
      data->Compass.Bytes[DMP_PQuat6_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= icm_20948_DMP_Compass_Bytes; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_ALS) > 0) // case DMP_header_bitmap_ALS:
  {
    if (fifo_count < icm_20948_DMP_ALS_Bytes) // Check if we need to read the FIFO count again
      icm20948_get_FIFO_count(&fifo_count);

    if (fifo_count < icm_20948_DMP_ALS_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    icm20948_read_FIFO(&fifoBytes[0], icm_20948_DMP_ALS_Bytes);

    for (int i = 0; i < icm_20948_DMP_ALS_Bytes; i++)
    {
      data->ALS[i] = fifoBytes[i];
    }
    fifo_count -= icm_20948_DMP_ALS_Bytes; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Quat6) > 0) // case DMP_header_bitmap_Quat6:
  {
    if (fifo_count < icm_20948_DMP_Quat6_Bytes) // Check if we need to read the FIFO count again
      icm20948_get_FIFO_count(&fifo_count);
    
    if (fifo_count < icm_20948_DMP_Quat6_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    icm20948_read_FIFO(&fifoBytes[0], icm_20948_DMP_Quat6_Bytes);
    for (int i = 0; i < icm_20948_DMP_Quat6_Bytes; i++)
    {
      data->Quat6.Bytes[DMP_Quat6_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= icm_20948_DMP_Quat6_Bytes; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Quat9) > 0) // case DMP_header_bitmap_Quat9:
  {
    if (fifo_count < icm_20948_DMP_Quat9_Bytes) // Check if we need to read the FIFO count again
      icm20948_get_FIFO_count(&fifo_count);
    
    if (fifo_count < icm_20948_DMP_Quat9_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    icm20948_read_FIFO(&fifoBytes[0], icm_20948_DMP_Quat9_Bytes);

    for (int i = 0; i < icm_20948_DMP_Quat9_Bytes; i++)
    {
      data->Quat9.Bytes[DMP_Quat9_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= icm_20948_DMP_Quat9_Bytes; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_PQuat6) > 0) // case DMP_header_bitmap_PQuat6:
  {
    if (fifo_count < icm_20948_DMP_PQuat6_Bytes) // Check if we need to read the FIFO count again
      icm20948_get_FIFO_count(&fifo_count);
    
    if (fifo_count < icm_20948_DMP_PQuat6_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    icm20948_read_FIFO(&fifoBytes[0], icm_20948_DMP_PQuat6_Bytes);

    for (int i = 0; i < icm_20948_DMP_PQuat6_Bytes; i++)
    {
      data->PQuat6.Bytes[DMP_PQuat6_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= icm_20948_DMP_PQuat6_Bytes; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Geomag) > 0) // case DMP_header_bitmap_Geomag:
  {
    if (fifo_count < icm_20948_DMP_Geomag_Bytes) // Check if we need to read the FIFO count again
      icm20948_get_FIFO_count(&fifo_count);
    
    if (fifo_count < icm_20948_DMP_Geomag_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    icm20948_read_FIFO(&fifoBytes[0], icm_20948_DMP_Geomag_Bytes);

    for (int i = 0; i < icm_20948_DMP_Geomag_Bytes; i++)
    {
      data->Geomag.Bytes[DMP_Quat9_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= icm_20948_DMP_Geomag_Bytes; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Pressure) > 0) // case DMP_header_bitmap_Pressure:
  {
    if (fifo_count < icm_20948_DMP_Pressure_Bytes) // Check if we need to read the FIFO count again
      icm20948_get_FIFO_count(&fifo_count);

    if (fifo_count < icm_20948_DMP_Pressure_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    icm20948_read_FIFO(&fifoBytes[0], icm_20948_DMP_Pressure_Bytes);

    for (int i = 0; i < icm_20948_DMP_Pressure_Bytes; i++)
    {
      data->Pressure[i] = fifoBytes[i];
    }
    fifo_count -= icm_20948_DMP_Pressure_Bytes; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Gyro_Calibr) > 0) // case DMP_header_bitmap_Gyro_Calibr:
  {
    // lcm20948MPUFifoControl.c suggests icm_20948_DMP_Gyro_Calibr_Bytes is not supported
    // and looking at DMP frames which have the Gyro_Calibr bit set, that certainly seems to be true.
    // So, we'll skip this...:
    /*
			if (fifo_count < icm_20948_DMP_Gyro_Calibr_Bytes) // Check if we need to read the FIFO count again
			{
					icm20948_get_FIFO_count(&fifo_count);
					if (result != ICM_20948_Stat_Ok)
							return result;
			}
			if (fifo_count < icm_20948_DMP_Gyro_Calibr_Bytes)
					return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
			icm20948_read_FIFO(&fifoBytes[0], icm_20948_DMP_Gyro_Calibr_Bytes);
			if (result != ICM_20948_Stat_Ok)
					return result;
			for (int i = 0; i < icm_20948_DMP_Gyro_Calibr_Bytes; i++)
			{
					data->Gyro_Calibr.Bytes[DMP_Quat6_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
			}
			fifo_count -= icm_20948_DMP_Gyro_Calibr_Bytes; // Decrement the count
			*/
  }

  if ((data->header & DMP_header_bitmap_Compass_Calibr) > 0) // case DMP_header_bitmap_Compass_Calibr:
  {
    if (fifo_count < icm_20948_DMP_Compass_Calibr_Bytes) // Check if we need to read the FIFO count again
      icm20948_get_FIFO_count(&fifo_count);
    
    if (fifo_count < icm_20948_DMP_Compass_Calibr_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    icm20948_read_FIFO(&fifoBytes[0], icm_20948_DMP_Compass_Calibr_Bytes);

    for (int i = 0; i < icm_20948_DMP_Compass_Calibr_Bytes; i++)
    {
      data->Compass_Calibr.Bytes[DMP_Quat6_Byte_Ordering[i]] = fifoBytes[i]; // Correct the byte order (map big endian to little endian)
    }
    fifo_count -= icm_20948_DMP_Compass_Calibr_Bytes; // Decrement the count
  }

  if ((data->header & DMP_header_bitmap_Step_Detector) > 0) // case DMP_header_bitmap_Step_Detector:
  {
    if (fifo_count < icm_20948_DMP_Step_Detector_Bytes) // Check if we need to read the FIFO count again
      icm20948_get_FIFO_count(&fifo_count);

    if (fifo_count < icm_20948_DMP_Step_Detector_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    icm20948_read_FIFO(&fifoBytes[0], icm_20948_DMP_Step_Detector_Bytes);

    uint32_t aWord = 0;
    for (int i = 0; i < icm_20948_DMP_Step_Detector_Bytes; i++)
    {
      aWord |= ((uint32_t)fifoBytes[i]) << (24 - (i * 8)); // MSB first
    }
    data->Pedometer_Timestamp = aWord;
    fifo_count -= icm_20948_DMP_Step_Detector_Bytes; // Decrement the count
  }

  // Now check for header2 features

  if ((data->header2 & DMP_header2_bitmap_Accel_Accuracy) > 0) // case DMP_header2_bitmap_Accel_Accuracy:
  {
    if (fifo_count < icm_20948_DMP_Accel_Accuracy_Bytes) // Check if we need to read the FIFO count again
      icm20948_get_FIFO_count(&fifo_count);

    if (fifo_count < icm_20948_DMP_Accel_Accuracy_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    aShort = 0;
    icm20948_read_FIFO(&fifoBytes[0], icm_20948_DMP_Accel_Accuracy_Bytes);

    for (int i = 0; i < icm_20948_DMP_Accel_Accuracy_Bytes; i++)
    {
      aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8));
    }
    data->Accel_Accuracy = aShort;
    fifo_count -= icm_20948_DMP_Accel_Accuracy_Bytes; // Decrement the count
  }

  if ((data->header2 & DMP_header2_bitmap_Gyro_Accuracy) > 0) // case DMP_header2_bitmap_Gyro_Accuracy:
  {
    if (fifo_count < icm_20948_DMP_Gyro_Accuracy_Bytes) // Check if we need to read the FIFO count again
      icm20948_get_FIFO_count(&fifo_count);

    if (fifo_count < icm_20948_DMP_Gyro_Accuracy_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    aShort = 0;
    icm20948_read_FIFO(&fifoBytes[0], icm_20948_DMP_Gyro_Accuracy_Bytes);

    for (int i = 0; i < icm_20948_DMP_Gyro_Accuracy_Bytes; i++)
    {
      aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8));
    }
    data->Gyro_Accuracy = aShort;
    fifo_count -= icm_20948_DMP_Gyro_Accuracy_Bytes; // Decrement the count
  }

  if ((data->header2 & DMP_header2_bitmap_Compass_Accuracy) > 0) // case DMP_header2_bitmap_Compass_Accuracy:
  {
    if (fifo_count < icm_20948_DMP_Compass_Accuracy_Bytes) // Check if we need to read the FIFO count again
      icm20948_get_FIFO_count(&fifo_count);

    if (fifo_count < icm_20948_DMP_Compass_Accuracy_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    aShort = 0;
    icm20948_read_FIFO(&fifoBytes[0], icm_20948_DMP_Compass_Accuracy_Bytes);

    for (int i = 0; i < icm_20948_DMP_Compass_Accuracy_Bytes; i++)
    {
      aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8));
    }
    data->Compass_Accuracy = aShort;
    fifo_count -= icm_20948_DMP_Compass_Accuracy_Bytes; // Decrement the count
  }

  if ((data->header2 & DMP_header2_bitmap_Fsync) > 0) // case DMP_header2_bitmap_Fsync:
  {
    // lcm20948MPUFifoControl.c suggests icm_20948_DMP_Fsync_Detection_Bytes is not supported.
    // So, we'll skip this just in case...:
    /*
			if (fifo_count < icm_20948_DMP_Fsync_Detection_Bytes) // Check if we need to read the FIFO count again
			{
					icm20948_get_FIFO_count(&fifo_count);
					if (result != ICM_20948_Stat_Ok)
							return result;
			}
			if (fifo_count < icm_20948_DMP_Fsync_Detection_Bytes)
					return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
			aShort = 0;
			icm20948_read_FIFO(&fifoBytes[0], icm_20948_DMP_Fsync_Detection_Bytes);
			if (result != ICM_20948_Stat_Ok)
					return result;
			for (int i = 0; i < icm_20948_DMP_Fsync_Detection_Bytes; i++)
			{
					aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8));
			}
			data->Fsync_Delay_Time = aShort;
			fifo_count -= icm_20948_DMP_Fsync_Detection_Bytes; // Decrement the count
			*/
  }

  if ((data->header2 & DMP_header2_bitmap_Pickup) > 0) // case DMP_header2_bitmap_Pickup:
  {
    if (fifo_count < icm_20948_DMP_Pickup_Bytes) // Check if we need to read the FIFO count again
      icm20948_get_FIFO_count(&fifo_count);

    if (fifo_count < icm_20948_DMP_Pickup_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    aShort = 0;
    icm20948_read_FIFO(&fifoBytes[0], icm_20948_DMP_Pickup_Bytes);

    for (int i = 0; i < icm_20948_DMP_Pickup_Bytes; i++)
    {
      aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8));
    }
    data->Pickup = aShort;
    fifo_count -= icm_20948_DMP_Pickup_Bytes; // Decrement the count
  }

  if ((data->header2 & DMP_header2_bitmap_Activity_Recog) > 0) // case DMP_header2_bitmap_Activity_Recog:
  {
    if (fifo_count < icm_20948_DMP_Activity_Recognition_Bytes) // Check if we need to read the FIFO count again
      icm20948_get_FIFO_count(&fifo_count);

    if (fifo_count < icm_20948_DMP_Activity_Recognition_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    icm20948_read_FIFO(&fifoBytes[0], icm_20948_DMP_Activity_Recognition_Bytes);

    for (int i = 0; i < icm_20948_DMP_Activity_Recognition_Bytes; i++)
    {
      data->Activity_Recognition.Bytes[DMP_Activity_Recognition_Byte_Ordering[i]] = fifoBytes[i];
    }
    fifo_count -= icm_20948_DMP_Activity_Recognition_Bytes; // Decrement the count
  }

  if ((data->header2 & DMP_header2_bitmap_Secondary_On_Off) > 0) // case DMP_header2_bitmap_Secondary_On_Off:
  {
    if (fifo_count < icm_20948_DMP_Secondary_On_Off_Bytes) // Check if we need to read the FIFO count again
      icm20948_get_FIFO_count(&fifo_count);

    if (fifo_count < icm_20948_DMP_Secondary_On_Off_Bytes)
      return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
    icm20948_read_FIFO(&fifoBytes[0], icm_20948_DMP_Secondary_On_Off_Bytes);

    for (int i = 0; i < icm_20948_DMP_Secondary_On_Off_Bytes; i++)
    {
      data->Secondary_On_Off.Bytes[DMP_Secondary_On_Off_Byte_Ordering[i]] = fifoBytes[i];
    }
    fifo_count -= icm_20948_DMP_Secondary_On_Off_Bytes; // Decrement the count
  }

  // Finally, extract the footer (gyro count)
  if (fifo_count < icm_20948_DMP_Footer_Bytes) // Check if we need to read the FIFO count again
    icm20948_get_FIFO_count(&fifo_count);

  if (fifo_count < icm_20948_DMP_Footer_Bytes)
    return ICM_20948_Stat_FIFOIncompleteData; // Bail if not enough data is available
  aShort = 0;
  icm20948_read_FIFO(&fifoBytes[0], icm_20948_DMP_Footer_Bytes);

  for (int i = 0; i < icm_20948_DMP_Footer_Bytes; i++)
  {
    aShort |= ((uint16_t)fifoBytes[i]) << (8 - (i * 8));
  }
  data->Footer = aShort;
  fifo_count -= icm_20948_DMP_Footer_Bytes; // Decrement the count

  if (fifo_count > 0) // Check if there is still data waiting to be read
    return ICM_20948_Stat_FIFOMoreDataAvail;

  return 0;
}

int8_t icm20948_load_firmware(const unsigned char *data_start, unsigned short size_start, unsigned short load_addr)
{
  int write_size;
  unsigned short memaddr;
  const unsigned char *data;
  unsigned short size;
  unsigned char data_cmp[MAX_SERIAL_R_W];
  int flag = 0;

/* //STATUS CHECKS
  if (pdev->_dmp_firmware_available == false)
    return ICM_20948_Stat_DMPNotSupported;

  if (pdev->_firmware_loaded)
    return ICM_20948_Stat_Ok; // Bail with no error if firmware is already loaded

  ICM_20948_sleep(pdev, false); // Make sure chip is awake
  if (result != ICM_20948_Stat_Ok)
  {
    return result;
  }

  ICM_20948_low_power(pdev, false); // Make sure chip is not in low power state
  if (result != ICM_20948_Stat_Ok)
  {
    return result;
  }
*/

  // Write DMP memory

  data = data_start;
  size = size_start;
  memaddr = load_addr;

  while (size > 0)
  {
    if (size <= MAX_SERIAL_R_W) // Break into maximally sized chunks
      write_size = size;
    else
      write_size = MAX_SERIAL_R_W;

    // Chunk moves across a bank
    if ((memaddr & 0xff) + write_size > 0x100)
    {
      // this sets the packet size to the remainder...
      write_size = (memaddr & 0xff) + write_size - 0x100;
      //...should it be instead set to the rest of the chunk?
      write_size = 0x100 - (memaddr & 0xff);
    }
    icm20948_write_mem(memaddr, write_size, (unsigned char *)data);

    data += write_size;
    size -= write_size;
    memaddr += write_size;
  }

  // Verify DMP memory

  data = data_start;
  size = size_start;
  memaddr = load_addr;
  while (size > 0)
  {
    if (size <= MAX_SERIAL_R_W) // Read in chunks of INV_MAX_SERIAL_READ
      write_size = size;
    else
      write_size = MAX_SERIAL_R_W;
    if ((memaddr & 0xff) + write_size > 0x100)
    {
      // this sets the packet size to the remainder...
      write_size = (memaddr & 0xff) + write_size - 0x100;
      //...should it be instead set to the rest of the chunk?
      write_size = 0x100 - (memaddr & 0xff);
    }
    icm20948_read_mem(memaddr, write_size, data_cmp);
    //if (result != ICM_20948_Stat_Ok)
    //  flag++;                               // Error, DMP not written correctly

    if (memcmp(data_cmp, data, write_size)) // Compare the data
      return -1;
    data += write_size;
    size -= write_size;
    memaddr += write_size;
  }
  /*
  //Enable LP_EN since we disabled it at begining of this function.
  ICM_20948_low_power(pdev, true); // Put chip into low power state
  if (result != ICM_20948_Stat_Ok)
    return result;
  */
  return 0;
}

void icm20948_set_i2c_bus(I2C_HandleTypeDef *bus)
{
	i2c_bus = *bus;
}

int8_t icm20948_initialize_DMP(void)
{

  // Normally, when the DMP is not enabled, startupMagnetometer (called by startupDefault, which is called by begin) configures the AK09916 magnetometer
  // to run at 100Hz by setting the CNTL2 register (0x31) to 0x08. Then the ICM20948's I2C_SLV0 is configured to read
  // nine bytes from the mag every sample, starting from the STATUS1 register (0x10). ST1 includes the DRDY (Data Ready) bit.
  // Next are the six magnetometer readings (little endian). After a dummy byte, the STATUS2 register (0x18) contains the HOFL (Overflow) bit.
  //
  // But looking very closely at the InvenSense example code, we can see in inv_icm20948_resume_akm (in Icm20948AuxCompassAkm.c) that,
  // when the DMP is running, the magnetometer is set to Single Measurement (SM) mode and that ten bytes are read, starting from the reserved
  // RSV2 register (0x03). The datasheet does not define what registers 0x04 to 0x0C contain. There is definitely some secret sauce in here...
  // The magnetometer data appears to be big endian (not little endian like the HX/Y/Z registers) and starts at register 0x04.
  // We had to examine the I2C traffic between the master and the AK09916 on the AUX_DA and AUX_CL pins to discover this...
  //
  // So, we need to set up I2C_SLV0 to do the ten byte reading. The parameters passed to i2cControllerConfigurePeripheral are:
  // 0: use I2C_SLV0
  // MAG_AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
  // AK09916_REG_RSV2: we start reading here (0x03). Secret sauce...
  // 10: we read 10 bytes each cycle
  // true: set the I2C_SLV0_RNW ReadNotWrite bit so we read the 10 bytes (not write them)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit to enable reading from the peripheral at the sample rate
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_GRP bit to show the register pairing starts at byte 1+2 (copied from inv_icm20948_resume_akm)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW to byte-swap the data from the mag (copied from inv_icm20948_resume_akm)
  icm20948_i2c_controller_configure_peripheral(0, MAG_AK09916_I2C_ADDR, AK09916_REG_RSV2, 10, true, true, false, true, true, 0);
  //
  // We also need to set up I2C_SLV1 to do the Single Measurement triggering:
  // 1: use I2C_SLV1
  // MAG_AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
  // AK09916_REG_CNTL2: we start writing here (0x31)
  // 1: not sure why, but the write does not happen if this is set to zero
  // false: clear the I2C_SLV0_RNW ReadNotWrite bit so we write the dataOut byte
  // true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit. Not sure why, but the write does not happen if this is clear
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_GRP bit
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW bit
  // AK09916_mode_single: tell I2C_SLV1 to write the Single Measurement command each sample
  icm20948_i2c_controller_configure_peripheral(1, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL2, 1, false, true, false, false, false, AK09916_mode_single);

  // Set the I2C Master ODR configuration
  // It is not clear why we need to do this... But it appears to be essential! From the datasheet:
  // "I2C_MST_ODR_CONFIG[3:0]: ODR configuration for external sensor when gyroscope and accelerometer are disabled.
  //  ODR is computed as follows: 1.1 kHz/(2^((odr_config[3:0])) )
  //  When gyroscope is enabled, all sensors (including I2C_MASTER) use the gyroscope ODR.
  //  If gyroscope is disabled, then all sensors (including I2C_MASTER) use the accelerometer ODR."
  // Since both gyro and accel are running, setting this register should have no effect. But it does. Maybe because the Gyro and Accel are placed in Low Power Mode (cycled)?
  // You can see by monitoring the Aux I2C pins that the next three lines reduce the bus traffic (magnetometer reads) from 1125Hz to the chosen rate: 68.75Hz in this case.
  icm20948_set_user_bank(3);
  uint8_t mstODRconfig = 0x04; // Set the ODR configuration to 1100/2^4 = 68.75Hz
  HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR, AGB3_REG_I2C_MST_ODR_CONFIG, I2C_MEMADD_SIZE_8BIT,  &mstODRconfig, 1, 1000);

  // Configure clock source through PWR_MGMT_1
  // ICM_20948_Clock_Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
  icm20948_set_clock_source(ICM_20948_Clock_Auto);

  // Enable accel and gyro sensors through PWR_MGMT_2
  // Enable Accelerometer (all axes) and Gyroscope (all axes) by writing zero to PWR_MGMT_2
  icm20948_set_user_bank(0); // Select Bank 0
  uint8_t pwrMgmt2 = 0x40;                                                          // Set the reserved bit 6 (pressure sensor disable?)
  HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_PWR_MGMT_2, I2C_MEMADD_SIZE_8BIT, &pwrMgmt2, 1, 1000);

  // Place _only_ I2C_Master in Low Power Mode (cycled) via LP_CONFIG
  // The InvenSense Nucleo example initially puts the accel and gyro into low power mode too, but then later updates LP_CONFIG so only the I2C_Master is in Low Power Mode
  icm20948_set_sample_mode(ICM_20948_Internal_Mst, ICM_20948_Sample_Mode_Cycled);

  // Disable the FIFO
  icm20948_enable_FIFO(false);

  // Disable the DMP
  icm20948_enable_DMP(false);

  // Set Gyro FSR (Full scale range) to 2000dps through GYRO_CONFIG_1
  // Set Accel FSR (Full scale range) to 4g through ACCEL_CONFIG
  ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
  myFSS.a = gpm4;        // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                         // gpm2
                         // gpm4
                         // gpm8
                         // gpm16
  myFSS.g = dps2000;     // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                         // dps250
                         // dps500
                         // dps1000
                         // dps2000
  icm20948_set_full_scale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);

  // The InvenSense Nucleo code also enables the gyro DLPF (but leaves GYRO_DLPFCFG set to zero = 196.6Hz (3dB))
  // We found this by going through the SPI data generated by ZaneL's Teensy-ICM-20948 library byte by byte...
  // The gyro DLPF is enabled by default (GYRO_CONFIG_1 = 0x01) so the following line should have no effect, but we'll include it anyway
  icm20948_enable_dlpf(ICM_20948_Internal_Gyr, true);

  // Enable interrupt for FIFO overflow from FIFOs through INT_ENABLE_2
  // If we see this interrupt, we'll need to reset the FIFO
  //intEnableOverflowFIFO( 0x1F ); // Enable the interrupt on all FIFOs

  // Turn off what goes into the FIFO through FIFO_EN_1, FIFO_EN_2
  // Stop the peripheral data from being written to the FIFO by writing zero to FIFO_EN_1
  icm20948_set_user_bank(0); // Select Bank 0
  uint8_t zero = 0;
  HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_FIFO_EN_1, I2C_MEMADD_SIZE_8BIT, &zero, 1, 1000);
  // Stop the accelerometer, gyro and temperature data from being written to the FIFO by writing zero to FIFO_EN_2
  HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_FIFO_EN_2, I2C_MEMADD_SIZE_8BIT, &zero, 1, 1000);

  // Turn off data ready interrupt through INT_ENABLE_1
  icm20948_int_enable_raw_data_ready(false);

  // Reset FIFO through FIFO_RST
  icm20948_reset_FIFO();

  // Set gyro sample rate divider with GYRO_SMPLRT_DIV
  // Set accel sample rate divider with ACCEL_SMPLRT_DIV_2
  ICM_20948_smplrt_t mySmplrt;
  mySmplrt.g = 19; // ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]). 19 = 55Hz. InvenSense Nucleo example uses 19 (0x13).
  mySmplrt.a = 19; // ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]). 19 = 56.25Hz. InvenSense Nucleo example uses 19 (0x13).
  //mySmplrt.g = 4; // 225Hz
  //mySmplrt.a = 4; // 225Hz
  //mySmplrt.g = 8; // 112Hz
  //mySmplrt.a = 8; // 112Hz
  icm20948_set_sample_rate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt);

  // Setup DMP start address through PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  icm20948_set_DMP_start_address(DMP_START_ADDRESS);

  // Now load the DMP firmware
  icm20948_load_firmware(dmp3_image, sizeof(dmp3_image), DMP_LOAD_START);

  // Write the 2 byte Firmware Start Value to ICM PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  icm20948_set_DMP_start_address(DMP_START_ADDRESS);

  // Set the Hardware Fix Disable register to 0x48
  icm20948_set_user_bank(0);
  uint8_t fix = 0x48;
  HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_HW_FIX_DISABLE, I2C_MEMADD_SIZE_8BIT, &fix, 1, 1000);

  // Set the Single FIFO Priority Select register to 0xE4
  icm20948_set_user_bank(0);
  uint8_t fifoPrio = 0xE4;
  HAL_I2C_Mem_Write(&i2c_bus, ICM_20948_I2C_ADDR, AGB0_REG_SINGLE_FIFO_PRIORITY_SEL, I2C_MEMADD_SIZE_8BIT, &fifoPrio, 1, 1000);

  // Configure Accel scaling to DMP
  // The DMP scales accel raw data internally to align 1g as 2^25
  // In order to align internal accel raw data 2^25 = 1g write 0x04000000 when FSR is 4g
  const unsigned char accScale[4] = {0x04, 0x00, 0x00, 0x00};
  icm20948_write_mem(ACC_SCALE, 4, &accScale[0]);
  // In order to output hardware unit data as configured FSR write 0x00040000 when FSR is 4g
  const unsigned char accScale2[4] = {0x00, 0x04, 0x00, 0x00};
  icm20948_write_mem(ACC_SCALE2, 4, &accScale2[0]);

  // Configure Compass mount matrix and scale to DMP
  // The mount matrix write to DMP register is used to align the compass axes with accel/gyro.
  // This mechanism is also used to convert hardware unit to uT. The value is expressed as 1uT = 2^30.
  // Each compass axis will be converted as below:
  // X = raw_x * CPASS_MTX_00 + raw_y * CPASS_MTX_01 + raw_z * CPASS_MTX_02
  // Y = raw_x * CPASS_MTX_10 + raw_y * CPASS_MTX_11 + raw_z * CPASS_MTX_12
  // Z = raw_x * CPASS_MTX_20 + raw_y * CPASS_MTX_21 + raw_z * CPASS_MTX_22
  // The AK09916 produces a 16-bit signed output in the range +/-32752 corresponding to +/-4912uT. 1uT = 6.66 ADU.
  // 2^30 / 6.66666 = 161061273 = 0x9999999
  const unsigned char mountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char mountMultiplierPlus[4] = {0x09, 0x99, 0x99, 0x99};  // Value taken from InvenSense Nucleo example
  const unsigned char mountMultiplierMinus[4] = {0xF6, 0x66, 0x66, 0x67}; // Value taken from InvenSense Nucleo example
  icm20948_write_mem(CPASS_MTX_00, 4, &mountMultiplierPlus[0]);
  icm20948_write_mem(CPASS_MTX_01, 4, &mountMultiplierZero[0]);
  icm20948_write_mem(CPASS_MTX_02, 4, &mountMultiplierZero[0]);
  icm20948_write_mem(CPASS_MTX_10, 4, &mountMultiplierZero[0]);
  icm20948_write_mem(CPASS_MTX_11, 4, &mountMultiplierMinus[0]);
  icm20948_write_mem(CPASS_MTX_12, 4, &mountMultiplierZero[0]);
  icm20948_write_mem(CPASS_MTX_20, 4, &mountMultiplierZero[0]);
  icm20948_write_mem(CPASS_MTX_21, 4, &mountMultiplierZero[0]);
  icm20948_write_mem(CPASS_MTX_22, 4, &mountMultiplierMinus[0]);

  // Configure the B2S Mounting Matrix
  const unsigned char b2sMountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char b2sMountMultiplierPlus[4] = {0x40, 0x00, 0x00, 0x00}; // Value taken from InvenSense Nucleo example
  icm20948_write_mem(B2S_MTX_00, 4, &b2sMountMultiplierPlus[0]);
  icm20948_write_mem(B2S_MTX_01, 4, &b2sMountMultiplierZero[0]);
  icm20948_write_mem(B2S_MTX_02, 4, &b2sMountMultiplierZero[0]);
  icm20948_write_mem(B2S_MTX_10, 4, &b2sMountMultiplierZero[0]);
  icm20948_write_mem(B2S_MTX_11, 4, &b2sMountMultiplierPlus[0]);
  icm20948_write_mem(B2S_MTX_12, 4, &b2sMountMultiplierZero[0]);
  icm20948_write_mem(B2S_MTX_20, 4, &b2sMountMultiplierZero[0]);
  icm20948_write_mem(B2S_MTX_21, 4, &b2sMountMultiplierZero[0]);
  icm20948_write_mem(B2S_MTX_22, 4, &b2sMountMultiplierPlus[0]);

  // Configure the DMP Gyro Scaling Factor
  // @param[in] gyro_div Value written to GYRO_SMPLRT_DIV register, where
  //            0=1125Hz sample rate, 1=562.5Hz sample rate, ... 4=225Hz sample rate, ...
  //            10=102.2727Hz sample rate, ... etc.
  // @param[in] gyro_level 0=250 dps, 1=500 dps, 2=1000 dps, 3=2000 dps
  icm20948_set_gyro_sf(19, 3); // 19 = 55Hz (see above), 3 = 2000dps (see above)

  // Configure the Gyro full scale
  // 2000dps : 2^28
  // 1000dps : 2^27
  //  500dps : 2^26
  //  250dps : 2^25
  const unsigned char gyroFullScale[4] = {0x10, 0x00, 0x00, 0x00}; // 2000dps : 2^28
  icm20948_write_mem(GYRO_FULLSCALE, 4, &gyroFullScale[0]);

  // Configure the Accel Only Gain: 15252014 (225Hz) 30504029 (112Hz) 61117001 (56Hz)
  const unsigned char accelOnlyGain[4] = {0x03, 0xA4, 0x92, 0x49}; // 56Hz
  //const unsigned char accelOnlyGain[4] = {0x00, 0xE8, 0xBA, 0x2E}; // 225Hz
  //const unsigned char accelOnlyGain[4] = {0x01, 0xD1, 0x74, 0x5D}; // 112Hz
  icm20948_write_mem(ACCEL_ONLY_GAIN, 4, &accelOnlyGain[0]);

  // Configure the Accel Alpha Var: 1026019965 (225Hz) 977872018 (112Hz) 882002213 (56Hz)
  const unsigned char accelAlphaVar[4] = {0x34, 0x92, 0x49, 0x25}; // 56Hz
  //const unsigned char accelAlphaVar[4] = {0x3D, 0x27, 0xD2, 0x7D}; // 225Hz
  //const unsigned char accelAlphaVar[4] = {0x3A, 0x49, 0x24, 0x92}; // 112Hz
  icm20948_write_mem(ACCEL_ALPHA_VAR, 4, &accelAlphaVar[0]);

  // Configure the Accel A Var: 47721859 (225Hz) 95869806 (112Hz) 191739611 (56Hz)
  const unsigned char accelAVar[4] = {0x0B, 0x6D, 0xB6, 0xDB}; // 56Hz
  //const unsigned char accelAVar[4] = {0x02, 0xD8, 0x2D, 0x83}; // 225Hz
  //const unsigned char accelAVar[4] = {0x05, 0xB6, 0xDB, 0x6E}; // 112Hz
  icm20948_write_mem(ACCEL_A_VAR, 4, &accelAVar[0]);

  // Configure the Accel Cal Rate
  const unsigned char accelCalRate[4] = {0x00, 0x00}; // Value taken from InvenSense Nucleo example
  icm20948_write_mem(ACCEL_CAL_RATE, 2, &accelCalRate[0]);

  // Configure the Compass Time Buffer. The I2C Master ODR Configuration (see above) sets the magnetometer read rate to 68.75Hz.
  // Let's set the Compass Time Buffer to 69 (Hz).
  const unsigned char compassRate[2] = {0x00, 0x45}; // 69Hz
  icm20948_write_mem(CPASS_TIME_BUFFER, 2, &compassRate[0]);

  // Enable DMP interrupt
  // This would be the most efficient way of getting the DMP data, instead of polling the FIFO
  //intEnableDMP(true);

  return 0;
}
