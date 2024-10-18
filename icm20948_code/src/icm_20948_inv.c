#include <stdint.h>
#include <stdbool.h>

#include "util/ICM_20948_REGISTERS.h"
#include "util/ICM_20948_ENUMERATIONS.h"


// select user bank 
void icm20948_set_user_bank(uint8_t bank){
    bank = (bank << 4) & 0x30; // bits 1:0 of bank -> 5:4 of icm bank select register
    HAL_I2C_Mem_Write(&hi2c1, ICM_20948_I2C_ADDR, REG_BANK_SEL, I2C_MEMADD_SIZE_8BIT, &bank, 1, 1000);
}

int icm20948_i2c_controller_configure_peripheral(uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap, uint8_t dataOut)
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
  HAL_I2C_Mem_Write(&hi2c1, ICM_20948_I2C_ADDR, periph_addr_reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&address, sizeof(ICM_20948_I2C_PERIPHX_ADDR_t), 1000);
  //ICM_20948_execute_w(pdev, periph_addr_reg, (uint8_t *)&address, sizeof(ICM_20948_I2C_PERIPHX_ADDR_t));


  // If we are setting up a write, configure the Data Out register too
  if (!Rw)
  {
    ICM_20948_I2C_PERIPHX_DO_t dataOutByte;
    dataOutByte.DO = dataOut;
    HAL_I2C_Mem_Write(&hi2c1, ICM_20948_I2C_ADDR, periph_do_reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&dataOutByte, sizeof(ICM_20948_I2C_PERIPHX_DO_t), 1000);
    //ICM_20948_execute_w(pdev, periph_do_reg, (uint8_t *)&dataOutByte, sizeof(ICM_20948_I2C_PERIPHX_DO_t));

  }

  // Set the peripheral sub-address (register address)
  ICM_20948_I2C_PERIPHX_REG_t subaddress;
  subaddress.REG = reg;
  HAL_I2C_Mem_Write(&hi2c1, ICM_20948_I2C_ADDR, periph_reg_reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&subaddress, sizeof(ICM_20948_I2C_PERIPHX_REG_t), 1000);
  //ICM_20948_execute_w(pdev, periph_reg_reg, (uint8_t *)&subaddress, sizeof(ICM_20948_I2C_PERIPHX_REG_t));


  // Set up the control info
  ICM_20948_I2C_PERIPHX_CTRL_t ctrl;
  ctrl.LENG = len;
  ctrl.EN = enable;
  ctrl.REG_DIS = data_only;
  ctrl.GRP = grp;
  ctrl.BYTE_SW = swap;
  HAL_I2C_Mem_Write(&hi2c1, ICM_20948_I2C_ADDR, periph_ctrl_reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl, sizeof(ICM_20948_I2C_PERIPHX_CTRL_t), 1000);
  //ICM_20948_execute_w(pdev, periph_ctrl_reg, (uint8_t *)&ctrl, sizeof(ICM_20948_I2C_PERIPHX_CTRL_t));

  return 0;
}

int icm20948_set_clock_source(ICM_20948_PWR_MGMT_1_CLKSEL_e source)
{
  ICM_20948_PWR_MGMT_1_t reg;
  icm20948_set_user_bank(0); // Must be in the right bank

  HAL_I2C_Mem_Read(&hi2c1, ICM_20948_I2C_ADDR, AGB0_REG_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg, sizeof(ICM_20948_PWR_MGMT_1_t), 1000);

  reg.CLKSEL = source;

  HAL_I2C_Mem_Write(&hi2c1, ICM_20948_I2C_ADDR, AGB0_REG_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg, sizeof(ICM_20948_PWR_MGMT_1_t), 1000);

  return 0;
}

int icm20948_set_sample_mode(ICM_20948_InternalSensorID_bm sensors, ICM_20948_LP_CONFIG_CYCLE_e mode)
{
  ICM_20948_LP_CONFIG_t reg;

  /*if (!(sensors & (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr | ICM_20948_Internal_Mst)))
  {
    return -1;
  }*/

  icm20948_set_user_bank(0); // Must be in the right bank

  HAL_I2C_Mem_Read(&hi2c1, ICM_20948_I2C_ADDR, AGB0_REG_LP_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg, sizeof(ICM_20948_LP_CONFIG_t), 1000);


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

  HAL_I2C_Mem_Write(&hi2c1, ICM_20948_I2C_ADDR, AGB0_REG_LP_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg, sizeof(ICM_20948_LP_CONFIG_t), 1000);

  // Check the data was written correctly
  //HAL_I2C_Mem_Read(&hi2c1, ICM_20948_I2C_ADDR, AGB0_REG_LP_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg, sizeof(ICM_20948_LP_CONFIG_t), 1000);
  return 0;
}

int icm20948_enable_FIFO(bool enable)
{
  ICM_20948_USER_CTRL_t ctrl;
  icm20948_set_user_bank(0);


  HAL_I2C_Mem_Read(&hi2c1, ICM_20948_I2C_ADDR, AGB0_REG_USER_CTRL, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl, sizeof(ICM_20948_USER_CTRL_t), 1000);

  if (enable)
    ctrl.FIFO_EN = 1;
  else
    ctrl.FIFO_EN = 0;

  HAL_I2C_Mem_Write(&hi2c1, ICM_20948_I2C_ADDR, AGB0_REG_USER_CTRL, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl, sizeof(ICM_20948_USER_CTRL_t), 1000);

  return 0;
}

int icm20948_reset_FIFO()
{
  ICM_20948_FIFO_RST_t ctrl;
  icm20948_set_user_bank(0);

  HAL_I2C_Mem_Read(&hi2c1, ICM_20948_I2C_ADDR, AGB0_REG_FIFO_RST, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl, sizeof(ICM_20948_FIFO_RST_t), 1000);

  ctrl.FIFO_RESET = 0x1F; // Datasheet says "FIFO_RESET[4:0]"

  HAL_I2C_Mem_Write(&hi2c1, ICM_20948_I2C_ADDR, AGB0_REG_FIFO_RST, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl, sizeof(ICM_20948_FIFO_RST_t), 1000);


  //delay ???

  ctrl.FIFO_RESET = 0x1E; // The InvenSense Nucleo examples write 0x1F followed by 0x1E
  HAL_I2C_Mem_Write(&hi2c1, ICM_20948_I2C_ADDR, AGB0_REG_FIFO_RST, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl, sizeof(ICM_20948_FIFO_RST_t), 1000);

  return 0;
}

int icm20948_enable_DMP(bool enable)
{
  ICM_20948_USER_CTRL_t ctrl;
  
  icm20948_set_user_bank(0);

  HAL_I2C_Mem_Read(&hi2c1, ICM_20948_I2C_ADDR, AGB0_REG_USER_CTRL,  I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl, sizeof(ICM_20948_USER_CTRL_t), 1000);


  if (enable)
    ctrl.DMP_EN = 1;
  else
    ctrl.DMP_EN = 0;

  HAL_I2C_Mem_Write(&hi2c1, ICM_20948_I2C_ADDR, AGB0_REG_USER_CTRL, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&ctrl, sizeof(ICM_20948_USER_CTRL_t), 1000);
  
  return 0;
}

int icm20948_set_full_scale(ICM_20948_InternalSensorID_bm sensors, ICM_20948_fss_t fss)
{
  if (!(sensors & (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr)))
  {
    return -1;
  }

  if (sensors & ICM_20948_Internal_Acc)
  {
    ICM_20948_ACCEL_CONFIG_t reg;
    icm20948_set_user_bank(2); // Must be in the right bank
    HAL_I2C_Mem_Read(&hi2c1, ICM_20948_I2C_ADDR, AGB2_REG_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t), 1000);
    reg.ACCEL_FS_SEL = fss.a;
    HAL_I2C_Mem_Write(&hi2c1, ICM_20948_I2C_ADDR,  AGB2_REG_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t), 1000);
    // Check the data was written correctly
    HAL_I2C_Mem_Read(&hi2c1, ICM_20948_I2C_ADDR, AGB2_REG_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t), 1000);
    
  }
  if (sensors & ICM_20948_Internal_Gyr)
  {
    ICM_20948_GYRO_CONFIG_1_t reg;
    icm20948_set_user_bank(2); // Must be in the right bank
    HAL_I2C_Mem_Read(&hi2c1, ICM_20948_I2C_ADDR,AGB2_REG_GYRO_CONFIG_1, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t), 1000);
    reg.GYRO_FS_SEL = fss.g;
    HAL_I2C_Mem_Write(&hi2c1, ICM_20948_I2C_ADDR,  AGB2_REG_GYRO_CONFIG_1, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t), 1000);
    // Check the data was written correctly
    HAL_I2C_Mem_Read(&hi2c1, ICM_20948_I2C_ADDR, AGB2_REG_GYRO_CONFIG_1, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t), 1000);
  }
  return 0;
}

int icm20948_enable_dlpf(ICM_20948_InternalSensorID_bm sensors, bool enable)
{
  if (!(sensors & (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr)))
  {
    return -1;
  }

  if (sensors & ICM_20948_Internal_Acc)
  {
    ICM_20948_ACCEL_CONFIG_t reg;
    icm20948_set_user_bank(2); // Must be in the right bank
    ICM_20948_execute_r(pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t));
    if (enable)
    {
      reg.ACCEL_FCHOICE = 1;
    }
    else
    {
      reg.ACCEL_FCHOICE = 0;
    }
    HAL_I2C_Mem_Write(&hi2c1, ICM_20948_I2C_ADDR,  AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t));
    // Check the data was written correctly
    ICM_20948_execute_r(pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t *)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t));

  }
  if (sensors & ICM_20948_Internal_Gyr)
  {
    ICM_20948_GYRO_CONFIG_1_t reg;
    icm20948_set_user_bank(2); // Must be in the right bank
    ICM_20948_execute_r(pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t));
    if (enable)
    {
      reg.GYRO_FCHOICE = 1;
    }
    else
    {
      reg.GYRO_FCHOICE = 0;
    }
    HAL_I2C_Mem_Write(&hi2c1, ICM_20948_I2C_ADDR,  AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t));
    // Check the data was written correctly
    ICM_20948_execute_r(pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t));
  }
  return 0;
}

int icm20948_set_sample_rate(ICM_20948_InternalSensorID_bm sensors, ICM_20948_smplrt_t smplrt)
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
    HAL_I2C_Mem_Write(&hi2c1, ICM_20948_I2C_ADDR,  AGB2_REG_ACCEL_SMPLRT_DIV_1, &div1, 1);
    HAL_I2C_Mem_Write(&hi2c1, ICM_20948_I2C_ADDR,  AGB2_REG_ACCEL_SMPLRT_DIV_2, &div2, 1);
  }
  if (sensors & ICM_20948_Internal_Gyr)
  {
    icm20948_set_user_bank(2); // Must be in the right bank
    uint8_t div = (smplrt.g);
    HAL_I2C_Mem_Write(&hi2c1, ICM_20948_I2C_ADDR,  AGB2_REG_GYRO_SMPLRT_DIV, &div, 1);
  }
  return 0;
}
