#include <stdint.h>
#include <stdbool.h>

#include "icm_20948_inv.h"

const uint8_t dmp3_image[] = {
#include "util/icm20948_img.dmp3a.h"
};


//icm_20948_load_firmware(dmp3_image, sizeof(dmp3_image), DMP_LOAD_START);

int icm20948_load_firmware(const unsigned char *data_start, unsigned short size_start, unsigned short load_addr)
{
  int write_size;
  int 0; // OK
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

// Combine all of the DMP start-up code from the earlier DMP examples
// This function is defined as __attribute__((weak)) so you can overwrite it if you want to,
//   e.g. to modify the sample rate
int initializeDMP(void)
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
  HAL_I2C_Mem_Write(&hi2c1, ICM_20948_I2C_ADDR, AGB3_REG_I2C_MST_ODR_CONFIG, I2C_MEMADD_SIZE_8BIT,  &mstODRconfig, 1, 1000);

  // Configure clock source through PWR_MGMT_1
  // ICM_20948_Clock_Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
  icm20948_set_clock_source(ICM_20948_Clock_Auto);

  // Enable accel and gyro sensors through PWR_MGMT_2
  // Enable Accelerometer (all axes) and Gyroscope (all axes) by writing zero to PWR_MGMT_2
  icm20948_set_user_bank(0); // Select Bank 0
  uint8_t pwrMgmt2 = 0x40;                                                          // Set the reserved bit 6 (pressure sensor disable?)
  HAL_I2C_Mem_Write(&hi2c1, ICM_20948_I2C_ADDR, AGB0_REG_PWR_MGMT_2, I2C_MEMADD_SIZE_8BIT, &pwrMgmt2, 1, 1000); 

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
  HAL_I2C_Mem_Write(&hi2c1, ICM_20948_I2C_ADDR, AGB0_REG_FIFO_EN_1, I2C_MEMADD_SIZE_8BIT, &zero, 1, 1000); 
  // Stop the accelerometer, gyro and temperature data from being written to the FIFO by writing zero to FIFO_EN_2
  HAL_I2C_Mem_Write(&hi2c1, ICM_20948_I2C_ADDR, AGB0_REG_FIFO_EN_2, I2C_MEMADD_SIZE_8BIT, &zero, 1, 1000);

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
  HAL_I2C_Mem_Write(&hi2c1, ICM_20948_I2C_ADDR, AGB0_REG_HW_FIX_DISABLE, I2C_MEMADD_SIZE_8BIT, &fix, 1, 1000); 

  // Set the Single FIFO Priority Select register to 0xE4
  icm20948_set_user_bank(0);
  uint8_t fifoPrio = 0xE4;
  HAL_I2C_Mem_Write(&hi2c1, ICM_20948_I2C_ADDR, AGB0_REG_SINGLE_FIFO_PRIORITY_SEL, I2C_MEMADD_SIZE_8BIT, &fifoPrio, 1, 1000); 

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
