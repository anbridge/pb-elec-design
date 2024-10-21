/*

This is a C-compatible interface to the features presented by the ICM 20948 9-axis device
The imementation of the interface is flexible

*/

#ifndef _ICM_20948_INV_H_
#define _ICM_20948_INV_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

#include "stm32wbxx_hal.h"

#include "ICM_20948_REGISTERS.h"
#include "ICM_20948_ENUMERATIONS.h" // This is to give users access to usable value definiitons
#include "AK09916_ENUMERATIONS.h"
#include "AK09916_REGISTERS.h"
#include "ICM_20948_DMP.h"


extern int memcmp(const void *, const void *, size_t); // Avoid compiler warnings

// Define if the DMP will be supported
// Note: you must have 14290/14301 Bytes of program memory available to store the DMP firmware!
#define ICM_20948_USE_DMP // Uncomment this line to enable DMP support. You can of course use ICM_20948_USE_DMP as a compiler flag too
#define ICM_20948_I2C_ADDR 0xD0 // 0x68 SHIFTED!!!
#define DMP_START_ADDRESS ((unsigned short)0x1000) //default program start address

// There are two versions of the InvenSense DMP firmware for the ICM20948 - with slightly different sizes
#define DMP_CODE_SIZE 14301 /* eMD-SmartMotion-ICM20948-1.1.0-MP */
//#define DMP_CODE_SIZE 14290 /* ICM20948_eMD_nucleo_1.0 */

//#define ICM_20948_I2C_ADDR_AD0 0x68 // Or 0x69 when AD0 is high
//#define ICM_20948_I2C_ADDR_AD1 0x69 //
#define ICM_20948_WHOAMI 0xEA

#define MAG_AK09916_I2C_ADDR 0x0C
#define MAG_AK09916_WHO_AM_I 0x4809
#define MAG_REG_WHO_AM_I 0x00

#define MAX_SERIAL_R_W 16 //16 bytes max serial read/write for ICM-20948



  typedef enum
  {
    ICM_20948_Stat_Ok = 0x00, // The only return code that means all is well
    ICM_20948_Stat_Err,       // A general error
    ICM_20948_Stat_NotImpl,   // Returned by virtual functions that are not implemented
    ICM_20948_Stat_ParamErr,
    ICM_20948_Stat_WrongID,
    ICM_20948_Stat_InvalSensor, // Tried to apply a function to a sensor that does not support it (e.g. DLPF to the temperature sensor)
    ICM_20948_Stat_NoData,
    ICM_20948_Stat_SensorNotSupported,
    ICM_20948_Stat_DMPNotSupported,    // DMP not supported (no #define ICM_20948_USE_DMP)
    ICM_20948_Stat_DMPVerifyFail,      // DMP was written but did not verify correctly
    ICM_20948_Stat_FIFONoDataAvail,    // FIFO contains no data
    ICM_20948_Stat_FIFOIncompleteData, // FIFO contained incomplete data
    ICM_20948_Stat_FIFOMoreDataAvail,  // FIFO contains more data
    ICM_20948_Stat_UnrecognisedDMPHeader,
    ICM_20948_Stat_UnrecognisedDMPHeader2,
    ICM_20948_Stat_InvalDMPRegister, // Invalid DMP Register

    ICM_20948_Stat_NUM,
    ICM_20948_Stat_Unknown,
  } ICM_20948_Status_e;

  typedef enum
  {
    ICM_20948_Internal_Acc = (1 << 0),
    ICM_20948_Internal_Gyr = (1 << 1),
    ICM_20948_Internal_Mag = (1 << 2),
    ICM_20948_Internal_Tmp = (1 << 3),
    ICM_20948_Internal_Mst = (1 << 4), // I2C Master Ineternal
  } ICM_20948_InternalSensorID_bm;     // A bitmask of internal sensor IDs

  typedef union
  {
    int16_t i16bit[3];
    uint8_t u8bit[6];
  } ICM_20948_axis3bit16_t;

  typedef union
  {
    int16_t i16bit;
    uint8_t u8bit[2];
  } ICM_20948_axis1bit16_t;

  typedef struct
  {
    uint8_t a : 2;
    uint8_t g : 2;
    uint8_t reserved_0 : 4;
  } ICM_20948_fss_t; // Holds full-scale settings to be able to extract measurements with units

  typedef struct
  {
    uint8_t a;
    uint8_t g;
  } ICM_20948_dlpcfg_t; // Holds digital low pass filter settings. Members are type ICM_20948_ACCEL_CONFIG_DLPCFG_e

  typedef struct
  {
    uint16_t a;
    uint8_t g;
  } ICM_20948_smplrt_t;

  typedef struct
  {
    uint8_t I2C_MST_INT_EN : 1;
    uint8_t DMP_INT1_EN : 1;
    uint8_t PLL_RDY_EN : 1;
    uint8_t WOM_INT_EN : 1;
    uint8_t REG_WOF_EN : 1;
    uint8_t RAW_DATA_0_RDY_EN : 1;
    uint8_t FIFO_OVERFLOW_EN_4 : 1;
    uint8_t FIFO_OVERFLOW_EN_3 : 1;
    uint8_t FIFO_OVERFLOW_EN_2 : 1;
    uint8_t FIFO_OVERFLOW_EN_1 : 1;
    uint8_t FIFO_OVERFLOW_EN_0 : 1;
    uint8_t FIFO_WM_EN_4 : 1;
    uint8_t FIFO_WM_EN_3 : 1;
    uint8_t FIFO_WM_EN_2 : 1;
    uint8_t FIFO_WM_EN_1 : 1;
    uint8_t FIFO_WM_EN_0 : 1;
  } ICM_20948_INT_enable_t;

  typedef union
  {
    ICM_20948_axis3bit16_t raw;
    struct
    {
      int16_t x;
      int16_t y;
      int16_t z;
    } axes;
  } ICM_20948_axis3named_t;

  typedef struct
  {
    ICM_20948_axis3named_t acc;
    ICM_20948_axis3named_t gyr;
    ICM_20948_axis3named_t mag;
    union
    {
      ICM_20948_axis1bit16_t raw;
      int16_t val;
    } tmp;
    ICM_20948_fss_t fss; // Full-scale range settings for this measurement
    uint8_t magStat1;
    uint8_t magStat2;
  } ICM_20948_AGMT_t;

  typedef struct
  {
    ICM_20948_Status_e (*write)(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user);
    ICM_20948_Status_e (*read)(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user);
    // void				(*delay)(uint32_t ms);
    void *user;
  } ICM_20948_Serif_t;                      // This is the vtable of serial interface functions
  extern const ICM_20948_Serif_t NullSerif; // Here is a default for initialization (NULL)

  typedef struct
  {
    const ICM_20948_Serif_t *_serif; // Pointer to the assigned Serif (Serial Interface) vtable
    bool _dmp_firmware_available;    // Indicates if the DMP firmware has been included. It
    bool _firmware_loaded;           // Indicates if DMP has been loaded
    uint8_t _last_bank;              // Keep track of which bank was selected last - to avoid unnecessary writes
    uint8_t _last_mems_bank;         // Keep track of which bank was selected last - to avoid unnecessary writes
    int32_t _gyroSF;                 // Use this to record the GyroSF, calculated by inv_icm20948_set_gyro_sf
    int8_t _gyroSFpll;
    uint32_t _enabled_Android_0;      // Keep track of which Android sensors are enabled: 0-31
    uint32_t _enabled_Android_1;      // Keep track of which Android sensors are enabled: 32-
    uint32_t _enabled_Android_intr_0; // Keep track of which Android sensor interrupts are enabled: 0-31
    uint32_t _enabled_Android_intr_1; // Keep track of which Android sensor interrupts are enabled: 32-
    uint16_t _dataOutCtl1;            // Diagnostics: record the setting of DATA_OUT_CTL1
    uint16_t _dataOutCtl2;            // Diagnostics: record the setting of DATA_OUT_CTL2
    uint16_t _dataRdyStatus;          // Diagnostics: record the setting of DATA_RDY_STATUS
    uint16_t _motionEventCtl;         // Diagnostics: record the setting of MOTION_EVENT_CTL
    uint16_t _dataIntrCtl;            // Diagnostics: record the setting of DATA_INTR_CTL
  } ICM_20948_Device_t;               // Definition of device struct type

extern void icm20948_set_i2c_bus(I2C_HandleTypeDef *bus);
extern void icm20948_set_user_bank(uint8_t bank);
extern int8_t sensor_type_2_android_sensor(enum inv_icm20948_sensor sensor);
extern int8_t icm20948_i2c_controller_configure_peripheral(uint8_t peripheral, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap, uint8_t dataOut);
extern int8_t icm20948_set_clock_source(ICM_20948_PWR_MGMT_1_CLKSEL_e source);
extern int8_t icm20948_set_sample_mode(ICM_20948_InternalSensorID_bm sensors, ICM_20948_LP_CONFIG_CYCLE_e mode);
extern int8_t icm20948_enable_FIFO(bool enable);
extern int8_t icm20948_reset_FIFO();
extern int8_t icm20948_enable_DMP(bool enable);
extern int8_t icm20948_reset_DMP();
extern int8_t icm20948_set_full_scale(ICM_20948_InternalSensorID_bm sensors, ICM_20948_fss_t fss);
extern int8_t icm20948_enable_dlpf(ICM_20948_InternalSensorID_bm sensors, bool enable);
extern int8_t icm20948_set_sample_rate(ICM_20948_InternalSensorID_bm sensors, ICM_20948_smplrt_t smplrt);
extern int8_t icm20948_int_enable(ICM_20948_INT_enable_t *write, ICM_20948_INT_enable_t *read);
extern int8_t icm20948_int_enable_raw_data_ready(bool enable);
extern int8_t icm20948_set_DMP_start_address(unsigned short address);
extern int8_t icm20948_write_mem(unsigned short reg, unsigned int length, const unsigned char *data);
extern int8_t icm20948_read_mem(unsigned short reg, unsigned int length, unsigned char *data);
extern int8_t icm20948_set_gyro_sf(unsigned char div, int gyro_level);
extern int8_t icm20948_enable_DMP_sensor(enum inv_icm20948_sensor sensor, int state);
extern int8_t icm20948_set_DMP_sensor_period(enum DMP_ODR_Registers odr_reg, uint16_t interval);
extern int8_t icm20948_get_FIFO_count(uint16_t *count);
extern int8_t icm20948_read_FIFO(uint8_t *data, uint8_t len);
extern int8_t icm20948_read_DMP_data(icm_20948_DMP_data_t *data);
extern int8_t icm20948_load_firmware(const unsigned char *data_start, unsigned short size_start, unsigned short load_addr);
extern int8_t icm20948_initialize_DMP(void);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _ICM_20948_INV_H_ */
