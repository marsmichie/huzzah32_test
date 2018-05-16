/* register addresses */
#define BMM150_CHIP_ID_ADDR          (0x40)
// register address for measurement data
// low refers to [7:3] bits, high to [15:8] bits of 12 bit integer for x/y
#define BMM150_DATA_X_LSB            (0x42)
#define BMM150_DATA_READY_STATUS     (0x48)
#define BMM150_INTERRUPT_STATUS      (0x4A)
#define BMM150_POWER_CONTROL_ADDR    (0x4B)
#define BMM150_OP_MODE_ADDR          (0x4C)
#define BMM150_INT_CONFIG_ADDR       (0x4D)
#define BMM150_AXES_ENABLE_ADDR      (0x4E)
#define BMM150_LOW_THRESHOLD_ADDR    (0x4F)
#define BMM150_HIGH_THRESHOLD_ADDR   (0x50)
#define BMM150_REP_XY_ADDR           (0x51)
#define BMM150_REP_Z_ADDR            (0x52)

/* Trim Extended Registers */
#define BMM150_DIG_X1               UINT8_C(0x5D)
#define BMM150_DIG_Y1               UINT8_C(0x5E)
#define BMM150_DIG_Z4_LSB           UINT8_C(0x62)
#define BMM150_DIG_Z4_MSB           UINT8_C(0x63)
#define BMM150_DIG_X2               UINT8_C(0x64)
#define BMM150_DIG_Y2               UINT8_C(0x65)
#define BMM150_DIG_Z2_LSB           UINT8_C(0x68)
#define BMM150_DIG_Z2_MSB           UINT8_C(0x69)
#define BMM150_DIG_Z1_LSB           UINT8_C(0x6A)
#define BMM150_DIG_Z1_MSB           UINT8_C(0x6B)
#define BMM150_DIG_XYZ1_LSB         UINT8_C(0x6C)
#define BMM150_DIG_XYZ1_MSB         UINT8_C(0x6D)
#define BMM150_DIG_Z3_LSB           UINT8_C(0x6E)
#define BMM150_DIG_Z3_MSB           UINT8_C(0x6F)
#define BMM150_DIG_XY2              UINT8_C(0x70)
#define BMM150_DIG_XY1              UINT8_C(0x71)

/* overflow definitions */
#define BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL  (-4096)
#define BMM150_ZAXIS_HALL_OVERFLOW_ADCVAL   (-16384)
#define BMM150_OVERFLOW_OUTPUT              (-32768)
#define BMM150_NEGATIVE_SATURATION_Z        (-32767)
#define BMM150_POSITIVE_SATURATION_Z        (32767)

// for two's complement
#define INT_13_BIT                     (0x1FFF)
#define INT_14_BIT                     (0x3FFF)
#define INT_15_BIT                     (0x7FFF)
#define INT_16_BIT                     (0xFFFF)

// number of sensors
#define SENSORS                        (64)

//measurement modesfor CNTL2; wait min. 100us before mode setting
const uint8_t powerBit       = 0b00000001;
const uint8_t sleepMode      = 0b00000110;
const uint8_t testMode       = 0b00000001;
const uint8_t forceMode      = 0b00000010;
const uint8_t continous10Hz  = 0b00000000;
const uint8_t continous30Hz  = 0b00111000;

const uint8_t dataReadyEn    = 0b10000100;

// number of averaged measured values for x/y
const uint8_t xy_three       = 0b00000001;
const uint8_t xy_five        = 0b00000010;
const uint8_t xy_seventeen   = 0b00001000;
// number of averaged measured values for z
const uint8_t z_six          = 0b00000101;
const uint8_t z_nine         = 0b00001000;
const uint8_t z_twenty       = 0b00010011;

