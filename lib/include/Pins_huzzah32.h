/*
 * Pin assigments for ESP32 dev board
 */
#define DFF_DATA            GPIO_NUM_33 // data input of first DFF
#define DFF_CLR_A           GPIO_NUM_15
#define DFF_PRE             GPIO_NUM_32
#define DFF_CLK             GPIO_NUM_21
#define DFF_LAST            GPIO_NUM_14 // data output of last DFF

//#define ALL_DATA_RDY        GPIO_NUM_22
//#define D_TRIG              6 // Trigger -> not needed
//#define LVL_SHFT_OE         5
//#define LVL_SHFT_OE_ENABLED (digitalWrite(LVL_SHFT_OE, HIGH))

/* default SPI */
//#define MOSI              18
//#define MISO              19
//#define SCK               5

#define HIGH                1
#define LOW                 0
