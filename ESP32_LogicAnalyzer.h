#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "config.h"
#include "rom/lldesc.h"
#include "soc/i2s_struct.h"
#include "soc/i2s_reg.h"
#include "driver/periph_ctrl.h"
#include "soc/io_mux_reg.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include <HardwareSerial.h>
#include "driver/ledc.h"
#include "Vrekrer_scpi_parser.h"

static const char* TAG = "ESP32_LogicAnalyzer";
static bool setTestOutput( int, int);

void startCapture(void);

// SUMP
// ====
void getSumpData(void);
void sendSumpMetadata(void);

// SCPI
// ====
#define SCPI_HASH_TYPE uint8_t //Default value = uint8_t
SCPI_Parser lascpi;

// Configuration
// =============
#define BOARD_esp32dev 1
//#define BOARD_esp32s2 1
#ifdef BOARD_esp32s2
      #define LOOPNEZS2 0                         // ESP32S2 loopnez problem
#endif

#undef DEBUG_OPERATION_TIMES              // If defined, print times for DMA and RLE operations

#define USE_SERIAL2_FOR_OLS   1           // If 1, UART2 = OLS and UART0=Debug; else, UART0 = OLS and UART2=Debug
                                          // XON/XOFF are not supported
#define RXD2 16
#define TXD2 17
#if BOARD_esp32s2
      HardwareSerial Serial2(2);
      #define Serial Serial0
#endif                                                                          

// If 1, UART2 = OLS and UART0=Debug
// else, UART0 = OLS and UART2=Debug
#if USE_SERIAL2_FOR_OLS
      #define Serial_Debug_Port Serial
      #define Serial_Debug_Port_Baud 115200
      //#define Serial_Debug_Port_Baud 921600
      #define OLS_Port Serial2
      #define OLS_Port_Baud 115200
      //#define OLS_Port_Baud 3000000
#else
      #define Serial_Debug_Port Serial2
      #define Serial_Debug_Port_Baud 115200
      #define OLS_Port Serial
      #define OLS_Port_Baud 115200
#endif

 // ALLOW_ZERO_RLE 1 is Fast mode.
 //  Add RLE Count 0 to RLE stack for non repeated values and postpone the RLE processing so faster
 //   8Bit Mode : ~28.4k clock per 4k block, captures 3000us while inspecting ~10Mhz clock at 20Mhz mode
 //  16Bit Mode : ~22.3k clock per 4k block, captures 1500us while inspecting ~10Mhz clock at 20Mhz mode
 
 // ALLOW_ZERO_RLE 0 is Slow mode.
 //  just RAW RLE buffer. It doesn't add 0 count values for non-repeated RLE values and process flags on the fly, so little slow but efficient.
 //   8Bit Mode : ~34.7k clock per 4k block, captures 4700us while inspecting ~10Mhz clock at 20Mhz mode
 //  16Bit Mode : ~30.3k clock per 4k block, captures 2400us while inspecting ~10Mhz clock at 20Mhz mode
#define ALLOW_ZERO_RLE        0           // Enable "Fast mode" RLE on data
#define CAPTURE_SIZE          128000
#define DMA_MAX               (4096-4)

#ifdef DEBUG_OPERATION_TIMES
      #define RLE_SIZE        92000
#else
      #define RLE_SIZE        96000
#endif

#define CHANPIN GPIO.in

// Map of digital input channel virtual pins to physical input pins
// ================================================================
// PIN Definitions for DEVKITC_V4
#ifdef BOARD_esp32dev
#define LED_PIN               2           //Led on while running and Blinks while transfering data.
#define CLK_PIN               0

#define D01_PIN               13    // GPIO
#define D02_PIN               12    // GPIO
#define D03_PIN               14    // GPIO
#define D04_PIN               27    // GPIO
#define D05_PIN               26    // GPIO
#define D06_PIN               25    // GPIO
#define D07_PIN               33    // GPIO
#define D08_PIN               32    // GPIO
#define D09_PIN                5    // SS
#define D10_PIN               18    // CLK
#define D11_PIN               23    // MOSI
#define D12_PIN               19    // MISO
#define D13_PIN               22    // SCL1
#define D14_PIN               21    // SDA1
#define D15_PIN                4    // RTC-10
#define D16_PIN               15    // RTC-13

#define A01_PIN               35    // ADC1-7
#define A02_PIN               34    // ADC1-6
#define A03_PIN               39    // ADC1-3
#define A04_PIN               36    // ADC1-0
#endif

#ifdef DEBUG_OPERATION_TIMES
unsigned int time_debug_indice_dma[1024];
unsigned int time_debug_indice_dma_p=0;

unsigned int time_debug_indice_rle[1024];
unsigned int time_debug_indice_rle_p=0;
#endif

int               chan_num          = 0;
int               sumpCmnd          = 0;
byte              sumpLongCmnd[5];

unsigned int      logicIndex        = 0;
unsigned int      triggerIndex      = 0;
uint32_t          readCount         = CAPTURE_SIZE;
unsigned int      delayCount        = 0;
uint16_t          trigger           = 0;
uint16_t          trigger_values    = 0;
unsigned int      useMicro          = 0;
unsigned int      delayTime         = 0;
unsigned long     divider           = 0;
bool              rleEnabled        = 0;
uint32_t          clock_per_read    = 0;

int i;
int z;

uint8_t channels_to_read=3;

// SUMP Protocol Definitions
// =========================

// Short commands
// --------------
/*
      These commands are exactly one byte long.
*/
#define SUMP_RESET                  0x00  // Resets the device. Send 5 times when the receiver status is unknown.
#define SUMP_ARM                    0x01  // Arms the trigger
#define SUMP_QUERY                  0x02  // Asks for device identification ("SLA1")


/* extended commands -- self-test unsupported, but metadata is returned. */
#define SUMP_SELF_TEST              0x03
#define SUMP_SEND_METADATA          0x04  // See sendSumpMetadata() function header
#define SUMP_FINISH_NOW             0x05  // Finish Now (disable RLE mode)
#define SUMP_QUERY_INPUTS           0x06  // The device responds with four bytes. Gives a snapshot of the current logic analyzer input bits
#define SUMP_ARM_ADVANCED_TRIGGER   0x0F  // Arm the advanced trigger. Conditional capturing (under control of trigger sequencer) begins immediately.
                                          //  Once trigger fires, the controller waits for "delay count" additional samples before returning captured data to client.

#define SUMP_XON                    0x11  // Deprecated
#define SUMP_XOFF                   0x13  // Deprecated

// Long commands
// -------------

/*
      These commands are five bytes long.
      The first byte contains the opcode.
      The bytes are sent LSB first.

      Serial Mode: In serial mode, a trigger stage monitors only a single sampled input bit, selected with the "serial-channel".
                        The selected bit is fed into a serial-to-parallel shift register.
                        Thus the last 32 samples of a single bit can be evaluated using the normal masked compare logic.
                        To be effective, you should use an external clock in serial mode.
*/
/* mask & values used, config ignored. only stage0 supported */
#define SUMP_TRIGGER_MASK_CH_A      0xC0  // Defines which trigger values must match. The opcodes refer to stage 0-3 (A, B, C, D)
#define SUMP_TRIGGER_MASK_CH_B      0xC4  //  In parallel mode each bit represents one channel,
#define SUMP_TRIGGER_MASK_CH_C      0xC8  //  in serial mode each bit represents one of the last 32 samples of the selected channel
#define SUMP_TRIGGER_MASK_CH_D      0xCC  //  A mask bit set to zero causes the corresponding trigger-value bit to be ignored

#define SUMP_TRIGGER_VALUES_CH_A    0xC1  // Defines which values individual bits must have. The opcodes refer to stage 0-3 (A, B, C, D)
#define SUMP_TRIGGER_VALUES_CH_B    0xC5  //  In parallel mode each bit represents one channel,
#define SUMP_TRIGGER_VALUES_CH_C    0xC9  //  in serial mode each bit represents one of the last 32 samples of the selected channel
#define SUMP_TRIGGER_VALUES_CH_D    0xCD

#define SUMP_TRIGGER_CONFIG_CH_A    0xC2  // Configures the selected trigger stage, The opcodes refer to stage 0-3 (A, B, C, D)
#define SUMP_TRIGGER_CONFIG_CH_B    0xC6  //  delay  (b15-00)  If a match occurs, the action of the stage is delayed by the given number of samples
#define SUMP_TRIGGER_CONFIG_CH_C    0xCA  //  level  (b17-16) Trigger level at which the stage becomes active
#define SUMP_TRIGGER_CONFIG_CH_D    0xCE  //  channel(b24,b23-20) Channel to be used in serial mode (Normal:0,31; Demux:0,15)
                                          //  serial (b26)
                                          //  start  (b27)

/* Most flags (except RLE) are ignored. */
#define SUMP_SET_DIVIDER            0x80  // When x is written, the sampling frequency is: sampleRate = clock / (x + 1)
                                          // x = sumpLongCmnd[1]:sumpLongCmnd[0]
#define SUMP_SET_READ_DELAY_COUNT   0x81  // (b15-b00) Read Count is the number of samples (divided by four) to read back from memory and sent to the host computer
                                          // (b31-b16) Delay Count is the number of samples (divided by four) to capture after the trigger fired
                                          // A Read Count bigger than the Delay Count means that data from before the trigger match will be read back
#define SUMP_SET_FLAGS              0x82  // demux          (b0)    Enables the demux input module. (Filter must be off.)
                                          // filter         (b1)    Enables the filter input module. (Demux must be off.)
                                          // channel groups (b5-b2) Disable channel group. Disabled groups are excluded from data transmissions.
                                          //  There are four groups, each represented by one bit.
                                          //  Starting with the least significant bit of the channel group field channels are assigned as follows: 0-7, 8-15, 16-23, 24-31
                                          // external       (b6)    Selects the clock to be used for sampling. Use external clock for sampling.
                                          //  If set to 0, the internal 100Mhz reference clock divided by the configured divider is used.
                                          //  (filter and demux are only available with internal clock)
                                          // inverted       (b7)    When set to 1, the external clock will be inverted before being used.
                                          //  The inversion causes a delay that may cause problems at very high clock rates.
                                          //  This option only has an effect with external set to 1.
                                          // EnableRLE      (b8)    For input samples which change infrequently, <value,rle-count> sent as pairs after max rle-count
                                          // SwapByteOrder  (b9)    Swap number schemes (swap upper/lower 16bits)
                                          // ExtTestMode    (b10)   External Test Mode: Output pattern on bits 31:16
                                          // IntTestMode    (b11)   Internal Test Mode
                                          // ExtraRLE       (b15-14)
                                          //                 00, 01 Issue <value> & <rle-count> as pairs
                                          //                 02     <values> reissued approximately every 256 <rle-count> fields
                                          //                 03     <values> can be followed by unlimited numbers of <rle-counts>


#define SUMP_SET_RLE                0x0100

#define MAX_CAPTURE_SIZE            CAPTURE_SIZE

int8_t rle_process=-1;
uint8_t rle_buff[RLE_SIZE];
uint8_t* rle_buff_p;
uint8_t* rle_buff_end;
uint8_t rle_sample_counter;
uint32_t rle_total_sample_counter;
uint8_t rle_value_holder;

bool rle_init(void){
  rle_buff_p=0;
  rle_sample_counter=0;
  rle_total_sample_counter=0;
  rle_value_holder=0;
  rle_process=-1;
  rle_buff_p=rle_buff;
  rle_buff_end = rle_buff+RLE_SIZE-4;
  memset( rle_buff, 0x00, RLE_SIZE);
  return true;
}


/* We have to encode RLE samples quickly.
 * Each sample needs to be encoded in under 12 clocks @240Mhz CPU 
 * for 20Mhz capture sampling speed.
 */

/* expected structure of DMA memory    : 00s1,00s2,00s3,00s4
 * actual data structure of DMA memory : 00s2,00s1,00s4,00s3
 */

void fast_rle_block_encode_asm_8bit_ch1(uint8_t *dma_buffer, int sample_size){ //size, not count
   uint8_t *desc_buff_end=dma_buffer;
   unsigned clocka=0;
   unsigned clockb=0;

   int dword_count=(sample_size/4) -1;   
   clocka = xthal_get_ccount();

    __asm__ __volatile__(
      "memw \n"
      "l32i a4, %0, 0        \n" // Load store dma_buffer address
      "l32i a6, %2, 0        \n" // Load store rle_buffer address
      "l8ui a8, a4, 2        \n" // a8 as rle_val (#2 is first)
      "l8ui a9, a4, 0        \n" // a9 as new_val
      "movi a7, 0xFF         \n" // store max RLE sample
      "movi a5, 0            \n" // init rle_counter
      "movi a10, 0x80        \n" // init rle_masks
      "movi a11, 0x7F        \n" // init rle_masks
"beq  a9, a8, rle_0          \n" // rle_val == new_val skip

#if not ALLOW_ZERO_RLE
      "and  a8, a8, a11      \n" // a11=0x7F
#endif
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
#if ALLOW_ZERO_RLE
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = rle_counter //not needed
      "addi a6, a6, 1        \n" // rle_buff_p ++              //not needed
#endif
      "movi a5, -1           \n" // rle_counter=-1
      "addi a6, a6, 1        \n" // rle_buff_p ++ 
      "mov  a8, a9           \n" // rle_val = new_val
            
      "rle_0:                \n"
      "addi a5, a5, 1        \n" // rle_counter++
#if not LOOPNEZS2
"loopnez %1, rle_loop_end    \n" // Prepare zero-overhead loop
#endif
      "loopStart:            \n"
      "addi a4, a4, 4        \n" // increase dma_buffer_p pointer by 4

      "rle_1_start:          \n"
      "l8ui a9, a4, 2        \n" // a8 as rle_val (#2 is first)
"beq  a9, a8, rle_1_end      \n" // rle_val == new_val skip

"bltui a5, 128, rle_1_add    \n" // if count >= 128 branch
      "rle_1_127:            \n"
#if not ALLOW_ZERO_RLE
      "and  a8, a8, a11      \n" // a11=0x7F
#endif
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val;
      "movi a7, 0xFF         \n"
      "s8i  a7, a6, 1        \n" // rle_buff_p+1 = 127 as count
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
      "addi a5, a5, -128     \n" // count=-127
"bgeui a5, 128, rle_1_127    \n" // if count >= 128 branch

      "rle_1_add:            \n" 
#if ALLOW_ZERO_RLE        // 4000 to 3140 bytes & 27219 clocks
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
#else                        // 4000 to 1988 bytes & 34348 clocks
      "and  a8, a8, a11      \n" // a11=0x7F
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val & 0x7F;
"beqz a5, rle_1_skip         \n" // if count == 0 , skip
      "or   a5, a5, a10      \n" // a10=0x80
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count | 0x80;
      "addi a6, a6, 1        \n" // rle_buff_p ++
      "rle_1_skip:           \n" 
      "addi a6, a6, 1        \n" // rle_buff_p ++
#endif
      "mov  a8, a9           \n" // rle_val = new_val
      "movi a5, -1           \n" // rle_counter=-1  , will be 0 at next instruction. 

      "rle_1_end:            \n"
      "addi a5, a5, 1        \n" // rle_counter++

      "rle_2_start:          \n"
      "l8ui a9, a4, 0        \n" // a9 as rle_val (#0 is second)
"beq  a9, a8, rle_2_end      \n" // rle_val == new_val continue

"bltui a5, 128, rle_2_add    \n" // if count >= 128 branch
      "rle_2_127:            \n"
#if not ALLOW_ZERO_RLE
      "and  a8, a8, a11      \n" // a11=0x7F
#endif
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val;
      "movi a7, 0xFF         \n"
      "s8i  a7, a6, 1        \n" // rle_buff_p+1 = 127 as count
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
      "addi a5, a5, -128     \n" // count=-127
"bgeui a5, 128, rle_2_127    \n" // if count >= 128 branch

      "rle_2_add:            \n" 
#if ALLOW_ZERO_RLE 
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
#else
      "and  a8, a8, a11      \n" // a11=0x7F
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val & 0x7F;
"beqz a5, rle_2_skip         \n"
      "or   a5, a5, a10      \n" // a10=0x80
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count | 0x80;
      "addi a6, a6, 1        \n" // rle_buff_p ++
      "rle_2_skip:           \n"
      "addi a6, a6, 1        \n" // rle_buff_p ++ 
#endif
      "mov  a8, a9           \n" // rle_val = new_val
      "movi a5, -1           \n" // rle_counter=-1
      "rle_2_end:            \n"
      "addi a5, a5, 1        \n" // rle_counter++

      "rle_loop_end:              \n"

"bltui a5, 128, rle_end_add  \n" // if count >= 128 branch
      "rle_end_127:          \n"
#if not ALLOW_ZERO_RLE
      "and  a8, a8, a11      \n" // a11=0x7F
#endif
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val;
      "movi a7, 0xFF         \n"
      "s8i  a7, a6, 1        \n" // rle_buff_p+1 = 127 as count
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
      "addi a5, a5, -128     \n" // count=-127
"bgeui a5, 128, rle_end_127  \n" // if count >= 128 branch

      "rle_end_add:          \n" 
#if ALLOW_ZERO_RLE 
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
#else
      "and  a8, a8, a11      \n" // 
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
"beqz a5, rle_end_skip       \n"
      "or   a5, a5, a10      \n" // 
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
      "addi a6, a6, 1        \n" // rle_buff_p ++ 
      "rle_end_skip:         \n"
      "addi a6, a6, 1        \n" // rle_buff_p ++
#endif

      "exit:                 \n"
      "s32i a6, %2, 0        \n" // update rle_buff_p value

      :
      :"a"(&dma_buffer), "a"(dword_count), "a"(&rle_buff_p):
      "a4","a5","a6","a7","a8","a9","a10","a11","memory");

    clockb = xthal_get_ccount();
#ifdef DEBUG_OPERATION_TIMES
    time_debug_indice_rle[time_debug_indice_rle_p++]=clockb;
#endif
 //   Serial_Debug_Port.printf("\r\n asm_process takes %d clocks\r\n",(clockb-clocka));
 //   Serial_Debug_Port.printf( "RX  Buffer = %d bytes\r\n", sample_size );
 //   Serial_Debug_Port.printf( "RLE Buffer = %d bytes\r\n", (rle_buff_p - rle_buff) );
    ESP_LOGD(TAG, "RLE Buffer = %d bytes\r\n", (rle_buff_p - rle_buff) );
}

void fast_rle_block_encode_asm_8bit_ch2(uint8_t *dma_buffer, int sample_size){ //size, not count
   uint8_t *desc_buff_end=dma_buffer;
   unsigned clocka;
   unsigned clockb=0;
    
   int dword_count=(sample_size/4) -1;
   clocka = xthal_get_ccount();

    __asm__ __volatile__(
      "memw \n"
      "l32i a4, %0, 0        \n" // Load store dma_buffer address
      "l32i a6, %2, 0        \n" // Load store rle_buffer address
      "l8ui a8, a4, 3        \n" // a8 as rle_val (#2 is first)
      "l8ui a9, a4, 1        \n" // a9 as new_val
      "movi a7, 0xFF         \n" // store max RLE sample
      "movi a5, 0            \n" // init rle_counter
      "movi a10, 0x80        \n" // init rle_masks
      "movi a11, 0x7F        \n" // init rle_masks
"beq  a9, a8, rle_0_ch2      \n" // rle_val == new_val skip

#if not ALLOW_ZERO_RLE
      "and  a8, a8, a11      \n" // a11=0x7F
#endif
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
#if ALLOW_ZERO_RLE
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = rle_counter //not needed
      "addi a6, a6, 1        \n" // rle_buff_p ++              //not needed
#endif
      "movi a5, -1           \n" // rle_counter=-1
      "addi a6, a6, 1        \n" // rle_buff_p ++ 
      "mov  a8, a9           \n" // rle_val = new_val
            
      "rle_0_ch2:            \n"
      "addi a5, a5, 1        \n" // rle_counter++
#if not LOOPNEZS2
"loopnez %1, rle_loop_end_ch2    \n" // Prepare zero-overhead loop
#endif
      "loopStart_ch2:            \n"
      "addi a4, a4, 4        \n" // increase dma_buffer_p pointer by 4

      "rle_1_start_ch2:          \n"
      "l8ui a9, a4, 3        \n" // a8 as rle_val (#2 is first)
"beq  a9, a8, rle_1_end_ch2      \n" // rle_val == new_val skip

"bltui a5, 128, rle_1_add_ch2    \n" // if count >= 128 branch
      "rle_1_127_ch2:            \n"
#if not ALLOW_ZERO_RLE
      "and  a8, a8, a11      \n" // a11=0x7F
#endif
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val;
      "movi a7, 0xFF         \n"
      "s8i  a7, a6, 1        \n" // rle_buff_p+1 = 127 as count
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
      "addi a5, a5, -128     \n" // count=-127
"bgeui a5, 128, rle_1_127_ch2    \n" // if count >= 128 branch

      "rle_1_add_ch2:            \n" 
#if ALLOW_ZERO_RLE        // 4000 to 3140 bytes & 27219 clocks
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
#else                        // 4000 to 1988 bytes & 34348 clocks
      "and  a8, a8, a11      \n" // a11=0x7F
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val & 0x7F;
"beqz a5, rle_1_skip_ch2         \n" // if count == 0 , skip
      "or   a5, a5, a10      \n" // a10=0x80
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count | 0x80;
      "addi a6, a6, 1        \n" // rle_buff_p ++
      "rle_1_skip_ch2:           \n" 
      "addi a6, a6, 1        \n" // rle_buff_p ++
#endif
      "mov  a8, a9           \n" // rle_val = new_val
      "movi a5, -1           \n" // rle_counter=-1  , will be 0 at next instruction. 

      "rle_1_end_ch2:            \n"
      "addi a5, a5, 1        \n" // rle_counter++

      "rle_2_start_ch2:          \n"
      "l8ui a9, a4, 1        \n" // a9 as rle_val (#0 is second)
"beq  a9, a8, rle_2_end_ch2      \n" // rle_val == new_val continue

"bltui a5, 128, rle_2_add_ch2   \n" // if count >= 128 branch
      "rle_2_127_ch2:            \n"
#if not ALLOW_ZERO_RLE
      "and  a8, a8, a11      \n" // a11=0x7F
#endif
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val;
      "movi a7, 0xFF         \n"
      "s8i  a7, a6, 1        \n" // rle_buff_p+1 = 127 as count
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
      "addi a5, a5, -128     \n" // count=-127
"bgeui a5, 128, rle_2_127_ch2    \n" // if count >= 128 branch

      "rle_2_add_ch2:            \n" 
#if ALLOW_ZERO_RLE 
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
#else
      "and  a8, a8, a11      \n" // a11=0x7F
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val & 0x7F;
"beqz a5, rle_2_skip_ch2         \n"
      "or   a5, a5, a10      \n" // a10=0x80
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count | 0x80;
      "addi a6, a6, 1        \n" // rle_buff_p ++
      "rle_2_skip_ch2:           \n"
      "addi a6, a6, 1        \n" // rle_buff_p ++ 
#endif
      "mov  a8, a9           \n" // rle_val = new_val
      "movi a5, -1           \n" // rle_counter=-1
      "rle_2_end_ch2:            \n"
      "addi a5, a5, 1        \n" // rle_counter++

      "rle_loop_end_ch2:              \n"

"bltui a5, 128, rle_end_add_ch2  \n" // if count >= 128 branch
      "rle_end_127_ch2:          \n"
#if not ALLOW_ZERO_RLE
      "and  a8, a8, a11      \n" // a11=0x7F
#endif
      "s8i  a8, a6, 0        \n" // rle_buff_p = rle_val;
      "movi a7, 0xFF         \n"
      "s8i  a7, a6, 1        \n" // rle_buff_p+1 = 127 as count
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
      "addi a5, a5, -128     \n" // count=-127
"bgeui a5, 128, rle_end_127_ch2  \n" // if count >= 128 branch

      "rle_end_add_ch2:          \n" 
#if ALLOW_ZERO_RLE 
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p ++2 
#else
      "and  a8, a8, a11      \n" // 
      "s8i  a8, a6, 0        \n" // rle_buff_p=rle_val;
"beqz a5, rle_end_skip_ch2       \n"
      "or   a5, a5, a10      \n" // 
      "s8i  a5, a6, 1        \n" // rle_buff_p+1 = count;
      "addi a6, a6, 1        \n" // rle_buff_p ++ 
      "rle_end_skip_ch2:         \n"
      "addi a6, a6, 1        \n" // rle_buff_p ++
#endif

      "exit_ch2:                 \n"
      "s32i a6, %2, 0        \n" // update rle_buff_p value

      :
      :"a"(&dma_buffer), "a"(dword_count), "a"(&rle_buff_p):
      "a4","a5","a6","a7","a8","a9","a10","a11","memory");

    clockb = xthal_get_ccount();
#ifdef DEBUG_OPERATION_TIMES
    time_debug_indice_rle[time_debug_indice_rle_p++]=clockb;
#endif
 //   Serial_Debug_Port.printf("\r\n asm_process takes %d clocks\r\n",(clockb-clocka));
 //   Serial_Debug_Port.printf( "RX  Buffer = %d bytes\r\n", sample_size );
 //   Serial_Debug_Port.printf( "RLE Buffer = %d bytes\r\n", (rle_buff_p - rle_buff) );
    ESP_LOGD(TAG, "RLE Buffer = %d bytes\r\n", (rle_buff_p - rle_buff) );
}

void fast_rle_block_encode_asm_16bit(uint8_t *dma_buffer, int sample_size){ //size, not count
   uint8_t *desc_buff_end=dma_buffer;
   unsigned clocka;
   unsigned clockb=0;

   int dword_count=(sample_size/4) -1;
   clocka = xthal_get_ccount();
   
    __asm__ __volatile__(
      "l32i a4, %0, 0        \n" // Load store dma_buffer address
      "l32i a6, %2, 0        \n" // Load store rle_buffer address
      "l16ui a8, a4, 2       \n" // a8 as rle_val #2 is first
      "l16ui a9, a4, 0       \n" // a9 as new_val
      "movi a7, 0xFF         \n" // store max RLE sample
      "movi a5, 0            \n" // init rle_counter
      "movi a10, 0x8000        \n" // init rle_masks for count
      "movi a11, 0x7FFF        \n" // init rle_masks for data

"beq  a9, a8, rle_0_16       \n" // rle_val == new_val skip

#if not ALLOW_ZERO_RLE
      "and  a8, a8, a11      \n" // a11=0x7FFF
#endif
      "s16i  a8, a6, 0        \n" // rle_buff_p=rle_val;
#if ALLOW_ZERO_RLE
      "s16i  a5, a6, 2        \n" // rle_buff_p+2 = rle_counter //not needed
      "addi a6, a6, 2        \n" // rle_buff_p +=2              //not needed
#endif
      "movi a5, -1           \n" // rle_counter=-1
      "addi a6, a6, 2        \n" // rle_buff_p +=2 
      "mov  a8, a9           \n" // rle_val = new_val

      "rle_0_16:             \n"
      "addi a5, a5, 1        \n" // rle_counter++
#if not LOOPNEZS2
"loopnez %1, rle_loop_end_16 \n" // Prepare zero-overhead loop
#endif
      "loopStart_16:         \n"
      "addi a4, a4, 4        \n" // increase dma_buffer_p pointer by 4

      "rle_1_start_16:       \n"
      "l16ui a9, a4, 2       \n" // a9 as rle_val #2 is first
"beq  a9, a8, rle_1_end_16   \n" // rle_val == new_val skip

      "rle_1_127_16:         \n" //not needed 
      
      "rle_1_add_16:         \n" 
#if ALLOW_ZERO_RLE        // 4000 to 3140 bytes & 22.2k clocks
      "s16i  a8, a6, 0       \n" // *rle_buff_p=rle_val;
      "s16i  a5, a6, 2       \n" // *rle_buff_p+1 = count;
      "addi a6, a6, 4        \n" // rle_buff_p ++4 
#else                        // 4000 to 1988 bytes & 34348 clocks
      "and  a8, a8, a11      \n" // * 0x7F
      "s16i  a8, a6, 0       \n"// *rle_buff_p=rle_val;
"beqz a5, rle_1_skip_16      \n" 
      "or   a5, a5, a10      \n" // *
      "s16i  a5, a6, 2       \n"// *rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p --
      "rle_1_skip_16:        \n" 
      "addi a6, a6, 2        \n" // rle_buff_p --
#endif
      "mov  a8, a9           \n" // rle_val = new_val
      "movi a5, -1           \n" // rle_counter=-1      

      "rle_1_end_16:         \n"
      "addi a5, a5, 1        \n" // rle_counter++

      "rle_2_start_16:       \n"
      "l16ui a9, a4, 0       \n" // a9 as rle_val #0 is second
"beq  a9, a8, rle_2_end_16   \n" // rle_val == new_val continue

      "rle_2_add_16:         \n" 
#if ALLOW_ZERO_RLE 
      "s16i  a8, a6, 0       \n" // *rle_buff_p=rle_val;
      "s16i  a5, a6, 2       \n" // *rle_buff_p+2 = count;
      "addi a6, a6, 4        \n" // rle_buff_p ++4
#else
      "and  a8, a8, a11      \n" // *
      "s16i  a8, a6, 0       \n" // *rle_buff_p=rle_val;
"beqz a5, rle_2_skip_16      \n"
      "or   a5, a5, a10      \n" // *
      "s16i  a5, a6, 2       \n" // *rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p -- 
      "rle_2_skip_16:        \n"
      "addi a6, a6, 2        \n" // rle_buff_p -- 
#endif
      "mov  a8, a9           \n" // rle_val = new_val
      "movi a5, -1           \n" // rle_counter=-1
      "rle_2_end_16:         \n"
      "addi a5, a5, 1        \n" // rle_counter++

      "rle_loop_end_16:      \n"

      "rle_end_add_16:       \n" 
#if ALLOW_ZERO_RLE 
      "s16i  a8, a6, 0       \n" // *rle_buff_p=rle_val;
      "s16i  a5, a6, 2       \n" // *rle_buff_p+2 = count;
      "addi a6, a6, 4        \n" // rle_buff_p ++4 
#else
      "and  a8, a8, a11      \n" // *
      "s16i  a8, a6, 0       \n" // *rle_buff_p=rle_val;
"beqz a5, rle_end_skip_16    \n"
      "or   a5, a5, a10      \n" // *
      "s16i  a5, a6, 2       \n" // *rle_buff_p+1 = count;
      "addi a6, a6, 2        \n" // rle_buff_p -- 
      "rle_end_skip_16:      \n"
      "addi a6, a6, 2        \n" // rle_buff_p -- 
#endif

      "exit_16:              \n"
      "s32i a6, %2, 0        \n" // update rle_buff_p value

      :
      :"a"(&dma_buffer), "a"(dword_count), "a"(&rle_buff_p):
      "a4","a5","a6","a7","a8","a9","a10","a11","memory");
    clockb = xthal_get_ccount();
#ifdef DEBUG_OPERATION_TIMES
    time_debug_indice_rle[time_debug_indice_rle_p++]=clockb;
#endif

//    delay(10);
    //Serial_Debug_Port.printf("\r\n asm_process takes %d clocks\r\n",(clockb-clocka));
    //Serial_Debug_Port.printf( "RX  Buffer = %d bytes\r\n", sample_size );
    //Serial_Debug_Port.printf( "RLE Buffer = %d bytes\r\n", (rle_buff_p - rle_buff) );

/*
    Serial_Debug_Port.printf("RLE Block Output:\r\n");
    for(int i=0; i < sample_size/40 ; i++ )
      Serial_Debug_Port.printf("0x%X, ", rle_buff[i]);
    Serial_Debug_Port.println();
*/
}
