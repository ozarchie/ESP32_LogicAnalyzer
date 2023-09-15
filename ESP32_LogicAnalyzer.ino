
  /*************************************************************************
 * 
 *  ESP32 Logic Analyzer
 *  Copyright (C) 2020 Erdem U. Altinyurt
 *    
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *************************************************************************
 *
 *  This project steals some code from 
 *  https://github.com/igrr/esp32-cam-demo for I2S DMA
 *  and
 *  https://github.com/gillham/logic_analyzer/issues for SUMP protocol as template.
 *  http://dangerousprototypes.com/docs/The_Logic_Sniffer%27s_extended_SUMP_protocol
 * 
 */ 

#include "ESP32_LogicAnalyzer.h"
#include "ESP32_LogicAnalyzer_I2S_DMA.h"

void setup(void) {
  Serial_Debug_Port.begin(Serial_Debug_Port_Baud, SERIAL_8N1, RXD2, TXD2);
  OLS_Port.begin(OLS_Port_Baud);

/*jma
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);

  btStop();
  esp_bluedroid_disable();
  esp_bluedroid_deinit();  //**
  esp_bt_controller_disable();
  esp_bt_controller_deinit(); //**
//*/
 
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  //setTestOutput(100000, LED_PIN);   // Output 100KHz square wave on LED pin.
  Serial_Debug_Port.printf("\r\nESP32 Logic Sniffer V23.09.11\r\n");
  Serial_Debug_Port.printf("Initial boot ..\r\n");
  dma_desc_init(CAPTURE_SIZE);

//  DEVKITc_V4 Pinout
//  =================
// Digital In
// ----------
// Map of digital input channel virtual pins to physical input pins
// The table is defined in the .h file
  cfg.gpio_bus[0]  = D01_PIN;   // GPIO13
  cfg.gpio_bus[1]  = D02_PIN;   // GPIO12
  cfg.gpio_bus[2]  = D03_PIN;   // GPIO14
  cfg.gpio_bus[3]  = D04_PIN;   // GPIO27
  cfg.gpio_bus[4]  = D05_PIN;   // GPIO26
  cfg.gpio_bus[5]  = D06_PIN;   // GPIO25
  cfg.gpio_bus[6]  = D07_PIN;   // GPIO33
  cfg.gpio_bus[7]  = D08_PIN;   // GPIO32

  cfg.gpio_bus[8]  = D09_PIN;    // SS       5
  cfg.gpio_bus[9]  = D10_PIN;    // CLK     18
  cfg.gpio_bus[10] = D11_PIN;    // MOSI    23
  cfg.gpio_bus[11] = D12_PIN;    // MISO    19
  cfg.gpio_bus[12] = D13_PIN;    // SCL1    22
  cfg.gpio_bus[13] = D14_PIN;    // SDA1    21
  cfg.gpio_bus[14] = D15_PIN;    // RTC-10   4
  cfg.gpio_bus[15] = D16_PIN;    // RTC-13  15

// Analog In
  cfg.gpio_bus[16] = A01_PIN;    // ADC1-7  35
  cfg.gpio_bus[17] = A02_PIN;    // ADC1-6  34
  cfg.gpio_bus[18] = A03_PIN;    // ADC1-3  39
  cfg.gpio_bus[19] = A04_PIN;    // ADC1-0  36

  cfg.gpio_clk = CLK_PIN;             // PinCLK_PIN used for XCK input from LedC
  cfg.bits = I2S_PARALLEL_BITS_16;
  cfg.clkspeed_hz = 2 * 1000 * 1000;  //resulting pixel clock = 1MHz
  cfg.buf = &bufdesc;

  //setSampleRate(I2S_HZ);
  //fill_dma_desc( bufdesc );
  i2s_parallel_setup(&cfg);
}

void loop()
{
  Serial_Debug_Port.printf("Main loop ..\r\n");
  while (1) {
    processgetSumpDatas();
  }
}

/*
  All communication is done using a standard RS232 connection with 8 data bits, 1 stop bit and no parity.
   The transfer rate can be set to 115200, 57600, 38400 or 19200 bps. XON/XOFF software flow control is available.
  When sending captured data the analyzer will send blocks of four bytes, the first containing the lowest channels.
   No start or end sequence exists.
    The host can assume an end of transmission if no data has been received for the duration of one byte.
  The protocol used by hardware version 0.5 and older is not covered here.
   Hardware 0.6 uses protocol version 0, and hardware 0.7 uses protocol version 1.
    Unless otherwise stated, commands exist in both versions.
*/

void processgetSumpDatas(void){

    if (OLS_Port.available() > 0) {
      z = OLS_Port.available();
      sumpCmnd = OLS_Port.read();
      Serial_Debug_Port.printf("CMD: 0x%02X -> ", sumpCmnd);
      chan_num = 0;
      switch (sumpCmnd) {
        case SUMP_RESET:
          break;
        case SUMP_QUERY:
          OLS_Port.print(F("1ALS"));    Serial_Debug_Port.print(F("1ALS\r\n"));
          break;
        case SUMP_ARM:
          startCapture();
          break;
        case SUMP_TRIGGER_MASK_CH_A:
          getSumpData();
          trigger = ((uint16_t)sumpLongCmnd[1] << 8 ) | sumpLongCmnd[0];
          if (trigger != 0) {
            Serial_Debug_Port.printf("Trigger Set for inputs : ");
            for ( int i = 0; i < 16 ; i++ ) {
              if (( trigger >> i) & 0x0001 )
                Serial_Debug_Port.printf("%02d, ", i );
              else
                Serial_Debug_Port.printf("--, ");
            }
            Serial_Debug_Port.println();
          }
          break;
        case SUMP_TRIGGER_VALUES_CH_A:
          getSumpData();
          trigger_values = ((uint16_t)sumpLongCmnd[1] << 8 ) | sumpLongCmnd[0];
          if (trigger) {
            Serial_Debug_Port.printf("Trigger Val for inputs : ");
            for ( int i = 0; i < 16 ; i++ ) {
              if (( trigger >> i) & 0x0001 )
                Serial_Debug_Port.printf("%C, ", (( trigger_values >> i ) & 0x1 ? 'H' : 'L') );
              else 
                Serial_Debug_Port.printf("X, ");
            }
            Serial_Debug_Port.println();
          }
          break;
        case SUMP_TRIGGER_MASK_CH_B:
        case SUMP_TRIGGER_MASK_CH_C:
        case SUMP_TRIGGER_MASK_CH_D:
        case SUMP_TRIGGER_VALUES_CH_B:
        case SUMP_TRIGGER_VALUES_CH_C:
        case SUMP_TRIGGER_VALUES_CH_D:
        case SUMP_TRIGGER_CONFIG_CH_A:
        case SUMP_TRIGGER_CONFIG_CH_B:
        case SUMP_TRIGGER_CONFIG_CH_C:
        case SUMP_TRIGGER_CONFIG_CH_D:
          getSumpData();
          // No config support
          break;
        case SUMP_SET_DIVIDER:
          /*
            the shifting needs to be done on the 32bit unsigned long variable
             so that << 16 doesn't end up as zero.
          */
          getSumpData();
          divider = sumpLongCmnd[2];
          divider = divider << 8;
          divider += sumpLongCmnd[1];
          divider = divider << 8;
          divider += sumpLongCmnd[0];
          setupDelay();
          break;
        case SUMP_SET_READ_DELAY_COUNT:
          getSumpData();
          readCount = 4 * (((sumpLongCmnd[1] << 8) | sumpLongCmnd[0]) + 1);
          if (readCount > MAX_CAPTURE_SIZE)
            readCount = MAX_CAPTURE_SIZE;
          delayCount = 4 * (((sumpLongCmnd[3] << 8) | sumpLongCmnd[2]) + 1);
          if (delayCount > MAX_CAPTURE_SIZE)
            delayCount = MAX_CAPTURE_SIZE;
          break;
        case SUMP_SET_FLAGS:
          getSumpData();
          rleEnabled = sumpLongCmnd[1] & 0x01;
          if (rleEnabled)
            Serial_Debug_Port.println("RLE Compression enabled");
          else
            Serial_Debug_Port.println("Non-RLE Operation");
          Serial_Debug_Port.printf("Demux  %c\r\n", sumpLongCmnd[0] & 0x01 ? 'Y' : 'N');
          Serial_Debug_Port.printf("Filter %c\r\n", sumpLongCmnd[0] & 0x02 ? 'Y' : 'N');
          channels_to_read = (~(sumpLongCmnd[0] >> 2) & 0x0F);
          Serial_Debug_Port.printf("Channels to read: 0x%X \r\n",  channels_to_read);
          if(channels_to_read == 3)
          Serial_Debug_Port.printf("External Clock %c\r\n", sumpLongCmnd[0] & 0x40 ? 'Y' : 'N');
          Serial_Debug_Port.printf("inv_capture_clock %c\r\n", sumpLongCmnd[0] & 0x80 ? 'Y' : 'N');
          break;
        case SUMP_SEND_METADATA:
          sendSumpMetadata();
          break;
        case SUMP_SELF_TEST:
          break;
        default:
          Serial_Debug_Port.printf("Unrecognized cmd 0x%02X\r\n", sumpCmnd );
          getSumpData();
          break;
      }
    }
}

void getSumpData() {
  delay(10);
  sumpLongCmnd[0] = OLS_Port.read();
  sumpLongCmnd[1] = OLS_Port.read();
  sumpLongCmnd[2] = OLS_Port.read();
  sumpLongCmnd[3] = OLS_Port.read();
  Serial_Debug_Port.printf("CMDs ");
  for (int q = 0; q < 4; q++) {
    Serial_Debug_Port.printf(" 0x%02X", sumpLongCmnd[q]);
  }
  Serial_Debug_Port.println();
}

/*
Metadata command
----------------
A new command byte is added to the sump protocol: 0x04 - get metadata.
In response, the device sends a series of 1-byte keys, followed by data pertaining to that key.
The series ends with the key 0x00. The system can be extended with new keys as more data needs to be reported.

Proposed meta data format
The keys are split up into two fields:
 the upper 3 bits denote the type, and the lower 5 bits denote the token.
The token is unique within the type.
  Thus a 0x01 null-terminated string token is not the same as a 0x01 integer token.

Type0 - null-terminated string, UTF-8 encoded
---------------------------------------------
0	0x00	not used, key means end of metadata
1	0x01	device name (e.g. "Openbench Logic Sniffer v1.0", "Bus Pirate v3b"
2	0x02	Version of the FPGA firmware
3	0x03	Ancillary version (PIC firmware)

Type2 - 32-bit unsigned integer
-------------------------------
0	0x20	Number of usable probes
1	0x21	Amount of sample memory available (bytes)
2	0x22	Amount of dynamic memory available (bytes)
3	0x23	Maximum sample rate (hz)
4	0x24	Protocol version (long) - see below

Type4 - 8-bit unsigned integer
------------------------------
0	0x40	Number of usable probes (short)
1	0x41	Protocol version (short)

The protocol version (0x24) key holds a 4-stage version,
  one per byte, where the MSB holds the major version number.
As of the first release to support this metadata command,
  the protocol version should be 2.
This would be encoded as 0x00000002.

3-7	unused
----------
*/
void sendSumpMetadata() {
  /* device name */
  OLS_Port.write((uint8_t)0x01);
#ifdef BOARD_esp32dev
  OLS_Port.write("ESPLAM32dev");
#else // ifdef BOARD_esp32s2
  OLS_Port.write("ESPLAM32s2");
#endif
  OLS_Port.write((uint8_t)0x00);

  /* firmware version */
  OLS_Port.write((uint8_t)0x02);
  OLS_Port.print("0.20");
  OLS_Port.write((uint8_t)0x00);

  /* sample memory */
  OLS_Port.write((uint8_t)0x21);
  uint32_t capture_size = CAPTURE_SIZE;
  OLS_Port.write((uint8_t) (capture_size >> 24) & 0xFF);
  OLS_Port.write((uint8_t) (capture_size >> 16) & 0xFF);
  OLS_Port.write((uint8_t) (capture_size >> 8) & 0xFF);
  OLS_Port.write((uint8_t) (capture_size >> 0) & 0xFF);

  /* Maximum sample rate (20MHz) */
//  uint32_t capture_speed = 200000000;     // 200MHz: FPGA internal clock
  uint32_t capture_speed = 80000000;      // 80MHz: ESP32 APB_CLK = 80MHz
  OLS_Port.write((uint8_t) 0x23);
  OLS_Port.write((uint8_t) (capture_speed >> 24) & 0xFF);
  OLS_Port.write((uint8_t) (capture_speed >> 16) & 0xFF);
  OLS_Port.write((uint8_t) (capture_speed >> 8) & 0xFF);
  OLS_Port.write((uint8_t) (capture_speed >> 0) & 0xFF);

  /* number of probes */
  OLS_Port.write((uint8_t)0x40);
  //OLS_Port.write((uint8_t)0x08);//8
  OLS_Port.write((uint8_t)0x10);  //16
  //OLS_Port.write((uint8_t)0x20);//32

  /* protocol version - short (0x41) */
  OLS_Port.write((uint8_t)0x41);
  OLS_Port.write((uint8_t)0x02);

  /* end of data */
  OLS_Port.write((uint8_t)0x00);
}

void setupDelay() {
  double rate = 100000000.0 / (divider + 1.0);  // Internal clock is 100MHz
  setSampleRate((int)rate);
  Serial_Debug_Port.printf("Sample Rate: %.2f MHz\r\n", rate/1000000.0);
}

void startCapture() {
  uint32_t a, b, c, d;
  Serial_Debug_Port.println("");  
  Serial_Debug_Port.println("Starting Capture");  
  Serial_Debug_Port.printf("FreeHeap         :%u\r\n", ESP.getFreeHeap());
  Serial_Debug_Port.printf("FreeHeap(64Byte) :%u\r\n", heap_caps_get_largest_free_block(64) );
  Serial_Debug_Port.printf("Trigger Values   :0x%X\r\n", trigger_values);
  Serial_Debug_Port.printf("Trigger          :0x%X\r\n", trigger);
  Serial_Debug_Port.printf("Running on CORE  :#%d\r\n", xPortGetCoreID());
  Serial_Debug_Port.printf("Samples          :%d\r\n", readCount);

  digitalWrite( LED_PIN, LOW );

  Serial_Debug_Port.printf("dma_sample_count :%d\r\n", s_state->dma_sample_count);
  //ESP_LOGD(TAG, "dma_sample_count: %d", s_state->dma_sample_count);
  rle_init();
  start_dma_capture();

  int Timeout = 20; // ~2s
  while ((! s_state->dma_done ) && (Timeout-- >0)) {
    delay(100);
  }
  Serial_Debug_Port.printf("dma_done         :%s\r\n", s_state->dma_done ? "true" : "false");

  yield();

  digitalWrite( LED_PIN, HIGH );

  Serial_Debug_Port.printf("ReadCount        :%d\r\n",readCount);
  Serial_Debug_Port.printf("DMA Desc Current :%d\r\n",  s_state->dma_desc_cur);

  ESP_LOGD(TAG, "Copying buffer");

  int filled_desc = ((readCount/2) / s_state->dma_sample_per_desc);
  int filled_sample_offset = ((readCount/2) % s_state->dma_sample_per_desc); //((readCount - 1) % s_state->dma_val_per_desc) % s_state->dma_sample_per_desc;
  int filled_full_sample_offset = s_state->dma_sample_per_desc;
  
  int tx_count = 0;
  Serial_Debug_Port.printf("useddesc         :%d\r\n", filled_desc);
  Serial_Debug_Port.printf("usedsampleoffset :%d\r\n", filled_sample_offset);

#ifdef DEBUG_OPERATION_TIMES
  Serial_Debug_Port.printf( "\r\nDMA Times:" );
  for(int i = 0 ; i <  50 ; i++){
    Serial_Debug_Port.printf( "%u\t", time_debug_indice_dma[i]-time_debug_indice_dma[0] );
    }
  
  Serial_Debug_Port.printf( "\r\nRLE Times:" );
  for(int i = 0 ; i <  50 ; i++){
    Serial_Debug_Port.printf( "%u\t", time_debug_indice_rle[i]-time_debug_indice_dma[0] );
    }
#endif

  Serial_Debug_Port.printf( "\r\nDone\r\n" );
  Serial_Debug_Port.flush();
  filled_desc--;
  filled_full_sample_offset--;
  if( filled_sample_offset-- == 0)
    filled_sample_offset = filled_full_sample_offset;

  dma_elem_t cur;

/*
  Serial_Debug_Port.printf("\r\nRAW BlocX \r\n");
  for ( int i = 0 ; i < 100 ; i++ ){
     cur = (dma_elem_t&)s_state->dma_buf[0][i];
     Serial_Debug_Port.printf("0x%X, ", cur.sample2);
     Serial_Debug_Port.printf("0x%X, ", cur.sample1);
   }

  Serial_Debug_Port.printf("\r\nRAW Block InpuX:\r\n");
  for ( int i = 0 ; i < 400 ; i+=2 ){
     uint8_t *crx = (uint8_t*)s_state->dma_buf[0];
     Serial_Debug_Port.printf("0x%X, ", *(crx+i) );
     }
   Serial_Debug_Port.println();
*/


  if(s_state->dma_desc_triggered < 0){ //if not triggered mode,
    s_state->dma_desc_triggered=0;  //first desc is 0
    ESP_LOGD(TAG, "Normal TX");
  }
  else {
    ESP_LOGD(TAG, "Triggered TX");
  }
    
  if(rleEnabled){
    ESP_LOGD(TAG, "RLE TX");
    int rle_fill = (rle_buff_p - rle_buff);
    Serial_Debug_Port.printf( "RLE Buffer = %d bytes\r\n", rle_fill );
    uint32_t rle_sample_count=0;
    uint32_t rle_sample_req_count=0;
    
    for(int i=0; i<rle_fill ; i++){
#if ALLOW_ZERO_RLE
      if( i%2 != 0 ){
        rle_sample_count += rle_buff[i];
        if( readCount > rle_sample_count )
          rle_sample_req_count= i;
        }

#else
#endif
    }
    Serial_Debug_Port.printf( "Total RLE Sample Count = %d\r\n", rle_sample_count ); 
    Serial_Debug_Port.printf( "Total RLE Sample Req Count = %d\r\n", rle_sample_req_count );
/*
    for(int i=0; i<32 ; i++){
     Serial_Debug_Port.printf("Processing DMA Desc: %d", i);
     fast_rle_block_encode_asm( (uint8_t*)s_state->dma_buf[i], s_state->dma_buf_width);
     if( rle_buff_p-rle_buff > RLE_SIZE - 4000 )
      break;
     }
*/   

    if( channels_to_read == 3 )
    {
      int a=0;
      Serial_Debug_Port.printf("Debug RLE BUFF:" );
      for( int i=0; i <50 ; i++){
        Serial_Debug_Port.printf("0x%X ", rle_buff[i] );
        }
      Serial_Debug_Port.printf("\r\n" );

#if ALLOW_ZERO_RLE
      for( int i =  rle_fill-4;  i>=0 ; i-=4  ){
      //for( int i =  rle_sample_req_count;  i>=0 ; i-=2  ){
        //if( rle_buff[i+1] !=0 )
       
        OLS_Port.write( rle_buff[i+2] | 0x00 ); //Count sent first
        OLS_Port.write( rle_buff[i+3] | 0x80 ); //Count sent later
        OLS_Port.write( rle_buff[i+0] & 0xFF ); //Value sent first
        OLS_Port.write( rle_buff[i+1] & 0x7F ); //Value sent later
        }
#else
      for( int i = rle_fill-2;  i>=0 ; i-=2  ){
        OLS_Port.write( rle_buff[i] );
        OLS_Port.write( rle_buff[i+1] );
        }

#endif
      }

      else{
//The buffer need to send from end to start due OLS protocol...
#if ALLOW_ZERO_RLE
      //for( int i =  0 ;  i < readCount - rle_sample_req_count ; i++  )
      //  OLS_Port.write( 0 );
      
      for( int i =  rle_fill-2;  i>=0 ; i-=2  ){
      //for( int i =  rle_sample_req_count;  i>=0 ; i-=2  ){
        if( rle_buff[i+1] !=0 )
        OLS_Port.write( rle_buff[i+1] | 0x80 ); //Count sent first
        OLS_Port.write( rle_buff[i+0] & 0x7F ); //Value sent later
        }
#else
      //for( int i =  (rle_buff_p - rle_buff)-1;  i>=0 ; i--  ){
      for( int i = rle_fill-1;  i>=0 ; i--  ){
        OLS_Port.write( rle_buff[i] );
        }

#endif
      }
      
    
/*

    for( int i = 0  ;  i < 200  ; i++  )
     OLS_Port.write( 8 );
    
    for( int i = s_state->dma_buf_width ;  i > 0   ; i-=4  ){
       //OLS_Port.write( *(((uint8_t*)s_state->dma_buf[0])+i) & 0x7F );
       uint8_t *crx = (uint8_t*)s_state->dma_buf[0];
       OLS_Port.write( *(crx+i+0) );
       OLS_Port.write( *(crx+i+2) );
     }
*/   
    /*
    for( int i = (rle_buff_p - rle_buff)-2; i > 0  ; i-=2  ){
      OLS_Port.write( rle_buff[i+1] -1 | 0x80 ) ;
      OLS_Port.write( rle_buff[i+0] & 0x7F ) ;
      }
      */
    /*
    for( int i = 0  ;  i < 200  ; i++  )
     OLS_Port.write( 8 );

    
    for ( int i =  filled_sample_full_offset ; i >= 0 ; i-- ) {
        cur = s_state->dma_buf[0][i];
        if (channels_to_read == 1) {
          OLS_Port.write(cur.sample2);
          OLS_Port.write(cur.sample1);
        }
        }
    */

    /*
    int x0 = readCount - (rle_buff_p-rle_buff)/2;
    for( int i = 0; i < x0 ; i+=2 ){
      OLS_Port.write( ((i/2)%2 ) | 0x80 );
      OLS_Port.write( (i/2)%2 );     
    }
*/      

    }
   else{
    for ( int j = filled_desc; j >= 0 ; j-- ) {
      ESP_LOGD(TAG, "filled_buff trgx = %d", (j + s_state->dma_desc_triggered + s_state->dma_desc_count) % s_state->dma_desc_count);
      digitalWrite( LED_PIN, !digitalRead(LED_PIN) );
      //for( int i=s_state->dma_buf_width/4 - 1; i >=0 ; i-- ){
      for ( int i = (j == filled_desc ? filled_sample_offset : filled_full_sample_offset ) ; i >= 0 ; i-- ) {
        //Serial.printf( "%02X %02X ", s_state->dma_buf[j][i].sample1,  s_state->dma_buf[j][i].sample2 );
        cur = s_state->dma_buf[ (j
                                +s_state->dma_desc_triggered
                                +s_state->dma_desc_count) % s_state->dma_desc_count][i];
        if (channels_to_read == 1) {
          OLS_Port.write(cur.sample2);
          OLS_Port.write(cur.sample1);
        }
        else if (channels_to_read == 2) {
          OLS_Port.write(cur.unused2);
          OLS_Port.write(cur.unused1);
        }
        else if (channels_to_read == 3) {
          OLS_Port.write(cur.sample2);
          OLS_Port.write(cur.unused2);
          OLS_Port.write(cur.sample1);
          OLS_Port.write(cur.unused1);
        }
        tx_count += 2;
        if (tx_count >= readCount)
          goto brexit;
      }
    }
  }
brexit:
  //OLS_Port.flush();
  //ESP_LOGD(TAG, "TX_Count: %d", tx_count);
  ESP_LOGD(TAG, "End. TX: %d", tx_count);
  digitalWrite( LED_PIN, HIGH );
}
