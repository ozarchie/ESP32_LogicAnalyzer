# ESP32 LogicAnalyzer

## A *SUMP* compatible 16Bit Logic Analyzer for ESP32 MCUs.

![PulseView](/images/ESP32_LogicAnalyzer_in_PulseViewSmall.png)

* Use Arduino to compile and flash your ESP32.
* Uses **ESP32 I2S DMA** and could capture speeds up to **20 Mhz**.
* Support **8 bit** and **16 bit** operations.
* Maximum **128k** samples. (Even using 8bit capturing mode.)
* RLE compression supported.
* Analog input is **NOT** available.
* ~~WROOVER modules support **2M** samples but only up to **2 Mhz** due bandwith limit on PSRAM access.~~ Under development.
* Default OLS port is **UART0** and default baudrate is **912600**.
* You can use **UART2** for high speed OLS communication by using **USE_SERIAL2_FOR_OLS** macro at ESP32_LogicAnalyzer.h file. Default OLS baudrate is **3M** on this mode.
* **WARNING:** 
  - For OLS port at UART0
    - Please pull **GPIO15** to ground (this will silence boot up messages.)
    - Set **"Core Debug Level"=None** before compiling code at arduino.
  - Set **"Core Debug Level"=None** for > 10Mhz capture operations.
  - **GPIO23** used for I2S input clk. Don't use it for IO or change it to an unused pin from code.

  
## Protocol  

https://sigrok.org/wiki/Openbench_Logic_Sniffer#Protocol

All **communication** is done using a standard RS232 connection with 8 data bits, 1 stop bit and no parity.  
The transfer rate is not emulated over USB and can be set to any speed supported by the operating system.  
XON/XOFF software flow control is available. 
  
When **sending captured data** the analyzer will send blocks of four bytes, the first containing the lowest channels.  
No start or end sequence exists.  
The host can assume an end of transmission if no data has been received for the duration of one byte.  

### Short Commands
These commands are exactly one byte long.  
#### Reset (0x00)
Resets the device. Should be sent 5 times when the receiver status is unknown. (It could be waiting for up to four bytes of pending long command data.)
#### Run (0x01)
Arms the trigger.
#### ID (0x02)
Asks for device identification.  
 - The device will respond with four bytes.  
The first three ("SLA") identify the device.  
The last one identifies the protocol version which is currently either "0" or "1".
#### XON (0x11)
Put transmitter out of pause mode.  
It will continue to transmit captured data if any is pending.  
This command is being used for xon/xoff flow control.
#### XOFF (0x13)
Put transmitter in pause mode.  
It will stop transmitting captured data.  
This command is being used for xon/xoff flow control.
### Long Commands
**Are five bytes long. The first byte contains the opcode. The bytes are displayed in the order in which they are sent to the serial port starting left. The bits within one byte are displayed most significant first.**  
#### Set Trigger Mask (0xc0, 0xc4, 0xc8, 0xcc)  
Defines which trigger values must match.  
In parallel mode each bit represents one channel, in serial mode each bit represents one of the last 32 samples of the selected channel. The opcodes refer to stage 0-3 in the order given above.  
(Protocol version 0 only supports stage 0.)  
![SetTriggerMask](/images/1100xx00.png)
#### Set Trigger Values (0xc1, 0xc5, 0xc9, 0xcd)  
Defines which values individual bits must have.  
In parallel mode each bit represents one channel, in serial mode each bit represents one of the last 32 samples of the selected channel. The opcodes refer to stage 0-3 in the order given above.  
(Protocol version 0 only supports stage 0.)  
![SetTriggerValues](/images/1100xx01.png
#### Set Trigger Configuration (0xc2, 0xc6, 0xca, 0xce)  
Configures the selected trigger stage.  
The opcodes refer to stage 0-3 in the order given above. The following parameters will be set:  
  - *delay*  
If a match occures, the action of the stage is delayed by the given number of samples.
  - *level*  
Trigger level at which the stage becomes active.
  - *channel*  
Channel to be used in serial mode. (0-31 in normal operation; 0-15 when demux flag is set)
  - *serial*  
When set to 1 the stage operates as serial trigger, otherwise it used as parallel trigger.
  - *start*  
When set to 1 a match will start the capturing process. The trigger level will rise on match regardless of this flag.  
(Command available as of protocol version 1.)  
![SetTriggerConfiguration](/images/1100xx10.png)  
#### Set Divider (0x80)  
When x is written, the sampling frequency is set to f = clock / (x + 1).  
![SetDivider](/images/10000000.png)
#### Set Read & Delay Count (0x81)  
**Read Count** is the number of samples (divided by four) to read back from memory and sent to the host computer.  
**Delay Count** is the number of samples (divided by four) to capture after the trigger fired.  
A Read Count bigger than the Delay Count means that data from before the trigger match will be read back. This data will only be valid if the device was running long enough before the trigger matched.  
![SetRead&DelayCount](/images/10000001.png)  
#### Set Flags (0x82)  
Sets the following flags:  
  - *demux*  
Enables the demux input module. (Filter must be off.)
  - *filter*  
Enables the filter input module. (Demux must be off.)
  - *channel groups*  
Disable channel group. Disabled groups are excluded from data transmissions. This can be used to speed up transfers. There are four groups, each represented by one bit. Starting with the least significant bit of the channel group field channels are assigned as follows: 0-7, 8-15, 16-23, 24-31.
  - *external*  
Selects the clock to be used for sampling. If set to 0, the internal clock divided by the configured divider is used, and if set to 1, the external clock will be used. Filter and demux are only available with internal clock.
  - *inverted*  
When set to 1, the external clock will be inverted before being used. The inversion causes a delay that may cause problems at very high clock rates. This option only has an effect with external set to 1.  
![SetFlags](/images/10000010.png)  
