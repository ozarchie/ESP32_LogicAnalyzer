#include "ESP32_LogicAnalyzer.h"
#include "ESP32_LogicAnalyzer_I2S_DMA.h"

static void IRAM_ATTR i2s_trigger_isr(void) {
  //I2S0.conf.rx_start = 1;
}

static void IRAM_ATTR i2s_isr(void* arg) {
  if(trigger==0){
    ESP_LOGD(TAG_I2S, "DMA INT Number %d Status 0x%x", s_state->dma_desc_cur, I2S0.int_raw.val );
  
  /*
  ESP_LOGD(TAG, "DMA INT take_data? %d", I2S0.int_raw.rx_take_data );
  ESP_LOGD(TAG, "DMA INT in_dscr_empty? %d", I2S0.int_raw.in_dscr_empty );
  ESP_LOGD(TAG, "DMA INT in_done? %d", I2S0.int_raw.in_done );
  ESP_LOGD(TAG, "DMA INT in_suc_eof? %d", I2S0.int_raw.in_suc_eof );
  ESP_LOGD(TAG, "DMA INT rx_rempty? %d", I2S0.int_raw.rx_rempty );
  ESP_LOGD( "\r\n" );
  */
  
  //ESP_LOGD(TAG, "Executing DMA ISR on core %d", xPortGetCoreID() );
  }
  
  //gpio_set_level(, 1); //Should show a pulse on the logic analyzer when an interrupt occurs
  //gpio_set_level(, 0);
  if(I2S0.int_raw.in_done){ //filled desc
    #ifdef DEBUG_OPERATION_TIMES
    time_debug_indice_dma[time_debug_indice_dma_p++]=xthal_get_ccount();
    #endif
    //Serial_Debug_Port.printf("DMA INT Number %d Status 0x%xX\r\n", s_state->dma_desc_cur, I2S0.int_raw.val);
    if(trigger && (stop_at_desc==-1)){
      static uint16_t trigger_helper_0;
      static uint16_t trigger_helper_1;
      trigger_helper_0 = buff_process_trigger_0((uint16_t*)s_state->dma_buf[ s_state->dma_desc_cur % s_state->dma_desc_count ], s_state->dma_buf_width, false);
      trigger_helper_1 = buff_process_trigger_1((uint16_t*)s_state->dma_buf[ s_state->dma_desc_cur % s_state->dma_desc_count ], s_state->dma_buf_width, false);
      ESP_LOGD(TAG_I2S, "DMA INT Number %d Status 0x%x TL0: x%X TH1: 0x%X", s_state->dma_desc_cur, I2S0.int_raw.val, trigger_helper_0, trigger_helper_1);

      //ESP_LOGD(TAG, "trigger ^ trigger_values 0x%X trigger ^ trigger_values ^ trigger_helper_0 0x%x ", trigger ^ trigger_values,  trigger ^ trigger_values ^ trigger_helper_0 );
      if(( trigger_values & trigger_helper_1 ) || ( (trigger ^ trigger_values) & ~trigger_helper_0 ) ){
        ESP_LOGD(TAG_I2S, "DMA Triggered at desc %d (%d)",  s_state->dma_desc_triggered, s_state->dma_desc_triggered%s_state->dma_desc_count );
        if(rleEnabled){
          if( channels_to_read == 1 )
              fast_rle_block_encode_asm_8bit_ch1( (uint8_t*)s_state->dma_buf[ s_state->dma_desc_cur % s_state->dma_desc_count], s_state->dma_buf_width);
          else if ( channels_to_read == 2 )
              fast_rle_block_encode_asm_8bit_ch2( (uint8_t*)s_state->dma_buf[ s_state->dma_desc_cur % s_state->dma_desc_count], s_state->dma_buf_width);
          else
              fast_rle_block_encode_asm_16bit( (uint8_t*)s_state->dma_buf[ s_state->dma_desc_cur % s_state->dma_desc_count], s_state->dma_buf_width);
          }
        else{
          stop_at_desc= (s_state->dma_desc_cur + (readCount / (s_state->dma_buf_width/2))-1) % s_state->dma_desc_count;
          ESP_LOGD(TAG_I2S, "DMA BREAK! Stop at desc %d", stop_at_desc);
          
          s_state->dma_desc_triggered = s_state->dma_desc_cur;
          I2S0.rx_eof_num = readCount/2 ;//- s_state->dma_buf_width/4000; // Total capacity - desc capacity
          I2S0.int_ena.in_suc_eof = 1;
          }        
        trigger=0;
        I2S0.int_clr.val = I2S0.int_raw.val;
        }
      }
    else if(rleEnabled){
      //ESP_LOGD(TAG,"Processing DMA Desc: %d (%d)\r\n", s_state->dma_desc_cur,  s_state->dma_desc_cur % s_state->dma_desc_count);
      //Serial_Debug_Port.printf("Processing DMA Desc: %d (%d)\r\n", s_state->dma_desc_cur,  s_state->dma_desc_cur % s_state->dma_desc_count);
      Serial_Debug_Port.printf(".");
        if(rleEnabled){
          if( channels_to_read == 1 )
              fast_rle_block_encode_asm_8bit_ch1( (uint8_t*)s_state->dma_buf[ s_state->dma_desc_cur % s_state->dma_desc_count], s_state->dma_buf_width);
          else if ( channels_to_read == 2 )
              fast_rle_block_encode_asm_8bit_ch2( (uint8_t*)s_state->dma_buf[ s_state->dma_desc_cur % s_state->dma_desc_count], s_state->dma_buf_width);
          else
              fast_rle_block_encode_asm_16bit( (uint8_t*)s_state->dma_buf[ s_state->dma_desc_cur % s_state->dma_desc_count], s_state->dma_buf_width);
          }

      if( (RLE_SIZE - (rle_buff_p - rle_buff )) < 4000) {
        //break;
        ESP_LOGD(TAG_I2S,"Break due rle_buff fill: %d\r\n", (RLE_SIZE - (rle_buff_p - rle_buff )));
        I2S0.int_ena.in_suc_eof = 1;
        esp_intr_disable(s_state->i2s_intr_handle);
        i2s_conf_reset();
        I2S0.conf.rx_start = 0;
        s_state->dma_done = true;
        }

      //rle doesn't need to stop at rx eof
      I2S0.int_clr.val = I2S0.int_raw.val;
      s_state->dma_desc_cur++;
      return;
      }
    s_state->dma_desc_cur++;
    }

    
  if(trigger==0)  //Not stop while not triggered.
  if(
    //I2S0.int_raw.in_dscr_empty ||
      I2S0.int_raw.in_suc_eof   ||
      I2S0.int_raw.rx_take_data || //>20 Mhz 
    //!I2S0.int_raw.rx_rempty || //<=20khz
    s_state->dma_desc_cur == s_state->dma_desc_count || //triggger
    0
    )
    {
      ESP_LOGD(TAG_I2S, "DMA INT take_data? %d", I2S0.int_raw.rx_take_data );
      ESP_LOGD(TAG_I2S, "DMA INT in_dscr_empty? %d", I2S0.int_raw.in_dscr_empty );
      ESP_LOGD(TAG_I2S, "DMA INT in_done? %d", I2S0.int_raw.in_done );
      ESP_LOGD(TAG_I2S, "DMA INT in_suc_eof? %d", I2S0.int_raw.in_suc_eof );
      ESP_LOGD(TAG_I2S, "DMA INT rx_rempty? %d", I2S0.int_raw.rx_rempty );

      //s_state->dma_desc_cur=0;
      esp_intr_disable(s_state->i2s_intr_handle);
      i2s_conf_reset();
      I2S0.conf.rx_start = 0;
      s_state->dma_done = true;
   }
 
  I2S0.int_clr.val = I2S0.int_raw.val;   
}

void dma_serializer( dma_elem_t *dma_buffer ){
  for ( int i = 0 ; i < s_state->dma_buf_width/4 ; i++ ){
     uint8_t y =  dma_buffer[i].sample2;
     dma_buffer[i].sample2 = dma_buffer[i].sample1;
     dma_buffer[i].sample1 = y;
   }
}

static void dma_desc_deinit(){
    ESP_ERROR_CHECK(esp_intr_disable(s_state->i2s_intr_handle));
    if (s_state->dma_buf) {
        for (int i = 0; i < s_state->dma_desc_count; ++i) {
            free(s_state->dma_buf[i]);
        }
    }
    free(s_state->dma_buf);
    free(s_state->dma_desc);
}

void start_dma_capture(void) {
  s_state->dma_done = false;
  s_state->dma_desc_cur = 0;
  s_state->dma_received_count = 0;
  s_state->dma_filtered_count = 0;
  s_state->dma_desc_triggered = 0;

#ifdef DEBUG_OPERATION_TIMES
time_debug_indice_dma_p=0;
time_debug_indice_rle_p=0;
for(int i=0; i < 1024 ; i++ )
time_debug_indice_dma[i]=time_debug_indice_rle[i]=0;
#endif

  ESP_ERROR_CHECK(esp_intr_disable(s_state->i2s_intr_handle));
  i2s_conf_reset();

  I2S0.in_link.addr = (uint32_t) &s_state->dma_desc[0];
  I2S0.in_link.start = 1;
  I2S0.int_clr.val = I2S0.int_raw.val;

  I2S0.int_ena.val = 0;
  I2S0.int_ena.in_done = 1;

  ESP_LOGD(TAG_I2S, "DMA Trigger : 0x%X", trigger );
  if (trigger || rleEnabled) {
    stop_at_desc = -1;
    I2S0.rx_eof_num = s_state->dma_buf_width;
    I2S0.int_ena.in_suc_eof = 0;
    I2S0.int_ena.rx_take_data = 0;
    lldesc_t* pd = &s_state->dma_desc[s_state->dma_desc_count - 1];
    pd->eof = 0;
    pd->qe.stqe_next = &s_state->dma_desc[0];
  }
  else {
    s_state->dma_desc_triggered=-1;
    I2S0.rx_eof_num = readCount/2;
    I2S0.int_ena.in_suc_eof = 1;
    I2S0.int_ena.rx_take_data = 1;
    lldesc_t* pd = &s_state->dma_desc[s_state->dma_desc_count - 1];
    pd->eof = 1;
    pd->qe.stqe_next = 0x0;
  }

  //clear dma buffers
  for (int i = 0; i < s_state->dma_desc_count; ++i)
    memset( s_state->dma_buf[i], 0, s_state->dma_desc[i].length);

  //Enable the Interrupt
  ESP_ERROR_CHECK(esp_intr_enable(s_state->i2s_intr_handle));

  //attachInterrupt(0, i2s_trigger_isr, RISING);
  I2S0.conf.rx_start = 1;
}

uint16_t buff_process_trigger_1(uint16_t *buff, int size, bool printit=true){
  uint16_t a = 0;
  for(int i=0 ; i < size/2 ; i+=2)
  a |= buff[i];
  if(printit)
    ESP_LOGD(TAG_I2S, "Process trigger 1: 0x%X", a);
  return a;
}

uint16_t buff_process_trigger_0(uint16_t *buff, int size, bool printit=true){
  uint16_t a = 0xFF;
  for(int i=0 ; i < size/2 ; i+=2)
  a &= buff[i];
  if(printit)
    ESP_LOGD(TAG_I2S, "Process trigger 0: 0x%X", a);
  return a;
}


static esp_err_t dma_desc_init(int raw_byte_size){
    s_state = (camera_state_t*) malloc (sizeof(camera_state_t));
    assert(raw_byte_size % 4 == 0);

    ESP_LOGD(TAG_I2S, "Buffer Total (for DMA): %d bytes", raw_byte_size);
    size_t dma_desc_count = 1;
    size_t buf_size = raw_byte_size;
    /*
    while (buf_size >= 4096)
    {
        buf_size /= 2;
        dma_desc_count *= 2;
    }
    s_state->dma_buf_width = buf_size;
    s_state->dma_val_per_desc = buf_size/2;
    s_state->dma_sample_per_desc = buf_size/4;
    s_state->dma_desc_count = dma_desc_count;
*/
    s_state->dma_buf_width = buf_size = 4000;
    s_state->dma_val_per_desc = 2000;
    s_state->dma_sample_per_desc = 1000;
    s_state->dma_desc_count = dma_desc_count = raw_byte_size/4000;
    
    ESP_LOGD(TAG_I2S, "DMA buffer size: %d", buf_size);
    ESP_LOGD(TAG_I2S, "DMA buffer count: %d", dma_desc_count);
    ESP_LOGD(TAG_I2S, "DMA buffer total: %d bytes", buf_size * dma_desc_count);
    
    s_state->dma_buf = (dma_elem_t**) malloc(sizeof(dma_elem_t*) * dma_desc_count);
    if (s_state->dma_buf == NULL) {
        return ESP_ERR_NO_MEM;
    }
    s_state->dma_desc = (lldesc_t*) malloc(sizeof(lldesc_t) * dma_desc_count);
    if (s_state->dma_desc == NULL) {
        return ESP_ERR_NO_MEM;
    }
    size_t dma_sample_count = 0;
    for (int i = 0; i < dma_desc_count; ++i) {
        ESP_LOGD(TAG_I2S, "Allocating DMA buffer #%d, size=%d", i, buf_size);
        dma_elem_t* buf = (dma_elem_t*) malloc(buf_size);
        if (buf == NULL) {
            ESP_LOGD(TAG_I2S, "NO_MEM!");  
            return ESP_ERR_NO_MEM;
        }
        s_state->dma_buf[i] = buf;
        ESP_LOGV(TAG_I2S, "dma_buf[%d]=%p", i, buf);

        lldesc_t* pd = &s_state->dma_desc[i];
        pd->length = buf_size/2;
        dma_sample_count += buf_size / 2; // indeed /4 because each sample is 4 bytes
        pd->size = buf_size;
        pd->owner = 1;
        pd->sosf = 1;
        pd->buf = (uint8_t*) buf;
        pd->offset = 0;
        pd->empty = 0;
        pd->eof = 0;
        pd->qe.stqe_next = &s_state->dma_desc[(i + 1) % dma_desc_count];
        if( i+1 == dma_desc_count ){
           pd->eof = 1;
           pd->qe.stqe_next = 0x0;
           //pd->eof = 0;
           //pd->qe.stqe_next = &s_state->dma_desc[0];
          }
    }
    s_state->dma_done = false;
    s_state->dma_sample_count = dma_sample_count;
    ESP_LOGD(TAG_I2S, "DMA dma_sample_count: %d", dma_sample_count);
    return ESP_OK;
}

static void gpio_setup_in(int gpio, int sig, int inv) {
  if (gpio == -1) return;
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);
  gpio_set_direction( (gpio_num_t)gpio, (gpio_mode_t)GPIO_MODE_DEF_INPUT);
  //gpio_matrix_in(gpio, sig, inv, false);
  gpio_matrix_in(gpio, sig, false);
}

void i2s_conf_reset(){
  // Toggle some reset bits in LC_CONF register
  I2S0.lc_conf.in_rst         = 1; I2S0.lc_conf.in_rst         = 0;
  I2S0.lc_conf.ahbm_rst       = 1; I2S0.lc_conf.ahbm_rst       = 0;
  I2S0.lc_conf.ahbm_fifo_rst  = 1; I2S0.lc_conf.ahbm_fifo_rst  = 0;

  // Toggle some reset bits in CONF register
  I2S0.conf.rx_reset          = 1; I2S0.conf.rx_reset          = 0;
  I2S0.conf.rx_fifo_reset     = 1; I2S0.conf.rx_fifo_reset     = 0;
  I2S0.conf.tx_reset          = 1; I2S0.conf.tx_reset          = 0;
  I2S0.conf.tx_fifo_reset     = 1; I2S0.conf.tx_fifo_reset     = 0;
  while (I2S0.state.rx_fifo_reset_back) {}
  }

void i2s_parallel_setup(const i2s_parallel_config_t *cfg) {
  //Figure out which signal numbers to use for routing
  ESP_LOGI(TAG_I2S, "Setting up parallel I2S bus at I2S%d\n", 0);
  int sig_data_base, sig_clk;

  sig_data_base = I2S0I_DATA_IN0_IDX;
  sig_clk       = I2S0I_WS_IN_IDX;
  //sig_clk       = I2S0I_BCK_IN_IDX;

  //Route the signals
  gpio_setup_in(cfg->gpio_bus[ 0], sig_data_base +  0, false); // D0
  gpio_setup_in(cfg->gpio_bus[ 1], sig_data_base +  1, false); // D1
  gpio_setup_in(cfg->gpio_bus[ 2], sig_data_base +  2, false); // D2
  gpio_setup_in(cfg->gpio_bus[ 3], sig_data_base +  3, false); // D3
  gpio_setup_in(cfg->gpio_bus[ 4], sig_data_base +  4, false); // HS
  gpio_setup_in(cfg->gpio_bus[ 5], sig_data_base +  5, false); // VS
  gpio_setup_in(cfg->gpio_bus[ 6], sig_data_base +  6, false); // VS
  gpio_setup_in(cfg->gpio_bus[ 7], sig_data_base +  7, false); // VS

  gpio_setup_in(cfg->gpio_bus[ 8], sig_data_base +  8, false); // VS
  gpio_setup_in(cfg->gpio_bus[ 9], sig_data_base +  9, false); // VS
  gpio_setup_in(cfg->gpio_bus[10], sig_data_base + 10, false); // VS
  gpio_setup_in(cfg->gpio_bus[11], sig_data_base + 11, false); // VS
  gpio_setup_in(cfg->gpio_bus[12], sig_data_base + 12, false); // VS
  gpio_setup_in(cfg->gpio_bus[13], sig_data_base + 13, false); // VS
  gpio_setup_in(cfg->gpio_bus[14], sig_data_base + 14, false); // VS
  gpio_setup_in(cfg->gpio_bus[15], sig_data_base + 15, false); // VS

  for (i=0;i<8;i++){
    Serial_Debug_Port.printf("%d->%d, ", cfg->gpio_bus[i], (sig_data_base + i));
  }
  Serial_Debug_Port.println();
  for (i=8;i<16;i++){
    Serial_Debug_Port.printf("%d->%d, ", cfg->gpio_bus[i], (sig_data_base + i));
  }
  Serial_Debug_Port.println();

  gpio_setup_in(cfg->gpio_clk, sig_clk, false);
  Serial_Debug_Port.printf("SIG_CLK: %d->%d\r\n", cfg->gpio_clk, sig_clk);

//gpio_matrix_in(0x38,    I2S0I_WS_IN_IDX, false);

// For I2S in parallel camera input mode, data reception starts with a falling edge on I2S0I_V_SYNC_IDX while H_SYNC = H_ENABLE = 1
// 0x3C sets signal low (0), 0x38 sets signal high (1)
  gpio_matrix_in(0x38,    I2S0I_V_SYNC_IDX, false);
//  gpio_matrix_in( 0x3C,   I2S0I_V_SYNC_IDX, false );  
  gpio_matrix_in(0x38,    I2S0I_H_SYNC_IDX, false);
  gpio_matrix_in(0x38,    I2S0I_H_ENABLE_IDX, false);

  // Enable and configure I2S peripheral
  periph_module_enable(PERIPH_I2S0_MODULE);
  
  //Initialize I2S dev
  i2s_conf_reset();               // Toggle some reset bits in LC_CONF register and CONF register
  I2S0.conf.rx_slave_mod = 1;     // Enable slave mode (sampling clock is external)

  I2S0.conf2.val = 0;             // Clear all conf2 bits
  I2S0.conf2.lcd_en = 1;          // Enable LCD mode = Enable parallel mode
  I2S0.conf2.camera_en = 1;       // Use HSYNC/VSYNC/HREF to control sampling

/*
    union {
        struct {
            uint32_t clkm_div_num: 8;                   /*Integral I2S clock divider value
            uint32_t clkm_div_b:   6;                   /*Fractional clock divider numerator value
            uint32_t clkm_div_a:   6;                   /*Fractional clock divider denominator value
            uint32_t clk_en:       1;                   /*Set this bit to enable clk gate
            uint32_t clk_sel:      2;                   /*Set this bit to enable clk_apll
            uint32_t reserved23:   9;
        };
        uint32_t val;
    } clkm_conf;
*/
  // f i2s = fpll / (Num + b/a )) where fpll=80Mhz
  // Configure clock divider
  I2S0.clkm_conf.val = 0;

#if (BOARD_esp32dev == 1)
// ESP32
// I2S_CLKA_ENA
//  Set this bit to enable APLL_CLK. Default is PLL_F160M_CLK. (R/W)
  I2S0.clkm_conf.clka_en = 0;    // select PLL_D2_CLK. Digital Multiplexer that select between APLL_CLK or PLL_D2_CLK.
//I2S0.clkm_conf.clk_en = 1;
#else // (BOARD_esp32s2 == 1)
// ESP32S2
// I2S_CLK_SEL 
// Set these bits to select I2S module clock source:
//    0: No clock. 1: APLL_CLK. 2:PLL_160M_CLK. 3: No clock. (R/W)
  I2S0.clkm_conf.clk_sel = 2;
#endif
  
  I2S0.clkm_conf.clkm_div_a = 1;    // Fractional clock divider denominator value
  I2S0.clkm_conf.clkm_div_b = 0;    // Fractional clock divider numerator value
  I2S0.clkm_conf.clkm_div_num = 4;  // Integral I2S clock divider value
  
  /*
  I2S0.clkm_conf.clkm_div_a = 3;//  24Mhz
  I2S0.clkm_conf.clkm_div_b = 1;
  I2S0.clkm_conf.clkm_div_num = 3;
  */
  
  // FIFO will sink data to DMA
  I2S0.fifo_conf.dscr_en = 1;       
  
  // FIFO configuration
  //I2S0.fifo_conf.rx_fifo_mod = s_state->sampling_mode;
  I2S0.fifo_conf.rx_fifo_mod = 1;   // SM_0A0B_0C0D = 1,
  I2S0.fifo_conf.rx_fifo_mod_force_en = 1;
  //dev->conf_chan.val = 0;

  I2S0.conf_chan.rx_chan_mod = 1;   // Store right channel data only
  // Clear flags which are used in I2S serial mode
  I2S0.sample_rate_conf.rx_bits_mod = 0;
  I2S0.conf.rx_right_first          = 1;
  I2S0.conf.rx_msb_right            = 0;
  I2S0.conf.rx_msb_shift            = 0;
  I2S0.conf.rx_mono                 = 1;
  I2S0.conf.rx_short_sync           = 1;
//  I2S0.conf.rx_mono = 0;
//  I2S0.conf.rx_short_sync = 0;

  I2S0.timing.val = 0;              // No delays on any of the timing sequences
//  I2S0.timing.rx_dsync_sw = 1;

  I2S0.sample_rate_conf.val = 0;
  // Clear flags which are used in I2S serial mode
  //I2S0.sample_rate_conf.rx_bits_mod = 8;
  I2S0.sample_rate_conf.rx_bits_mod = 16;
  //dev->sample_rate_conf.rx_bck_div_num = 16;  //ToDo: Unsure about what this does...
  I2S0.sample_rate_conf.rx_bck_div_num = 1;     // datasheet says this must be 2 or greater (but 1 seems to work)
  
  // this combination is 20MHz
  //dev->sample_rate_conf.tx_bck_div_num=1;
  //dev->clkm_conf.clkm_div_num=3; // datasheet says this must be 2 or greater (but lower values seem to work)

  //Allocate DMA descriptors
  i2s_state[0]                = (i2s_parallel_state_t*)malloc(sizeof(i2s_parallel_state_t));
  i2s_parallel_state_t *st    = i2s_state[0];
  s_state->dma_done           = false;
  s_state->dma_desc_cur       = 0;
  s_state->dma_received_count = 0;
  s_state->dma_filtered_count = 0;
  //esp_intr_disable(s_state->i2s_intr_handle);
  // i2s_conf_reset();

  ESP_LOGD(TAG_I2S, "dma_sample_count: %d", s_state->dma_sample_count);      // The length of data to be received
  I2S0.rx_eof_num             = s_state->dma_sample_count;
  I2S0.in_link.addr           = (uint32_t) &s_state->dma_desc[0];
  I2S0.in_link.start          = 1;                                            // Start the inlink descriptor
  I2S0.int_clr.val            = I2S0.int_raw.val;                             // Clear interrupt flags
  I2S0.int_ena.val            = 0;                                            // Disable all interrupts
  I2S0.int_ena.in_done        = 1;

  //Setup I2S DMA Interrupt
  esp_err_t err = esp_intr_alloc( ETS_I2S0_INTR_SOURCE,
                    ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM,
                    &i2s_isr,  NULL, &s_state->i2s_intr_handle );

  //Enable the Interrupt
  //ESP_ERROR_CHECK(esp_intr_enable(s_state->i2s_intr_handle));
  //  I2S0.conf.rx_start = 1;                                                 // Start I2S & the DMA
}

static bool setSampleRate( int freq_in_hz ) {
  //  ledcSetup(0, freq_in_hz, 1);
  //  ledcAttachPin(cfg.gpio_clk, 0);
  //  ledcWrite( 0, 1);
  //  delay(10);

  int freq=0; int resn = 1; int duty;
  do {
    duty = (1<<(resn - 1));
    freq = ledcSetup(0, freq_in_hz, resn);
    ledcWrite( 0, duty);
    Serial_Debug_Port.printf("Duty,Resolution: %d,%d => frequency: %d\r\n", duty, resn, freq);
  } while ((freq == 0) && (resn++<=20));
  if (freq == 0) return (false);
  else {
    ledcAttachPin(cfg.gpio_clk, 0);
    duty = ledcRead(0);
    freq = ledcReadFreq(0);
    Serial_Debug_Port.printf("Pin,targetFreq,ledcFreq: %d,%d,%d (Resn,Duty: %d,%d)\r\n", cfg.gpio_clk, freq_in_hz, freq, resn, duty);
    return (true);
  }
    /*
    esp_err_t err;
    periph_module_enable(PERIPH_LEDC_MODULE);
    ledc_timer_config_t timer_conf;
    timer_conf.bit_num = LEDC_TIMER_1_BIT;
    //timer_conf.freq_hz = I2S_HZ;
    timer_conf.freq_hz = freq_in_hz;
    timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    timer_conf.timer_num = LEDC_TIMER_0;
    err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed, rc=%x", err);
    }

    ledc_channel_config_t ch_conf;
    ch_conf.channel = LEDC_CHANNEL_0;
    ch_conf.timer_sel = LEDC_TIMER_0;
    ch_conf.intr_type = LEDC_INTR_DISABLE;
    ch_conf.duty = 1;
    ch_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    ch_conf.gpio_num = CLK_PIN; //s_config.pin_xclk; 
    err = ledc_channel_config(&ch_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed, rc=%x", err);
    }
    */
}

static bool setTestOutput( int freq_in_hz, int pin ) {
  int freq=0; int resn = 1; int duty;
  do {
    duty = (1<<(resn - 1));
    freq = ledcSetup(0, freq_in_hz, resn);
    ledcWrite( 0, duty);
    Serial_Debug_Port.printf("Duty,Resolution: %d,%d => frequency: %d\r\n", duty, resn, freq);
  } while ((freq == 0) && (resn++<=20));
  if (freq == 0) return (false);
  else {
    ledcAttachPin(pin, 0);
    duty = ledcRead(0);
    freq = ledcReadFreq(0);
    Serial_Debug_Port.printf("Pin,targetFreq,ledcFreq: %d,%d,%d (Resn,Duty: %d,%d)\r\n", pin, freq_in_hz, freq, resn, duty);
    return (true);
  }
}
