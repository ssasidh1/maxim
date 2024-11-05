/*
 * Copyright (c) 2022 - 2024, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <zephyr/kernel.h>
#include <nrfx_example.h>
#include <nrfx_spim.h>
#include <zephyr/drivers/gpio.h>
#include <includes/MAX30003.h>
#include <nrfx_log.h>
#include <zephyr/sys/time_units.h>
#include <stdlib.h>
#include <zephyr/timing/timing.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/BLE.h>
// #include <zephyr/sys/printk.h>
#define NRFX_LOG_MODULE                 EXAMPLE
#define NRFX_EXAMPLE_CONFIG_LOG_ENABLED 1
#define NRFX_EXAMPLE_CONFIG_LOG_LEVEL   3

// SPIM pin initializaiton 
#define SPIM_INST_IDX 1
#define MO_PIN 3
#define MI_PIN 31
#define SCLK_PIN 4
#define MSG_TO_SEND "Nordic Semiconductor"
#define SS_PIN 27
nrfx_spim_t spim_inst = NRFX_SPIM_INSTANCE(SPIM_INST_IDX);
static const struct gpio_dt_spec intB_gpio_pin =   GPIO_DT_SPEC_GET_OR(DT_NODELABEL(gpiocust1), gpios, {0});
static const struct gpio_dt_spec intB_gpio_pin_time =   GPIO_DT_SPEC_GET_OR(DT_NODELABEL(gpiocust2), gpios, {0});
// static const struct gpio_dt_spec MISO =   GPIO_DT_SPEC_GET_OR(DT_NODELABEL(gpiocust4), gpios, {0});
// static const struct gpio_dt_spec MOSI =   GPIO_DT_SPEC_GET_OR(DT_NODELABEL(gpiocust3), gpios, {0});


// Maxim30003 bit constants
int EINT_STATUS =  1 << 23;
int RTOR_STATUS =  1 << 10;
int DCL_OFF = 1<<20;
int DCL_ON = 1<<11; 
int RTOR_REG_OFFSET = 10;
float RTOR_LSB_RES = 0.0078125f;
int FIFO_OVF =  0x7;
int FIFO_VALID_SAMPLE =  0x0;
int FIFO_FAST_SAMPLE =  0x1;
int ETAG_BITS = 0x7;
int FIFO_EMPTY = 0x6;

#define ADC_NUMBER_OF_BITS   (18u)
#define ADC_SIGN_MASK        (1u << (ADC_NUMBER_OF_BITS-1))
#define ADC_NUMBER_MASK      (ADC_SIGN_MASK-1u)
#define ADC_FIELD_MASK       (ADC_SIGN_MASK | ADC_NUMBER_MASK)

//Maxim registers
union GeneralConfiguration_u CNFG_GEN_r;
union CalConfiguration_u CAL_CONFG_r;
  union EnableInterrupts_u EN_INT_r;
	union ECGConfiguration_u CNFG_ECG_r;
   	union RtoR1Configuration_u CNFG_RTOR_r;
    union ManageInterrupts_u MNG_INT_r;
     union ManageDynamicModes_u MNG_DYN_r;
    union MuxConfiguration_u CNFG_MUX_r;
// ECG related buffers and variables
volatile bool ecgIntFlag = false;
volatile uint8_t m_tx_buffer[4];
volatile uint8_t m_rx_buffer[4];
static uint8_t ecg_tx_buffer[13];
static uint8_t ecg_rx_buffer[13];

static uint8_t txB[1];
static uint8_t rxB;
uint32_t RtoR, idx, stat;

uint8_t ETAG[32];
uint32_t ecgFIFO;
int32_t ecgSample[17];
float BPM=0.0f;

volatile bool spiXferComplete = false;

// #define nodeId DT_NODELABEL(test)
// timing vars
int64_t ecg_start, ecg_end, ecg_start_m, ecg_end_m;
  

void spin(){
    while(!spiXferComplete){
      // printk("\n spinning \n");
    }
    spiXferComplete = false;
}
void writeRegister(enum Registers_e reg, const uint32_t data)
{
    
	m_tx_buffer[0] = reg << 1;
	m_tx_buffer[1] = ((0x00FF0000 & data) >> 16);
	m_tx_buffer[2] = ((0x0000FF00 & data) >> 8);
	m_tx_buffer[3] = ( 0x000000FF & data);
    nrfx_spim_xfer_desc_t spim_xfer_desc = NRFX_SPIM_XFER_TRX(m_tx_buffer, sizeof(m_tx_buffer), m_rx_buffer, sizeof(m_rx_buffer));
    gpio_pin_set_dt(&intB_gpio_pin_time, 1);
    nrfx_spim_xfer(&spim_inst, &spim_xfer_desc, 0);
    
    spin();
    gpio_pin_set_dt(&intB_gpio_pin_time, 0);
    k_msleep(100);
    printk("\n");
}

uint32_t readRegister(enum Registers_e reg)
{
  
  nrfx_err_t err;
    uint32_t data = 0;
	m_tx_buffer[0] = ((reg << 1) | 1);
    m_tx_buffer[1] = 0x00;
    m_tx_buffer[2] = 0x00;
    m_tx_buffer[3] = 0x00;

    m_rx_buffer[0] = 7;
    m_rx_buffer[1] = 7;
    m_rx_buffer[2] = 7;
    m_rx_buffer[3] = 7;
    txB[0] = m_tx_buffer[0];
    gpio_pin_set_dt(&intB_gpio_pin_time, 1);
    nrfx_spim_xfer_desc_t spim_xfer_desc = NRFX_SPIM_XFER_TRX(m_tx_buffer, sizeof(m_tx_buffer), m_rx_buffer, sizeof(m_rx_buffer));
    nrfx_spim_xfer(&spim_inst, &spim_xfer_desc, 0);
    spin();
    gpio_pin_set_dt(&intB_gpio_pin_time, 0);
	data |= m_rx_buffer[1] ;
    data = data << 8;
    data |= m_rx_buffer[2] ;
    data = data << 8;
     data |= m_rx_buffer[3] ;
    //  k_msleep(100);
     //printk("\ndata rxd %u %u %u %u\n", m_rx_buffer[0],m_rx_buffer[1],m_rx_buffer[2],m_rx_buffer[3]);
    return data;
   
}


uint32_t* readECG(enum Registers_e reg)
{
    ecg_tx_buffer[0] = ((reg << 1) | 1);
    nrfx_spim_xfer_desc_t spim_xfer_desc = NRFX_SPIM_XFER_TRX(ecg_tx_buffer, sizeof(ecg_tx_buffer), ecg_rx_buffer, sizeof(ecg_rx_buffer));
    nrfx_spim_xfer(&spim_inst, &spim_xfer_desc, 0);
    uint32_t* data = malloc(sizeof(uint32_t)* 4);
    for(int i = 0; i<4  ; i++){
        uint32_t dIn = 0;
        dIn |= ecg_rx_buffer[(3*i)+1] ;
        dIn = dIn << 8;
        dIn |= ecg_rx_buffer[(3*i)+2] ;
        dIn = dIn << 8;
        dIn |= ecg_rx_buffer[(3*i)+3] ;
        data[i] = dIn;
    }
    return data;
}

  
    // union ECG_FIFO_BURST

void regCheck(){
    EN_INT_r.all = 0x80000;
  writeRegister( EN_INT , EN_INT_r.all);
    printk("%x\n",readRegister(EN_INT));
    NRFX_EXAMPLE_LOG_PROCESS();
}


void ecg_config2(){
  writeRegister( SW_RST , 0);
  CNFG_GEN_r.all = 0x80000;
    writeRegister( CNFG_GEN , 0x80000);
    CAL_CONFG_r.all = 0;
      writeRegister( CNFG_CAL , 0);
        CNFG_MUX_r.all = 0;
        writeRegister( CNFG_EMUX , 0);
        CNFG_ECG_r.all = 0x805000;
          writeRegister( CNFG_ECG , 0x805000);
          CNFG_RTOR_r.all = 0x3F6300;
            writeRegister( CNFG_RTOR1 , 0x3F6300); 
            writeRegister( CNFG_RTOR2 , 0x202400);
            //  writeRegister( CNFG_RTOR1 , 0x3F6300);
}
void ecg_config() { 
    writeRegister( SW_RST , 0);
     
    // General config register setting
    
    CNFG_GEN_r.bits.en_ecg = 1;     // Enable ECG channel
    CNFG_GEN_r.bits.rbiasn = 1   ;  // Enable resistive bias on negative input
    CNFG_GEN_r.bits.rbiasp = 1;     // Enable resistive bias on positive input
    CNFG_GEN_r.bits.en_rbias = 1;   // Enable resistive bias
     CNFG_GEN_r.bits.imag = 2;       // Current magnitude = 10nA
    // CNFG_GEN_r.bits.fmstr = 0;
    CNFG_GEN_r.bits.en_dcloff = 1;  // Enable DC lead-off detection   
    //printk("CNFG_GEN_r.all %x\n",CNFG_GEN_r.all);
    //CNFG_GEN_r.all = 0x081007;
    writeRegister( CNFG_GEN , CNFG_GEN_r.all);
 
    
    // ECG Config register setting
   
    CNFG_ECG_r.bits.dlpf = 1;       // Digital LPF cutoff = 40Hz
    CNFG_ECG_r.bits.dhpf = 1;       // Digital HPF cutoff = 0.5Hz
    CNFG_ECG_r.bits.gain = 3;       // ECG gain = 160V/V, 3
    CNFG_ECG_r.bits.rate = 2;   //chng ss from 2(128sps) to 0(512sps)    // Sample rate = 128 sps
    //CNFG_ECG_r.all = 0x005000;
    writeRegister( CNFG_ECG , CNFG_ECG_r.all);
    //printk("CNFG_ECG_r.all %u\n",CNFG_ECG_r.all);  

 
    
   
    CNFG_RTOR_r.bits.wndw = 0b0011;         // WNDW       = 96ms
    CNFG_RTOR_r.bits.en_rtor = 1; // enabling rtor detection 
    CNFG_RTOR_r.bits.rgain = 0b1111;        // Auto-scale gain
    CNFG_RTOR_r.bits.pavg = 0b11;           // 16-average
    CNFG_RTOR_r.bits.ptsf = 0b0011;         // PTSF = 4/16
    CNFG_RTOR_r.bits.en_rtor = 1;           // Enable R-to-R detection 1->0
    writeRegister( CNFG_RTOR1 , CNFG_RTOR_r.all);
     
        
    //Manage interrupts register setting
    
    MNG_INT_r.bits.efit = 1;//;          // Assert EINT w/ 4 unread samples
    MNG_INT_r.bits.clr_rrint = 1;// Clear R-to-R on RTOR reg. read back
    //  MNG_INT_r.bits.clr_samp= 1;
    writeRegister( MNGR_INT , MNG_INT_r.all);
  
    // printk("MNG_INT_r.all %u\n",MNG_INT_r.all);
    
    
    //Enable interrupts register setting
    
    EN_INT_r.bits.en_eint = 1;              // Enable EINT interrupt
    // EN_INT_r.bits.en_dcloffint = 0; // ak
    // EN_INT_r.bits.en_loint = 0 ;//ak
    // EN_INT_r.bits.en_pllint = 1 ;//ak
// EN_INT_r.bits.en_samp = 0;
    EN_INT_r.bits.en_rrint = 1;             // Enable R-to-R interrupt
    EN_INT_r.bits.intb_type = 3;            // Open-drain NMOS with internal pullup
    writeRegister( EN_INT , EN_INT_r.all);
  
       
    //Dyanmic modes config
    MNG_DYN_r.bits.fast = 0;                // Fast recovery mode disabled
    writeRegister( MNGR_DYN , MNG_DYN_r.all);
    // MUX Config
    CNFG_MUX_r.bits.openn = 0;          // Connect ECGN to AFE channel ss-> (0->1)
    CNFG_MUX_r.bits.openp = 0;          // Connect ECGP to AFE channel ss-> (0->1)
    //ss
    // CNFG_MUX_r.bits.caln_sel = 1;
   writeRegister( CNFG_EMUX , CNFG_MUX_r.all);

// //ss
//    union CalConfiguration_u CAL_CONFG_r;
//    CAL_CONFG_r.bits.en_vcal = 1;
//    CAL_CONFG_r.bits.vmode = 0;
//    CAL_CONFG_r.bits.fcal = 1;
//    writeRegister( CNFG_CAL , CAL_CONFG_r.all);
    
    return;
}  
static struct gpio_callback maxim_intB;

void maxim_interrupt(){
    // ecg_start = k_uptime_get();
    //printk("\nintterupted\n");
    ecgIntFlag = true; 
    
}

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define CON_STATUS_LED DK_LED2
#define RUN_LED_BLINK_INTERVAL 25

#define USER_LED DK_LED3

#define USER_BUTTON DK_BTN1_MSK

static bool app_button_state;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LBS_VAL),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err %u)\n", err);
		return;
	}

	printk("Connected\n");

	//dk_set_led_on(CON_STATUS_LED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    
	printk("Disconnected (reason %u)\n", reason);

	//dk_set_led_off(CON_STATUS_LED);
}
BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
#ifdef CONFIG_BT_LBS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};
void configureGPIO()
{
	int ret;
	ret = gpio_pin_configure_dt(&intB_gpio_pin, GPIO_INPUT );
    ret = gpio_pin_configure_dt(&intB_gpio_pin_time, GPIO_OUTPUT );
    bool status = false;
    ret = gpio_pin_interrupt_configure_dt(&intB_gpio_pin, GPIO_INT_EDGE_TO_ACTIVE );
    gpio_init_callback(&maxim_intB, maxim_interrupt, BIT(intB_gpio_pin.pin));
    gpio_add_callback(intB_gpio_pin.port, &maxim_intB);
}
static void spi_event_handler(nrfx_spim_evt_t const *event, void *context)
{
  if(event->type == NRFX_SPIM_EVENT_DONE)
  {
    spiXferComplete = true;
    //printk("\n event done \n");
  }
}
int main(void)
{
    nrfx_err_t status1;
    (void) status1;
   int err;
#if defined(__ZEPHYR__)
    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIM_INST_GET(SPIM_INST_IDX)), IRQ_PRIO_LOWEST,
                NRFX_SPIM_INST_HANDLER_GET(SPIM_INST_IDX), 0, 0);
#endif

    
    NRFX_EXAMPLE_LOG_INIT();

		err = bt_enable(NULL);
		if (err) {
			printk("Bluetooth init failed (err %d)\n", err);
			return;
		}

		printk("Bluetooth initialized\n");

		if (IS_ENABLED(CONFIG_SETTINGS)) {
			settings_load();
		}


		err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
		if (err) {
			printk("Advertising failed to start (err %d)\n", err);
			return;
		}

		printk("Advertising successfully started\n");  
        printk("---------------------main funciton-------------\n");
        NRFX_EXAMPLE_LOG_PROCESS();
    
    nrfx_spim_config_t spim_config = NRFX_SPIM_DEFAULT_CONFIG(SCLK_PIN,
                                                              MO_PIN,
                                                              MI_PIN,
                                                              27);
                   // spim_config.miso_pull      = NRF_GPIO_PIN_PULLDOWN;
                    spim_config.frequency = 125000;
                  //  spim_config.orc = 0x00;
                    spim_config.mode = NRF_SPIM_MODE_0;
    status1 = nrfx_spim_init(&spim_inst, &spim_config, spi_event_handler, NULL);
    NRFX_ASSERT(status1 == NRFX_SUCCESS);
    configureGPIO();
 
//     writeRegister( SW_RST , 0);
//     //EN_INT_r.bits.en_eint = 1;
// EN_INT_r.all = 0x00800000;
// //     // writeRegister( EN_INT , EN_INT_r.all);
   
//    // writeRegister( EN_INT , EN_INT_r.all);
//      writeRegister( SYNCH, 0);
// //     readRegister(EN_INT);
//     while(true){
//         // printk("\n ")
        
//         regCheck();
      
     
//     }
     ecg_config();
    //printk("MNG_INT_r.all %u\n",MNG_INT_r.all);
    //while(1){}


//     bool state = false;
//     int ecg_count = 0;
    
    
//     NRFX_EXAMPLE_LOG_PROCESS();
//     ecg_start = 0;
   
//    int toggle = 0;
//     uint32_t arr ;
//         int pllCheck =0;
//     writeRegister( SYNCH , 0);
    // while (  true  ){ 
        
    // uint32_t en_int = readRegister(EN_INT);
    // uint32_t rtor = readRegister(CNFG_RTOR1);
    // uint32_t mngr_int = readRegister(MNGR_INT);
    // uint32_t cnfg_gen = readRegister(CNFG_GEN);
    // uint32_t cnfg_ecg = readRegister(CNFG_ECG);

    // printk("\nEN_INT: read -> %x written -> %x \n",en_int, EN_INT_r.all);
    // printk("\nCNFG_RTOR1: read -> %x written -> %x\n",rtor, CNFG_RTOR_r.all);
    // printk("\nMNGR_INT: read -> %x writetn -> %x\n",mngr_int, MNG_INT_r.all);
    // printk("\nCNFG_GEN: read -> %x written -> %x\n",cnfg_gen,  CNFG_GEN_r.all);
    // printk("\nCNFG_ECG: read -> %x written -> %x\n",cnfg_ecg, CNFG_ECG_r.all);

//   if(en_int == EN_INT_r.all && rtor == CNFG_RTOR_r.all && mngr_int == MNG_INT_r.all && cnfg_gen == CNFG_GEN_r.all && cnfg_ecg ==  CNFG_ECG_r.all ){
//         printk("All registers done");
//         break;
//     }
    

    // while(true){
        
    //     arr = readRegister(STATUS );

        
    //     if((arr & 256)){
    //     printk("\nPLL Interrupt - pre SYNCH\n");
    //         // break;
    //     }
        
    //     else {
    //            printk(" \n No PLL \n");
    //         break;
         
    //     }
    // }
       
         NRFX_EXAMPLE_LOG_PROCESS();
  
   //// writeRegister( SYNCH , 0);
   
        
// break;
   
	while (1) 
	{
    //   printk("\n looping \n");
        NRFX_EXAMPLE_LOG_PROCESS();

		if(intB_gpio_pin.port)
		{
            
           // printk("\necg %d\n",ecgIntFlag);
                 
 			if(ecgIntFlag) 
			{
                // Samples obtained in 1000 ms
                
                // if(ecg_end-ecg_start >= 1000){
                //     printk("{time for %d in ms = %lld}\n",ecg_count,ecg_end-ecg_start);
                //     ecg_count = 0;
                //     ecg_start = k_uptime_get();
                // }
                uint8_t sampleCount = 0;
				ecgIntFlag = false;
				uint32_t cond_bits = readRegister( STATUS );      // Read the STATUS register
/*              //  if((cond_bits & 256)){
                // printk("\nPLL INTERRUPT\n");

                // 	writeRegister( SYNCH , 0);
                // }
				// NRFX_ASSERT(!cond_bits & DCL_OFF);
                // NRFX_ASSERT(!cond_bits & DCL_ON);
                    // while(true){
                    //     if((cond_bits & 256)){
                    //    // printk("\nPLL Interrupt -Inside\n");

                    //         // break;
                    //     }
                        
                    //     else {
                    //         //printk(" \n No PLL \n");
                    //         break;
                        
                    //     }
                     //   cond_bits = readRegister( STATUS );
                    //}

*/
				if( ( cond_bits & RTOR_STATUS ) == RTOR_STATUS ){           
					RtoR = readRegister( RTOR ) >>  RTOR_REG_OFFSET;   
					BPM = 1.0f / ( RtoR * RTOR_LSB_RES / 60.0f );   
                }  
				// Check if EINT interrupt asserted
				if ( ( cond_bits & EINT_STATUS ) == EINT_STATUS ) {  
                    
                        // uint32_t* ecg = readECG( ECG_FIFO_BURST );       // Read FIFO
                        // for(int i = 0; i < 4; i++){
                        //     ecgFIFO = ecg[i];
                        //     ecgSample[sampleCount] = ( ecgFIFO & 0x00FFFFFF ) >> 6;
                        //     if( ecgSample[sampleCount]  & 0x0002FFFF )   ecgSample[sampleCount] = ecgSample[sampleCount] | 0xFFFC0000;

                        //     ETAG[sampleCount] = ( ecgFIFO >> 3 ) & ETAG_BITS;  // Isolate ETAG
                        // // printk("\necgFIFO %x - %x - %x %d\n", ecgFIFO, (( ecgFIFO >> 3 )), ( ecgFIFO >> 3 ) & ETAG_BITS, ETAG[sampleCount]);
                        // //  printk("\nuint32 fifo %x 18bits ECG - %x ETAG - %u\n",ecgFIFO, ecgSample[sampleCount], ETAG[sampleCount] );
                        //     // printk("\n%x -> %x|0x%u %u\n", ecgFIFO, ecgSample[sampleCount],ETAG[sampleCount] , sampleCount);
                        //     printk("\n%d", ecgSample[sampleCount]  );
                        
                        // sampleCount++;   
                        // }      
					do {
						                                 // Increment sample counter                                          
                        ecgFIFO = readRegister( ECG_FIFO );
                        // int32_t ecgVal = (m_rx_buffer[1]<<16 ) | (m_rx_buffer[2] << 8) | (m_rx_buffer[3] & 0b11000000);
                        // if ( ecgVal & ADC_SIGN_MASK )
                        // {
                        //     // Sign extend the -ve 24-bit value to a -ve 32-bit value
                        //     ecgVal |= ~(int32_t)ADC_NUMBER_MASK;
                        // }
                        // int32_t ecgData = (signed long) ecgVal;
                        // int32_t d = (int32_t)m_rx_buffer[1] << 16;
                        // d = d | ((m_rx_buffer[2] << 8) & 0x0000FF00);
                        // d = d | ((m_rx_buffer[3] ) & 0x000000FF);

                            // ecgSample[sampleCount] = ecgFIFO  >> 8;
                        //     uint32_t temp =  ( ecgFIFO & 0x00FFFFFF ) >> 6;
                        //     //  ecgSample[sampleCount] =  temp;
                        // //  //   printk(" %u-",temp);
                        //     if ( temp >= 131072)
                        //     temp =  temp - 262144 ;
                        // //  //   printk(" %u-",temp);
                            ecgSample[sampleCount] = ecgFIFO >> 8 ; //ecgVal;
                            //if( ecgSample[sampleCount]  & 0x00020000 )   ecgSample[sampleCount] = ecgSample[sampleCount] | 0xFFFC0000;
                            
                       //printk("%x-%x-%d\n", ecgFIFO & 0x00FFFFFF , (ecgFIFO & 0x00FFFFFF) >> 6,  ecgSample[sampleCount]);         
                            ETAG[sampleCount] = ( ecgFIFO >> 3 ) & ETAG_BITS;  // Isolate ETAG
                        // printk("\necgFIFO %x - %x - %x %d\n", ecgFIFO, (( ecgFIFO >> 3 )), ( ecgFIFO >> 3 ) & ETAG_BITS, ETAG[sampleCount]);
                        //  printk("\nuint32 fifo %x 18bits ECG - %x ETAG - %u\n",ecgFIFO, ecgSample[sampleCount], ETAG[sampleCount] );
                            // printk("\n%x -> %x|0x%u %u\n", ecgFIFO, ecgSample[sampleCount],ETAG[sampleCount] , sampleCount);
                            //printk("\n%d", ecgSample[sampleCount]);
                     printk(" %d\n",ecgSample[sampleCount]);//,   ecgSample[sampleCount] )  ;
                        
                      NRFX_EXAMPLE_LOG_PROCESS();
   
                        sampleCount++;   
                        // ecg_count++;
                    //  printk("\nFifo overflow check  %u\n",  sampleCount);
                    //   NRFX_EXAMPLE_LOG_PROCESS();
                        // if(sampleCount > 32){
                        //     printk("\nFifo overflow %u\n",  sampleCount);
                        //     //  NRFX_EXAMPLE_LOG_PROCESS();
                        //     break;
                        // }
                        
                       
       
					} while ( ETAG[sampleCount-1] == FIFO_VALID_SAMPLE || 
							ETAG[sampleCount-1] == FIFO_FAST_SAMPLE); 
                    //   printk("\n size : %d\n", sampleCount);
                    // printk("\n ecg read %u \n",sampleCount);
					if( ETAG[sampleCount - 1] == FIFO_OVF ){    
                         printk("\n writing FIFO RST \n");              
						writeRegister( FIFO_RST , 0); // Reset FIFO
                            // writeRegister( SYNCH , 0);
                    
					}

                    for( idx = 0; idx < sampleCount; idx++ ) {
                          printk("%6d\r\n", ecgSample[idx]);           
                        
                    }
                    // if(cN==1){
                    //    // printk("Notified enabled\n");
                    //     notifyECG(ecgSample, sampleCount);
                    //     }
                    // if( ETAG[sampleCount - 1] == FIFO_EMPTY ){
                    //       printk("\n  FIFO empty/invalid \n");
                    // }

                   
                  
					
				}
               
               
        	}
            //  printk("\n ENDIF\n");
            //  NRFX_EXAMPLE_LOG_PROCESS();
		}
        // else printk("\n port error\nn");
          
    }
    printk("\n while broken \n");
}
