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
// #include <zephyr/irq.h>

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
#include <nrfx_rtc.h>
#include <nrfx_timer.h>
// #include <zephyr/sys/printk.h>
//-- ECG IMPORTS FROM NORDIC VERSION ---
/* STEP 4.1 - Define the SAADC sample interval in microseconds */
#define SAADC_SAMPLE_INTERVAL_US 2500
//RING_BUF_DECLARE(my_ring_buf, MY_RING_BUF_WORDS);
/* STEP 5.1 - Define the buffer size for the SAADC */
#define SAADC_BUFFER_SIZE   10
#define RESOLUTION NRF_SAADC_RESOLUTION_12BIT
#define STACKSIZE 1024
#define THREAD_HR 4
#define THREAD_HRV 3
uint32_t volatile countSAADC = 0, countLoop = 0, ctick=0, cInt = 0,hrv_set=0;
/* STEP 4.2 - Declaring an instance of nrfx_timer for TIMER2. */
//const nrfx_rtc_t timer_instance = NRFX_RTC_INSTANCE(2);

/* STEP 5.2 - Declare the buffers for the SAADC */
// static nrf_saadc_value_t saadc_sample_buffer[2][SAADC_BUFFER_SIZE];
//volatile uint16_t saadc_sample_buffer[2][SAADC_BUFFER_SIZE];
//struct k_sem hrv;
//K_SEM_DEFINE(hrv,0,1);

/* STEP 5.3 - Declare variable used to keep track of which buffer was last assigned to the SAADC driver */
static uint32_t saadc_current_buffer = 0;
timing_t start_time, end_time,start_time_tick,end_time_tick, start_time_int,end_time_int,mainLoop, endLoop ;
    uint64_t total_cycles;
    uint64_t total_ns;
	   uint64_t total_cycles_int;
    uint64_t total_ns_int;
	   uint64_t total_cycles_tick;
    uint64_t total_ns_tick;
	uint64_t t1;
	uint64_t t2;
  #define SIMPLEPROFILE_CHAR3_LEN 20
#define SIMPLEPROFILE_CHAR1_LEN 10
#define SIMPLEPROFILE_CHAR2_LEN 4
#define SIMPLEPROFILE_CHAR4_LEN 1
#define SIMPLEPROFILE_CHAR5_LEN 3

static int tempTrigCount = 0;
static void resetAll(void);

#define PEAK_BUFFER_SIZE 2//64
#define ECG_BUFFER_SIZE 2048

#define RISING_SLP  1
#define FALLING_SLP -1
#define DEFAULT_SLP 0

#define NUM_HR_VARS 10 //in terms of bytes, each var is converted from uint16_t to uint8_t
#define NNint_BUF_SIZE 600//1200 //in Bytes (uint8_t)

#define NORMALISE_CONSTANT  1000//350
#define HR_VAR_PKT_HEADER   7

static int slope =DEFAULT_SLP;

    

static struct peaks {
    uint16_t peakVal;
    uint16_t position;
}peakBuff[PEAK_BUFFER_SIZE];//8];//PEAK_BUFFER_SIZE];
/*
static struct altBuff {
    uint16_t ecgVal;
    uint16_t position;
}adcBuff_alt[512];//2048];
*/
static void SleepyTime(uint8_t);
static uint16_t adcBuff[ECG_BUFFER_SIZE];//2560];//3072];//2048];//1536];//2*SCIF_ADC_DATA_STREAMER_BUFFER_SIZE];//SCIF_ADC_DATA_LOGGER_BUFFER_SIZE];
static uint8_t TX_Buff[SIMPLEPROFILE_CHAR3_LEN];

static uint8_t TX_Buff_HR[SIMPLEPROFILE_CHAR1_LEN];
//static uint16_t TX_Buff[SIMPLEPROFILE_CHAR3_LEN];

//static uint16_t TX_Buff16[SIMPLEPROFILE_CHAR3_LEN/2];
static uint8_t peakIdx = 0;
static uint16_t avgHR[5];// = 0;
static unsigned long meanNNSamples = 0;
//static unsigned long meanNNSamples_tmp = 0;
static uint16_t totalNN = 0;

static uint16_t NNint_arr[NNint_BUF_SIZE];
static uint16_t NNint_idx = 0;

static uint8_t hrVars[NUM_HR_VARS];
static uint8_t HrVarTimerFlag = 0;
//for test purpose of multiple characteristics (new design approach)
static uint8_t TX_Buff_char2[SIMPLEPROFILE_CHAR2_LEN];

//static uint16_t Detect_Peak_HR(void);//int,int);
static uint16_t min(uint16_t ,uint16_t);
//static void VerifyRR(void);
//static void SortBuff(uint16_t,uint16_t,uint16_t);//int, int);
//static int partition(int, int, int);
//static void Swap(int, int);
static uint16_t max(uint16_t, uint16_t);
//static uint16_t incr_i = 0; // incrementing step counter for test purpose only (Android host)
//static float incr_temp = 30.0; // incrementing temperature counter for test purpose only (Android host)
//static uint8_t  incr_hr = 30;   // incrementing HR counter for test purpose only (Android host)
//static uint32 battVol = 100; // battery voltage
static uint8_t AlarmsTX[SIMPLEPROFILE_CHAR5_LEN];

//static uint16_t pkt_idx = 0;
//static uint8_t *gapState;
static uint16_t ecgBuff_wrIdx = 0;
static uint16_t ecgBuff_rdIdx = 0;
//static uint16_t ecgTail = 0;
static uint16_t thermVol=0;
static uint16_t volDiv=0;

static uint8_t txStatus = 0;
//static uint8_t lookAhd = 0;

static uint8_t CONNECTED = 0;

static uint8_t magnitudeDelta;
static uint16_t positionDelta;
//static uint16_t positionDelta_avg = 0;
//static uint8_t magnitudeDelta_avg = 0;
static uint8_t prevHR =0;
static uint8_t instantHR = 0;
//static uint8_t sendECG = 0;

//static int sleep_time = 256000; // sleep time in us
static void uTaskSleep(void);
//static void controlConn(UArg);
//static void fnHrVarCalc(UArg);
static uint8_t DutyToff = 0;
static uint8_t numTx = 0;

static uint16_t R_Threshold = 0;
static uint16_t Q_Threshold = 4000;
//static uint16_t thresCounter = 0;
//static uint16_t trig_thresCounter =0;
static uint16_t ecgSamplesCount = 0;
//static uint8_t SBP_SLEEP = 0;
static uint8_t oneTimeClkShutDwn = 0;

static uint16_t avgR =0, avgQ =0, avgVpp =0, cntRR =0;
//static uint8_t HOLD_N_SEND =0;
static uint8_t FIRST_RESET =0;



#define HR_VAR_CLOCK_PERIOD 5000 // 5 secs
#define FIVE_MINUTES    60
#define ONE_MINUTE      12
#define TWO_SECONDS     5
#define ONE_SECOND     2
#define ABS_MAX_ADC     4095
#define ABS_MIN_ADC     0

//#define timeLimit(x) (x==0 ? FIVE_MINUTES : ONE_MINUTE )
#define timeLimit(x) (x==0 ? TWO_SECONDS : ONE_SECOND )
#define limit(x)    (x==0 ? 1500 : avgVpp)

#define NNSamplesToMilliSecs(x) (2.5*x)

static int TrigHrValCount = 0;

/*
 * * Duty cycle configuration values:
 * * ---------------------------------------------
 * *    value(from Host)    |    Duty Cycle (in %)
 * * ---------------------------------------------
 * *    11 - 19             |   10% - 50%
 * *    21 - 29             |   55% - 95%
 * *    31                  |   100%
 * *----------------------------------------------
 */
static uint8_t dutyConfig = 31;

static uint16_t lLimitVpp = 150;
//static uint16_t uLimitVpp = 4000;//1000;
//static uint16_t uLimitVppCnt = 0;

#define getDelta(x,y) (x>y ? x-y : y-x)
#define getDutyCycle(x) (((x>10)&&(x<20)) ? (((x%10)*5) + 5) : ((x>20)&&(x<30)) ? (((x%20)*5)+50) : (x==31) ? (((x%30)*5)+95) : 100) // x -> input: dutyConfig
#define getSleepLimit(x)    (1000 - (10*x))/x // x -> input: duty cycle val

#define TURN_OFF_BLE_ADV_OFF  0
#define TURN_ON_BLE_ADV_ON  1


static uint16_t prev_ecgSamplesCount = 0;
static long double baseline =0.0; // average of raw ecg values
volatile uint8_t POTENTIAL_ARTIFACT = 0;
static uint16_t hrTxPauseTimer = 0;

volatile uint16_t insQ = 0;
static uint16_t events;

//static uint16_t insVpp =0;
void scsEvt_Handler(int evt);
static uint8_t msgFromHost = 0;
static uint8_t currTxPwrConfig = 0;
static uint8_t NUM_PEAKS_WAIT_TO_SEND = 1;
static uint8_t peakCnt_PeakBuff = 0;
int art = -1;
void scsEvt_Handler(int evtArg)
{
    //Display_print1(dispHandle, 4, 0, "Event_triggered %d", 1);
    events |= evtArg;
   // Display_print1(dispHandle, 5, 0, "evtIdx : %d", evtIdx++);
  //  Semaphore_post(sem);
}

static uint16_t min(uint16_t x,uint16_t y){

    if(x<y)
        return x;
    else
        return y;
}

static uint16_t max(uint16_t x, uint16_t y){

    if(x>y){
        return x;
    }
    else
        return y;
}

static void resetNNint(uint8_t avgSPM)
{
    int i;

    NNint_idx = 0;//totalNN;//0;
    for (i=0;i<NNint_BUF_SIZE;i++){
        NNint_arr[i] = 0;
    }
    
}
static void resetAll()
{
    FIRST_RESET = 0;
    R_Threshold= 0;
    Q_Threshold = 4000;
    tempTrigCount = 0;
    ecgSamplesCount = 0;
    avgR =0, avgQ =0, avgVpp =0, cntRR =0;
    meanNNSamples = 0;
    //meanNNSamples_tmp = 0;
    totalNN = 0;
    resetNNint(0);
   //AK scifTaskData.adcDataStreamer.state.head = 0;
   //Ak scifTaskData.adcDataStreamer.state.tail = 0;
    ecgBuff_wrIdx = 0;
    peakBuff[0].peakVal=0;
    peakBuff[0].position =0;
    peakBuff[1].peakVal =0;
    peakBuff[1].position =0;
    peakIdx =0;
    baseline = 0;

}
static void artifactDetectSet()
{
    POTENTIAL_ARTIFACT =1;
    hrTxPauseTimer = 500; // Number of ECG samples
    peakIdx = 0;
    prevHR = 0;
   // printk("Aritfact\n");

    //scsEvt_Handler(ARTIFACT_TX_EVT);

}
static void findPeaks()
{
//printk("Entry |%d, %d, %d, %d, %d|\n",adcBuff[ecgBuff_wrIdx],adcBuff[ecgBuff_wrIdx - 1],R_Threshold,Q_Threshold,ecgBuff_wrIdx);
    if((adcBuff[ecgBuff_wrIdx] < adcBuff[ecgBuff_wrIdx - 1]) && (adcBuff[ecgBuff_wrIdx - 1] > (R_Threshold - 150)) &&
            (adcBuff[ecgBuff_wrIdx - 1] < (R_Threshold + 150)) && ((adcBuff[ecgBuff_wrIdx - 1] - Q_Threshold) >= lLimitVpp) /*&&
            (getDelta(adcBuff[ecgBuff_wrIdx - 1], insQ) <= limit(FIRST_RESET))*/ && (slope == RISING_SLP || slope == DEFAULT_SLP)) {


        slope = FALLING_SLP;


      //  if((adcBuff[ecgBuff_wrIdx - 1] - Q_Threshold) < uLimitVpp){
        //    uLimitVppCnt = 0;
           // uLimitVpp = (adcBuff[ecgBuff_wrIdx - 1] - Q_Threshold) + 250;

            peakBuff[peakIdx].peakVal = adcBuff[ecgBuff_wrIdx - 1];
            peakBuff[peakIdx].position = ecgBuff_wrIdx - 1;//ecgSamplesCount;

            if(++peakIdx >= PEAK_BUFFER_SIZE){
                peakIdx =1;//0; // Most recent previous potential peak is always in index 0

                if(peakBuff[0].position < peakBuff[1].position){
                    positionDelta = peakBuff[1].position - peakBuff[0].position;//(ECG_BUFFER_SIZE - peakBuff[1].position) + peakBuff[0].position;//abs(peakBuff[0].position - peakBuff[1].position);
                }
                else{
                    positionDelta = (ECG_BUFFER_SIZE - peakBuff[0].position) + peakBuff[1].position;//ECG_BUFFER_SIZE - (peakBuff[0].position - peakBuff[1].position); // ecgBuff_wrIdx has wrapped around the adcBuff
                }

                magnitudeDelta = getDelta(peakBuff[1].peakVal,peakBuff[0].peakVal);
                
                if(positionDelta >= 100 && positionDelta <= 800){//ECG_BUFFER_SIZE){//positionDelta <= (ECG_BUFFER_SIZE - 250) ){

                    if(magnitudeDelta <= 175){//200){//NORMALISE_CONSTANT ){//&& (peakBuff[0].peakVal > 2500 && peakBuff[1].peakVal > 2500)){



                        instantHR = 24000/positionDelta;
                        printk("prevHR=|%d||%d||%d|\n",prevHR,instantHR,getDelta(prevHR,instantHR));
                        printk("peakVals|%d, %d|\n", peakBuff[0].position, peakBuff[1].position);
                        if(prevHR == 0){
                            prevHR = instantHR;
                            peakCnt_PeakBuff++;
                            NUM_PEAKS_WAIT_TO_SEND = 1;
                        }
                        else{

                            if(getDelta(prevHR,instantHR) > 10){

                                printk("delta|%d, %d|\n", peakBuff[0].position, peakBuff[1].position);
                                artifactDetectSet();
                            }
                            else{
                                prevHR = instantHR;
                                if(++peakCnt_PeakBuff >= 3){
                                    peakCnt_PeakBuff =0;
                                    NUM_PEAKS_WAIT_TO_SEND = 0;
                                }
                            }
                        }


                        //if(POTENTIAL_ARTIFACT == 0 && NUM_PEAKS_WAIT_TO_SEND == 0){
                        prev_ecgSamplesCount = ecgSamplesCount;

                        if(cntRR > 0){
                            cntRR++;
                            if(cntRR > 5){
                                FIRST_RESET = 1;
                            }
                            avgR = (avgR*(cntRR-1) + R_Threshold)/cntRR;

                            avgQ = (avgQ*(cntRR -1) + Q_Threshold)/cntRR;
                            avgVpp = (avgVpp*(cntRR-1) + (R_Threshold -Q_Threshold))/cntRR;

                            if(cntRR >= 65000){
                                cntRR =0;
                            }
                        }
                        else{
                            cntRR++;
                        }




                        avgHR[0] = positionDelta;//(peakBuff[1].position - peakBuff[0].position); //Num of samples between the actual peaks. Always peaBuff[1] > peakBUff[0]
                        avgHR[1] = peakBuff[1].peakVal;//peakBuff[0].peakVal;
                        avgHR[2] = peakBuff[0].peakVal;//avgVpp;//R_Threshold;//peakBuff[0].position;
                        avgHR[3] = avgQ;//Q_Threshold;//peakBuff[1].peakVal;
                        avgHR[4] = ecgSamplesCount;//

                            //Making peakBuff[0] as most recent peak for next calculation/comparison
                        peakBuff[0].position = peakBuff[1].position;
                        peakBuff[0].peakVal = peakBuff[1].peakVal;

                            //Send HR (avgHR) if it changed
                            //currHR = (1000 * 60)/positionDelta;
                            /*Calculating the mean number of RR intervals, to be used for long-term(5min) HR variability Calcution */
                        //printk("potential peak %d %d",POTENTIAL_ARTIFACT,NUM_PEAKS_WAIT_TO_SEND);
                        if(POTENTIAL_ARTIFACT == 0 && NUM_PEAKS_WAIT_TO_SEND == 0){


                            NNint_arr[NNint_idx] = positionDelta;//instantHR;
                            if(++NNint_idx >= NNint_BUF_SIZE){
                                NNint_idx =0;
                            }
                            
                            totalNN++;
                            //printk("totalNN++|%d|",totalNN);
                            // meanNNSamples = ((totalNN*meanNNSamples) + positionDelta)/(++totalNN);
/*
                            meanNNSamples *= totalNN;
                            meanNNSamples += positionDelta;
                            meanNNSamples *= 1000;
                            meanNNSamples /= (++totalNN);
                            meanNNSamples /= 1000;
*/
                            if(totalNN >= 600 || HrVarTimerFlag == 1){
                                uint8_t avgSPM=0;// = 24000/meanNNSamples;  //average Samples per Minute

                                meanNNSamples = 0;//resetMeanNNSamples(avgSPM);//0;
                                totalNN = 0;//resetTotalNN(avgSPM);//0;


                                if(HrVarTimerFlag == 1){
                                    resetNNint(avgSPM);
                                    HrVarTimerFlag = 0;
                                }

                            }

                        }


                        //scsEvt_Handler(TX_HR_EVT);
                        printk("\nPA: %d, PWTS:%d\n",POTENTIAL_ARTIFACT, NUM_PEAKS_WAIT_TO_SEND);
						if(POTENTIAL_ARTIFACT == 0 && NUM_PEAKS_WAIT_TO_SEND == 0){
                		//  SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, SIMPLEPROFILE_CHAR1_LEN, TX_Buff_HR); // uncomment
                        // for(int i =0; i< 5; i++){
                        //      printk("HR %d\n", avgHR[i]);
                        // }
            //adhr
            printk("\nBPM:%d\n", 24000/avgHR[0]);
						notifyHR(avgHR, 5);
            }
					

                    }
                    else{
                    // Noise between peaks
                    // Lesser magnitude value is the potential peak. Make peakBuff[0] as most recent potential peak
                        peakBuff[0].position = max(peakBuff[0].position, peakBuff[1].position);
                        if(peakBuff[0].position == peakBuff[1].position){
                            peakBuff[0].peakVal = peakBuff[1].peakVal;
                        }
                    }
                }
                else{
                //irrelevant for HR, and collect the most recent potential peak
                    if(ecgSamplesCount - prev_ecgSamplesCount > 800)
                    {
                        peakIdx = 0;
                    }
                    peakBuff[0].position = min(peakBuff[0].position, peakBuff[1].position);
                    if(peakBuff[0].position == peakBuff[1].position){
                        peakBuff[0].peakVal = peakBuff[1].peakVal;
                    }
                }

            }
       /* }
        else if(++uLimitVppCnt >= 20){

                uLimitVppCnt =0;
                uLimitVpp += 500;
                R_Threshold = 0;
                Q_Threshold = 4000;




        }*/

    }
    else if((adcBuff[ecgBuff_wrIdx] < adcBuff[ecgBuff_wrIdx - 1]) && (slope == RISING_SLP || slope == DEFAULT_SLP)){
        slope = FALLING_SLP;
    }
    else if(adcBuff[ecgBuff_wrIdx] > adcBuff[ecgBuff_wrIdx - 1] && (slope == FALLING_SLP || slope == DEFAULT_SLP)){
        slope = RISING_SLP;
        insQ = adcBuff[ecgBuff_wrIdx - 1];
    }



}


 void sandeep(int16_t sample){
	// -- sandeep code --
	//printk("\n sandeep's code\n");

	     adcBuff[ecgBuff_wrIdx] = sample;

               if(++tempTrigCount >= 2000){ // 5000
                   tempTrigCount =0;
                  // HOLD_N_SEND =         1;


                   if((R_Threshold > (1.1*avgR) || R_Threshold < (0.9*avgR)) && FIRST_RESET == 1){
                       R_Threshold = avgR;
                   }
                   else{
                       R_Threshold = 0.95*R_Threshold;//(R_Threshold - Q_Threshold)/2; //0;
                   }
                   if((Q_Threshold > (1.1*avgQ) || Q_Threshold < (0.9*avgQ)) && FIRST_RESET == 1) {
                       Q_Threshold = avgQ;
                   }
                   else{
                       Q_Threshold = 1.05*Q_Threshold;//(R_Threshold - Q_Threshold)/2;//4000;
                   }
               }



               //ecgSamplesCount++;
               if(++ecgSamplesCount >= 65000){//sizeof(ecgSamplesCount)){
                   ecgSamplesCount = 0;
                   FIRST_RESET = 0;
               }

               if(ecgSamplesCount > 0){
                    //printk("%f\n",(baseline*(ecgSamplesCount - 1) + adcBuff[ecgBuff_wrIdx])/ecgSamplesCount);

                   //baseline = (baseline*(ecgSamplesCount - 1) + adcBuff[ecgBuff_wrIdx])/ecgSamplesCount;
                   baseline *= (ecgSamplesCount -1);
                   //printk("float1 |%Lf|\n",baseline);
                   baseline += adcBuff[ecgBuff_wrIdx];
                   //printk("float2 |%Lf|\n",baseline);
                   baseline /= ecgSamplesCount;
                  

               }

               if(hrTxPauseTimer > 0){
                   hrTxPauseTimer--;
               }
               else if(POTENTIAL_ARTIFACT){

                   POTENTIAL_ARTIFACT = 0;
                   //Util_startClock(&HrVarClock);
               }

               //look for slope change and collect potential peaks
               if(ecgBuff_wrIdx > 0){ // RAM has atleast two values to compare
                   //if(++thresCounter <= 1000) {
                  if(FIRST_RESET == 0){
                       if(adcBuff[ecgBuff_wrIdx] > R_Threshold && ((adcBuff[ecgBuff_wrIdx] - Q_Threshold) < 1500)) {
                           R_Threshold = adcBuff[ecgBuff_wrIdx];// - 49500;// - 1000;//NORMALISE_CONSTANT; // 350 is a normalizing constant chosen to be fixed

                       }
                       else if(adcBuff[ecgBuff_wrIdx] < Q_Threshold && ((R_Threshold - adcBuff[ecgBuff_wrIdx]) < 1500)) {
                           Q_Threshold = adcBuff[ecgBuff_wrIdx];

                       }

                   }
                   else{
                       if(adcBuff[ecgBuff_wrIdx] > R_Threshold && ((adcBuff[ecgBuff_wrIdx] - Q_Threshold) <= 1.2*avgVpp) &&
                               ((adcBuff[ecgBuff_wrIdx] - Q_Threshold) >= 0.6*avgVpp)) {
                           R_Threshold = adcBuff[ecgBuff_wrIdx];// - 49500;// - 1000;//NORMALISE_CONSTANT; // 350 is a normalizing constant chosen to be fixed

                       }
                       else if(adcBuff[ecgBuff_wrIdx] < Q_Threshold && ((R_Threshold - adcBuff[ecgBuff_wrIdx]) <= 1.2*avgVpp) &&
                               ((R_Threshold - adcBuff[ecgBuff_wrIdx]) >= 0.6*avgVpp)) {
                           Q_Threshold = adcBuff[ecgBuff_wrIdx];

                       }
                   }
                  /*findPeaks() will run potential and real peak detection for all that's filtered in baseline*/
                  if(FIRST_RESET > 0){
                      //uint16_t diff = adcBuff[ecgBuff_wrIdx - 1] - (uint16_t)(baseline);
                      if((adcBuff[ecgBuff_wrIdx - 1] > (uint16_t)(baseline + 100.0))){// || (adcBuff[ecgBuff_wrIdx - 1] < (uint16_t)(baseline - 100.0))){
                          findPeaks();
                      }
                        // changes baseline from 1000 - 1500 for any confusion look into sandeep original code
                      if((adcBuff[ecgBuff_wrIdx - 1] > (uint16_t)(baseline + 1500.0)) || (adcBuff[ecgBuff_wrIdx - 1] < (uint16_t)(baseline - 1500.0))){
                        //printk("outoff base|%d, %Lf|\n",adcBuff[ecgBuff_wrIdx - 1],baseline);
                          artifactDetectSet();
                      }
                      //findPeaks();
                  }
                  else{

                      if((adcBuff[ecgBuff_wrIdx ] > (ABS_MAX_ADC - 100)) || (adcBuff[ecgBuff_wrIdx] < (ABS_MIN_ADC + 100))){
                        //printk("below base|%d|\n",adcBuff[ecgBuff_wrIdx - 1]);
                          artifactDetectSet();
                      }
                      else {
                          findPeaks();
                      }
                  }



               }

            //AK    if (++tail >= SCIF_ADC_DATA_STREAMER_BUFFER_SIZE) {
            //        tail = 0;
            //    }
               if(++ecgBuff_wrIdx >= ECG_BUFFER_SIZE) {
                   ecgBuff_wrIdx = 0;

               }

	//-------------------
 }
 void HRV_parameters(){
     if(++numTx >= timeLimit(0)){
                  DutyToff = 1;
                  numTx = 0;
              }
              if(++TrigHrValCount >= timeLimit(oneTimeClkShutDwn)){ // added totalNN condition to avoid divide by zero error(in cases like artifact)--> in future donot call hrvParameter when totalNN ==0
                // printk("\nInside hrv\n");
                  TrigHrValCount = 0;

                  HrVarTimerFlag = 1;

                  if(!oneTimeClkShutDwn){
                      oneTimeClkShutDwn = 1;
                  }


                  hrVars[0] = HR_VAR_PKT_HEADER & 0xFF;
                  hrVars[1] = (HR_VAR_PKT_HEADER >> 8) & 0xFF;

                  int i;
                  for(i=0;i<totalNN;i++){
                      meanNNSamples += NNint_arr[i];
                  }
                  printk("totNN|%d| Mean|%d|",totalNN,meanNNSamples);
                  meanNNSamples /= totalNN;
                  
                  hrVars[2] = (uint8_t) meanNNSamples & 0xFF; //AVNN
                  hrVars[3] = (uint8_t) (meanNNSamples >> 8) & 0xFF;

                  //int i;
                  uint64_t sdnn = 0, rmssd = 0, pNN50 = 0;

                  for(i=0;i<totalNN;i++){

                      sdnn += pow((unsigned long)NNSamplesToMilliSecs(NNint_arr[i]) - NNSamplesToMilliSecs(meanNNSamples), 2);

                      if(totalNN > 2 && i > 0){
                          rmssd += (unsigned long) pow(NNSamplesToMilliSecs(NNint_arr[i]) - NNSamplesToMilliSecs(NNint_arr[i-1]), 2);
                          if(getDelta(NNSamplesToMilliSecs(NNint_arr[i]), NNSamplesToMilliSecs(NNint_arr[i-1])) >= 50){
                              pNN50++;
                          }
                      }
                  }
                  sdnn = sqrt(sdnn/ (unsigned long) totalNN); // SDNN
                  hrVars[4] = (uint8_t) sdnn & 0xFF;
                  hrVars[5] = (uint8_t) (sdnn >> 8) & 0xFF;

                  if(rmssd){
                      rmssd = sqrt(rmssd/(unsigned long)totalNN);    // RMSSD
                      hrVars[6] = (uint8_t) rmssd & 0xFF;
                      hrVars[7] = (uint8_t) (rmssd >> 8) & 0xFF;
                  }
                  if(pNN50){
                      pNN50 = ((pNN50*1000)/ (unsigned long) totalNN);  //pNN50
                      hrVars[8] = (uint8_t) pNN50 & 0xFF;
                      hrVars[9] = (uint8_t) (pNN50 >> 8) & 0xFF;
                  }
                  printk("\n-- HRV --\n");
                  // for(int i =0; i<10; i++){
                  //   printk("HRV %u, ",hrVars[i]);
                  // }

                  printk("\t meanNN %ul: \n", meanNNSamples);
                  printk("\t sdnn %u: \n", sdnn);
                  printk("\t rmssd %u: \n", rmssd);
                  printk("\t pnn50 %u: \n", pNN50);
                  printk("\n---------\n");                 
                  notifyHRV(hrVars);
              }
              
 }
 
//  int notifyHR(uint16_t hrData[]);
//  int notifyHRV(uint8_t hrvData[]);
//  void TX_HRV(){
// 	uint64_t time_stamp;
//     int64_t delta_time;
// 	time_stamp = k_uptime_get();
// 	if(!k_sem_take(&hrv,K_FOREVER)){
// 		delta_time = k_uptime_delta(&time_stamp);
// 		printk("woke after %lld min ",delta_time);
// 	}
//  }
 //K_THREAD_DEFINE(tx_hrv_id, STACKSIZE,TX_HRV,NULL,NULL,NULL,THREAD_HRV,0,0);
 atomic_t hrv_bit;
 int32_t temperature=-100, temp=-100;
const nrfx_rtc_t timer_instance = NRFX_RTC_INSTANCE(2);

static void rtc_handler(nrfx_rtc_int_type_t int_type)
{
    nrfx_err_t err;
	switch (int_type)
	{
		case NRFX_RTC_INT_TICK:
			//printk("tick interrupt received\n");
			ctick++;
			
		end_time_tick = k_uptime_get();
					
					start_time = end_time_tick;
		
			break;
		case NRFX_RTC_INT_COMPARE0:
		{
			//printk("compare 0 event received\n");
			nrfx_rtc_counter_clear(&timer_instance);
			//err = nrfx_rtc_cc_set(&timer_instance, 0, 1, true);
			// if(err!=NRFX_SUCCESS)
			// {
			// 	printk(" compare channel initialization error %d",err);
			// }
            if(cInt == 0)start_time_int = k_uptime_get();
						cInt++;
                        /*
                        cInt == 2000 for 5 Second hrv timer as prescaler = 400 => 400 * 2000 = 5s
                        cInt == 640 for 5 Second hrv timer as prescaler = 128 => 128 * 640 = 5s
                        */ 

                        if(cInt == 640){ 
                            end_time = k_uptime_get();
                            //printk("cint %lld\n",end_time-start_time_int);
                            //k_sem_give(&hrv);
                            cInt = 0;
                            hrv_set = 1;
			            }
				
			if(cInt % 1 ==0){
					// end_time_int = timing_counter_get();
					// total_cycles_int = timing_cycles_get(&start_time, &end_time_int);
    				// total_ns_int = timing_cycles_to_ns(total_cycles_int);
					//printk( " |%d interrupt %lld, %d| ",cInt,total_ns_int,cBuf.end);
			}
            break;
		}
		default:
			printk("rtc interrupt %d\n", int_type);
			break;
	}
}

static void configure_RTC(void)
{
    nrfx_err_t err;

    /* STEP 4.3 - Declaring timer config and intialize nrfx_timer instance. */
    nrfx_rtc_config_t timer_config = NRFX_RTC_DEFAULT_CONFIG;
	printk("before Prescaler value = %d",timer_config.prescaler);
    timer_config.prescaler = NRF_RTC_FREQ_TO_PRESCALER(128);
	printk("Prescaler value = %d",timer_config.prescaler);

    IRQ_CONNECT(DT_IRQN(DT_NODELABEL(rtc2)),
                DT_IRQ(DT_NODELABEL(rtc2), priority),
                nrfx_rtc_2_irq_handler, NULL, 0);  

    err = nrfx_rtc_init(&timer_instance, &timer_config, rtc_handler );

    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_timer_init error: %08x", err);
        return;
    }
     nrfx_rtc_tick_enable(&timer_instance, true);
	 err = nrfx_rtc_cc_set(&timer_instance, 0, 1, true);
	
	start_time = k_uptime_get();
	start_time_tick = start_time;
    /* STEP 4.4 - Set compare channel 0 to generate event every SAADC_SAMPLE_INTERVAL_US. */
    // err = nrfx_rtc_cc_set(&timer_instance, 0, 1, true);
    // if(err!=NRFX_SUCCESS)
    // {
    //  printk(" compare channel initialization error %d",err);
    // }
    nrfx_rtc_enable(&timer_instance);

}




//--------------------------------------
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
int16_t ecgSample[32];
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
    gpio_pin_set_dt(&intB_gpio_pin_time, 1);    nrfx_spim_xfer(&spim_inst, &spim_xfer_desc, 0);
    
    spin();    gpio_pin_set_dt(&intB_gpio_pin_time, 0);
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
    
    MNG_INT_r.bits.efit = 0b00011;//;          // Assert EINT w/ 4 unread samples
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
    // printk("\ninterupted\n");
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
                   spim_config.miso_pull      = NRF_GPIO_PIN_PULLDOWN;
                    spim_config.frequency = 8000000;
                  //  spim_config.orc = 0x00;
                    spim_config.mode = NRF_SPIM_MODE_0;
    status1 = nrfx_spim_init(&spim_inst, &spim_config, spi_event_handler, NULL);
    NRFX_ASSERT(status1 == NRFX_SUCCESS);
    configureGPIO();
     ecg_config();    
     writeRegister( SYNCH , 0); 

    //  while(1){
    //     printk("reading\n");
    //     readRegister( CNFG_ECG);
    //  }
         NRFX_EXAMPLE_LOG_PROCESS();
  
	while (1) 
	{
        // printk("\r\n--\r\n");
        NRFX_EXAMPLE_LOG_PROCESS();
		if(intB_gpio_pin.port)
		{        
 			if(ecgIntFlag) 
			{
                uint8_t sampleLen = 0;
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
                  //  printk("Status : 0x%x\n"
                    //  "Current BPM is %3.2f\n", cond_bits, BPM);
                }  
				// Check if EINT interrupt asserted
				if ( ( cond_bits & EINT_STATUS ) == EINT_STATUS ) {  
					do {
                        ecgFIFO = readRegister( ECG_FIFO );
                        ecgSample[sampleLen] = ecgFIFO >> 8 ; //ecgVal;
                        ETAG[sampleLen] = ( ecgFIFO >> 3 ) & ETAG_BITS;  // Isolate ETAG
                        //printk("%d\n",ecgSample[sampleCount]);//,   ecgSample[sampleCount] )  ;
                        NRFX_EXAMPLE_LOG_PROCESS();
                        sampleLen++;   
					} while ( ETAG[sampleLen-1] == FIFO_VALID_SAMPLE || 
							ETAG[sampleLen-1] == FIFO_FAST_SAMPLE); 
                    
					if( ETAG[sampleLen - 1] == FIFO_OVF ){    
                         printk("\n writing FIFO RST \n");              
						 writeRegister( FIFO_RST , 0); // Reset FIFO                    
					}
                    notifyECG(ecgSample, sampleLen);
                    // printk("size: %d\n",sampleLen);  
                    for( idx = 0; idx < sampleLen; idx++ ) {
                        //   printk("%6d\n", ecgSample[idx]); 
                          sandeep(ecgSample[idx]);          
                    }
                     if(hrv_set){
                HRV_parameters();
                // printk("done sending");
                hrv_set=0;
            }
				}               
        	}            
        } 
    }
    printk("\n while broken \n");
}
