//////////////////////////////////////////////////////////////////////
// Author: Manuel Remmele
// Date: 29.11.2021
// Version: V1.0
//
// About: This is the eighth versin for the code of the project 
// in embedded systems (see: Manuel Remmele project approach.pdf)
//
// Versin discription: This version includes color changing 
// with the buttons along with several modes.
///////////////////////////////////////////////////////////////////// 




//RTOS ---------------------------------------------------------------
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include "fix_fft.h"
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif


// semaphores
static SemaphoreHandle_t sem_ADC_new_val;
static SemaphoreHandle_t sem_rb_full;
static SemaphoreHandle_t sem_led_ready;
static SemaphoreHandle_t sem_fft_done;

void TaskWS2812b( void *pvParameters );
void TaskRead( void *pvParameters );
void TaskFFT( void *pvParameters );
void TaskSettings( void *pvParameters );


//sound
#define RB_L 64                   // Sound Ring buffer length
volatile uint8_t sound_once;      // new measured ADC value
volatile uint8_t sound_rb[RB_L];  // ring buffer for ADC values. 1024 16 Bit variables need 130% of the dynamic memory
int8_t fft_rb[RB_L];              // FFT real buffer
int8_t fft_cb[RB_L];              // FFT compelx buffer
uint8_t fft_abs[RB_L/2];           // FFT compelx buffer
uint8_t rb_ctr = 0;                   // current possition for a new value in the ring buffer

#define DATA_PIN 9
#define NUM_LEDS 50

//Buttons
#define Bt0_GPIO  5
#define Bt1_GPIO  4
#define Bt2_GPIO  3
#define Bt3_GPIO  2
uint8_t Bt0_state = 0;
uint8_t Bt1_state = 0;
uint8_t Bt2_state = 0;
uint8_t Bt3_state = 0;
uint8_t Button_states = 0;
uint8_t mode = 0;
#define MODES 4 //how many modes are there


//color
uint8_t HSV[] = {155,0,0};



// Debugg LED
#define LED_debugg 19


Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LEDS, DATA_PIN, NEO_GRB + NEO_KHZ800);


const TickType_t xDelay = 3000 / portTICK_PERIOD_MS;



////////////////////////////////////////////////////////////////////////////////
void setup() {

  // WS2812b
  Serial.begin(9600);
  pixels.begin();

  // ADC interrupt
  ADCSRA = B10101111;
  
  /*bit(ADEN) // Turn ADC on
           | bit(ADATE) // ADC Auto Trigger Enable
           | bit(ADIE) // Enable interrupt
           | bit(ADPS0) | bit(ADPS1) | bit(ADPS2); // Prescaler of 128*/
  ADMUX  = bit(REFS0)   // AVCC
           | B00000001; // ADC1
  ADCSRB = B01000000;   // free running mode


  // Buttons
  pinMode(Bt0_GPIO,INPUT);
  pinMode(Bt1_GPIO,INPUT);
  pinMode(Bt2_GPIO,INPUT);
  pinMode(Bt3_GPIO,INPUT);

  //Debugg LED
  pinMode(LED_debugg,OUTPUT);

  // Semaphores
  if ( sem_ADC_new_val == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    sem_ADC_new_val= xSemaphoreCreateBinary();  // Create a binary semaphore
  }
  if ( sem_rb_full == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    sem_rb_full= xSemaphoreCreateBinary();  // Create a binary semaphore
  }
  if ( sem_led_ready == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    sem_led_ready= xSemaphoreCreateBinary();  // Create a binary semaphore
  }
  if ( sem_fft_done == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    sem_fft_done= xSemaphoreCreateBinary();  // Create a binary semaphore
  }






  // Tasks
  xTaskCreate(
    TaskWS2812b
    ,  "WS2812b"  // A name just for humans
    ,  110  // This stack size can be checked & adjusted by reading the Stack Highwater -------- 66 is the minimum stack size
    ,  NULL //Parameters for the task
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL ); //Task Handle

  xTaskCreate(
    TaskADC
    ,  "Read" // A name just for humans
    ,  75  // Stack size
    ,  NULL //Parameters for the task
    ,  3  // Priority
    ,  NULL ); //Task Handle

  xTaskCreate(
    TaskFFT
    ,  "FFT" // A name just for humans
    ,  110  // Stack size
    ,  NULL //Parameters for the task
    ,  1  // Priority
    ,  NULL ); //Task Handle

  xTaskCreate(
    TaskSettings
    ,  "Settings and Buttons" // A name just for humans
    ,  66  // Stack size
    ,  NULL //Parameters for the task
    ,  0  // Priority
    ,  NULL ); //Task Handle
    
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////
// ADC Interrupt
ISR(ADC_vect) {
  BaseType_t task_woken = pdFALSE;
  sound_once = ADC>>2;
  xSemaphoreGiveFromISR(sem_ADC_new_val, &task_woken);
  if (task_woken) {
    portYIELD_FROM_ISR();
  }
}




// main program
void loop() {
}





void TaskWS2812b( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  while (1){


    xSemaphoreTake( sem_led_ready, ( TickType_t ) 100 );
    switch(mode){

      // FFT mode
      case 0:
        for (uint8_t i = 0; i < (NUM_LEDS); i++){
          if (i<24){
            pixels.setPixelColor (i, (fft_abs[i+8]>>2), (fft_abs[i+8]>>2), (fft_abs[i+8]>>2));
          }else{
            pixels.setPixelColor (i, 0,0,0);
          }
        }
        break;

      // Amplitude mode
      case 1:
        digitalWrite(LED_debugg, !digitalRead(LED_debugg));
        for (uint8_t i = 0; i < (NUM_LEDS); i++){
          if ((sound_once-(i))>127){
            pixels.setPixelColor (i, 20, 10, 3);
          }else{
            pixels.setPixelColor (i, 0, 0, 0);
          }
        }
        break;

      // simple one color constant mode
      case 2:
        for (uint8_t i = 0; i < 3; i++){ //(NUM_LEDS) 
          pixels.setPixelColor (i, pixels.ColorHSV((uint16_t(HSV[0])<<8), (255-HSV[1]), HSV[2] ));
        }
        break;
      default: mode = 0;


      // Christmas mode
      case 3: 
        // this mode uses the fft_rb buffer that is used for the FFT in FFT mode to store variables because storage is rar.
        for (uint8_t i = 0; i < (NUM_LEDS); i++){
          if ((i & B00000011) == B00000010){
            if ((i & B00000100) == 0){
              pixels.setPixelColor (i, 30, 25, 0);
            }else{
              pixels.setPixelColor (i, 50, 0, 0);
            }
          }else{
            pixels.setPixelColor (i, 1, 3, 0);
          }
        }
        break;

    }
    pixels.show();
  }
}



void TaskADC( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  bitSet(ADCSRA, ADSC); //starting the first conversation
  while(1){

    xSemaphoreTake(sem_ADC_new_val, portMAX_DELAY);
    uint8_t current_ADC = ADMUX ;
    if ((current_ADC & B00001111) == B00000001){    //if ADC1 is selected
      ADMUX = ADMUX & B11110000;      //select ADC0
      sound_rb[rb_ctr] = sound_once;
      if (rb_ctr >= RB_L-1){ // if ringbuffer is at one end change the index back to the beginning
        rb_ctr = 0;
        xSemaphoreGive(sem_rb_full);
        xSemaphoreTake( sem_fft_done, ( TickType_t ) 100 );
      }else{rb_ctr++;}
    }else{
      ADMUX = ADMUX | B0000001;  //select ADC1
    }
  }
}







void TaskFFT( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  uint8_t test = 0;
  while(1){
    if (1){
      if( xSemaphoreTake( sem_rb_full, ( TickType_t ) 100 ) == pdTRUE ){
        for (uint8_t i=0 ; i<(RB_L) ; i++){
          fft_cb[i] = 0;
          fft_rb[i] = sound_rb[i];
        }

        fix_fft(fft_rb,fft_cb,6,0);

        //process the 16th to 32th value of the fft
        uint16_t semi_pyth;
        uint8_t highest = 0;
        uint8_t highest_pos = 0;

        for (uint8_t i=16 ; i<(RB_L/2) ; i++){
          semi_pyth = (abs(fft_rb[i])) + (abs(fft_cb[i]));
          if ((semi_pyth>highest)&(semi_pyth>18)){
            fft_abs[highest_pos] = 0;
            highest_pos = i;
            highest = semi_pyth;
            fft_abs[i] = 1<<7;
          }else{
            if (fft_abs[i]>6){
              fft_abs[i]=fft_abs[i]-6;
            }else{
              fft_abs[i]=0;
            }
          }
        }

        // repeat the same thing for 8 to 16
        for (uint8_t i=8 ; i<16 ; i++){
          semi_pyth = (abs(fft_rb[i])) + (abs(fft_cb[i]));
          if ((semi_pyth>highest)&(semi_pyth>32)){
            fft_abs[highest_pos] = 0;
            highest_pos = i;
            highest = semi_pyth;
            fft_abs[i] = 1<<7;
          }else{
            if (fft_abs[i]>6){
              fft_abs[i]=fft_abs[i]-6;
            }else{
              fft_abs[i]=0;
            }
          }
        }
        
        xSemaphoreGive(sem_led_ready);
        xSemaphoreGive(sem_fft_done);
      }
      else
      {
        digitalWrite(LED_debugg, !digitalRead(LED_debugg));
          /* We could not obtain the semaphore and can therefore not access
          the shared resource safely. */
      }
    }
  }
}



void TaskSettings( void *pvParameters __attribute__((unused)) )  
{
  uint8_t state = 0;
  while(1){
    state = digitalRead(Bt0_GPIO);
    
    if ((state != Bt0_state) & (state == HIGH)){
      mode ++;
      if (mode >= MODES){
        mode = 0;
      }
    }
    Bt0_state = state;

    
    
    if (digitalRead(Bt1_GPIO)){
      HSV[0]= HSV[0]+4;
    }
    


    
    state = digitalRead(Bt2_GPIO);
    if ((state == HIGH)){
      if (HSV[1]<255){
        HSV[1] = HSV[1]<<1;
        HSV[1] = HSV[1]+1;
      }else{
        if (state != Bt1_state){
          HSV[1] = 0;
        }
      }
    }
    Bt2_state = state;


    
    state = digitalRead(Bt3_GPIO);
    if ((state == HIGH)){
      if (HSV[2]<255){
        HSV[2] = HSV[2]<<1;  
        HSV[2] ++;
      }else{
        if (state != Bt3_state){
          HSV[2] = 0;
        }
      }
    }
    Bt3_state = state;



    vTaskDelay( 100 / portTICK_PERIOD_MS );
  }
}

