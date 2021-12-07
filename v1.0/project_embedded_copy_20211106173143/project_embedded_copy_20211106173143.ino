//////////////////////////////////////////////////////////////////////
// Author: Manuel Remmele
// Date: 05.11.2021
// Version: V0.1
//
// About: This is the secound beta versin for the code of the project 
// in embedded systems (see: Manuel Remmele project approach.pdf)
//
// Versin discription: this version tryes to implement only the tasks 
// WS2812b and ADC + the ADCISR. The target is to write into a buffer 
// from TaskADC and control the lights out of that buffer.
///////////////////////////////////////////////////////////////////// 




//RTOS ---------------------------------------------------------------
#include <Arduino_FreeRTOS.h>
#include <semphr.h>


// semaphores
static SemaphoreHandle_t sem_ADC_new_val;

void TaskWS2812b( void *pvParameters );
void TaskRead( void *pvParameters );

// ADC interrupt
const byte adc_pin = A0; // = 14 (pins_arduino.h)
volatile int adc_value;
volatile bool adc_done;
volatile bool adc_busy;
unsigned int something_different = 0;

//sound
#define RB_L 128                  // Sound Ring buffer length
volatile uint8_t sound_once;      // new measured ADC value
volatile uint8_t sound_rb[RB_L];  // ring buffer for ADC values. 1024 16 Bit variables need 130% of the dynamic memory
//volatile uint8_t fft_b[RB_L];     // ring buffer for ADC values. 1024 16 Bit variables need 130% of the dynamic memory
int rb_ctr = 0;                   // current possition for a new value in the ring buffer

//int sound_GPIO = A1;

//WS2812b
#include "FastLED.h"
#include <pixeltypes.h>
#define DATA_PIN 9
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define NUM_LEDS 60
#define BRIGHTNESS 96

//Buttons
int Bt0 = 0;
int Bt0_GPIO = 5;
int Bt1 = 0;
int Bt1_GPIO = 4;
int Bt2 = 0;
int Bt2_GPIO = 3;
int Bt3 = 0;
int Bt3_GPIO = 2;

// Debugg LED
#define LED_debugg 19

//walk round
int wanderer=0;
CRGB leds[NUM_LEDS];
int val = 0;
int val_alt = 0;
int wanderung = 1;

//system variables
int speed = 9;
int brightness = 96;







////////////////////////////////////////////////////////////////////////////////
void setup() {

  // WS2812b
  Serial.begin(9600);
  //TCCR2B = TCCR2B & B11111000 | B00000011; // for PWM frequency of 980.39 Hz
  //delay(3000); // initial delay of a few seconds is recommended
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip); //+ initializes LED strip
  FastLED.setBrightness(BRIGHTNESS);// global brightness
  #define cc_speed 5

  // ADC interrupt
  ADCSRA = bit(ADEN) // Turn ADC on
           | bit(ADATE) // ADC Auto Trigger Enable
           | bit(ADIE) // Enable interrupt
           | bit(ADPS0) | bit(ADPS1) | bit(ADPS2); // Prescaler of 128
  ADMUX  = bit(REFS0) // AVCC
           | B00000001; // ADC1
  ADCSRB = B01000000; // free running mode


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







  // Tasks
  xTaskCreate(
    TaskWS2812b
    ,  "WS2812b"  // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL //Parameters for the task
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL ); //Task Handle

  xTaskCreate(
    TaskADC
    ,  "Read" // A name just for humans
    ,  128  // Stack size
    ,  NULL //Parameters for the task
    ,  1  // Priority
    ,  NULL ); //Task Handle
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////
// ADC Interrupt
ISR(ADC_vect) {
  sound_once = ADC>>2;
  //xSemaphoreGive(sem_ADC_new_val);
  //Serial.println(sound);
}


// main program
void loop() {
}



void TaskWS2812b( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  while (1){
    for (int i = 0; i < (NUM_LEDS + 1); i++){
      if ((sound_rb[0]-(i))>505){
        leds[i]= CRGB(20,10,3);
      }else{
        leds[i]= CRGB(0,0,0);
      }
      //Serial.println((abs(sound)-i*10));
    }
    FastLED.show();
  }
}



void TaskADC( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  const TickType_t xDelay = 5000 / portTICK_PERIOD_MS;
  bitSet(ADCSRA, ADSC); //starting the first conversation
  while(1){
    //xSemaphoreTake(sem_ADC_new_val, portMAX_DELAY);
    digitalWrite(LED_debugg, !digitalRead(LED_debugg));
    sound_rb[rb_ctr] = sound_once;
    fft_b[rb_ctr] = sound_once;
    if (rb_ctr > RB_L){ 
      rb_ctr = 0;
      Serial.println(sound_rb[0]);
      Serial.println(fft_b[1]);
      Serial.println(sound_rb[2]);
      Serial.println(sound_rb[3]);
      vTaskDelay( xDelay );
    }else{rb_ctr++;}
  
  }
}