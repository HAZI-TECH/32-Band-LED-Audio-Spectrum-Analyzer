#include <arduinoFFT.h>
#include <MD_MAX72xx.h>
#include <SPI.h>
#include <EEPROM.h>

#define SAMPLES 64                                           
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW                     
#define MAX_DEVICES  4                                       
#define CLK_PIN   13                                         
#define DATA_PIN  11                                         
#define CS_PIN    10                                         
#define XRES 32                                              
#define YRES 8                                               
#define BUTTON_PIN 6                                         
#define DEBOUNCE_DELAY 50
#define ARRAY_SIZE 9
#define MODE_COUNT 4
#define ADC_OFFSET 512

const int audio_response = 20;                                   

int modes[MODE_COUNT][ARRAY_SIZE] = {
  {0, 128, 192, 224, 240, 248, 252, 254, 255},
  {0, 128, 64, 32, 16, 8, 4, 2, 1},
  {0, 128, 192, 160, 144, 136, 132, 130, 129},
  {0, 128, 192, 160, 208, 232, 244, 250, 253}
};

double vReal[SAMPLES];
double vImag[SAMPLES];
char data_avgs[XRES];

int yvalue;
int displaycolumn , displayvalue;
int peaks[XRES];
int displaymode; 
unsigned long lastDebounceTime = 0; 
int MY_ARRAY[ARRAY_SIZE];

MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);   
arduinoFFT FFT = arduinoFFT();                                     

void updateMode(int mode) {
  for (int i = 0; i < ARRAY_SIZE; i++) {
    MY_ARRAY[i] = modes[mode - 1][i];
  }
  displaymode = mode;
  EEPROM.update(1, mode);
}

void setup() {
  EEPROM.update(1,1);                                           
  displaymode = EEPROM.read(1);
  ADCSRA = 0b11100101;                                          
  ADMUX = 0b00000000;                                           
  pinMode(BUTTON_PIN, INPUT);
  mx.begin();                                                   
  delay(50);                                                    
  updateMode(displaymode);
}

void loop() {
   // ++ Sampling
   for(int i=0; i<SAMPLES; i++) {
      while(!(ADCSRA & 0x10));                                    
      ADCSRA = 0b11110101 ;                                       
      int value = ADC - ADC_OFFSET ;                              
      vReal[i]= value/8;                                          
      vImag[i] = 0;                         
    }

    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

    int step = (SAMPLES/2)/XRES; 
    int c=0;
    for(int i=0; i<(SAMPLES/2); i+=step)  {
      data_avgs[c] = 0;
      for (int k=0 ; k< step ; k++) {
          data_avgs[c] = data_avgs[c] + vReal[i+k];
      }
      data_avgs[c] = data_avgs[c]/step; 
      c++;
    }

    for(int i=0; i<XRES; i++) {
      data_avgs[i] = constrain(data_avgs[i],0,audio_response);              
      data_avgs[i] = map(data_avgs[i], 0, audio_response, 0, YRES);        
      yvalue=data_avgs[i];

      peaks[i] = peaks[i]-1;
      if (yvalue > peaks[i]) 
          peaks[i] = yvalue ;
      yvalue = peaks[i];    
      displayvalue=MY_ARRAY[yvalue];
      displaycolumn=31-i;
      mx.setColumn(displaycolumn, displayvalue);                
     }
     
    int reading = digitalRead(BUTTON_PIN);
    if (reading == HIGH && previousState == LOW && millis() - lastDebounceTime > debounceDelay) {
      updateMode((displaymode % MODE_COUNT) + 1);
      lastDebounceTime = millis();
    }
    previousState = reading;
}
