/*
MS1	MS2	Microstep Resolution
L	L	Full Step (2 Phase)
H	L	Half Step
L	H	Quarter Step
H	H	Eigth Step (Default configuration)
*/
#include <LiquidCrystal.h>
#include <HX711.h>
#define step_pin 2  // Pin 3 connected to Steps pin on EasyDriver
#define dir_pin 3   // Pin 2 connected to Direction pin
#define MS1 13       // Pin 5 connected to MS1 pin
#define MS2 12       // Pin 4 connected to MS2 pin
#define ENABLE 11     // Pin 7 connected to SLEEP pin

static int steps = 0; // start and return position
static bool direction = 0;   // Variable to set Rotation (CW-CCW) of the motor

LiquidCrystal lcd(8,9,4,5,6,7);
int lcd_key = 0;
int adc_key_in = 0;

HX711 scale;
uint8_t dataPin = A0;
uint8_t clockPin = A1;

#define btnRight 0
#define btnUp 1
#define btnDown 2
#define btnLeft 3
#define btnSelect 4
#define btnNone 5

int stepDelay = 1;

int read_LCD_buttons(){
  adc_key_in = analogRead(0);
  if(adc_key_in > 840)
    return btnNone;
  if(adc_key_in < 50)
    return btnRight;
  if (adc_key_in < 225)
    return btnUp;
  if (adc_key_in < 423)
    return btnDown;
  if (adc_key_in < 640)
    return btnLeft;
  if (adc_key_in < 851)
    return btnSelect;
  return btnNone;
}

void turn_motor(int stepsPerCall, bool direction){
  digitalWrite(ENABLE, LOW);
  digitalWrite(dir_pin, direction);  // (HIGH = anti-clockwise / LOW = clockwise)
  for(int i = 0; i < stepsPerCall; i++){
    digitalWrite(step_pin, HIGH);
    delay(stepDelay);
    digitalWrite(step_pin, LOW);
    delay(stepDelay);  
    digitalWrite(step_pin, HIGH);
    delay(stepDelay);
    digitalWrite(step_pin, LOW);
    delay(stepDelay);  
  }
  if(!direction){
    steps+=2;
    Serial.println(steps);
  }
  else{
    steps-=2;
  }  
}

void write_display_fast_speed(){
lcd.clear();
      lcd.setCursor(0, 0); //position 0, row0
      lcd.print("Steps = ");
      lcd.setCursor(8, 0);
      lcd.print(steps);
      lcd.setCursor(0, 1);
      lcd.print("mm=");
      lcd.setCursor(3, 1);
      lcd.print(steps * 0.01);
      lcd.setCursor(8, 1);
      lcd.print("in=");
      lcd.setCursor(11, 1);
      lcd.print(steps * 0.0004);
}

void write_display_slow_speed(){
lcd.clear();     
      lcd.setCursor(0, 0); //position 0, row0
      lcd.print("Steps = ");
      lcd.setCursor(8, 0);  //position 8, top row
      lcd.print(steps/8);
      lcd.setCursor(0, 1);
      lcd.print("mm=");
      lcd.setCursor(3, 1);
      lcd.print(steps * 0.00125);
      lcd.setCursor(8, 1);
      lcd.print("in=");
      lcd.setCursor(11, 1);
      lcd.print(steps * 0.00005);
}

void setup() {
  steps = 0; 

  lcd.begin(16,2);
  lcd.setCursor(0,0);
  lcd.print("Press Right/Up");
  lcd.setCursor(1, 1);
  lcd.print("to Compress.");
  
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  pinMode(step_pin, OUTPUT);
  pinMode(ENABLE, OUTPUT);

  digitalWrite(MS1, HIGH);      // Configures to Full Steps
  digitalWrite(MS2, HIGH);    // Configures to Full Steps
  digitalWrite(step_pin, LOW);
  digitalWrite(dir_pin, LOW);
  //digitalWrite(SLEEP, HIGH);

  Serial.begin(115200);
  //calibrateScale();
}

void loop() {
 // digitalWrite(MS1, LOW);      // Configures to Full Steps for full speed
  //    digitalWrite(MS2, LOW);    // Configures to Full Steps       
  lcd.setCursor(9,1);
  lcd.setCursor(0,1);
  lcd_key = read_LCD_buttons();
  //default to low speed
  lcd.clear();
  switch (lcd_key) {
    

    //Compress (clockwise) at low speed until 12.7mm or 1/2" is reached (1270 steps);
    case btnRight:{
      digitalWrite(MS1, LOW);      // Configures to 1/8 Steps
      digitalWrite(MS2, HIGH);    // Configures to 1/8 Steps
      direction = 0;
      digitalWrite(dir_pin, direction);
      
      write_display_slow_speed(); //take into account microstepping
          
      if (steps < 1270 *8) {  //10160 turns for 1/2" travel when using microstepping
        turn_motor(1, direction);       
      }      
      break;      
    }

    //Rewind at low speed until steps = 0;
    case btnLeft:{
      digitalWrite(MS1, LOW);      // Configures to 1/8 Steps
      digitalWrite(MS2, HIGH);    // Configures to 1/8 Steps
      direction = 1;

      write_display_slow_speed(); 
     
      if(steps > 0){
        turn_motor(1, direction);      
      }
      break;
    }

    //Compress (clockwise) at fast speed until 12.7mm or 1/2" is reached (1270 steps);
    case btnUp:{
      direction = 0;
      digitalWrite(MS1, LOW);      // Configures to Full Steps for full speed
      digitalWrite(MS2, LOW);    // Configures to Full Steps       
      
      write_display_fast_speed();
      
      if (steps < 1270) {  //1/2" @ 200 steps per rev
        turn_motor(1, direction); 
      }
      break;
    }    
    
    //If steps > 0, rewind at a fast speed.  Rewind stops when steps = 0.
    case btnDown:{
      direction = 1;
      digitalWrite(MS1, LOW);      // Configures to Full Steps
      digitalWrite(MS2, LOW);    // Configures to Full Steps
      
      write_display_fast_speed();
      
      if (steps > 0) {  //1/2" @ 200 steps per rev
        turn_motor(1, direction);       
       }
      break;
    }

    //Unwind with no restrictions.  Reset steps to 0.
    case btnSelect:{
      lcd.clear();
      digitalWrite(MS1, LOW);      // Configures to 1/8 Steps
      digitalWrite(MS2, HIGH);    // Configures to 1/8 Steps
      turn_motor(1, direction);
      lcd.setCursor(0, 0);
      lcd.print("Override and");
      lcd.setCursor(0, 1);
      lcd.print("Reset to 0");
      lcd.setCursor(0, 1);
      steps = 0;  //reset to zero

      break;
    }
    case btnNone:{
      lcd.setCursor(0, 1);
     
      break;
    }
  }
}
