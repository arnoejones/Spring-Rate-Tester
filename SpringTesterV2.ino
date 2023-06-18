#include <Arduino.h>

/*
A4988 & DRV8825 hookups different from EasyDriver board:
5v -> RST (pin 5)
5v -> SLP (pin 6)
*/

#include <HX711.h>
#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>
#include <EncoderButton.h>
#include <Adafruit_NeoPixel.h>

#define step_pin 20  // Pin 3 connected to Steps pin on EasyDriver
#define dir_pin 21   // Pin 2 connected to Direction pin

#define MS1 14  // Pin connected to MS1 pin
#define MS2 15  // Pin  connected to MS2 pin
#define MS3 16

#define CW LOW  //motor direction
#define CCW HIGH

#define adc_key_in_pin A2   //analog pin to read ladder bridge switch set

// HX711 Module connections
#define CLK 5   // CLOCK of HX711 connected to pin 5 of Arduino
#define DOUT 6  // DOUT of HX711 connected to pin 6 of Arduino

#define btnLLeft 0    //ladder button switch assignment (not pin assignments)
#define btnLeft 1
#define btnCenter 2
#define btnRight 3
#define btnRRight 4
#define btnNone 5

//Everything for the MINI 12864 LCD module, encoder, reset switch
#define encBtn 4
#define encA 2
#define encB 3
#define NEOPIXEL_PIN 17
#define encResetBtn 18
#define LCD_CLOCK 10
#define LCD_MOSI 11
#define LCD_RESET 7
#define LCD_CS 9
#define LCD_RS 8
U8G2_ST7567_OS12864_2_4W_SW_SPI u8g2(U8G2_R2,  //rotate 180
                                     LCD_CLOCK,
                                     LCD_MOSI,
                                     LCD_CS,
                                     LCD_RS,
                                     LCD_RESET);


// Rotary Encoder Module connections
#define RotaryCLK 2  // Rotary encoder CLK pin connected to pin 2 of Arduino
#define RotaryDT 3   // Rotary encoder DT pin connected to pin 3
#define RotarySW 4   // Rotary encoder Switch pin connected to pin 4

HX711 scale;
uint8_t dataPin = A0;
uint8_t clockPin = A1;

static int count = 0;
static float distance = 0;
static int factor = 0;
static int step = 0;
int lcd_key = 0;
int adc_key_in = 0;

int step_time_delay = 500;

volatile boolean TurnDetected;  // variable used to detect rotation of Rotary encoder
volatile int Rotary_Flag = 0;   // flag to indicate rotation as occured

int reset_screen_counter = 0;    // Variable used to decide what to display on Oled
volatile int current_units = 0;  // Used to select which measuring unit to use (KG,Grams,Pounds)
float unit_conversion;           // Used to convert between measuring units
int decimal_place;               // how many decimal number to display

int read_LCD_buttons() {
  adc_key_in = analogRead(adc_key_in_pin);

  if (adc_key_in > 950)
    return btnNone;
  if (adc_key_in < 50)  //left to right, 0 - 5
    return btnLLeft;
  if (adc_key_in < 400)
    return btnLeft;
  if (adc_key_in < 600)
    return btnCenter;
  if (adc_key_in < 750)
    return btnRight;
  if (adc_key_in < 900)
    return btnRRight;
  return btnNone;
}

void runMotor(bool direction, int steps, bool ms1, bool ms2, bool ms3, int factor) {
  digitalWrite(dir_pin, direction);
  digitalWrite(MS1, ms1);
  digitalWrite(MS2, ms2);
  digitalWrite(MS3, ms3);

  while (count < steps) {
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(step_time_delay);
    digitalWrite(step_pin, LOW);
    delayMicroseconds(step_time_delay);
    count++;  //only care about steps, not direction

    if (direction) {
      step++;  //global running count, only reset by reset button.
    } else {
      step--;
    }
  }
  int _distance = (step / factor);
  distance = distance + _distance;
  step = 0;
  count = 0;
  return;
}

// Interrupt routine runs if Rotation detected from Rotary encoder
void rotarydetect() {
  Rotary_Flag = 1;  // Set Rotary flag from 0 to 1
  //delay(2);
}

void draw(void) {
  u8g2.firstPage();
  do {
    u8g2.drawStr(1, 25, "Resetting Origin");
    char buf[9];
    sprintf(buf, "%3d", Rotary_Flag);
    u8g2.drawStr(5, 40, buf);
  } while (u8g2.nextPage());
}

void onEncBtnReset() {
  distance = 0;
  draw();
}

void write_display_compressing() {
  u8g2.firstPage();
  do {
    u8g2.drawStr(0, 30, "Compressing...");
  } while (u8g2.nextPage());
}

void write_display_extending() {
  u8g2.firstPage();
  do {
    u8g2.drawStr(0, 30, "Extending...");
  } while (u8g2.nextPage());
}

void setup() {
  pinMode(A2, INPUT_PULLUP);  //keypad input pin
  pinMode(encResetBtn, OUTPUT);
  digitalWrite(encResetBtn, HIGH);

  // Attach interrupt 0 (Pin 2 on UNO) to the Rotary Encoder
  attachInterrupt(0, rotarydetect, RISING);  // interrupt 0 always connected to pin 2 on Arduino UNO
  attachInterrupt(digitalPinToInterrupt(encResetBtn), onEncBtnReset, FALLING);
  
  // light up the module
  Adafruit_NeoPixel strip(3, NEOPIXEL_PIN, NEO_GRB);
  strip.begin();
  strip.setBrightness(255);
  strip.setPixelColor(0, strip.Color(150, 255, 0));    //knob LED, upper left
  strip.setPixelColor(1, strip.Color(0, 50, 255));     //knob LED, lower right
  strip.setPixelColor(2, strip.Color(255, 255, 255));  //backlight
  strip.show();

  // stepper motor init
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  pinMode(step_pin, OUTPUT);

  digitalWrite(MS1, HIGH);  // Configures to Full Steps
  digitalWrite(MS2, HIGH);  // Configures to Full Steps
  digitalWrite(MS3, HIGH);
  digitalWrite(step_pin, LOW);
  digitalWrite(dir_pin, LOW);

  pinMode(RotarySW, INPUT_PULLUP);
  pinMode(RotaryCLK, INPUT_PULLUP);
  pinMode(RotaryDT, INPUT_PULLUP);

  scale.begin(DOUT, CLK);  // Init of the HX711
  
  String start_count_string = "Starting up....";  // Message to display at Startup
  char start_count[15];                           // Used to String to Char conversion

  u8g2.begin();
  u8g2.setContrast(25);
  u8g2.setFont(u8g_font_unifont);
  // Loop to display counting dots
  for (int x = 12; x < 16; x++) {  // Select the first 12 to 16 character of String
    start_count_string.toCharArray(start_count, x);
    u8g2.firstPage();
    do {
      u8g2.drawStr(0, 10, "ARDUINO SCALE");
      u8g2.drawStr(0, 28, start_count);
    } while (u8g2.nextPage());
    delay(500);  // Delay between dots
  }
}

// Reset Scale to zero
void tare_scale(void) {
  scale.set_scale(-452395.00);  //Calibration Factor obtained from calibration sketch
  scale.tare();                 //Reset the scale to 0
}

// Used to change the measurement units (0=grams, 1=KG, 2=pounds)
void change_units() {
  if (current_units == 0) current_units = 1;
  else if (current_units == 1) current_units = 2;
  else if (current_units == 2) current_units = 3;
  else if (current_units == 3) current_units = 0;
}

// Run at Startup and when Resetting with Rotary encoder switch
void startupscreen(void) {
  u8g2.firstPage();
  do {
    u8g2.drawStr(0, 10, "Clear Scale");
    u8g2.drawStr(0, 28, "Click to zero...");
  } while (u8g2.nextPage());
}

// Start displaying information on display
void start_scale(void) {
  char temp_current_units[15];  // Needed to store String to Char conversion
  String KG = "KG";
  String GRAMS = "GRAMS";
  String LBS = "POUNDS";
  String LBSInch = "LBS/Inch";

  if (current_units == 0) {                     // 0 = grams
    GRAMS.toCharArray(temp_current_units, 15);  // Convert String to Char for OLED display
    unit_conversion = 1000;                     // conversion value for grams
    decimal_place = 0;                          // how many decimal place numbers to display
  } else if (current_units == 1) {              // 1 = Kilograms
    KG.toCharArray(temp_current_units, 15);
    unit_conversion = 1;
    decimal_place = 3;
  } else if (current_units == 2) {  // else 2 = Pounds
    LBS.toCharArray(temp_current_units, 15);
    unit_conversion = 2.2046226218;
    decimal_place = 3;
  } else {
    LBSInch.toCharArray(temp_current_units, 15);
    unit_conversion = 2.2046226218 * 2;
    decimal_place = 3;
  }

  u8g2.firstPage();
  do {
    u8g2.setCursor(0, 10);
    u8g2.print(scale.get_units(3) * unit_conversion, decimal_place);  // Display the average of 3 scale value reading
    u8g2.drawStr(60, 10, temp_current_units);                         // Display the current measurement unit
    u8g2.setCursor(0, 28);
    u8g2.print("mm  ");
    u8g2.print(distance/100, decimal_place);
    u8g2.setCursor(0, 46);
    u8g2.print("inch  ");
    decimal_place = 2;
    u8g2.print(((distance)/25.4)/100, decimal_place);

  } while (u8g2.nextPage());
}
/*
Using DRV8825 stepper driver 
Data sheet: https://www.ti.com/lit/ds/symlink/drv8825.pdf
MS1	  MS2	  MS3	  Microstep Resolution
Low	  Low	  Low	  Full step
High	Low	  Low	  Half step
Low	  High	Low	  Quarter step
High	High	Low	  Eighth step
Low   Low   High  Sixteenth step
High	Low 	High	32nd step  (not available on A4988 (Big Easyddriver))
*/
void loop() {
  // If Switch is pressed on Rotary Encoder
  if (!digitalRead(RotarySW)) {  // Check to see which action to take
    if (reset_screen_counter == 1) {
      tare_scale();  // 1 = zero and start scale
      reset_screen_counter = 2;
      //delay(500);
    } else {
      if (reset_screen_counter == 2) {  // 2 = Scale already started so restart from begining
        reset_screen_counter = 0;
        //delay(500);
      }
    }
  }

  // If Rotation was detected
  if (Rotary_Flag == 1) {
    change_units();   // change the measuring units
    Rotary_Flag = 0;  // reset flag to zero
  }

  // If system was just started display intro screen
  if (reset_screen_counter == 0) {
    startupscreen();
    reset_screen_counter = 1;  // set to 1 and wait for Rotary click to zero scale
  }

  // if zero (tare) of scale has occured start display of weight
  if (reset_screen_counter == 2) {
    start_scale();
  }

  lcd_key = read_LCD_buttons();
  /* End product is to get everything in the form of distance travelled, or 
    rotation.  200 steps equals 1 full rotation, which equals 2mm of travel
    for the slide.  To keep everything in sync, convert all the steps into
    final distance travelled in terms of 200, or one rotation.
    1:200
    2:400
    4:800
    8:1600
    16:3200
  */
  switch (lcd_key) {
    case btnRight:  // Inner right button, regulated distance, fast speed.
      {
        write_display_compressing();  //take into account microstepping
        factor = 2;
        runMotor(CCW, (80*factor), HIGH, LOW, LOW, factor);  //1270 * 2
        break;
      }

    //Rewind at low speed until steps = 0;
    case btnLeft:  // Inner left button, regulated distance, fast speed.
      {
        factor = 2;
        write_display_extending();
        runMotor(CW, 80*factor, HIGH, LOW, LOW, factor);
        break;
      }

    //Compress (clockwise) at fast speed until 12.7mm or 1/2" is reached (1270 steps);
    case btnRRight:  //Outer right button, slow speed, unregulated distance
      {
        factor = 4;
        write_display_compressing();
        runMotor(CCW, 1270*factor, LOW, HIGH, LOW, factor);
        break;
      }

    //If steps > 0, rewind at a fast speed.  Rewind stops when steps = 0.
    case btnLLeft:  // Outer left button, slow speed, unregulated distance
      {
        factor = 4;
        write_display_extending();
        runMotor(CW, 1270*factor, LOW, HIGH, LOW, factor);
        break;
      }

    case btnCenter:
        break;
    case btnNone:
        break;
      break;
  }
}