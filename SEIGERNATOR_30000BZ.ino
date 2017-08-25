#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>
#define PWM_PIN 3
#define POT_PIN A1
#define THERM_PIN A0
#define HEAT_PIN 13
#define RPM_PIN 0
#define WINDOWSIZE 1024

#define BCLED1_P 11
#define BCLED1_N 10
#define BCLED2_P 8
#define BCLED2_N 9

#define PB1 5
#define PB2 4

#define SHORTDELAY  1
#define LONGDELAY  200

#define I2C_ADDR    0x27 // <<----- Add your address here.  Find it from I2C Scanner
#define BACKLIGHT_PIN     3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7

int pwm_power = 0;
double setpoint, input, output;
unsigned long windowStartTime;
unsigned long screen_update;
unsigned long stop_update;
PID temp_controller(&input, &output, &setpoint, 2,0.01,4, DIRECT);

volatile long stirRPM;
double RPM_LAST;
volatile char measureState = 0;
volatile unsigned long rpm_timestore;
double RPM[10], PWM_power, stir_setpoint;
PID stir_controller(&RPM[0], &PWM_power, &stir_setpoint, .8, 3, .05, DIRECT);
LiquidCrystal_I2C lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);

char stir_halt;
char heat_halt;
char stir_set;
char heat_set;

unsigned long PB1_press;
char PB1_state;
unsigned long PB2_press;
char PB2_state;

void RPM_MEASURE()
{
  if(measureState == 0)
  {
    rpm_timestore = micros();
    measureState = 1;
  } else if((measureState == 1) & (micros() > rpm_timestore+1000))
  {
    stirRPM = 60000000/(micros() - rpm_timestore);
    measureState = 0;
  }
}

void setup()
{
  lcd.begin(16,2);
  // Switch on the backlight
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.home (); // go home
  lcd.print("   SEIGERNATOR   "); 
  lcd.setCursor(0,1);
  lcd.print("     30000BZ     "); 
  //PWM Pin initialize
  pinMode(PWM_PIN, OUTPUT);
  //initialize heater pin
  pinMode(HEAT_PIN, OUTPUT);
  //Ensure PWM is at zero;
  analogWrite(PWM_PIN, 0);
  //attach the interrupt for measuring RPM
  attachInterrupt(RPM_PIN, RPM_MEASURE, RISING);
  //initialize BC LED's
  pinMode(BCLED1_P, OUTPUT);
  pinMode(BCLED1_N, OUTPUT);
  pinMode(BCLED2_P, OUTPUT);
  pinMode(BCLED2_N, OUTPUT);
  //initialize buttons
  pinMode(PB1, INPUT);
  pinMode(PB2, INPUT);
  //illuminate them opposite colors
  for(int i = 0; i < 1500; i++){
    digitalWrite(BCLED1_P, HIGH);
    digitalWrite(BCLED1_N, LOW);
    digitalWrite(BCLED2_P, LOW);
    digitalWrite(BCLED2_N, HIGH);
    delay(1);
    digitalWrite(BCLED1_P, LOW);
    digitalWrite(BCLED1_N, HIGH);
    digitalWrite(BCLED2_P, HIGH);
    digitalWrite(BCLED2_N, LOW);
    delay(1);
  }
  lcd.clear(); 
  //init temp controller in manual state
  setpoint = 4*(95-55);
  //setpoint = 100;
  temp_controller.SetOutputLimits(0, WINDOWSIZE);
  temp_controller.SetSampleTime(500);
  temp_controller.SetMode(MANUAL);
  //increase PWM frequency!
  TCCR2B = TCCR2B & 0b11111000 | 0x01;
  //init stir controller in manual state
  stir_setpoint = 250;
  stir_controller.SetOutputLimits(0, 25500);
  stir_controller.SetSampleTime(25);
  stir_controller.SetMode(MANUAL);
  
  screen_update = millis();
  
  //ensure modes are correct;
  stir_halt = 1;
  heat_halt = 1;
  stir_set = 0;
  heat_set = 0;
}

void loop()
{
  RPM[0] = (RPM[1] + RPM[2] + RPM[3] + RPM[4] + RPM[5] + RPM[6] + stirRPM)/7;
  RPM[1] = RPM[0];
  RPM[2] = RPM[1];
  RPM[3] = RPM[2];
  RPM[4] = RPM[3];
  RPM[5] = RPM[4];
  RPM[6] = RPM[5];
  Serial.println(analogRead(THERM_PIN));
  input = analogRead(THERM_PIN);
  temp_controller.Compute();
  stir_controller.Compute();
  analogWrite(PWM_PIN, PWM_power/100);
  
  if(millis() - windowStartTime>WINDOWSIZE)
  {
    windowStartTime += WINDOWSIZE;
  }
  if(output < millis() - windowStartTime)
  {
    digitalWrite(HEAT_PIN,LOW);
    lcd.setCursor(15,1);
    lcd.print(" ");
  } else {
    digitalWrite(HEAT_PIN,HIGH);
    lcd.setCursor(15,1);
    lcd.print("*");
  }
 
 
  //detect when stirrer has stopped 
  if(millis() > stop_update + 500);
  { 
    
    if(RPM[0] == RPM_LAST)
    {
      stirRPM = 0;
    }
    RPM_LAST = RPM[0];
    stop_update = millis();    
  }
  
  
  //Update the screen!!!
  if(millis() > screen_update + 200)
  { 
    //print stuff to LCD
    lcd.setCursor(0,0);
    //print the stir status
    if(!stir_halt){
      if((int)RPM[0] > 999)
      {
        //nada
      } else if((int)RPM[0] > 99) {
        lcd.print(" ");
      } else if((int)RPM[0] > 9) {
        lcd.print("  ");
      } else {
        lcd.print("   ");
      }
      lcd.print((int)RPM[0]);
      lcd.setCursor(4,0);
      lcd.print("RPM  ");
    } else {
      lcd.print("HALT   ");
    }
    //print the stir setpoint
    lcd.setCursor(9,0);
    if(stir_setpoint > 999){
      //nada
    } else if(stir_setpoint > 99){
      lcd.print(" ");
    } else if(stir_setpoint > 9){
      lcd.print("  ");
    } else {
      lcd.print("   ");
    }
    lcd.print((int)stir_setpoint);
    lcd.print("RPM");
    
    lcd.setCursor(0,1);
    if(((input/4)+55) < 100){
      lcd.print(" ");
    }
    if(!heat_halt){
      lcd.print((input/4)+55);
      lcd.print((char)223);
      lcd.print("F    ");
    } else {
      lcd.print("HALT    ");
    }
  
    //display temperature setpoint
    lcd.setCursor(10,1);
    if(((setpoint/4)+55) < 100){
      lcd.print(" ");
    }
    lcd.print((int)(setpoint/4)+55); 
    lcd.print((char)223);
    lcd.print("F    "); 
    screen_update = millis();
  }
  
  //make sure the LED's fit the correct color
  if((!stir_halt) & (!stir_set))
  {
    //green
    digitalWrite(BCLED1_P, LOW);
    digitalWrite(BCLED1_N, HIGH);
  } else if(stir_halt & (!stir_set)){
    //RED
    digitalWrite(BCLED1_P, HIGH);
    digitalWrite(BCLED1_N, LOW);
  } else if(stir_set) {
    //ORANGE
    digitalWrite(BCLED1_P, !digitalRead(BCLED1_P));
    digitalWrite(BCLED1_N, !digitalRead(BCLED1_N));
  }
  //now make sure the heat LED is correct
  if(!heat_halt & !heat_set)
  {
    //GREEN
    digitalWrite(BCLED2_P, LOW);
    digitalWrite(BCLED2_N, HIGH);
  } else if(heat_halt & !heat_set) {
    //RED
    digitalWrite(BCLED2_P, HIGH);
    digitalWrite(BCLED2_N, LOW);
  } else if(heat_set) {
    //ORANGE
    digitalWrite(BCLED2_P, !digitalRead(BCLED2_P));
    digitalWrite(BCLED2_N, !digitalRead(BCLED2_N));
  }
  
  //check if button 2 is pressed
  if(!digitalRead(PB2)){
    if(PB2_state == 1){
      PB2_press = millis();
      PB2_state = 2;
    }
    if(millis() > PB2_press + LONGDELAY){
      PB2_state = 4;
    } else if(millis() > PB2_press + SHORTDELAY){
      PB2_state = 3;
    } 
  } else {
    if(PB2_state == 3) {
      stir_halt = !stir_halt;
    } else if(PB2_state == 4) {
      stir_set = !stir_set;
    }
    PB2_state = 1;
  }
  
  //check if button 1 is pressed
  if(!digitalRead(PB1)){
    if(PB1_state == 1){
      PB1_press = millis();
      PB1_state = 2;
    }
    if(millis() > PB1_press + LONGDELAY){
      PB1_state = 4;
    } else if(millis() > PB1_press + SHORTDELAY){
      PB1_state = 3;
    } 
  } else {
    if(PB1_state == 3) {
      heat_halt = !heat_halt;
    } else if(PB1_state == 4) {
      heat_set = !heat_set;
    }
    PB1_state = 1;
  }
  
  //set the manual/automatic nature of the stirrer
  if(!stir_halt){
    stir_controller.SetMode(AUTOMATIC);
  } else {
    stir_controller.SetMode(MANUAL);
    PWM_power = 0;
  }
  
  //set the manual/automatic nature of the Heater
  if(!heat_halt){
    temp_controller.SetMode(AUTOMATIC);
    //setpoint = 4*(100-55); //set to 100 degrees
  } else {
    temp_controller.SetMode(MANUAL);
    output = 0;
  }
  
  //allow setting of stirrer
  if(stir_set){
    stir_setpoint = (analogRead(POT_PIN)*4)+250;
  }
  
  //allow setting of Heater
  if(heat_set){
    setpoint = analogRead(POT_PIN)/1.4;
  }
  
  if(stir_set & heat_set){
    stir_set = 0;
    heat_set = 0;
  }
  
  //OVER_TEMPERATURE PROETECTION
  if(input > 750)
  {
    temp_controller.SetMode(MANUAL);
    stir_controller.SetMode(MANUAL);
    output = 0;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("TEMP ERROR");
    stir_halt = 1;
    heat_halt = 1;
  }
  
}
