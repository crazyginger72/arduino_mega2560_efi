/*

====================================================================================
====================================================================================
                   ____   ______ __  ___          ___  ____       
                  / __ \ / ____//  |/  /   _   __/  / / __ \  ____ 
                 / /_/ // /    / /|_/ /   | | / // / / / / / / __ \
                / ____// /___ / /  / /    | |/ // / / /_/ / / /_/ / 
               /_/     \____//_/  /_/     |___//_/(_)____/  \__,__| 
       __       ___       ____       ______          __          _            __
      / /      /   |     / __ \     /_  __/__  _____/ /_  ____  (_)________ _/ /
     / /      / /| |    / /_/ /      / / / _ \/ ___/ __ \/ __ \/ / ___/ __ `/ / 
    / /____  / ___ |_  / ____/      / / /  __/ /__/ / / / / / / / /__/ /_/ / /  
   /_____(_)/_/  |_(_)/_/   (_)    /_/  \___/\___/_/ /_/_/ /_/_/\___/\__,_/_/

                 ©2014 Ginger Pollard gingerpollard72@gmail.com
                            under license GNU GPLv3
====================================================================================
====================================================================================

*Full timing map with adjustments on the fly
*Auto calibration of sensors
*Builtin Diag/debug mode
*Eeprom storage of all adjustments
*
*

*/

#include <EEPROM.h>

const char* ver = "1.0a";
const char* model = "test";
const char* extras = "";
const int8_t sw_hw_reset_pin = 6;
int8_t rpm_raw = 0;
float time_last = 0;
int8_t rpm_array[5] = {0,0,0,0,0};
int8_t rpm = 0; 
const int8_t encoder_pin_A = 26;  
const int8_t encoder_pin_B = 28;  
uint8_t encoder_A, encoder_B, encoder_B_prev, settings_button_State, error_loaded, rpm_delay;
float val, adj_val;
int8_t settings_mode = 0;
int8_t last_settings_mode = 0;
int8_t settings_set = 0x0;
uint16_t inj_pw = 5000;
uint8_t idle = 751;
float advance = 0;
uint8_t rev_limit = 3800;
uint8_t mode_set = 0x0;
const int8_t settings_button_Pin = 35;    
uint8_t last_settings_Button_State = 0x0;
int16_t last_settings_check_Time = 0; 
const uint16_t check_Delay = 300;
const uint8_t efi_led = B00000000;  //efi logo on 7seg
const uint8_t efi_led2 = B00000000; //efi logo on 7seg
const uint8_t red_led = 33;
const uint8_t green_led = 27;
const uint8_t blue_led = 23;
const uint8_t amber_led = 24;
const uint8_t inj_pin = 34;
const uint8_t ign_pin = 36;
const int8_t eeprom_addr[5] = {1, 2, 3, 4, 5};
int16_t inj_pw_loaded, last_inj_time, inj_pulse;
int8_t error = 0;
int16_t last_inj_pw = inj_pw;
int8_t last_idle = idle;
int8_t last_rev_limit = rev_limit;
float last_advance = advance;
boolean diag_mode = 0x0;
boolean encoder_turned = 0x0;
volatile uint8_t rev;
boolean over_rev = 0x0;
boolean inj_pin_state = 0x0;
uint16_t inj_pause = 20000;
boolean ign_pin_state = 0x0;
uint16_t dwell = 150;
uint16_t timing = 0;
boolean load_defaults = 0x0;
uint8_t load = 0;

//sensors========================================================
#define TPS 0
#define OIL 1
#define FUEL 2
uint8_t sensors[3][4] = { // change 3 as needed please
//===pin=====val======cor=====c_pin==============================
    {A15,     0,       0,      A15}, //TPS  0 
    {A14,     0,       0,      A14}, //OIL  1 (press)
    {A13,     0,       0,      A14}, //FUEL 2 (press)
};
int sensors_array_size = sizeof(sensors)/sizeof(sensors[0]);
//========================================================sensors

//timing map=====================================================
int8_t adv_curve [11][11] = {         //---tune later---
//rpm===0.5k==1k==1.5k==2k==2.5k==3k==3.5k==4k==5k==6k=====load%
  {15,   7,   8,   8,   9,   9,  10,  11,  13,  17,  20}, //100%
  {15,   8,   8,   9,  10,  11,  12,  13,  15,  18,  21}, //90%
  {15,   9,   9,  11,  12,  13,  14,  15,  16,  20,  22}, //80%
  {15,  10,  11,  13,  15,  15,  16,  17,  18,  22,  24}, //70%
  {15,  11,  13,  15,  17,  17,  18,  19,  20,  24,  26}, //60%
  {15,  11,  15,  17,  19,  19,  20,  20,  22,  26,  29}, //50%
  {15,  12,  16,  19,  21,  22,  23,  23,  25,  29,  31}, //40%
  {15,  12,  18,  21,  24,  25,  25,  25,  28,  31,  33}, //30%
  {15,  13,  19,  23,  26,  27,  28,  28,  31,  34,  35}, //20%
  {15,  14,  20,  24,  28,  30,  30,  31,  33,  36,  36}, //10%
  {15,  15,  21,  26,  30,  32,  33,  34,  35,  36,  37}, //0%
};
//=====================================================timing map

//fuel map======================================================
int8_t fuel_curve [11][11] = {         //---tune later---
//rpm==0.5k==1k==1.5k==2k==2.5k==3k==3.5k==4k==5k==6k=====load%
  {5,  14,  21,  26,  30,  32,  33,  34,  35,  36,  37}, //100%
  {5,  14,  20,  24,  28,  30,  30,  31,  33,  36,  36}, //90%
  {5,  13,  19,  23,  26,  27,  28,  28,  31,  34,  35}, //80%
  {5,  12,  18,  21,  24,  25,  25,  25,  28,  31,  33}, //70%
  {5,  12,  16,  19,  21,  22,  23,  23,  25,  29,  31}, //60%
  {5,  11,  15,  17,  19,  19,  20,  20,  22,  26,  29}, //50%
  {5,  11,  13,  15,  17,  17,  18,  19,  20,  24,  26}, //40%
  {5,  10,  11,  13,  15,  15,  16,  17,  18,  22,  24}, //30%
  {5,   9,   9,  11,  12,  13,  14,  15,  16,  20,  22}, //20%
  {5,   8,   8,   9,  10,  11,  12,  13,  15,  18,  21}, //10%
  {5,   7,   8,   8,   9,   9,  10,  11,  13,  17,  20}, //0%
};
//======================================================fuel map

// auto zero for sensors
int zero_sensor(uint8_t pin){
  uint8_t val = analogRead(pin);
  return val;
}
  
int zero_sensor(uint8_t pin, uint8_t c_pin){
  uint8_t state = digitalRead(c_pin);
  analogWrite(c_pin, 255);
  uint8_t val_h = analogRead(pin);
  analogWrite(c_pin, 0);
  uint8_t val_l = analogRead(pin);
  return val_h, val_l;
}

//calibrate sensors
int sensor_calibrate(){ 
  for(int i = 0;i <= sensors_array_size;i++){
    sensors[i][3] = zero_sensor(sensors[i][1]);
  }
}

//caculate timing deg to Us
int timing_us(uint8_t load, uint8_t rpms){
  cli();
  if(load > 100){load = 100;}
  if(rpm > 6000){rpm = 6000;}
  uint8_t total_adv = adv_curve[load/10][rpms/500] + advance;
  uint16_t us = total_adv*(1/(rpms *6));
  sei();
  return us;
}

//loop for sw to hw reset
int swhwReset(void)
{
  tone(8,4000,500);
  delay(500);
  pinMode(sw_hw_reset_pin, 0x1);
  digitalWrite(sw_hw_reset_pin, 0x0);
}

// custom shiftout for different shiftregiser setups, length 1=8bit, 2=16bit, 3=24bit
int shiftregister_Out(int8_t dataPin, int8_t clockPin, int16_t val, int8_t length)
{
  for (int8_t i = 0; i < 8*length; i++)  {
    digitalWrite(dataPin, !!(val & (1 << i)));
    digitalWrite(clockPin, 0x1);
    digitalWrite(clockPin, 0x0);    
  }
}

int isr_delay(uint16_t wait){
  if (--wait != 0)
    wait <<= 2;
    wait -= 2;
    __asm__ __volatile__ (
    "1: sbiw %0,1" "\n\t" // 2 cycles
    "brne 1b" : "=w" (wait) : "0" (wait) // 2 cycles
  );
}

//hall effect trigger
void rpm_interrupt()  // fix ISR to be c++ not arduino
{
  cli();
  rev++;
  if(over_rev == 0x0){
    timing = timing_us(load, rpm);
    isr_delay(timing); //set delay to the timing adv/rtd
    ign_pin_state = !ign_pin_state;
    digitalWrite(ign_pin, ign_pin_state);
    isr_delay(dwell);
    ign_pin_state = !ign_pin_state;
    digitalWrite(ign_pin, ign_pin_state);
  }
}

//caculates rpm
int rpms(void){
  //detachInterrupt(0);
  cli();
  rpm_raw = 60000/(millis() - time_last)*rev;
  time_last = millis();
  rev = 0;
  //attachInterrupt(0, rpm_interrupt, FALLING);
  sei();
}

int updateEncoder(){
  encoder_A = digitalRead(encoder_pin_A);
  encoder_B = digitalRead(encoder_pin_B);
  if (((encoder_A == 1 && encoder_B_prev == 1) || (encoder_A == 0 && encoder_B_prev == 0))
  && ((encoder_B == 0 && encoder_B_prev ==1) || (encoder_B == 1 && encoder_B_prev == 0))) {
      val += adj_val;
      encoder_B_prev = encoder_B;
      encoder_turned = 0x1;
  }
  else if (((encoder_A == 1 && encoder_B_prev == 0) || (encoder_A == 0 && encoder_B_prev == 1))
  && ((encoder_B == 1 && encoder_B_prev == 0) || (encoder_B == 0 && encoder_B_prev == 1))) {
      val -= adj_val;
      encoder_B_prev = encoder_B;
      encoder_turned = 0x1;
  }
}

int read_eeprom(){
  if (load_defaults == 0x0){
    inj_pw = EEPROM.read(1)*100;
    idle = EEPROM.read(2)*10;
    advance = EEPROM.read(3);
    rev_limit = EEPROM.read(4)*100;
  }
  error_loaded = EEPROM.read(5);
  if (error_loaded != error){
    error = error_loaded;
  }
  if (diag_mode ==1 && load_defaults == 0x0){
    Serial.println("EEPROM loaded");
    Serial.print("inj_pw= ");
    Serial.println(inj_pw);
    Serial.print("idle= ");
    Serial.println(idle);
    Serial.print("advance= ");
    Serial.println(advance);
    Serial.print("rev_limit= ");
    Serial.println(rev_limit);
    Serial.print("error= ");
    Serial.println(error);
  }else if (load_defaults == 0x1){
    Serial.println("EEPROM not loaded, using defaults!!!");
  }
}

int write_eeprom(int8_t eeprom_pw, int8_t eeprom_idl, float eeprom_adv, int8_t eeprom_rl){
  EEPROM.write(1, eeprom_pw/100);
  EEPROM.write(2, eeprom_idl/10);
  EEPROM.write(3, eeprom_adv);
  EEPROM.write(4, eeprom_rl/100);
  if (diag_mode ==1){
    Serial.println("EEPROM updated");
  }
}

int light_check(){
  digitalWrite(red_led,0x0);
  digitalWrite(green_led,0x0);
  digitalWrite(blue_led,0x0);
  digitalWrite(amber_led,0x0);
  digitalWrite(inj_pin,0x1);
  delay(50);
  digitalWrite(inj_pin,0x0);
  tone(8,2000,200);
  delay(2000);
  digitalWrite(red_led,0x1);
  digitalWrite(green_led,0x1);
  digitalWrite(blue_led,0x1);
  digitalWrite(amber_led,0x1);
}

int main(void)
{
  sw_reset: //a software reset point
  ign_pin_state = 0x0;
  sei();
  init();
  pinMode(13, 0x1);
  digitalWrite(13,0x0);
  pinMode(ign_pin, 0x1);
  digitalWrite(ign_pin, ign_pin_state);
  Serial.begin(115200);
  Serial.println("");
  Serial.print("EFI Ver:");
  Serial.print(ver);
  Serial.print(", Model: ");
  Serial.println(model);
  Serial.println("L.A.P. Technical");
  Serial.println("©2014 Ginger Pollard");
  if (extras != ""){
    Serial.println(extras);
  }
  Serial.println("firmware loaded, booting up...");
  
  pinMode(red_led, 0x1);
  pinMode(green_led, 0x1);
  pinMode(blue_led, 0x1);
  pinMode(amber_led, 0x1);
  pinMode(encoder_pin_A, 0x0);
  pinMode(encoder_pin_B, 0x0);
  pinMode(settings_button_Pin, 0x0);
  digitalWrite(red_led,0x1);
  digitalWrite(green_led,0x1);
  digitalWrite(blue_led,0x1);
  digitalWrite(amber_led,0x1);
  int8_t diag_timer = millis();
  int8_t wait_time;
  diag_wait:
  if (digitalRead(settings_button_Pin) == 0x1){
    if (millis() > diag_timer + 5000 && digitalRead(settings_button_Pin) == 0x1){
      diag_mode = 0x1;
      digitalWrite(red_led, 0x0);
      delay(1000);
      digitalWrite(red_led, 0x1);
      digitalWrite(amber_led, 0x0);
      delay(1000);
      digitalWrite(amber_led, 0x1);
      digitalWrite(green_led, 0x0);
      delay(1000);
      digitalWrite(green_led, 0x1);
      digitalWrite(blue_led, 0x0);
      delay(1000);
      digitalWrite(blue_led, 0x1);
      Serial.print("now entering diagnostis mode");
      for (int8_t i=0;i<=20;i++){
        delay(100);
        if (i != 20){
          Serial.print(".");
        }
        else if (i == 20){
          Serial.println(".");
        }
      }
    }
    else {
      goto diag_wait;
    }
  }
  pinMode(inj_pin, 0x1);
  uint16_t currentTime_enc = millis();
  uint16_t currentTime_rpm = millis();
  uint16_t looptime_enc = currentTime_enc;
  uint16_t looptime_rpm = currentTime_rpm;
  Serial3.begin(9600);
  attachInterrupt(0, rpm_interrupt, FALLING);
  int16_t last_millis = millis();
  read_eeprom();
  last_inj_pw = inj_pw;
  last_idle = idle;
  last_rev_limit = rev_limit;
  last_advance = advance;
  light_check();
  sensor_calibrate();

//Main Loop To Calculate RPM and shit
for(;;){
  
  start:

  if(rpm >= rev_limit){
    over_rev = 0x1;
  }

  uint16_t st = millis();
  //annoying beep if engine isnt started in 3min!
  if (millis()>180000 && rpm < 100 && settings_mode == 0 && diag_mode == 0x0){
    tone(8,4000,2000);
  }

  //the fucking math, FTW!!!
  int8_t displacment = 420;
  int8_t ve = 90;


  













  //button check to enter settings mode
  if ((digitalRead(settings_button_Pin) == 0x1) 
    && (last_settings_Button_State != digitalRead(settings_button_Pin)) 
    && (millis() - last_settings_check_Time > check_Delay)){
    last_settings_check_Time = millis();
    last_settings_Button_State = 0x1;
    last_settings_mode = settings_mode;
    settings_mode++;
    if(settings_mode > 4){
      settings_mode = 0;
      digitalWrite(red_led,0x1);
      digitalWrite(green_led,0x1);
      digitalWrite(blue_led,0x1);
      digitalWrite(amber_led,0x1);
    }
    settings_set = 0x0;  
  }
  else if (digitalRead(settings_button_Pin) == 0x0){ 
    last_settings_Button_State = 0x0;
  }

  //mode for setting params of EFI
  if((settings_mode != last_settings_mode) && (settings_set == 0x0)){
    settings_set = 0x0;
    digitalWrite(red_led,0x1);
    digitalWrite(green_led,0x1);
    digitalWrite(blue_led,0x1);
    digitalWrite(amber_led,0x1);
    switch(last_settings_mode){
      case 0:
        break;
      case 1:
        inj_pw = val;
        break;
      case 2:
        idle = val;
        break;
      case 3:
        advance = val;
        break;
      case 4:
        rev_limit = val;
        break;
      default:
        break;
      }
    if(settings_mode != 0){ 
    switch(settings_mode){
      case 1:
        val = inj_pw;
        adj_val = 50;
        settings_set = 0x1;
        digitalWrite(green_led,0x0);
        break;
      case 2:
        val = idle;
        adj_val = 25;
        settings_set = 0x1;
        digitalWrite(blue_led,0x0);
        break;
      case 3:
        val = advance;
        settings_set = 0x1;
        adj_val = 0.5;
        digitalWrite(amber_led,0x0);
        break;
      case 4:
        val = rev_limit;
        adj_val = 50;
        settings_set = 0x1;
        digitalWrite(red_led,0x0);
        break;
      default:
        break;
        Serial.println("[ERROR] set_sel not within params!!!");
    }
  }
  else if (settings_mode == 0){
    if (last_advance != advance || last_idle != idle || last_rev_limit != rev_limit || last_inj_pw != inj_pw){
      write_eeprom(inj_pw, idle, advance, rev_limit);
      last_inj_pw = inj_pw;
      last_idle = idle;
      last_advance = advance;
      last_rev_limit = rev_limit;
    }
    val = 0;
    adj_val = 0;
    settings_set = 0x1;
    digitalWrite(red_led,0x1);
    digitalWrite(green_led,0x1);
    digitalWrite(blue_led,0x1);
    digitalWrite(amber_led,0x1);
    }
  }
  //check delay and mode
  currentTime_enc = millis();
  if(currentTime_enc >= (looptime_enc + 10) && settings_mode !=0){
    updateEncoder();
    looptime_enc = currentTime_enc; 
    if (millis() > last_millis + 100 && diag_mode == 0x1 && encoder_turned == 0x1){
      Serial.print(settings_mode);
      Serial.print("<settings_mode, ");
      Serial.print(val);
      Serial.print("<val, ");
      Serial.print(adj_val*2);
      Serial.println("<adj_val");
      Serial.println("");
      last_millis = millis();
      encoder_turned = 0x0;
    }
  }
  if(rpm > 2500){
    rpm_delay = 20;
  }
  else{
    rpm_delay = 200;
  }
  currentTime_rpm = millis();
  if(currentTime_rpm >= (looptime_rpm + rpm_delay)){
    rpms();
    looptime_rpm = currentTime_rpm;
    if(diag_mode == 0x1){
      Serial.print("  rpm: ");
      Serial.println(rpm);
    } 
    //5 Sample Average
    rpm_array[0] = rpm_array[1];
    rpm_array[1] = rpm_array[2];
    rpm_array[2] = rpm_array[3];
    rpm_array[3] = rpm_array[4];
    rpm_array[4] = rpm_raw;
    rpm = (rpm_array[0] + rpm_array[1] + rpm_array[2] + rpm_array[3] + rpm_array[4]) / 5;
  }

  //fuel injector pulsing
  if(over_rev == 0x0 && inj_pin_state == 0x0 && last_inj_time + inj_pulse >= micros()){
    inj_pin_state = !inj_pin_state;
    digitalWrite(inj_pin, inj_pin_state);
  }else if(inj_pin_state == 0x1 && last_inj_time + inj_pause >= micros())

  if (serialEventRun) serialEventRun();

   }
}