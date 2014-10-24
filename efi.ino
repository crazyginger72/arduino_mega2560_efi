/*

====================================================================================
====================================================================================
                     ______ ______ ____         ___  ____       
                    / ____// ____//  _/  _   __/  / / __ \  ____ 
                   / __/  / /_    / /   | | / // / / / / / / __ \
                  / /___ / __/  _/ /    | |/ // / / /_/ / / /_/ / 
                 /_____//_/    /___/    |___//_/(_)____/  \__,__| 
       __       ___       ____       ______          __          _            __
      / /      /   |     / __ \     /_  __/__  _____/ /_  ____  (_)________ _/ /
     / /      / /| |    / /_/ /      / / / _ \/ ___/ __ \/ __ \/ / ___/ __ `/ / 
    / /____  / ___ |_  / ____/      / / /  __/ /__/ / / / / / / / /__/ /_/ / /  
   /_____(_)/_/  |_(_)/_/   (_)    /_/  \___/\___/_/ /_/_/ /_/_/\___/\__,_/_/

                 ©2014 Ginger Pollard gingerpollard72@gmail.com
                            under license GNU GPLv3
====================================================================================
====================================================================================

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
uint8_t encoder_A;
uint8_t encoder_B;
uint8_t encoder_B_prev;
float val ;    
float adj_val ;
int8_t settings_mode = 0;
int8_t last_settings_mode = 0;
int8_t settings_set = false;
uint16_t inj_pw = 20001;
uint8_t idle = 751;
float advance = 21;
uint8_t rev_limit = 3801;
int8_t mode_set = false;
const int8_t settings_button_Pin = 35;    
int8_t settings_button_State;
int8_t last_settings_Button_State = 0x0;
int16_t last_settings_check_Time = 0; 
const int16_t check_Delay = 300;
const int8_t tps_input_pin = A15;
int8_t tps_value = 0;
const float tps_correction_factor = 1; // adjust this after reading the tps sensor!!!
const int8_t efi_led = 0;  //efi logo on 7seg
const int8_t efi_led2 = 0; //efi logo on 7seg
const int8_t red_led = 33;
const int8_t green_led = 27;
const int8_t blue_led = 23;
const int8_t amber_led = 24;
const int8_t inj_pin = 34;
const int8_t eeprom_addr[5] = {1, 2, 3, 4, 5};
int16_t inj_pw_loaded;
int8_t idle_loaded;
int8_t rev_limit_loaded;
float advance_loaded;
int8_t error = 0;
int8_t error_loaded;
int16_t last_inj_pw = inj_pw;
int8_t last_idle = idle;
int8_t last_rev_limit = rev_limit;
float last_advance = advance;
boolean diag_mode = false;
boolean encoder_turned = false;
volatile uint8_t rev;
uint8_t rpm_delay;




//loop for sw to hw reset
int swhwReset(void)
{
  tone(8,4000,500);
  pinMode(sw_hw_reset_pin, 0x1);
  digitalWrite(sw_hw_reset_pin, 0x0);    // sets the LED off
}

// custom shiftout for different shiftregiser setups, length 1=8bit, 2=16bit
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
  isr_delay(0); //set delay to the timing adv/rtd
}

//caculates rpm
int rpms(void){
  detachInterrupt(0);
  rpm_raw = 60000/(millis() - time_last)*rev;
  time_last = millis();
  rev = 0;
  attachInterrupt(0, rpm_interrupt, FALLING);
}

int updateEncoder(){
  encoder_A = digitalRead(encoder_pin_A);
  encoder_B = digitalRead(encoder_pin_B);
  if (((encoder_A == 1 && encoder_B_prev == 1) || (encoder_A == 0 && encoder_B_prev == 0))
  && ((encoder_B == 0 && encoder_B_prev ==1) || (encoder_B == 1 && encoder_B_prev == 0))) {
      val += adj_val;
      encoder_B_prev = encoder_B;
      encoder_turned = true;
      //Serial.println("up");
  }
  else if (((encoder_A == 1 && encoder_B_prev == 0) || (encoder_A == 0 && encoder_B_prev == 1))
  && ((encoder_B == 1 && encoder_B_prev == 0) || (encoder_B == 0 && encoder_B_prev == 1))) {
      val -= adj_val;
      encoder_B_prev = encoder_B;
      encoder_turned = true;
      //Serial.println("down");
  }
}

int read_eeprom(){
  inj_pw_loaded = EEPROM.read(1);
  if (inj_pw_loaded*100 != inj_pw){
    inj_pw = inj_pw_loaded*100;
  }
  idle_loaded = EEPROM.read(2);
  if (idle_loaded*100 != idle){
    idle = idle_loaded*10;
  }
  advance_loaded = EEPROM.read(3);
  if (advance_loaded != advance){
    advance = advance_loaded;
  }
  rev_limit_loaded = EEPROM.read(4);
  if (rev_limit_loaded*100 != rev_limit){
    rev_limit = rev_limit_loaded*100;
  }
  error_loaded = EEPROM.read(5);
  if (error_loaded != error){
    error = error_loaded;
  }
  if (diag_mode ==1){
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
  sei();
  init();
  pinMode(13, 0x1);
  digitalWrite(13,0x0);
  Serial.begin(115200);
  Serial.println("");
  Serial.print("EFI Ver:");
  Serial.print(ver);
  Serial.print(", Model: ");
  Serial.println(model);
  Serial.println("L.A.P. Technical");
  Serial.println("©2014 Ginger Pollard");
  Serial.println(extras);
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
      diag_mode = true;
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

//Main Loop To Calculate RPM and shit
for(;;){
  
  start:
uint16_t st = millis();
  //annoying beep if engine isnt started in 3min!
  if (millis()>180000 && rpm < 100 && settings_mode == 0 && diag_mode == false){
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
    settings_set = false;  
  }
  else if (digitalRead(settings_button_Pin) == 0x0){ 
    last_settings_Button_State = 0x0;
  }

  //mode for setting params of EFI
  if((settings_mode != last_settings_mode) && (settings_set == false)){
    settings_set = false;
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
        settings_set = true;
        digitalWrite(green_led,0x0);
        break;
      case 2:
        val = idle;
        adj_val = 25;
        settings_set = true;
        digitalWrite(blue_led,0x0);
        break;
      case 3:
        val = advance;
        settings_set = true;
        adj_val = 0.5;
        digitalWrite(amber_led,0x0);
        break;
      case 4:
        val = rev_limit;
        adj_val = 50;
        settings_set = true;
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
    settings_set = true;
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
    looptime_enc = currentTime_enc;  // Updates looptime_enc
    if (millis() > last_millis + 100 && diag_mode == true && encoder_turned == true){
      Serial.print(settings_mode);
      Serial.print("<settings_mode, ");
      Serial.print(val);
      Serial.print("<val, ");
      Serial.print(adj_val*2);
      Serial.println("<adj_val");
      Serial.println("");
      last_millis = millis();
      encoder_turned = false;
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
    if(diag_mode == true){
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

  if (serialEventRun) serialEventRun();

   }
}
