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

int sw_hw_reset_pin = 6;
volatile float time = 0;
volatile float time_last = 0;
volatile int rpm_array[5] = {0,0,0,0,0};
volatile int rpm = 0; 
const int encoder_pin_A = 26;  
const int encoder_pin_B = 28;  
unsigned int encoder_A;
unsigned int encoder_B;
//unsigned int encoder_A_prev;
unsigned int encoder_B_prev;
float val ;    
float adj_val ;
int settings_mode = 0;
int last_settings_mode = 0;
int settings_set = false;
long inj_pw = 20001;
int idle = 751;
float advance = 21;
int rev_limit = 3801;
int mode_set = false;
const int settings_button_Pin = 35;    
int settings_button_State;
int last_settings_Button_State = LOW;
long last_settings_check_Time = 0; 
long check_Delay = 300;
int tps_input_pin = A15;
int tps_value = 0;
float tps_correction_factor = 1; // adjust this after reading the tps sensor!!!
const int efi_led = 0;  //efi logo on 7seg
const int efi_led2 = 0; //efi logo on 7seg
const int red_led = 33;
const int green_led = 27;
const int blue_led = 23;
const int amber_led = 24;
const int inj_pin = 34;
const int eeprom_addr[5] = {1, 2, 3, 4, 5};
long inj_pw_loaded;
int idle_loaded;
int rev_limit_loaded;
float advance_loaded;
int error = 0;
int error_loaded;
long last_inj_pw = inj_pw;
int last_idle = idle;
int last_rev_limit = rev_limit;
float last_advance = advance;
boolean diag_mode = false;







//loop for sw to hw reset
int swhwReset(void)
{
  tone(8,4000,500);
  pinMode(sw_hw_reset_pin, OUTPUT);      // sets pin only if needed
  digitalWrite(sw_hw_reset_pin, LOW);    // sets the LED off
}

// custom shiftout for different shiftregiser setups, length 1=8bit, 2=16bit
int shiftregister_Out(int dataPin, int clockPin, long val, int length)
{
  for (int i = 0; i < 8*length; i++)  {
    digitalWrite(dataPin, !!(val & (1 << i)));
    digitalWrite(clockPin, HIGH);
    digitalWrite(clockPin, LOW);    
  }
}

//hall effect trigger
void rpm_interrupt()
{
   time = (micros() - time_last); 
   time_last = micros();
}

int updateEncoder(){
  encoder_A = digitalRead(encoder_pin_A);
  encoder_B = digitalRead(encoder_pin_B);
  if (((encoder_A == 1 && encoder_B_prev == 1) || (encoder_A == 0 && encoder_B_prev == 0))
  && ((encoder_B == 0 && encoder_B_prev ==1) || (encoder_B == 1 && encoder_B_prev == 0))) {
      val += adj_val;
      encoder_B_prev = encoder_B;
      //Serial.println("up");
  }
  else if (((encoder_A == 1 && encoder_B_prev == 0) || (encoder_A == 0 && encoder_B_prev == 1))
  && ((encoder_B == 1 && encoder_B_prev == 0) || (encoder_B == 0 && encoder_B_prev == 1))) {
      val -= adj_val;
      encoder_B_prev = encoder_B;
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
  Serial.println("EEPROM loaded...");
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

int write_eeprom(int eeprom_pw, int eeprom_idl, float eeprom_adv, int eeprom_rl){
  EEPROM.write(1, eeprom_pw/100);
  EEPROM.write(2, eeprom_idl/10);
  EEPROM.write(3, eeprom_adv);
  EEPROM.write(4, eeprom_rl/100);
  if (diag_mode ==1){
    Serial.println("EEPROM updated...");
  }
}

int light_check(){
  digitalWrite(red_led,LOW);
  digitalWrite(green_led,LOW);
  digitalWrite(blue_led,LOW);
  digitalWrite(amber_led,LOW);
  digitalWrite(inj_pin,HIGH);
  delay(50);
  digitalWrite(inj_pin,LOW);
  tone(8,2000,200);
  delay(2000);
  digitalWrite(red_led,HIGH);
  digitalWrite(green_led,HIGH);
  digitalWrite(blue_led,HIGH);
  digitalWrite(amber_led,HIGH);
}

int main(void)
{
  sw_reset: //a software reset point
  init();
  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);
  Serial.begin(115200);
  Serial.println("");
  Serial.println("EFI v1.0a");
  Serial.println("L.A.P. Technical");
  Serial.println("©2014 Ginger Pollard");
  Serial.println("");
  pinMode(red_led, OUTPUT);
  pinMode(green_led, OUTPUT);
  pinMode(blue_led, OUTPUT);
  pinMode(amber_led, OUTPUT);
  pinMode(encoder_pin_A, INPUT);
  pinMode(encoder_pin_B, INPUT);
  pinMode(settings_button_Pin, INPUT);
  digitalWrite(red_led,HIGH);
  digitalWrite(green_led,HIGH);
  digitalWrite(blue_led,HIGH);
  digitalWrite(amber_led,HIGH);
  int diag_timer = millis();
  int wait_time;
  diag_wait:
  if (digitalRead(settings_button_Pin) == HIGH){
    if (millis() > diag_timer + 5000 && digitalRead(settings_button_Pin) == HIGH){
      diag_mode = true;
      digitalWrite(red_led, LOW);
      delay(1000);
      digitalWrite(red_led, HIGH);
      digitalWrite(amber_led, LOW);
      delay(1000);
      digitalWrite(amber_led, HIGH);
      digitalWrite(green_led, LOW);
      delay(1000);
      digitalWrite(green_led, HIGH);
      digitalWrite(blue_led, LOW);
      delay(1000);
      digitalWrite(blue_led, HIGH);
      Serial.print("now entering diagnostis mode");
      for (int i=0;i<=20;i++){
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
      //Serial.println("wait");
      goto diag_wait;
    }
  }
  pinMode(inj_pin, OUTPUT);
  unsigned long currentTime = millis();
  unsigned long loopTime = currentTime;
  Serial3.begin(9600);
  attachInterrupt(0, rpm_interrupt, FALLING);
  long last_millis = millis();//////////////
  read_eeprom();
  last_inj_pw = inj_pw;
  last_idle = idle;
  last_rev_limit = rev_limit;
  last_advance = advance;
  light_check();

//Main Loop To Calculate RPM and shit
for(;;){
  
  start:

  //annoying beep if engine isnt started in 3min!
  if (millis()>180000 && rpm < 100 && settings_mode == 0 && diag_mode == false){
    tone(8,4000,2000);
  }

  //the fucking math, FTW!!!
  int displacment = 420;
  int ve = 90;
















  //button check to enter settings mode
  if ((digitalRead(settings_button_Pin) == HIGH) 
    && (last_settings_Button_State != digitalRead(settings_button_Pin)) 
    && (millis() - last_settings_check_Time > check_Delay)){
    last_settings_check_Time = millis();
    last_settings_Button_State = HIGH;
    last_settings_mode = settings_mode;
    settings_mode++;
    if(settings_mode > 4){
      settings_mode = 0;
      digitalWrite(red_led,HIGH);
      digitalWrite(green_led,HIGH);
      digitalWrite(blue_led,HIGH);
      digitalWrite(amber_led,HIGH);
    }
    settings_set = false;  
  }
  else if (digitalRead(settings_button_Pin) == LOW){ 
    last_settings_Button_State = LOW;
  }

  //mode for setting params of EFI
  if((settings_mode != last_settings_mode) && (settings_set == false)){
    settings_set = false;
    digitalWrite(red_led,HIGH);
    digitalWrite(green_led,HIGH);
    digitalWrite(blue_led,HIGH);
    digitalWrite(amber_led,HIGH);
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
        digitalWrite(green_led,LOW);
        break;
      case 2:
        val = idle;
        adj_val = 25;
        settings_set = true;
        digitalWrite(blue_led,LOW);
        break;
      case 3:
        val = advance;
        settings_set = true;
        adj_val = 0.5;
        digitalWrite(amber_led,LOW);
        break;
      case 4:
        val = rev_limit;
        adj_val = 50;
        settings_set = true;
        digitalWrite(red_led,LOW);
        break;
      default:
        break;
       // Serial.println("[ERROR] set_sel not within params!!!");
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
    digitalWrite(red_led,HIGH);
    digitalWrite(green_led,HIGH);
    digitalWrite(blue_led,HIGH);
    digitalWrite(amber_led,HIGH);
    }
  }
  
  //check delay 
  currentTime = millis();
  if(currentTime >= (loopTime + 10) && settings_mode !=0){
    updateEncoder();
    loopTime = currentTime;  // Updates loopTime

if (millis() > last_millis + 2000 && diag_mode == true){
Serial.print(settings_mode);
Serial.print("<settings_mode, ");
Serial.print(val);
Serial.print("<val, ");
Serial.print(adj_val*2);
Serial.println("<adj_val");
Serial.println("");
last_millis = millis();
}


}


    //5 Sample Moving Average
    rpm_array[0] = rpm_array[1];
    rpm_array[1] = rpm_array[2];
    rpm_array[2] = rpm_array[3];
    rpm_array[3] = rpm_array[4];
    rpm_array[4] = 60*(1000000/(time*4));    
    rpm = (rpm_array[0] + rpm_array[1] + rpm_array[2] + rpm_array[3] + rpm_array[4]) / 5;
 

  if (serialEventRun) serialEventRun();
  goto start;
 
  }
}

