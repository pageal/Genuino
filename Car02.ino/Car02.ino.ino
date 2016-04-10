#include <OneWire.h>

//#include <IRremote.h>
//#include <IRremoteInt.h>
//#include <IRremoteTools.h>

// sources:
// - IR: https://arduino-info.wikispaces.com/IR-RemoteControl
// - IR Lib: https://gist.github.com/Robotonics/9b956eb356552ecc6dfb (download .zip and use Sketch/Include Library/Add .zip library menu)
// - Motors: http://www.instructables.com/id/Arduino-Modules-L298N-Dual-H-Bridge-Motor-Controll/?ALLSTEPS
// 
// IRRemote lib: https://github.com/z3t0/Arduino-IRremote
// BAsic: 
// - millisecnds: https://www.arduino.cc/en/Reference/Millis
// - getting started: https://www.arduino.cc/en/Guide/Arduino101
// - AVR library: http://www.nongnu.org/avr-libc/, https://www.arduino.cc/en/Reference/UsingAVR
// - AVR Library Download and Install: 
//    - http://www.ladyada.net/learn/avr/setup-win.html 
//    - https://sourceforge.net/projects/winavr/ 
//    - !!! Assistance request: http://forum.arduino.cc/index.php?topic=389365.0
// 


//#include <time.h>

//pins
////////////////////////////////////////////////////
//H-BRIDGE
int OUT_PIN_HBRIHGE_INT1_MOT_A = 2;
int OUT_PIN_HBRIHGE_INT2_MOT_A = 3;
int OUT_PIN_HBRIHGE_INT3_MOT_B = 4;  
int OUT_PIN_HBRIHGE_INT4_MOT_B = 5;
int OUT_PIN_HBRIHGE_EN_A = 6;
int OUT_PIN_HBRIHGE_EN_B = 7;

//Keyboard Controls:
#define MOTOR_A_FWD 49 // '1' -Motor 1 forward
#define MOTOR_A_STP 50 // '2' -Motor 1 stop
#define MOTOR_A_BKW 51 // '3' -Motor 1 backward

#define MOTOR_B_FWD 52 // '4' -Motor 2 forward
#define MOTOR_B_STP 53 // '5' -Motor 2 stop
#define MOTOR_B_BKW 54 // '6' -Motor 2 backward

// motors
#define MOTOR_A 0
#define MOTOR_B 1

// commands
#define MOTOR_STP 1
#define MOTOR_FWD 2
#define MOTOR_BKW 3

// Declare L298N Dual H-Bridge Motor Controller directly since there is not a library to load.

struct MOTOR
{
  int dir1Pin;
  int dir2Pin;
  int speedPin; // Needs to be a PWM pin to be able to control motor speed
};
typedef struct MOTOR T_MOTOR;

T_MOTOR motors[2];

// XINDA IR 
int INP_PIN_IR_RECV = 12;
// INDICATION
int OUT_PIN_BLINK_PIN = 13;


//timeout vars
bool status_led_high = false;
int blink_timeout = 500;
unsigned long t1 = 0;
unsigned long t2 = 0;
int cycle_time_sum = 0;

// analog pins
#define DIR_FWD 0
#define DIR_RGT 1
#define DIR_LFT 2

void log_serial(char* msg)
{
  Serial.println(msg);
}

//IR 
//IRrecv irrecv(INP_PIN_IR_RECV);
//decode_results results;
void motor_do(int motor_idx, int action)
{
  T_MOTOR motor = motors[motor_idx]; 
  char msg[20];
  switch(action)
  {
//https://www.sparkfun.com/datasheets/Robotics/L298_H_Bridge.pdf
//Turn-On and Turn-Off : Before to Turn-ON the Supply
//Voltage and before to Turn it OFF, the Enable input
//must be driven to the Low state.
    
      case MOTOR_FWD: 
        digitalWrite(motor.dir1Pin, HIGH);
        digitalWrite(motor.dir2Pin, LOW);
        digitalWrite(motor.speedPin, HIGH);
        //sprintf(msg, "motor%d FWD", motor_idx);
        //Serial.println(msg);
        break;
      
      case MOTOR_STP:
        if(digitalRead(motor.dir1Pin)==LOW)
          digitalWrite(motor.dir1Pin, HIGH);
        if(digitalRead(motor.dir2Pin)==LOW)
          digitalWrite(motor.dir2Pin, HIGH);
        digitalWrite(motor.speedPin, 0);
        //sprintf(msg, "motor%d STP", motor_idx);
        //Serial.println(msg);
       break;
      
      case MOTOR_BKW:
        digitalWrite(motor.dir1Pin, LOW);
        digitalWrite(motor.dir2Pin, HIGH);
        digitalWrite(motor.speedPin, HIGH);
        //sprintf(msg, "motor%d BKW", motor_idx);
        //Serial.println(msg);
        break;
    }
}

// Motors
void handle_motors() {
  // Check the Serial interface:
  int inByte = 0;
  if (Serial.available() > 0) 
  {
    inByte = Serial.read();
    char echo_input[20];
    //int speed; // Local variable
    sprintf(echo_input, "Echo: %d", inByte);
    Serial.println(echo_input); 
  }
    
  switch (inByte) {
    //______________Motor 1______________
    case MOTOR_A_FWD: // Motor 1 Forward
      motor_do(MOTOR_A, MOTOR_FWD);
      break;
    case MOTOR_A_STP: // Motor 1 Stop (Freespin)
      motor_do(MOTOR_A, MOTOR_STP);
      break;
    case MOTOR_A_BKW: // Motor 1 Reverse
      motor_do(MOTOR_A, MOTOR_BKW);
      break;
    
    //______________Motor 2______________
    case MOTOR_B_FWD: // Motor 2 Forward
      motor_do(MOTOR_B, MOTOR_FWD);
      break;
    case MOTOR_B_STP: // Motor 1 Stop (Freespin)
      motor_do(MOTOR_B, MOTOR_STP);
      break;
    case MOTOR_B_BKW: // Motor 2 Reverse
      motor_do(MOTOR_B, MOTOR_BKW);
      break;    
  }
}

bool motors_stopped = false;

// the setup function runs once when you press reset or power the board
void setup() {
 
  // initialize digital pin 13 as an output.
  pinMode(OUT_PIN_BLINK_PIN, OUTPUT);

  Serial.begin(9600);
  motors[MOTOR_A].dir1Pin = OUT_PIN_HBRIHGE_INT1_MOT_A;
  motors[MOTOR_A].dir2Pin = OUT_PIN_HBRIHGE_INT2_MOT_A;
  motors[MOTOR_A].speedPin = OUT_PIN_HBRIHGE_EN_A;
  
  motors[MOTOR_B].dir1Pin = OUT_PIN_HBRIHGE_INT3_MOT_B;
  motors[MOTOR_B].dir2Pin = OUT_PIN_HBRIHGE_INT4_MOT_B;
  motors[MOTOR_B].speedPin = OUT_PIN_HBRIHGE_EN_B;

//  irrecv.enableIRIn(); // Start the receiver}

  //Define L298N Dual H-Bridge Motor Controller Pins
  pinMode(OUT_PIN_HBRIHGE_INT1_MOT_A,OUTPUT);
  pinMode(OUT_PIN_HBRIHGE_INT2_MOT_A,OUTPUT);
  pinMode(OUT_PIN_HBRIHGE_INT3_MOT_B,OUTPUT);
  pinMode(OUT_PIN_HBRIHGE_INT4_MOT_B,OUTPUT);
  pinMode(OUT_PIN_HBRIHGE_EN_A,OUTPUT);
  pinMode(OUT_PIN_HBRIHGE_EN_B,OUTPUT);
}

unsigned long diff32(unsigned long a, unsigned long b)
{
  long delta = b-a;
  if(delta <0)
    delta = (0xFFFFFFFF-t1)+t2;
  return (unsigned long)delta;
}

/*
unsigned long get_ir_keypress()
{
  if (irrecv.decode(&results)) {
      Serial.println(results.value, HEX);
      irrecv.resume(); // Receive the next value
    }
  return results.value;
}
*/
void invert_status_led()
{
  cycle_time_sum += diff32(t1,t2);
  t1 = millis();
  if(cycle_time_sum>blink_timeout)
  {
    cycle_time_sum = 0;
    if(!status_led_high)
    {
      digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
      status_led_high = true;
    }
    else
    {
      digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
      status_led_high = false;
    }
  }
}


bool can_move(int dir)
{
  int ir_distance_head = analogRead(dir);
  //if (Serial.available() > 0)
  //{
  //  Serial.println(ir_distance_head); 
  //}
  if(ir_distance_head<200)
    return true;
  return false;
}

bool move()
{
      motor_do(MOTOR_A, MOTOR_FWD);
      motor_do(MOTOR_B, MOTOR_FWD);
}

// the loop function runs over and over again forever
void loop() {

  if(motors_stopped == false)
  {
      motor_do(MOTOR_A, MOTOR_STP);
      motor_do(MOTOR_B, MOTOR_STP);
      motors_stopped = true;
  }

//  unsigned long ir_key = 0; 
  invert_status_led();
  //ir_key = get_ir_keypress();
  handle_motors();
  delay(blink_timeout/10);              // wait for a second
  t2=millis();

  if(can_move(DIR_FWD) and can_move(DIR_RGT) and can_move(DIR_LFT))
  {   
    move();
  }else
  {
      motor_do(MOTOR_A, MOTOR_BKW);
      motor_do(MOTOR_B, MOTOR_BKW);
      delay(200);
      motor_do(MOTOR_A, MOTOR_STP);
      motor_do(MOTOR_B, MOTOR_STP);
  }
}
