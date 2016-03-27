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


#include <time.h>

//pins
////////////////////////////////////////////////////
//H-BRIDGE
//int OUT_PIN_HBRIHGE_INT1_MOT_A = 1;
//int OUT_PIN_HBRIHGE_INT2_MOT_A = 2;
//int OUT_PIN_HBRIHGE_INT3_MOT_B = 3;
//int OUT_PIN_HBRIHGE_INT4_MOT_B = 4;
//int OUT_PIN_HBRIHGE_EN_A = 9;
//int OUT_PIN_HBRIHGE_EN_B = 10;
//Keyboard Controls:
//
// 1 -Motor 1 Left
// 2 -Motor 1 Stop
// 3 -Motor 1 Right
//
// 4 -Motor 2 Left
// 5 -Motor 2 Stop
// 6 -Motor 2 Right

#define MOTOR_A_FWD 49
#define MOTOR_A_STP 50
#define MOTOR_A_BKW 51

#define MOTOR_B_FWD 52
#define MOTOR_B_STP 53
#define MOTOR_B_BKW 54

#define MOTOR_A 0
#define MOTOR_B 1

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

int dir1PinA = 2;
int dir2PinA = 3;
int speedPinA = 9; // Needs to be a PWM pin to be able to control motor speed

// Motor 2
int dir1PinB = 4;
int dir2PinB = 5;
int speedPinB = 10; // Needs to be a PWM pin to be able to control motor speed
////////////////////////////////////////////////////

// XINDA IR 
int INP_PIN_IR_RECV = 12;
// INDICATION
int OUT_PIN_BLINK_PIN = 13;



//timeout vars
bool status_led_high = false;
int blink_timeout = 1000;
unsigned long t1 = 0;
unsigned long t2 = 0;
int cycle_time_sum = 0;

//IR 
//IRrecv irrecv(INP_PIN_IR_RECV);
//decode_results results;
void motor_do(int motor_idx, int action)
{
motors[0].dir1Pin = 2;
motors[0].dir2Pin = 3;
motors[0].speedPin = 9;

motors[1].dir1Pin = 4;
motors[1].dir2Pin = 5;
motors[1].speedPin = 10;

  T_MOTOR motor = motors[motor_idx]; 
  switch(action)
  {
      case MOTOR_FWD: 
        analogWrite(motor.speedPin, 0);
        digitalWrite(motor.dir1Pin, LOW);
        digitalWrite(motor.dir2Pin, HIGH);
        analogWrite(motor.speedPin, 255);
        break;
      
      case MOTOR_STP: // Motor 1 Stop (Freespin)
        digitalWrite(motor.dir1Pin, LOW);
        digitalWrite(motor.dir2Pin, LOW);
        analogWrite(motor.speedPin, 0);
        break;
      
      case MOTOR_BKW: // Motor 2 Reverse
        analogWrite(motor.speedPin, 0);
        digitalWrite(motor.dir1Pin, HIGH);
        digitalWrite(motor.dir2Pin, LOW);
        analogWrite(motor.speedPin, 255);
        break;
    }
}

// Motors
void handle_motors() {
  // Check the Serial interface:
  if (Serial.available() > 0) {
    int inByte = Serial.read();
    char echo_input[20];
    //int speed; // Local variable
    sprintf(echo_input, "Echo: %d", inByte);
    Serial.println(echo_input); 
    
    switch (inByte) {
      
      //______________Motor 1______________
      
      case MOTOR_A_FWD: // Motor 1 Forward
        Serial.println("Motor 1 Forward"); // Prints out “Motor 1 Forward” on the serial monitor
        motor_do(MOTOR_A, MOTOR_FWD);
        break;
      
      case MOTOR_A_STP: // Motor 1 Stop (Freespin)
        Serial.println("Motor 1 Stop");
        motor_do(MOTOR_A, MOTOR_STP);
        break;
      
      case MOTOR_A_BKW: // Motor 1 Reverse
        Serial.println("Motor 1 Reverse");
        motor_do(MOTOR_A, MOTOR_BKW);
        break;
      
      //______________Motor 2______________
      
      case MOTOR_B_FWD: // Motor 2 Forward
        motor_do(MOTOR_B, MOTOR_FWD);
        Serial.println("Motor 2 Forward");
        break;
      
      case MOTOR_B_STP: // Motor 1 Stop (Freespin)
        motor_do(MOTOR_B, MOTOR_STP);
        Serial.println("Motor 2 Stop");
        break;
      
      case MOTOR_B_BKW: // Motor 2 Reverse
        motor_do(MOTOR_B, MOTOR_BKW);
        Serial.println("Motor 2 Reverse");
        break;
      
      default:
        Serial.println("Stop ALL");
        // turn all the connections off if an unmapped key is pressed:
        for (int thisPin = 2; thisPin < 11; thisPin++) {
          digitalWrite(thisPin, LOW);
        }
    }
  }// if (Serial.available() > 0)
}

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 13 as an output.
  pinMode(OUT_PIN_BLINK_PIN, OUTPUT);
//  pinMode(OUT_PIN_HBRIHGE_EN_A, OUTPUT);
//  pinMode(OUT_PIN_HBRIHGE_EN_B, OUTPUT);
//  pinMode(OUT_PIN_HBRIHGE_EN_B, OUTPUT);

  Serial.begin(9600);
//  irrecv.enableIRIn(); // Start the receiver}

  //Define L298N Dual H-Bridge Motor Controller Pins
  pinMode(dir1PinA,OUTPUT);
  pinMode(dir2PinA,OUTPUT);
  pinMode(speedPinA,OUTPUT);
  pinMode(dir1PinB,OUTPUT);
  pinMode(dir2PinB,OUTPUT);
  pinMode(speedPinB,OUTPUT);
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
// the loop function runs over and over again forever
void loop() {
//  unsigned long ir_key = 0; 
  invert_status_led();
  //ir_key = get_ir_keypress();
  handle_motors();
  delay(blink_timeout/100);              // wait for a second
  t2=millis();
}
