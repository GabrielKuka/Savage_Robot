// This is the 1.0 version !
#include <Arduino.h>
#include <XBOXRECV.h> //import the wireless coms library
#include <DCMOTOR.h> //import the motor control library

USB Usb; //Create Objects for program to use
XBOXRECV Xbox(&Usb);
// Motor 1 pins

uint8_t dirPin1_M1 = 8;
uint8_t dirPin2_M1 = 7;
uint8_t enablePin_M1 = 6;

// Motor 2 pins

uint8_t dirPin1_M2 = 2;
uint8_t dirPin2_M2 = 4;
uint8_t enablePin_M2 = 5;

// Motor objects
DCMOTOR Motor1;
DCMOTOR Motor2;

// Joystick directions
int16_t hatXInput;
int16_t hatYInput;

// These variable will hold the intensity of each direction
int16_t sideIntensity;
int16_t fbIntensity;

bool motorDir;

uint8_t pwmMap(uint16_t input);

uint8_t xboxPort = 0;//The port on the receiver that the controller is connected to

void setup() {//The setup code initializes the rest of your program
  Serial.begin(115200);
  if (Usb.Init() == -1) {//check whether the USB connects
    Serial.print(F("\r\nOSC did not start"));
    while (1); //Stop the program if we didn't connect
  }

  // Attach motors to pin in the arduino
  Motor1.attach( dirPin1_M1, dirPin2_M1, enablePin_M1 );
  Motor2.attach( dirPin1_M2, dirPin2_M2, enablePin_M2 );
  
  Serial.print(F("\r\nXbox Wireless Receiver Library Started"));
}

void loop() {// The loop runs repeatedly from top to bottom after the setup
  Usb.Task();
  if(Xbox.XboxReceiverConnected)  {
    if(Xbox.Xbox360Connected[xboxPort]){
      hatXInput = Xbox.getAnalogHat(RightHatX, xboxPort);
      hatYInput = Xbox.getAnalogHat(RightHatY, xboxPort);
      
      sideIntensity = abs(hatXInput);
      fbIntensity = (abs)(hatYInput);
      
      if(hatXInput > 7500){ // <- If joystick moves right
        checkMovement(2);
      } else if (hatXInput < -7500 ){ // <- If joystick moves left
        checkMovement(1);
      } else if(hatYInput > 7500){ // <- If joystick moves up
        checkMovement(3);
      }else if(hatYInput < -7500){ // <- If joystick moves down
        checkMovement(4);
      }else if(hatXInput <= 7500 && hatXInput >= -7500 && hatYInput <= 7500 && hatYInput >= -7500) {
        checkMovement(5);
      }
    }
  }
}

uint8_t pwmMap(uint16_t input){
  uint8_t temp = (input-1)/128;//overflows at full neg without the -1
  Serial.print("Intensity: ");
  Serial.println(input);
  Serial.print("Pwm Out: ");
  Serial.println(temp);
  return temp;
}
void checkMovement(int movement){
  switch(movement){
    case 1: // <- Turn robot left
      Serial.println("Turns left");
      Motor1.motorRunCW(pwmMap(sideIntensity));
      Motor2.motorRunCW(pwmMap(sideIntensity)); 
    break;
    case 2: // <- Turn robot right
      Serial.println("Turns right");
      Motor1.motorRunCCW(pwmMap(sideIntensity));
      Motor2.motorRunCCW(pwmMap(sideIntensity));
    break;

    case 3: // <- Moves robot forward
      Serial.println("Forward");
      Motor1.motorRunCCW(pwmMap(fbIntensity));
      Motor2.motorRunCW(pwmMap(fbIntensity));
    break;

    case 4: // Moves robot backward
      Serial.println("Backward");
      Motor1.motorRunCW(pwmMap(fbIntensity));
      Motor2.motorRunCCW(pwmMap(fbIntensity));
    break;
    default: // The defualt state will stop the motors from rotating
      Serial.println("Robot stops");
      Motor1.motorBrake();
      Motor2.motorBrake();
    } 
  }
