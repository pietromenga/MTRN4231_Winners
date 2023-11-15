#include <Stepper.h>

// Number of steps per output rotation
const int stepsPerRevolution = 200;

// Create Instance of Stepper library
Stepper myStepper(stepsPerRevolution, 9, 10, 11, 12);

// Motor Left connections
int enA = 8;
int in1 = 7;
int in2 = 6;
// Motor Right connections
int enB = 3;
int in3 = 5;
int in4 = 4;

// for incoming serial data
int incomingByte = 0; 

void setup() {
    // Set all the motor control pins to outputs
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    
    // Turn off motors - Initial state
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);

    // set the speed at 60 rpm:
    myStepper.setSpeed(100);

    // opens serial port, sets data rate to 9600
    Serial.begin(9600); 
    Spin();
}

void loop() {
  
    // check for data from serial, any data is fine
    if (Serial.available() > 0) {
        if (Serial.read() > 0) {
          // if there is a serial data then activate the launcher loop
          Serial.println("Starting Launching Process");
  
          // Start spinning the motors
          
          
          // Step forward
          for (int i = 0; i < 12; i++) {
          myStepper.step(stepsPerRevolution);
          delay(500);
          }

          delay(1000);
          
  
          // Step back to origin
          for (int i = 0; i < 12; i++) {
          myStepper.step(-stepsPerRevolution);
          delay(500);
          }
         
        }
    }

    // If there is no data on the serial, keep looking until there is
}

void Spin() {
    analogWrite(enA, 255);
    analogWrite(enB, 255);

    // Turn on motor A & B
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    delay(2000);
}

void Stop() {
    // turn off motors
    Serial.println("Stoping");
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    delay(2000);
}
