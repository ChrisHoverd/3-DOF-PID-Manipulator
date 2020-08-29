
#include <Servo.h>

Servo myservo; //create servo object


const int m1_EncoderA = 2; //declares input pins for motor 1 encoder
const int m1_EncoderB = 3;


const int m2_EncoderA = 18; //declares input pins for motor 2 encoder
const int m2_EncoderB = 19;

double m1_DesiredAngle = 0 ; //declares desired angle for motor 1
double m1_CurrentAngle = 0;  //declares current angle for motor 1

double m2_DesiredAngle = -90; //declares desired angle for motor 2
double m2_CurrentAngle = 0; //declares current angle for motor 2

int servo_pos = 0;

double m1_Error; //declares error values for PID motor 1
double m1_D_Error; 
double m1_i_Error;
double m1_LastError;

double m2_Error; //declares error values for PID motor 2
double m2_D_Error;
double m2_i_Error;
double m2_LastError;

String m1_CurrentDirection = ""; //declares current direction for both motors
String m2_CurrentDirection = "";


double  m1_Voltage; //declares output voltage for both motors
double  m2_Voltage; //PID will modify this from the error values

double  m1_Kp = 10; //tuning values for motor 1
double  m1_Kd = 0.8;
double  m1_Ki = 0.000625;

double  m2_Kp = 6; //tuning values for motor 2
double  m2_Kd = 11;
double  m2_Ki = 0.001;

int  m1_H_bridge_enable = 8; //h_bridge pins for motor 1
int  m1_in_1= 10;
int  m1_in_2 = 11;

int  m2_H_bridge_enable = 4; //h_bridge pins for motor 2
int  m2_in_1= 5;
int  m2_in_2 = 6;

volatile int  m1_Counter = 0; //increment/decrements encoder 1 position
volatile int  m1_CurrentStateA; //state of channel A
volatile int m1_CurrentStateB; //state of channel B
volatile int m1_LastStateA; //last state of channel A
double m1_MotorAngle; //angle in degrees of motor 1
double m1_CurrentTime; //current millis from arduino
double m1_LastTime; //used for calculating time past for calculations
double m1_TimeChange; //time change

volatile int  m2_Counter = 0; //refer to above but for motor 2
volatile int  m2_CurrentStateA;
volatile int m2_CurrentStateB;
volatile int m2_LastStateA;
double m2_MotorAngle;
double m2_CurrentTime;
double m2_LastTime;
double m2_TimeChange;


 void setup() { 

    
   
   pinMode (m1_EncoderA, INPUT); //sets encoder pins as input for motor 1
   pinMode (m1_EncoderB, INPUT);
   
   pinMode (m2_EncoderA, INPUT); //sets encoder pins as input for motor 2
   pinMode (m2_EncoderB, INPUT);

   myservo.attach(13);

   Serial.begin (9600);
   Serial.setTimeout(1000);
   
   pinMode (m1_H_bridge_enable, OUTPUT); //sets h_bridge pins as output for motor 1
   pinMode (m1_in_1, OUTPUT);
   pinMode (m1_in_2, OUTPUT);

   pinMode (m2_H_bridge_enable, OUTPUT); //sets h_bridge pins as output for motor 2
   pinMode (m2_in_1, OUTPUT);
   pinMode (m2_in_2, OUTPUT);

  // Reads the initial state of the outputA
   m1_LastStateA = digitalRead(m1_EncoderA); //finds current state of channel A for motor 1
   m2_LastStateA = digitalRead(m2_EncoderA); //finds current state of channel A for motor 1
  
   //Attach interrupts
   attachInterrupt(digitalPinToInterrupt(m1_EncoderA), m1_EncoderCount, RISING); //attaches rising interrupt for channel A motor 1
   attachInterrupt(digitalPinToInterrupt(m2_EncoderA), m2_EncoderCount, RISING); //attaches rising interrupt for channel A motor 2
  
   m1_Error = m1_DesiredAngle - m1_CurrentAngle; //finds error between desired and current angle for motor 1
   m2_Error = m2_DesiredAngle - m2_CurrentAngle; //finds error between desired and current angle for motor 1

    
      
 } 

 void loop() {
    
    m1_CurrentTime = millis(); //PID for motor 1
    m1_TimeChange =  m1_CurrentTime -  m1_LastTime;
    m1_Error = m1_DesiredAngle - m1_CurrentAngle;
    m1_D_Error = (m1_Error - m1_LastError)/m1_TimeChange;
    m1_i_Error = m1_i_Error + (m1_Error*m1_TimeChange);

    m2_CurrentTime = millis(); //PID for motor 2
    m2_TimeChange =  m2_CurrentTime -  m2_LastTime;
    m2_Error = m2_DesiredAngle - m2_CurrentAngle;
    m2_D_Error = (m2_Error - m2_LastError)/m2_TimeChange;
    m2_i_Error = m2_i_Error + (m2_Error*m2_TimeChange);
    
    m1_CurrentAngle = m1_MotorAngle;
    m2_CurrentAngle = m2_MotorAngle;

   
    
     Serial.print(m1_MotorAngle);
     Serial.print(",");
     Serial.println(m1_DesiredAngle);
    
    // Serial.print(",");

     // Serial.print(m2_MotorAngle);
     //Serial.print(",");
     //Serial.println(m2_DesiredAngle);
     
     //Serial.print(millis());
    // Serial.print(",");
     //Serial.print(m2_MotorAngle);
     //Serial.print(",");
     //Serial.println(m2_DesiredAngle);

 
   
    
    
    m1_Voltage = m1_Error*m1_Kp + m1_D_Error*m1_Kd + m1_Ki*m1_i_Error; //PID calc for motor 1
    m2_Voltage = m2_Error*m2_Kp + m2_D_Error*m2_Kd + m2_Ki*m2_i_Error; //PID calc for motor 2

    if (m1_Voltage > 255) { //sets a max output duty cycle to 255
      m1_Voltage = 255;
    }
    
    if (m1_Voltage < -255) { //sets a max output duty cycle to 255 (other direction)
      m1_Voltage = -255;
    }
    if (m1_Voltage > 0) { //sends output signal through h bridge
    analogWrite(m1_H_bridge_enable, m1_Voltage);
    digitalWrite(m1_in_1, LOW);
    digitalWrite(m1_in_2, HIGH);
    }
    if (m1_Voltage < 0 ) { //sends output signal through h bridge (other direction)
    analogWrite(m1_H_bridge_enable, abs(m1_Voltage));
    digitalWrite(m1_in_1, HIGH);
    digitalWrite(m1_in_2, LOW);
    }


    if (m2_Voltage > 255) { //sets a max output duty cycle to 255
      m2_Voltage = 255;
    }
    
    if (m2_Voltage < -255) { //sets a max output duty cycle to 255 (other direction)
      m2_Voltage = -255;
    }
    if (m2_Voltage > 0) {  //sends output signal through h bridge
    analogWrite(m2_H_bridge_enable, m2_Voltage);
    digitalWrite(m2_in_1, LOW);
    digitalWrite(m2_in_2, HIGH);
    }
    if (m2_Voltage < 0 ) { //sends output signal through h bridge (other direction)
    analogWrite(m2_H_bridge_enable, abs(m2_Voltage));
    digitalWrite(m2_in_1, HIGH);
    digitalWrite(m2_in_2, LOW);
    }

    myservo.write(servo_pos);
    
    m1_LastError = m1_Error; //updates last error for calcs
    m1_LastTime = m1_CurrentTime;
    
    m2_LastError = m2_Error; //updates last error for calcs
    m2_LastTime = m2_CurrentTime;
 }
    
void m1_printMotorValues(){ //function for printing motor 1 values
  Serial.print("Direction: ");
  Serial.print(m1_CurrentDirection);
  Serial.print(" | Counter: ");
  Serial.print(m1_Counter);
  Serial.print(" | Angle: ");
  Serial.println(m1_MotorAngle);
}

void m2_printMotorValues(){ //function for printing motor 2 values
  Serial.print("Direction: ");
  Serial.print(m2_CurrentDirection);
  Serial.print(" | Counter: ");
  Serial.print(m2_Counter);
  Serial.print(" | Angle: ");
  Serial.println(m2_MotorAngle);
}

void m1_EncoderCount() //interrupt that gets called on rising state for motor 1 channel A
                       //updates the encoder count and calculates the motor angle
{
 
m1_CurrentStateA = digitalRead(m1_EncoderA);
m1_CurrentStateB = digitalRead(m1_EncoderB);

  if(m1_CurrentStateA == m1_CurrentStateB)
  {
    m1_CurrentDirection = "CW";
    m1_Counter = m1_Counter +1;
  }

  if(m1_CurrentStateA != m1_CurrentStateB)
  {
    m1_CurrentDirection = "CCW";
    m1_Counter = m1_Counter -1;
  }

m1_MotorAngle = m1_Counter*0.4373709;
}

void m2_EncoderCount()//interrupt that gets called on rising state for motor 1 channel A
                       //updates the encoder count and calculates the motor angle
{
 
m2_CurrentStateA = digitalRead(m2_EncoderA);
m2_CurrentStateB = digitalRead(m2_EncoderB);

  if(m2_CurrentStateA == m2_CurrentStateB)
  {
    m2_CurrentDirection = "CW";
    m2_Counter = m2_Counter +1;
  }

  if(m2_CurrentStateA != m2_CurrentStateB)
  {
    m2_CurrentDirection = "CCW";
    m2_Counter = m2_Counter -1;
  }

m2_MotorAngle = m2_Counter*0.4373709;
}
