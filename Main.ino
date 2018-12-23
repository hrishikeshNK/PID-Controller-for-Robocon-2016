#include<Arduino.h>
#include<Servo.h>
#include<math.h> //Used to calculate the arctans for finding angles


float kp = 0.45, ki = 0.009, kd = 0.5;
/*kp, ki and kd each represent the gains or weights of the 'P', 'I' and 'D' variables 
  and determine how much of a say each of those terms are going ot have in the final actuation/output of the controller.
  Each of these gains have been fine tuned using trial and error method, so that the bot follows the white line on the track
  with minimal wobble
*/


bool blue = false; 
bool rightAngle = false;
float RawError, LastError = 0;
int ctr = 0, count = -1;



Servo k; // This object represents the servo motor, which is the final actuator of the bot and controls its direction.

class ldr { //Represents each of the 8 LDR sensors on the bot
  public:
    int whiteMean;  // Most occuring reading of white color for a particular sensor under current lighting conditions
    int up, down;   // Tolerance bounds(upper and lower) for white color
    void calibrate(int ID); // Method to calibrate the sensor
};

int s[8]; //Boolean values representing correponding LDRs being on or not being on the white line.(1 for on the white line, 0 for not)
int pr[8], prev[8];
ldr uno[8]; // Array of LDR sensor objects to store real-time readings

void calibrateAll()
{
  for (int i = 0; i < 8; i++) //Calibrate each of the sensors 
    uno[i].calibrate(i);


  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);              // wait for a second
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);              // wait for a second

}

void ldr::calibrate(int ID)
{
  int a[100];
  for (int i = 0; i < 100; i++)
  {
    switch(ID) // Take 100 readings for given sensor(denoted by 'ID')
    {
      case 0: a[i] = analogRead(A0); break;
      case 1: a[i] = analogRead(A1); break;
      case 2: a[i] = analogRead(A2); break;
      case 3: a[i] = analogRead(A3); break;
      case 4: a[i] = analogRead(A4); break;
      case 5: a[i] = analogRead(A5); break;
      case 6: a[i] = analogRead(A6); break;
      case 7: a[i] = analogRead(A7); break;
    }
    delay(10);
  }
  for (int i = 0; i < 100; i++)
  {
    int ct = 0; // Stores the number of readings that fall in the acceptable range for current lighting conditions
    for (int j = 0; j < 100; j++)  
      if (a[j] == a[i] || a[j] == (a[i] + 1) || a[j] == (a[i] - 1))
        ct += 1;
    if (ct >= 90) // If more than 90/100 values are consistent and fall within appropriate range
    {
      Serial.print("count=");
      Serial.println(ct);
      whiteMean = a[i]; // Set the WhiteMean to the central value
      break;
    }
    else    //Else try calibrating again
      return calibrate(ID); 
  }
  up = whiteMean + 10;
  down = whiteMean - 10;
  // Tolerance of +/-10 around the most occuring value
}                                //Calibrate Ends.

void ErrorCalc() {
  int sum = 0; // Represents how many LDRs in the LDR array are currently on the white line
  for (int i = 0; i < 8; i++)
  {
    sum = sum + s[i];
  }
  //RawError is simply the angle between the LDRs currently of the white line and the center of the LDR array which is the setpoint
  recheck:
  if (sum == 2)                 //This statement checks whether two ldrs are on the white line
  {
    if (s[3] == 1 && s[4] == 1)
      RawError = 0;
    else if (s[4] == 1 && s[5] == 1)
      RawError = 10.42;
    else if (s[5] == 1 && s[6] == 1)
      RawError = 20.20;
    else if (s[6] == 1 && s[7] == 1)
      RawError = 28.89;
    else if (s[2] == 1 && s[3] == 1)
      RawError = -10.42;
    else if (s[1] == 1 && s[2] == 1)
      RawError = -20.20;
    else if (s[0] == 1 && s[1] == 1)
      RawError = -28.89;
  }

  else if (sum == 1)             //Checks whether only 1 ldr is on white surface
  {
    if (s[0] == 1)
      RawError = -32.78;
    else if (s[1] == 1)
      RawError = -24.70;
    else if (s[2] == 1)
      RawError = -15.43;
    else if (s[3] == 1)
      RawError = -5.26;
    else if (s[4] == 1)
      RawError = 5.26;
    else if (s[5] == 1)
      RawError = 15.43;
    else if (s[6] == 1)
      RawError = 24.70;
    else if (s[7] == 1)
      RawError = 32.78;
  }

  else if (sum == 0) 
    RawError = 0;
  
  else if (sum >= 3) // More than 2 LDRs are on the white line
  {
    {
      for (int i = 3, j = 4; i >= 0, j >= 1; i--, j--)
        pr[i] = s[i] * (j); 

      for (int i = 4, j = 1; i <= 7, j <= 4; i++, j++)
        pr[i] = s[i] * (j);

      //Sensors on the extremities of the array get higher weights

      int greatest = pr[0]; // Greatest value after multiplying by weights as described above
      int gloc = 0;     //Sensor location of said greatest value
      for (int i = 0; i < 7; i++)
      {
        if (pr[i + 1] > greatest)
        {
          greatest = pr[i + 1];
          gloc = i + 1; // 
        }
      }
      for (int i = 0; i < 8; i++)
        s[i] = 0;           //Every sensor value except the greatest after weighting, is set to 0
      s[gloc] = 1;      // The greatest weighted sensor is set to 1 and thus determines the error
      sum = 1;
      a = gloc;
      rightAngle = true; //Bot is at the 90 degree turn on the track
      goto recheck;   // Jump back to recheck, with the highest valued sensor on the white line determining the error
    }
  }
  PID();   //Call the PID controller after error calculation                   
}    //End of findDist.



void AnatoBool() { //Converts the analog readings from the ldrs to boolean using ranges defined by the calibrate function
  for (int i = 0; i < 8; i++)
  {
    if (s[i] >= uno[i].down && s[i] <= uno[i].up)
      s[i] = 1;
    else
      s[i] = 0;
  }
}

void read() {
  int flag=0; 
  s[0] = analogRead(A0);
  s[1] = analogRead(A1);
  s[2] = analogRead(A2);
  s[3] = analogRead(A3);
  s[4] = analogRead(A4);
  s[5] = analogRead(A5);
  s[6] = analogRead(A6);
  s[7] = analogRead(A7); //Collect analog readings from each of the 8 sensors in the LDR array

  if(count>2){
    for(int i=0; i<8; i++){
       if(prev[i]>s[i]+5)
       flag+=1;
    }
    if(flag>=4)
       kp = 1; ki = 0; kd = 0.5; //Downhill detected
  }

  for(int i=0; i<8; i++) //Assign previous array readings
     prev[i]=s[i];
}

void display() {
  for (int i = 0; i < 8; i++)
  {
    Serial.print(s[i]);
    Serial.print("\t");
  }
  Serial.println("");
}


void PID()
{
  float P, I, D; // 'P', 'I', 'D' represent each the Proportional, Derivative and Integral values for the error respectively
  float Drive;  //The final actuation in terms of angle, given to the servo motor

  RawError = -RawError;

  P = RawError; // Proportional value equal to the instantaneous Error 
  I = I + RawError;

  if (LastError != RawError)
    D = LastError - RawError;
  else
    D = 0;
  if (RawError < 0)
  {
    Drive = (P * kp) + (I * ki) + (D * kd);
  }

  else
    Drive = (P * kp) + (I * ki) + (D * (-1 * kd));
  (int)Drive;
  if (rightAngle == true)
  {
    Drive = Drive * 3; //3 Times the actuation if the bot is at a 90 degree turn
    rightAngle = false;
    count++;
  }

  if (count == 1)
    Drive = 50;
  if (count == 2) {
    kp = 1.5; ki = 0; kd = 0.5;
  }
  if (count == 5) {
    kp = 1; ki = 0; kd = 0.5;
  }
  
  if (LastError != RawError && RawError != 0)
  {

    k.write(90 + Drive);
    if (count == -1)
      delay(5000);
  }
  Serial.print("\nDrive=");
  Serial.println(90 + Drive);
  LastError = RawError;
}

void setup() {
  Serial.begin(9600); // Sets the baud rate at 9600
  k.attach(9);    // Attach the servo motor on digital pin 9(Arduino Mega 2560)
  k.write(70);    
  calibrateAll(); // Calibrate LDR sensors
  k.write(90);
  for (int i = 0; i < 8; i++)
    Serial.print(uno[i].up); Serial.print("\t");  Serial.println(uno[i].down);
  Serial.println("\n\n");
}



void loop()
{
  read(); 
  display();
  AnatoBool();
  display();
  ErrorCalc();
}
