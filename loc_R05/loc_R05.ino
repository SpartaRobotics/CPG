// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ VARIABLES ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


#define EM 10
#define DOCK 11
#define COUNT 0
#define A 9999
#define pi 3.1415926535
#define THETAINERT 0
#include <math.h>

int SwitchCase;
int RMsignal;
int MasterSwitch;
int xfToln;
int XfTolp;
int YfToln;
int YfTolp;
int THRUSTER[] = {2, 3, 4, 5, 6, 7, 8, 9};
float THETA;
float THETAF;
float theta2;
float thetaTol;
float thetaCheckp;
float thetaCheckn;
float thetaCheckpf;
float thetaChecknf;
float X1;
float X2;
float X;
float Y2;
float Y1;
float Y;
float XF;
float YF;
float thetaf;

long THETA_deg = 0;
long THETAF_deg = 0;

float J0 = 0.087; //Cost Function for X translation
float J1 = 0.087; //Cost Function for Y translation
float J2 = 0.087;
float J3 = 0.087; //Must implement J4 and J5. Help the program generate automatic cost functions


float line1_y_vect;
float line1_x_vect;
float line2_x_vect;
float line2_y_vect;
int flag; //This is the flag that determines if the vehicles are in a position to dock or not.
int tol = 50; //YTolerance is 5 centimeters
int line1; //[x, y]Line 1. Determines a single line manuver
int line2; //[x, y]Line 2 . Dettermes the two line manuever
int line3; //The first distance in the 3 line manuever
int line22; //The second distance in the second line manuever
int line32; //The second distance in the 3 line manuever
int line33; //The third distance in the 3 line manuver
int CounterX = 5;
int CounterY = 5;
int Xtdiv;
int Ytdiv;

bool rlyflag = false;
long timer1 = 0;

int xDP = 1828.8; //X-dimension of the demonstartion platform
int yDP = 2438.4; //Y-Dimension of the demonstration platform
int xtolop = 20; //Iniital positive Xtolorence for path generator
int xtolon = -200; //[cm]Iniital negative Xtolorence for pathgenerator
int ytolop = 200; //[cm]Ininital Ytolorence for path generator that is positive - 20 cm
int ytolon = -200; //[cm]Y initial tologernece for path generator that is negative 20cm
int tolm = 0.5; //tolorence for slope of the line - used by path generator 20 cm

int pathGenSwitch = 0; //Determines the line vectors
float mDP = yDP / xDP; //Slope of the Demonstration Platform
float mDPinv = -1/mDP; //Slope of the perpendicular line drawn to this line
double thetaDP = atan((double)mDP); //[Radians] angle formed by the diagonal of the table
double thetaDPinv = atan((double)mDPinv); //[Radians] inverse of the angle formed by the diagonal of the table

int allowedX = 60; // Allowable distance between the chase and target during translation in the X direction
int allowedY = 60; // Allowable distance between the chase and target during translation in the y diection
float xtoltrans = 0.4; //Allowed X disatnce between the Chase and target while translation
float ytoltrans = 0.4; //Allowed Y disatnce between the Chase and target while translation
float txdiv; //[S]time differenc between firing thrusters while translting in x
float tydiv; //[S]time differenc between firing thrusters while translting in y
int xfirst = 0; //keeps track of in what direction the vehicle needs to move first - in X first
int yfirst = 0; //keeps track of in what direction the vehicle needs to move first - in Y first
int line = 0; //Keeps track of how many lines

int ROTATE = 0;
int TRANSLATE = 0;
int ROTATE2BODY = 0;
int STATIONKEEP = 0;
int RETURN_rotate = 0;
int RETURN_translate = 0; 
int TURNOFF = 0;

int xory;
int divisor;
int len = 0;
int j = 0;
int dir;
int rem;
float ydiv;
float xdiv;

int dir2;
int dir3;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~StationKeeping initializations~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
boolean freeze = true; // Variable to capture current point
long timer2 = 0;
int DBtol; // Dead Band Tolerence
int X_bound_p;
int X_bound_n;
int Y_bound_p;
int Y_bound_n;
int X_SK;
int Y_SK;
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ SETUP ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



void setup() {

  // put your setup code here, to run once:
  Serial1.begin(38400); // For the HC-05 Bluetooth
  Serial3.begin(115200); // To the Robotic Manipulator
  Serial1.setTimeout(20); //configures timeout for comms to 20 ms

  for (int thisPin = 0; thisPin < 8; thisPin++) {
    pinMode(THRUSTER[thisPin], OUTPUT); //Sets the pins as an output
  }

  pinMode(EM, OUTPUT);             //Sets the pin as an output
  pinMode(DOCK, INPUT);            //Sets the pin as an input

}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ VOID LOOP ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void loop() {
/* REMOVE
  // put main code here, to run repeatedly:


  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~Check for signals from VTS and define inputs
  if (Serial1.available() > 0) {

    if (Serial1.peek() == 'c')  {
      X = Serial1.parseInt(); //x pos of castor in mm
      Y = Serial1.parseInt(); // y pos of castor in mm
      THETA_deg = Serial1.parseInt(); //angle of castor in degrees*10
      THETA = THETA_deg * pi / 180 / 10; //angle of castor in radians
      XF = Serial1.parseInt(); //x pos of pollux in mm
      YF = Serial1.parseInt(); // y pos of pollux in mm
      THETAF_deg = Serial1.parseInt(); //angle of pollux in degrees*10
      THETAF = THETAF_deg * pi / 180 / 10; //angle of pollux in radians
      ROTATE = 1;
    }
    else {
      Serial1.read();
    }
  }

  
  // Signal passing for docking and refueling

  if (ROTATE){
    Rotate2Inertial(THETAINERT, THETA, THRUSTER, pi, thetaTol, thetaCheckp, thetaCheckn, SwitchCase);
    ROTATE = 0;
    TRANSLATE = 1;
  }

  if (TRANSLATE || RETURN_translate) {
     if (TRANSLATE) {
            if (pathGenSwitch > 300)
            {
              len = 3;
            }
            else if (pathGenSwitch > 200 && pathGenSwitch < 300)
            {
              len = 2;
            }
            else
            {
              len = 1;
            }
          
            xory = (pathGenSwitch % 100)/10;
            rem = pathGenSwitch % 10;
          
          
            if (len == 1 && xory == 1)
            {
              ydiv = abs(line1) / 5;
              tydiv = ydiv / J1;
              float Yarray[] = {ydiv, ydiv, ydiv, ydiv, ydiv}; 
              ytranslation(rem, Yarray, tydiv);;
            }
            else if (len == 1 && xory == 0)
            {
              xdiv = abs(line1) / 5;
              txdiv = xdiv / J0;
              float Xarray[] = {xdiv, xdiv, xdiv, xdiv, xdiv};
              xtranslation(rem, Xarray, txdiv);
            }  
            else if (len == 2 && xory == 1)
            {
              ydiv = abs(line1) / 5;
              tydiv = xdiv / J1;
              float Yarray[] = {ydiv, ydiv, ydiv, ydiv, ydiv}; 
              ytranslation(rem, Yarray, tydiv);
          
              xdiv = abs(line2) / 5;
              txdiv = xdiv / J0;
              if (line2 < 0)
              {
                dir2 = 0;
              }
              else
              {
                dir2 = 1;
              }
              float Xarray[] = {xdiv, xdiv, xdiv, xdiv, xdiv};
              xtranslation(dir2, Xarray, txdiv);
            }
            else if(len == 2 && xory == 0)
            {
              xdiv = line1 / 5;
              txdiv = xdiv / J0;
              float Xarray[] = {xdiv, xdiv, xdiv, xdiv, xdiv};
              xtranslation(rem, Xarray, txdiv);
          
              ydiv = line2 / 5;
              tydiv = ydiv / J1;
              if (line2 < 0)
              {
                dir2 = 0;
              }
              else
              {
                dir2 = 1;
              }
              Yarray[] = {ydiv, ydiv, ydiv, ydiv, ydiv}; 
              ytranslation(dir2, Yarray, tydiv);
            }
            else if(len == 3 && xory == 0)
            {
              xdiv = abs(line1);
              txdiv = xdiv / J0;
              dir = 1;
              Xarray[] = {xdiv};
              xtranslation(dir, Xarray, txdiv);
          
              ydiv = abs(line2) / 5;
             
              tydiv = ydiv / J1;
              if (line2 < 0)
              {
                dir2 = 0;
              }
              else
              {
                dir2 = 1;
              }
              Yarray[] = {ydiv, ydiv, ydiv};
              ytranslation(dir2, Yarray, tydiv);
          
              xdiv = abs(line3);
              txdiv = xdiv / J0;
              dir3 = !dir;
              xtranslation(dir3, Xarray, txdiv);
              
            }
            else if (len == 3 && xory == 1)
            {
              ydiv = abs(line1);
              tydiv = ydiv / J1;
              dir = 1;
              Yarray[] = {ydiv};
              ytranslation(dir, Yarray, tydiv);
          
              xdiv = abs(line2) / 5;
              txdiv = line2 / J0;
              if (line2 < 0)
              {
                dir2 = 0;
              }
              else
              {
                dir2 = 1;
              }
              Xarray[] = {xdiv, xdiv, xdiv, xdiv, xdiv};
              xtranslation(dir2, Xarray, txdiv);
          
              ydiv = line3;
              tydiv = line3 / J1;
              dir3 = !dir;
              Yarray[] = {ydiv};
              ytranslation(dir3, Yarray, tydiv);
              
            }
          TRANSLATE = 0;
          ROTATE2BODY = 1;
      }
    
      int line1return;
      //Line1 used to return to the path
      
       if (RETURN_translate)
      {
        
        if (len == 3 && xory == 0)
        {
          xtranslation(!dir3, Xarray, txdiv);
          ytranslation(!dir2, Yarray, tydiv);
          xtranslation(!dir, Xarray, txdiv);      
        }
    
        else if(len == 3 && xory == 1)
        {
          ytranslation(!dir3, Yarray, tydiv);
          xtranslation(!dir2, Xarray, txdiv);
          ytranslation(!dir, Yarray, tydiv);
        }
    
        else if(len == 2 && xory == 0)
        {
          ytranslation(!dir2, Yarray, tydiv);
          xtranslation(!dir, Xarray, txdiv); 
        }
        else if(len == 2 && xory == 1)
        {
          xtranslation(!dir2, Xarray, txdiv);
          ytranslation(!dir, Yarray, tydiv);
        }
        else if(len ==1 && xory == 1)
        {
          ytranslation(!rem, Yarray, tydiv); 
        }
        else if(len ==1 && xory == 1)
        {
          xtranslation(!rem, Xarray, txdiv);
        }
        RETURN_translate = 0;
        TURNOFF = 1;
      }
 }

  if (ROTATE2BODY) {
    Rotate2Body(THETAF, pi, thetaTol, thetaCheckpf, thetaChecknf, THETA, THRUSTER, SwitchCase)
    ROTATE2BODY = 0;
    STATIONKEEP = 1;
  }
  
  if (STATIONKEEP) {
    float StationKeeping();
    
    int docking = 1;
    Serial3.write(docking); // begin pulling us closer
    
    while (Serial3.read() != 1) {
    }
    
    digitalWrite(EM) = HIGH; // Check
    
    while (digitalRead(DOCK) != LOW) { // docking succesful (in contact)
    }
    
    int refuel = 1;
    Serial3.write(refuel);
    
    while (Serial3.read != 1) {
    }
    digitalWrite(EM) = LOW;
    
    STATIONKEEP = 0;
    RETURN_rotate = 1;
  }

  if (RETURN_rotate) {
    Rotate2Inertial(THETAINERT, THETA, THRUSTER, pi, thetaTol, thetaCheckp, thetaCheckn, SwitchCase);
    RETURN_rotate = 0;
    RETURN_translate = 1;
    // Put return translation code here
  }
  
 if (TURNOFF) { // STATIONKEEPING IS NEEDED HERE AS WELL
    for (COUNT = 0; COUNT < 8; COUNT++) {
      digitalWrite(THRUSTER[COUNT], LOW); // Turn off controller - singal 0 to all solenoids
    }
    
  }
REMOVE */
}


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ STATION KEEPING AND SIGNAL MANAGEMENT~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

int StationKeeping() {
// Add station keeping code here

if (freeze) {
      X_SK = X; // Target X
      Y_SK = Y; // Target Y
      freeze = false;
    }
        
    X_bound_p = X_SK + DBtol;
    X_bound_n = X_SK - DBtol;
    Y_bound_p = Y_SK + DBtol;
    Y_bound_n = Y_SK - DBtol;
    
    if (THETA < pi/4 && THETA > 7*pi/4) {
      
      while (X > X_bound_p) {
        digitalWrite(THRUSTER[5], HIGH);
      }
      digitalWrite(THRUSTER[2,3,4,5], LOW);
      
      while (X < X_bound_n) {
        digitalWrite(THRUSTER[4], HIGH);
      }
      digitalWrite(THRUSTER[2,3,4,5], LOW);
      
      while (Y > Y_bound_p) {
        digitalWrite(THRUSTER[3], HIGH);
      }
      digitalWrite(THRUSTER[2,3,4,5], LOW);
      
      while (Y < Y_bound_n) {
        digitalWrite(THRUSTER[2], HIGH);
      }
      digitalWrite(THRUSTER[2,3,4,5], LOW);
      
    }
    
    else if (THETA > pi/4 && THETA < 3*pi/4) {
      while (X > X_bound_p) {
        digitalWrite(THRUSTER[2], HIGH);
      }
      digitalWrite(THRUSTER[2,3,4,5], LOW);
      
      while (X < X_bound_n) {
        digitalWrite(THRUSTER[3], HIGH);
      }
      digitalWrite(THRUSTER[2,3,4,5], LOW);
      
      while (Y > Y_bound_p) {
        digitalWrite(THRUSTER[5], HIGH);
      }
      digitalWrite(THRUSTER[2,3,4,5], LOW);
      
      while (Y < Y_bound_n) {
        digitalWrite(THRUSTER[4], HIGH);
      }
      digitalWrite(THRUSTER[2,3,4,5], LOW);
    }
      
      else if (THETA < 3*pi/4 && THETA > 5*pi/4) {
      
      while (X > X_bound_p) {
        digitalWrite(THRUSTER[4], HIGH);
      }
      digitalWrite(THRUSTER[2,3,4,5], LOW);
      
      while (X < X_bound_n) {
        digitalWrite(THRUSTER[5], HIGH);
      }
      digitalWrite(THRUSTER[2,3,4,5], LOW);
      
      while (Y > Y_bound_p) {
        digitalWrite(THRUSTER[2], HIGH);
      }
      digitalWrite(THRUSTER[2,3,4,5], LOW);
      
      while (Y < Y_bound_n) {
        digitalWrite(THRUSTER[3], HIGH);
      }
      digitalWrite(THRUSTER[2,3,4,5], LOW);
      
    }
    else {
      while (X > X_bound_p) {
        digitalWrite(THRUSTER[3], HIGH);
      }
      digitalWrite(THRUSTER[2,3,4,5], LOW);
      
      while (X < X_bound_n) {
        digitalWrite(THRUSTER[2], HIGH);
      }
      digitalWrite(THRUSTER[2,3,4,5], LOW);
      
      while (Y > Y_bound_p) {
        digitalWrite(THRUSTER[4], HIGH);
      }
      digitalWrite(THRUSTER[2,3,4,5], LOW);
      
      while (Y < Y_bound_n) {
        digitalWrite(THRUSTER[5], HIGH);
      }
      digitalWrite(THRUSTER[2,3,4,5], LOW);
    }
        
  }

  
  int caputure = 1;
  Serial3.write(capture);

  while (Serial3.read() != 1) {
    for (COUNT = 0; COUNT < 8; COUNT++) {
      digitalWrite(THRUSTERS[COUNT], LOW); // Turn off controller - singal 0 to all solenoids
    }
  }

  
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ROTATE TO INERTIAL FUNCTION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void Rotate2Inertial(THETAINERT, THETA, THRUSTER, pi, thetaTol, thetaCheckp, thetaCheckn, SwitchCase)
{
  //function for the rotation to the inertial frame code
  thetaTol = 5 * pi / 180; // Tolerence of +- 5°
  thetaCheckp = THETAINERT + thetaTol; // Max value
  thetaCheckn = THETAINERT - thetaTol; // Min value

  // If the RMsignal is a 0, then we have not performed docking, if it is a 1, then docking has been performed

  if THETA > thetaCheckp {
  SwitchCase = 1;
}
else if THETA < thetaCheckn {
  SwitchCase = 2;
}
else {
  SwitchCase = 0
}

switch (SwitchCase) {
    case 1:
      while THETA > thetaCheckp {
      digitalWrite(THRUSTER[8, 9], HIGH); // Turn on - thrusters
        timer1 = millis();
        rlyflag = true;
        if (rlyflag && (millis() - timer) > 1000) {
          digitalWrite(THRUSTER[8, 9], LOW); // Turn off - thrusters
          rlyflag = false;
        }
        timer1 = millis();
        rlyflag = true;
        if (rlyflag && (millis() - timer) > 500) {
          digitalWrite(THRUSTER[6, 7], HIGH); // Turn on + thrusters
          rlyflag = false;
        }
        timer1 = millis();
        rlyflag = true;
        if (rlyflag && (millis() - timer) > 1000) {
          digitalWrite(THRUSTER[6, 7], LOW); // Turn off + thrusters
          rlyflag = false;
        }
      }
      break;

    case 2:
      while THETA < thetaCheckn {
      digitalWrite(THRUSTER[6, 7], HIGH); // Turn on - thrusters
        timer1 = millis();
        rlyflag = true;
        if (rlyflag && (millis() - timer) > 1000) {
          digitalWrite(THRUSTER[6, 7], LOW); // Turn off - thrusters
          rlyflag = false;
        }
        timer1 = millis();
        rlyflag = true;
        if (rlyflag && (millis() - timer) > 500) {
          digitalWrite(THRUSTER[8, 9], HIGH); // Turn on + thrusters
          rlyflag = false;
        }
        timer1 = millis();
        rlyflag = true;
        if (rlyflag && (millis() - timer) > 1000) {
          digitalWrite(THRUSTER[8, 9], LOW); // Turn off + thrusters
          rlyflag = false;
        }
      }
      break;

    default:
      // Station keeping
      if THETA < thetaCheckn {
      digitalWrite(THRUSTER[8, 9], HIGH); // Turn on - thrusters
        timer1 = millis();
        rlyflag = true;
        if (rlyflag && (millis() - timer) > 10) {
          digitalWrite(THRUSTER[6, 7], HIGH); // Turn off - thrusters
          rlyflag = false;
        }
        timer1 = millis();
        rlyflag = true;
        if (rlyflag && (millis() - timer) > 10) {
          digitalWrite(THRUSTER[8, 9], LOW); // Turn on + thrusters
          rlyflag = false;
        }
        timer1 = millis();
        rlyflag = true;
        if (rlyflag && (millis() - timer) > 10) {
          digitalWrite(THRUSTER[6, 7], LOW); // Turn off + thrusters
          rlyflag = false;
        }


        else if THETA > thetaCheckp {
        digitalWrite(THRUSTER[6, 7], HIGH); // Turn on - thrusters
          timer1 = millis();
          rlyflag = true;
          if (rlyflag && (millis() - timer) > 100) {
            digitalWrite(THRUSTER[8, 9], HIGH); // Turn off - thrusters
            rlyflag = false;
          }
          timer1 = millis();
          rlyflag = true;
          if (rlyflag && (millis() - timer) > 100) {
            digitalWrite(THRUSTER[6, 7], LOW); // Turn on + thrusters
            rlyflag = false;
          }
          timer1 = millis();
          rlyflag = true;
          if (rlyflag && (millis() - timer) > 100) {
            digitalWrite(THRUSTER[8, 9], LOW); // Turn off + thrusters
            rlyflag = false;
          }


          else{
            break;
          }
          break;
        }
      }

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ROTATE TO BODY FUNCTION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void Rotate2Body(THETAF, pi, thetaTol, thetaCheckpf, thetaChecknf, THETA, THRUSTER, SwitchCase)
      {
        //function for the rotation to the target angle
        thetaTol = 5 * pi / 180; // Tolerence of +- 5°
        thetaCheckpf = THETAF + pi + thetaTol; // Max value
        thetaChecknf = THETAF + pi - thetaTol; // Min value

        // If the RMsignal is a 0, then we have not performed docking, if it is a 1, then docking has been performed

        if THETA > thetaCheckpf {
        SwitchCase = 1;
      }
      else if THETA < thetaChecknf {
        SwitchCase = 2;
      }
      else {
        SwitchCase = 0
      }

      switch (SwitchCase) {
          case 1:
            while THETA > thetaCheckpf {
            digitalWrite(THRUSTER[8, 9], HIGH); // Turn on - thrusters
              timer1 = millis();
              rlyflag = true;
              if (rlyflag && (millis() - timer) > 1000) {
                digitalWrite(THRUSTER[8, 9], LOW); // Turn off - thrusters
                rlyflag = false;
              }
              timer1 = millis();
              rlyflag = true;
              if (rlyflag && (millis() - timer) > 500) {
                digitalWrite(THRUSTER[6, 7], HIGH); // Turn on + thrusters
                rlyflag = false;
              }
              timer1 = millis();
              rlyflag = true;
              if (rlyflag && (millis() - timer) > 1000) {
                digitalWrite(THRUSTER[6, 7], LOW); // Turn off + thrusters
                rlyflag = false;
              }
            }
            break;

          case 2:
            while THETA < thetaChecknf {
            digitalWrite(THRUSTER[6, 7], HIGH); // Turn on - thrusters
              timer1 = millis();
              rlyflag = true;
              if (rlyflag && (millis() - timer) > 1000) {
                digitalWrite(THRUSTER[6, 7], LOW); // Turn off - thrusters
                rlyflag = false;
              }
              timer1 = millis();
              rlyflag = true;
              if (rlyflag && (millis() - timer) > 500) {
                digitalWrite(THRUSTER[8, 9], HIGH); // Turn on + thrusters
                rlyflag = false;
              }
              timer1 = millis();
              rlyflag = true;
              if (rlyflag && (millis() - timer) > 1000) {
                digitalWrite(THRUSTER[8, 9], LOW); // Turn off + thrusters
                rlyflag = false;
              }
            }
            break;

          default:
            // Station keeping
            if THETA < thetaChecknf {
            digitalWrite(THRUSTER[8, 9], HIGH); // Turn on - thrusters
              timer1 = millis();
              rlyflag = true;
              if (rlyflag && (millis() - timer) > 100) {
                digitalWrite(THRUSTER[6, 7], HIGH); // Turn off - thrusters
                rlyflag = false;
              }
              timer1 = millis();
              rlyflag = true;
              if (rlyflag && (millis() - timer) > 100) {
                digitalWrite(THRUSTER[8, 9], LOW); // Turn on + thrusters
                rlyflag = false;
              }
              timer1 = millis();
              rlyflag = true;
              if (rlyflag && (millis() - timer) > 100) {
                digitalWrite(THRUSTER[6, 7], LOW); // Turn off + thrusters
                rlyflag = false;
              }
            }
            else if THETA > thetaCheckpf {
            digitalWrite(THRUSTER[6, 7], HIGH); // Turn on - thrusters
              timer1 = millis();
              rlyflag = true;
              if (rlyflag && (millis() - timer) > 100) {
                digitalWrite(THRUSTER[8, 9], HIGH); // Turn off - thrusters
                rlyflag = false;
              }
              timer1 = millis();
              rlyflag = true;
              if (rlyflag && (millis() - timer) > 100) {
                digitalWrite(THRUSTER[6, 7], LOW); // Turn on + thrusters
                rlyflag = false;
              }
              timer1 = millis();
              rlyflag = true;
              if (rlyflag && (millis() - timer) > 100) {
                digitalWrite(THRUSTER[8, 9], LOW); // Turn off + thrusters
                rlyflag = false;
              }
            }
            else {
              break;
            }
            break;
        }
      }


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ PATHGENERATOR FUNCTION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

float pathgenerator(float xdiff, float ydiff)
{

      float diffx = 0;
      float diffy = 0;

      diffx = (XF - X);
      //Difference in the x coordinates
      
      diffy = (YF - Y);
      //Differenc in the y ordinates

      if (((abs(diffx) >= xtolon && abs(diffx) <= xtolop)) && (((round(THETA) - THETAtol) >= (3*pi/2)) && (round(THETA + THETAtol) || (((THETA+THETAtol) <= pi/2) && ((THETA-THETAtol) >= pi/2)))))
      {
          if (((round(THETA+THETAtol) <= pi/2) && (round(THETA-THETAtol) >= pi/2)) && (diffy > 0))
          {
            pathGenSwitch = 301;
            //Three line manuever first in x direction. TheTarget vehicle is in the forward direction WRT the inertial frame. 
            //This follows xyx. In the y direction it has to travel extra 25 cm.
          }
          else if (((round(THETA+THETAtol) <= pi/2) && (round(THETA-THETAtol) >= pi/2)) && (diffy < 0))
          {
            pathGenSwitch = 110;
            //One line manuever first in Y direction. The traget vechicle is in the backward direction WRT the intertial frame
          }

          else if ((((round(THETA) - THETAtol) >= (3*pi/2)) && (round(THETA + THETAtol) <= (3*pi/2 ))) && (diffy > 0))
          {
            pathGenSwitch = 111;
            //One line manuever first in Y direction. The Target vehicel is inn forward direction WRT the inertial frame
          }

          else if ((((round(THETA) - THETAtol) >= (3*pi/2)) && (round(THETA + THETAtol) <= (3*pi/2 ))) && (diffy < 0))
          {
            pathGenSwitch = 300;
            //Three line manuever first in Y direction. The target vechile is in the backward direction WRT the inertial frame
          }

      }

      else if ( (diffy >= ytolon && diffy <= ytolop) &&((( 0 <= (THETA+THETAtol)) && (0 >=  (THETA-THETAtol))) ||(( pi <= (THETA+THETAtol)) && (pi >=  (THETA-THETAtol))))) 
      {
          if ((( 0 <= (THETA+THETAtol)) && (0 >=  (THETA-THETAtol))) && (xdiff > 0))
          {
            pathGenSwitch = 311;
            //Three line manuever. First translate in X. The target vechole is in the forward direction WRT in the intertial frame
          }
          else if ((( 0 <= (THETA+THETAtol)) && (0 >=  (THETA-THETAtol))) && (xdiff < 0))
          {
            pathGenSwitch = 100;
            //One line manyecer, first translate in X. The target vehicle is in the backward direction WRT the Chase vehicle in the inertial frame
          }

          else if ((( pi <= (THETA+THETAtol)) && (pi >=  (THETA-THETAtol))) && (xdiff > 0))
          {
            pathGenSwitch = 101;
            //One line manuver first translate in X. The target vehicle is in the forward durection WRT the chase vehicle in the inertial frame
          }
          else if ((( pi <= (THETA+THETAtol)) && (pi >=  (THETA-THETAtol))) && (xdiff < 0))
          {
            pathGenSwitch = 310;
            //Three line manuever first translate in X. The target vehicle is in the backward direction WRT the chase vehicle in the inertial frame
          }
        
      }

      else
     {
       
       float m = 0;
        //Creates  a new variable m for the slope of the line formed by the ceneter of the coordinate frame and the target vehicle

        m = YF / XF;
        //Slope of the line formed by the ceneter of the target vehicle and the coordinate frame
       
        if ((((X < 0) && (Y < 0))  && ((XF > 0) && (YF > 0))) || (((X < 0) && (Y > 0))  && ((XF > 0) && (YF < 0))) || (((X > 0) && (Y > 0)) && ((XF < 0) && (YF > 0))) || (((X > 0) && (Y < 0)) && ((XF < 0) && (YF > 0))))
        //Case 1, 2, 3, and 4 when the chase and the target vehicles are in the opposite quadrants.
        {
          if ((XF > 0 && YF > 0) && (THETAF > thetaDP && THETAF < pi+thetaDP))
          //Check for the first quadrant
          {
             pathGenSwitch = 211;
             //This means - it is a two line manuever. Translate in y first. The target vehicle is in th forawd direction WRT the Chase Vehicle
          }   

          else if ((XF > 0 && YF > 0) && (THETAF < thetaDP && THETAF > pi+thetaDP))
          {
            pathGenSwitch = 201;
            //This means - it is a two line manuever. Translate in x first. The target vehicle is in th forawd direction WRT the Chase Vehicle in the inertial coordinate frame         
          }

          else if ((XF < 0 && YF < 0) && (THETAF > thetaDP && THETAF < pi+thetaDP))
          {
            pathGenSwitch = 200;
            //This means - it is a two line manuever. Translate in x first. The target vehicle is in the backward direction WRT the Chase Vehiclei in the Inertial coordinate frame
          }

          else if  ((XF < 0 && YF < 0) && (THETAF < thetaDP && THETAF > pi+thetaDP))
          {
            pathGenSwitch = 210;
            //This means - it is a two line manuever. Translate in y first. The target vehicle is in the backward direction WRT the Chase Vehiclei in the Inertial coordinate frame
          }
        }  

        else
        {
          if ((xdiff > 0 && ydiff > 0) && ((THETAF > thetaDP && THETAF < pi+thetaDP)))
          {
            pathGenSwitch = 211;
            //This means - it is a two line manuever. Translate in y first. The target vehicle is in th forawd direction WRT the Chase Vehicle in the inertial coordinate frame         
          }
          else if ((xdiff > 0 && ydiff > 0) && (THETAF < thetaDP && THETAF > pi+thetaDP))
          {
             pathGenSwitch = 201;
             //This means - it is a two line manuever. Translate in x first. The target vehicle is in th forawd direction WRT the Chase Vehicle in the inertial coordinate frame   
            
          }
          else if ((xdiff > 0 && ydiff < 0) && ((THETAF > thetaDPinv && THETAF < pi+thetaDPinv)))
          {
            pathGenSwitch = 200;
            //This means - it is a two line manuever. Translate in x first. The target vehicle is in th backward direction WRT the Chase Vehicle in the inertial coordinate frame   
          }

          else if ((xdiff > 0 && ydiff < 0) && ((THETAF < thetaDPinv && THETAF > pi+thetaDPinv)))
          {
            pathGenSwitch = 210;
            //This means - it is a two line manuever. Translate in y first. The target vehicle is in the backward direction WRT the Chase Vehicle in the inertial coordinate frame   
            
          }

          else if((xdiff < 0 && ydiff > 0) && ((THETAF > thetaDPinv && THETAF < pi+thetaDPinv)))
          {
            pathGenSwitch = 211;
             //This means - it is a two line manuever. Translate in y first. The target vehicle is in the forward direction WRT the Chase Vehicle in the inertial coordinate frame   
          }
          else if((xdiff < 0 && ydiff > 0) && ((THETAF < thetaDPinv && THETAF > pi+thetaDPinv)))
          {
             pathGenSwitch = 201;
             //This means - it is a two line manuever. Translate in x first. The target vehicle is in the forward direction WRT the Chase Vehicle in the inertial coordinate frame 
          }
          else if((xdiff < 0 && ydiff < 0) && ((THETAF > thetaDP && THETAF < pi+thetaDP)))
          {
             pathGenSwitch = 200;
             //This means - it is a two line manuever. Translate in x first. The target vehicle is in the backward direction WRT the Chase Vehicle in the inertial coordinate frame 
          }
          else if((xdiff < 0 && ydiff < 0) && ((THETAF < thetaDP && THETAF > pi+thetaDP)))
          {
             pathGenSwitch = 210;
             //This means - it is a two line manuever. Translate in y first. The target vehicle is in the backward direction WRT the Chase Vehicle in the inertial coordinate frame 
          }
                    
        }
        
     }

     return xdiff, ydiff;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ TRANSLATION FUNCTION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void Translation(float xdiff, float ydiff)
//This fumction determines the vectors for the motion
{
  line = 0;
  xfirst = 0;
  yfirst = 0;
    switch (pathGenSwitch)
    {
      case 100:
        line1 = xdiff - xtoltrans;  
        break;
        
      case 101:
        line1 = xdiff - xtoltrans;
        //Distance in x
  
        break;
  
      case 110:
        line1 = ydiff - ytoltrans;
        //Distance in y
        break;
      
      case 111:
  
        line1 = ydiff - ytoltrans;
        //Distance the chase vehicle has to move in the y-direction
        break;
            
      case 200:
        line1 = xdiff - xtoltrans;
        line2 = ydiff - ytoltrans;
        break;
      case 201:
        line1 = xdiff - xtoltrans;
        line2 = ydiff - ytoltrans;
        break;
      case 210:
        line1 = ydiff - ytoltrans;
        line2 = xdiff - xtoltrans;
        break;
      case 211:
        line1 = ydiff - ytoltrans;
        line2 = xdiff - xtoltrans;
        break;
      case 300:
        line1 = allowedX;
        line2 = -(ydiff + ytoltrans);
        line3 = -allowedX;
       break;
      case 301:
        line1 = allowedX;
        line2 = (ydiff + ytoltrans);
        line3 = -allowedX;
       break;
      case 310:
        line1 = allowedY;
        line2 = -(xdiff + xtoltrans);
        line3 = -allowedY;
        break;
      case 311:
        line1 = allowedY;
        line2 = (xdiff + xtoltrans);
        line3 = -allowedY;
     break;
      default: break;
    }
}
              
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ y translation FUNCTION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void ytranslation(int SwitchTranslation, float Yarray[], int tydiv)
{

  //[]Determines what case is valid
  int tol = 1000;
  //[cm]Y tolorenece

   int timer;
   //[]Used to keep track of time the thrusters are going to be on for

   int count = 1;
   
  while ((Y+tol >= YF) || (Y-tol <= YF))
  {
 
    switch (SwitchTranslation)
    {
      case 1:

        digitalWrite(THRUSTER[2], HIGH); //Turns thruster 2 on such that the vehicle moves forward
        timer1 = millis();
        rlyflag = true;
        if (rlyflag && (millis()-timer) > tydiv/2)
        {
          digitalWrite(THRUSTER[2], LOW); // Turns thruster 2 off off - thrusters 
          rlyflag = false; 
        }
        
        digitalWrite(THRUSTER[3], HIGH); //Turns the backward thruster on sich that the vehicle comes
        //to a a stop at the division
         timer1 = millis();
        rlyflag = true;
        if (rlyflag && (millis()-timer) > tydiv/2)
        {
          digitalWrite(THRUSTER[3], LOW); // Turns thruster 2 off off - thrusters 
          rlyflag = false; 
          
        }
        count++;
        break;

      case 0:
        digitalWrite(THRUSTER[3], HIGH); //Turns thruster 3 on such that the vehicle moves forward
         timer1 = millis();
        rlyflag = true;
        if (rlyflag && (millis()-timer) > tydiv/2)
        {
          digitalWrite(THRUSTER[3], LOW); // Turns thruster 2 off off - thrusters 
          rlyflag = false; 
        }
        
        digitalWrite(THRUSTER[2], HIGH); //Turns the backward thruster on sich that the vehicle comes
        //to a a stop at the division
         timer1 = millis();
        rlyflag = true;
        if (rlyflag && (millis()-timer) > tydiv/2)
        {
          digitalWrite(THRUSTER[2], LOW); // Turns thruster 2 off off - thrusters 
          rlyflag = false; 
         
        }

      count++;
      default:
        break;
    }
    float error = (XF - count*Yarray[0]) - X;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ x-translation FUNCTION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void xtranslation(int SwitchTranlation, float Xarray[], int txdiv)
{

  //[]Determines what case is valid
  int tol = 1000;
  //[cm]Y tolorenece

   int timer;
   //[]Used to keep track of time the thrusters are going to be on for

   int count = 1;
   
  while ((X+tol >= XF) || (X-tol <= XF))
  {
 
    switch (SwitchTranslation)
    {
      case 1:

        digitalWrite(THRUSTER[4], HIGH); //Turns thruster 2 on such that the vehicle moves forward
        timer1 = millis();
        rlyflag = true;
        if (rlyflag && (millis()-timer) > txdiv/2)
        {
          digitalWrite(THRUSTER[4], LOW); // Turns thruster 2 off off - thrusters 
          rlyflag = false; 
        }
        
        digitalWrite(THRUSTER[5], HIGH); //Turns the backward thruster on sich that the vehicle comes
        //to a a stop at the division
         timer1 = millis();
        rlyflag = true;
        if (rlyflag && (millis()-timer) > txdiv/2)
        {
          digitalWrite(THRUSTER[5], LOW); // Turns thruster 2 off off - thrusters 
          rlyflag = false; 
          
        }
        count++;
        break;

      case 0:
        digitalWrite(THRUSTER[5], HIGH); //Turns thruster 3 on such that the vehicle moves forward
         timer1 = millis();
        rlyflag = true;
        if (rlyflag && (millis()-timer) > txdiv/2)
        {
          digitalWrite(THRUSTER[5], LOW); // Turns thruster 2 off off - thrusters 
          rlyflag = false; 
        }
        
        digitalWrite(THRUSTER[4], HIGH); //Turns the backward thruster on sich that the vehicle comes
        //to a a stop at the division
         timer1 = millis();
        rlyflag = true;
        if (rlyflag && (millis()-timer) > txdiv/2)
        {
          digitalWrite(THRUSTER[4], LOW); // Turns thruster 2 off off - thrusters 
          rlyflag = false; 
         
        }

      count++;
      default:
        break;
    }
    float error = (XF - count*Xarray[0]) - X;
}


