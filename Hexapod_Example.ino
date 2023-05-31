#include <ESP32Servo.h>
#include <Math.h>
#include <Ramp.h>

// Constants
const double J2L = 57.0; // Length of J2 (57 - 2.35)mm
const double J3L = 110.0; // Length of J3 110mm

const double Y_Rest =  80.0;
const double Z_Rest = -90.0;

const double J3_LegAngle = 15.4;

const double walk[13][3] = {{0.0, 0.0, 00.0},
                            {-0.2, 0.8, 00.0},
                            {-0.5, 0.5, 00.0},
                            {-1.0, 1.0, 00.0},
                            {-0.8, 0.2, 15.0},
                            {-0.5, 0.5, 25.0},
                            {0.0, 1.0, 30.0},
                            {0.5, 1.5, 25.0},
                            {0.8, 1.8, 15.0},
                            {1.0, 2.0, 00.0},
                            {0.5, 1.5, 00.0},
                            {0.2, 1.2, 00.0},
                            {0.0, 1.0, 00.0}};
// Debug
//#define DEBUG
#ifdef DEBUG
  #define DEBUG_PRINTL(x)  Serial.println(x)
  #define DEBUG_PRINT(x)  Serial.print(x)
#else
  #define DEBUG_PRINTL(x)
  #define DEBUG_PRINT(x)
#endif

// CO-ORDINATE SYSTEM
//          x
//          ^
//          |
//          |
// y <------o
// +ve z = OUT OF SCREEN

// LEG CONFIGURATION
//
//        FRONT
// L3-------|-------L4
// L2-------|-------L5
// L1-------|-------L6

// ##### ##### ##### ##### ##### ##### ##### ##### ##### JOINT CLASS
class Joint {
  public:
  bool Inverted;
  int8_t AngleOffset;

  // CTOR
  Joint(){
  }

  // Methods
  void Setup(uint8_t Pin, bool I, int8_t Ao){        
    _Pin = Pin;
    Inverted = I;
    AngleOffset = Ao;
    _Servo.attach(_Pin, 500, 2500);
  }

  void Update(double A){
    if (Inverted){
      _Servo.write(180 - A + AngleOffset);
    }
    else{
      _Servo.write(A + AngleOffset);
    }
  }

  private:
  // Variables
  Servo _Servo;
  uint8_t _Pin;  

};

// ##### ##### ##### ##### ##### ##### ##### ##### ##### LEG CLASS
class Leg {
  public:  

  // CTOR
  Leg(){
  }

  // Methods
  void Setup(double Angle){
    _LegAngle = Angle;
  }
  
  void CartesianMove(double X, double Y, double Z, Joint *Joint1, Joint *Joint2, Joint *Joint3){
    // Apply XYZ Offsets
    Y += Y_Rest;
    Z += Z_Rest;

    // Pivot by angle
    if (_LegAngle != 0){
      double aR = _LegAngle / (180 / PI);
      double xn = ((X * cos(aR)) - (Y * sin(aR)));
      double yn = ((Y * cos(aR)) + (X * sin(aR)));

      X = xn;
      Y = yn;
    }

  // CALCULATE INVERSE KINEMATIC SOLUTION
  double J1 = atan(X / Y) * (180 / PI);
  double H = sqrt((Y * Y) + (X * X));
  double L = sqrt((H * H) + (Z * Z));
  double J3 = acos(   ((J2L * J2L) + (J3L * J3L) - (L * L))   /   (2 * J2L * J3L)   ) * (180 / PI);
  double B = acos(   ((L * L) + (J2L * J2L) - (J3L * J3L))   /   (2 * L * J2L)   ) * (180 / PI);
  double A = atan(Z / H) * (180 / PI);  // BECAUSE Z REST IS NEGATIVE, THIS RETURNS A NEGATIVE VALUE
  double J2 = (B + A);  // BECAUSE 'A' IS NEGATIVE AT REST WE NEED TO INVERT '-' TO '+'

  Joint1->Update(90 - J1);
  Joint2->Update(90 - J2);
  if (Joint3->Inverted){
    Joint3->Update(J3 - J3_LegAngle);
  }
  else{
    Joint3->Update(J3 + J3_LegAngle);
  }

  }

  private:
  // Variables
  double _LegAngle;
};

// Servo pins
#define L1J1Pin 33
#define L1J2Pin 32
#define L1J3Pin 14

#define L2J1Pin 27
#define L2J2Pin 26
#define L2J3Pin 25

#define L3J1Pin 13
#define L3J2Pin 12


#define L4J1Pin 21
#define L4J2Pin 23
#define L4J3Pin 16

#define L5J1Pin 17
#define L5J2Pin 18
#define L5J3Pin 19

#define L6J1Pin 15
#define L6J2Pin 4

// Legs
Leg L1;
Leg L2;
Leg L3;
Leg L4;
Leg L5;
Leg L6;

// Joints
Joint L1J1;
Joint L1J2;
Joint L1J3;

Joint L2J1;
Joint L2J2;
Joint L2J3;

Joint L3J1;
Joint L3J2;


Joint L4J1;
Joint L4J2;
Joint L4J3;

Joint L5J1;
Joint L5J2;
Joint L5J3;

Joint L6J1;
Joint L6J2;


// User Input
int8_t X_Speed = 40;  // DISTANCE PER HALF STEP, SO 80mm STEP LENGTH
int8_t Y_Speed = 0;

// Motion Control
bool started = false;
uint8_t commandStepA = 12;
uint8_t commandStepB = 6;

// Joint Variables
double AXAct = 0.0;
double AYAct = 0.0;
double AZAct = 0.0;

rampDouble AXTar = 0.0;
rampDouble AYTar = 0.0;
rampDouble AZTar = 0.0;

double BXAct = 0.0;
double BYAct = 0.0;
double BZAct = 0.0;

rampDouble BXTar = 0.0;
rampDouble BYTar = 0.0;
rampDouble BZTar = 0.0;

uint8_t walkSpeed = 75; // SMALLER IS FASTER, TIME FOR EACH STEP, 75, 50, 30 & 20 SEEM GOOD

void setup() {
  // Debug
  #ifdef DEBUG
    Serial.begin(115200);
  #endif
  // THE FIRST VALUE IS THE PIN

  // THE SECOND VALUE IS TO INVERT THE JOINT

  // THE LAST VALUE IS AN OFFSET TO ACCOUNT FOR ASSEMBLY DIFFERENCES AND OFF CENTRE WEIGHT,
  // THIS SHOULDN'T BE NEEDED FOR STRONG SERVOS WITH ACCURATE CONSTRUCTION.
  L1J1.Setup(L1J1Pin, false, 0);
  L1J2.Setup(L1J2Pin, true,  -10);
  L1J3.Setup(L1J3Pin, false, 0);

  L2J1.Setup(L2J1Pin, false, 0);
  L2J2.Setup(L2J2Pin, true,  10);
  L2J3.Setup(L2J3Pin, false, -10);

  L3J1.Setup(L3J1Pin, false, 0);
  L3J2.Setup(L3J2Pin, true,  0);

  L4J1.Setup(L4J1Pin, true,  0);
  L4J2.Setup(L4J2Pin, false, 5);
  L4J3.Setup(L4J3Pin, true,  0);

  L5J1.Setup(L5J1Pin, true,  0);
  L5J2.Setup(L5J2Pin, false, -5);
  L5J3.Setup(L5J3Pin, false, 0);

  L6J1.Setup(L6J1Pin, true,  0);
  L6J2.Setup(L6J2Pin, false, 10);

  // THE OFFSETS ALLOW ALL THE LEGS TO 'REST'
  // AT THE SAME PLACE RELATIVE TO J1, THE ANGLE
  // ALLOWS THEM TO MOVE IN THE SAME XY PLANE

  double DegToRad = (180 / PI);
  L1.Setup(-60);
  L2.Setup(0);
  L3.Setup(60);
  L4.Setup(60);
  L5.Setup(0);
  L6.Setup(-60);
  
  // Stand Up
  L1.CartesianMove(0, 0, 0, &L1J1, &L1J2, &L1J3);
  L2.CartesianMove(0, 0, 0, &L2J1, &L2J2, &L2J3);
  L3.CartesianMove(0, 0, 0, &L3J1, &L3J2, &L1J3);
  L4.CartesianMove(0, 0, 0, &L4J1, &L4J2, &L4J3);
  L5.CartesianMove(0, 0, 0, &L5J1, &L5J2, &L5J3);
  L6.CartesianMove(0, 0, 0, &L6J1, &L6J2, &L4J3);

  delay(20000);
}

void loop() {

MotionControl();

}

void MotionControl(){

  // Update position
  AXAct = AXTar.update();
  AYAct = AYTar.update();
  AZAct = AZTar.update();

  BXAct = BXTar.update();
  BYAct = BYTar.update();
  BZAct = BZTar.update();

  L1.CartesianMove(AXAct, AYAct, AZAct, &L1J1, &L1J2, &L1J3);
  L2.CartesianMove(BXAct, BYAct, BZAct, &L2J1, &L2J2, &L2J3);
  L3.CartesianMove(AXAct, AYAct, AZAct, &L3J1, &L3J2, &L1J3);
  L4.CartesianMove(BXAct, BYAct, BZAct, &L4J1, &L4J2, &L4J3);
  L5.CartesianMove(AXAct, AYAct, AZAct, &L5J1, &L5J2, &L5J3);
  L6.CartesianMove(BXAct, BYAct, BZAct, &L6J1, &L6J2, &L4J3);

  // Check if command finished
  if (started == false | AXTar.isFinished() == true){
    commandStepA++;
    commandStepB++;
    if (commandStepA > 12){
      commandStepA = 1;
    }
    if (commandStepB > 12){
      commandStepB = 1;
    }

    double xMove = walk[commandStepA][0] * X_Speed;
    double yMove = walk[commandStepA][1] * Y_Speed;
    double zMove = walk[commandStepA][2];
    uint8_t duration = walkSpeed;

    AXTar.go(xMove, duration);
    AYTar.go(yMove, duration);
    AZTar.go(zMove, duration);

    xMove = walk[commandStepB][0] * X_Speed;
    yMove = walk[commandStepB][1] * Y_Speed;
    zMove = walk[commandStepB][2];
    duration = walkSpeed;

    BXTar.go(xMove, duration);
    BYTar.go(yMove, duration);
    BZTar.go(zMove, duration);

    started = true;
  }
}









