#include <Wire.h>
#include <LiquidCrystal_I2C.h>        //Include for I2C LCD
#include <Adafruit_PWMServoDriver.h>  //Include for Servo Shield
#include <PS2X_lib.h>                 //PSX Controller Include
#include <DBH1.h>

#define PRESSURES false         //Define PSX Pressure/Rumble Function
#define RUMBLE false
#define LXMIN 0                 //Minimum X Value for Left Stick
#define LXMAX 255               //Maximum X Value for Left Stick
#define LYMIN 0                 //Minimum Y Value for Left Stick
#define LYMAX 255               //Maximum Y Value for Left Stick
#define RXMIN 0                 //Minimum X Value for Right Stick
#define RXMAX 255               //Maximum X Value for Right Stick
#define RYMIN 0                 //Minimum Y Value for Right Stick
#define RYMAX 255               //Maximum Y Value for Right Stick
//Define the servo minimum and maximum pulse lengths (out of 4096)
#define SERVO1MIN  175          //Servo 1 (Left Pan)
#define SERVO1MAX  530
#define SERVO2MIN  175          //Servo 2 (Left Tilt)
#define SERVO2MAX  530
#define SERVO3MIN  175          //Servo 3 (Right Pan)
#define SERVO3MAX  530
#define SERVO4MIN  175          //Servo 4 (Right Tilt)
#define SERVO4MAX  530
#define SERVO5MIN  175          //Servo 5 (Left Elbow)
#define SERVO5MAX  530
#define SERVO6MIN  175          //Servo 6 (Right Elbow)
#define SERVO6MAX  530
#define PS2_DAT  12           //Define PSX Controller Pins  
#define PS2_CMD  11
#define PS2_SEL  10
#define PS2_CLK  13
#define STICK_CENTER 128
#define DEADZONE_SPACING 3
#define DEBOUNCE 20
#define THROTTLEREDUX 0.90
#define SERVOITERATION 30

#define MINPWM 0
#define MAXPWM 250

DBH1 DBH1;
LiquidCrystal_I2C lcd(0x27, 16, 2);   //Set the LCD address 0x27 and to 16 char, 2 lines
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);  //Set Servo Shield Address
PS2X ps2x;                // create PS2 Controller Class

const int SERVO1 = 0;         //Map Servo to Adafruit Servo Shield Headers
const int SERVO2 = 1;
const int SERVO3 = 2;
const int SERVO4 = 3;
const int SERVO5 = 4;
const int SERVO6 = 5;
boolean STARTUP = true;
boolean BRAKE_TOGGLE = false;
boolean SERVOMODE = false;
long timer = 0;
int counter = 0;
int Elbow[] = {330, 330};             //Set default servo position for Left/Right Elbow
int L_DEADZONE[] = {0, 0, 0, 0};      //{X Min, X Max, Y Min, Y Max}
int R_DEADZONE[] = {0, 0, 0, 0};
int PanTilt[] = {0,0};
int LSPanTilt[] = {0,0};
int RSPanTilt[] = {0,0};
int error = 0;              //PSX Controller Error Checks
byte type = 0;
byte vibrate = 0;
int LSPTout[] = {330, 330};
int RSPTout[] = {330, 330};
boolean FORWARD = true;
int POWERLEVEL[] = {0, 0};

void ElbowMove() {                    //Movement control for elbows
  int i;
  if (ps2x.Button(PSB_L2) && ps2x.Button(PSB_TRIANGLE)) {
      Elbow[1] = Elbow[1] + SERVOITERATION;
  }
  if (ps2x.Button(PSB_L2) && ps2x.Button(PSB_CROSS)) {
      Elbow[1] = Elbow[1] - SERVOITERATION;
  }
  if (ps2x.Button(PSB_R2) && ps2x.Button(PSB_TRIANGLE)) {
      Elbow[2] = Elbow[2] + SERVOITERATION;
  }
  if (ps2x.Button(PSB_R2) && ps2x.Button(PSB_CROSS)) {
      Elbow[2] = Elbow[2] - SERVOITERATION;
  }
  Elbow[1] = constrain(Elbow[1], SERVO5MIN, SERVO5MAX);
  Elbow[2] = constrain(Elbow[2], SERVO6MIN, SERVO6MAX);
  pwm.setPWM(SERVO5, 0, Elbow[1]);
  pwm.setPWM(SERVO6, 0, Elbow[2]);    //Send Servo Position
}

void LShoulder() {          //Control for Left Shoulder
  PanTilt[1] = ps2x.Analog(PSS_RX);    //Read Right Stick X Axis into Pan
  PanTilt[2] = ps2x.Analog(PSS_RY);   //Read Right Stick Y Axis into Tilt
  if (SERVOMODE == true){
    if (ps2x.Button(PSB_L1)){
      LSPanTilt[1] = map(PanTilt[1], RXMIN, RXMAX, -SERVOITERATION, SERVOITERATION);
      LSPanTilt[2] = map(PanTilt[2], RYMIN, RYMAX, -SERVOITERATION, SERVOITERATION);
      LSPTout[1] = LSPTout[1] + LSPanTilt[1];
      LSPTout[2] = LSPTout[2] + LSPanTilt[2];
    }
  }
  else if (ps2x.Button(PSB_L1)) {
    LSPTout[1] = map(PanTilt[1], RXMIN, RXMAX, SERVO1MIN, SERVO1MAX);  //Map pan/tilt to Servo pulse lengths
    LSPTout[2] = map(PanTilt[2], RXMIN, RYMAX, SERVO2MIN, SERVO2MAX);
  }
    LSPTout[1] = constrain(LSPTout[1], SERVO1MIN, SERVO1MAX);
    LSPTout[2] = constrain(LSPTout[2], SERVO2MIN, SERVO2MAX);
    pwm.setPWM(SERVO1, 0, LSPTout[1]);    //Set Servo Positions
    pwm.setPWM(SERVO2, 0, LSPTout[2]);
}

void RShoulder() {          //Movement control for Right Shoulder
  PanTilt[1] = ps2x.Analog(PSS_RX);    //Read Right Stick X Axis into Pan
  PanTilt[2] = ps2x.Analog(PSS_RY);   //Read Right Stick Y Axis into Tilt
  if (SERVOMODE == true){
    if (ps2x.Button(PSB_R1)){
      RSPanTilt[1] = map(PanTilt[1], RXMIN, RXMAX, -SERVOITERATION, SERVOITERATION);
      RSPanTilt[2] = map(PanTilt[2], RYMIN, RYMAX, -SERVOITERATION, SERVOITERATION);
      RSPTout[1] = RSPTout[1] + RSPanTilt[1];
      RSPTout[2] = RSPTout[2] + RSPanTilt[2];
    }
  }
  else if (ps2x.Button(PSB_R1)) {   //Run Only if Right DPad Button is Pressed
    RSPTout[1] = map(PanTilt[1], RXMIN, RXMAX, SERVO3MIN, SERVO3MAX);  //Map pan/tilt to Servo pulse lengths
    RSPTout[2] = map(PanTilt[2], RXMIN, RYMAX, SERVO4MIN, SERVO4MAX);
  }
    RSPTout[1] = constrain(RSPTout[1], SERVO3MIN, SERVO3MAX);
    RSPTout[2] = constrain(RSPTout[2], SERVO4MIN, SERVO4MAX);
    pwm.setPWM(SERVO3, 0, RSPTout[1]);    //Set Servo Positions
    pwm.setPWM(SERVO4, 0, RSPTout[2]);
}

void JoystickDeadzone() {          //Sets stick deadzone
  L_DEADZONE[1] = STICK_CENTER - DEADZONE_SPACING;            
  L_DEADZONE[2] = STICK_CENTER + DEADZONE_SPACING;
  L_DEADZONE[3] = STICK_CENTER - DEADZONE_SPACING;
  L_DEADZONE[4] = STICK_CENTER + DEADZONE_SPACING;
  R_DEADZONE[1] = STICK_CENTER - DEADZONE_SPACING;
  R_DEADZONE[2] = STICK_CENTER + DEADZONE_SPACING;
  R_DEADZONE[3] = STICK_CENTER - DEADZONE_SPACING;
  R_DEADZONE[4] = STICK_CENTER + DEADZONE_SPACING;
}

void Forward(int lpwm, int rpwm) {    //Forward movement function
  lpwm = PWMScale(lpwm, MAXPWM);
  rpwm = PWMScale(rpwm, MAXPWM);
  POWERLEVEL[1] = abs(lpwm);         //Translate value to non-negative
  POWERLEVEL[2] = abs(rpwm);
  FORWARD = true;
  DBH1.Forward(lpwm, rpwm);
}

void Reverse(int lpwm, int rpwm) {    //Reverse movement function
  lpwm = PWMScale(lpwm, MAXPWM);
  rpwm = PWMScale(rpwm, MAXPWM);
  POWERLEVEL[1] = abs(lpwm);         //Translate value to non-negative
  POWERLEVEL[2] = abs(rpwm);
  FORWARD = false;
  DBH1.Reverse(lpwm, rpwm);
}

void Coasting() {                     //Function defines motor control when no throttle applied
  POWERLEVEL[1] = 0;
  POWERLEVEL[2] = 0;
  FORWARD = true;
  DBH1.Coasting();
}

void Braking() {                      //Brake function activated when brake button pressed
  if (millis() != timer){
    if (ps2x.Button(PSB_START)) {
      if (counter >= DEBOUNCE){
        BRAKE_TOGGLE = !BRAKE_TOGGLE;
        counter = 0;
      }  
    timer = millis();
    }
  counter++;
  timer = millis();
  }
  
  if (BRAKE_TOGGLE == true){
    POWERLEVEL[1] = 0;
    POWERLEVEL[2] = 0;
    DBH1.Braking();
    Serial.println("Brake Enabled");
    if (STARTUP = true) {
      return;
    }
  }
  else {
    Serial.println("Brake Disabled");
}
}

int PWMScale(int pwm, int scale) {        //Scale PWM to user defined scale
  pwm = abs(pwm);                           //Get absolute value of PWM
  pwm = constrain(pwm, 0, scale);           //Set bounderies for PWM (ex. 0-100)
  pwm = map(pwm, MINPWM, MAXPWM, 0, scale);   //Map pwm to scale
  return pwm;                               //Pass PWM value back
}

void Movement(int Dir, int Vec) {
  int LS_POS[] = {0, 0};
  LS_POS[1] = ps2x.Analog(PSS_LX); //Read joystick X/Y position
  LS_POS[2] = ps2x.Analog(PSS_LY);
  int L_Scale[] = {0, 0};
  int Level[] = {0, 0};
  L_Scale[1] = map(LS_POS[1], LXMIN, LXMAX, -100, 100); //Map X and Y from 0 to 255 to -100 to 100
  L_Scale[2] = map(LS_POS[2], LYMIN, LYMAX, -100, 100); //For ease of PWM control
  Braking();
  if (BRAKE_TOGGLE == false) {
    if (Dir == 2 && Vec == 1) {                   //If no input on Y axis, set motor to coast mode
      Coasting();
    }
    if (Vec == 1 && Dir == 1) {       //If X Axis Centered, scale motor PWM based off Y Axis
      Forward(L_Scale[2], L_Scale[2]);    //for forward movement
    }
    if (Vec == 1 && Dir == 0) {         //If X Axis Centered, scale motor PWM based off Y Axis
      Reverse(L_Scale[2], L_Scale[2]);    //for reverse movement
    }
    if (Vec == 0 && Dir == 1) {         //If X Axis moved left of center, adjust PWM for left motor
      Level[1] = ThrottleMath(L_Scale[1]); //by 90% of throttle and turn on right motor for throttle value
      Level[2] = L_Scale[2];               //to make turn left
      Forward(Level[1], Level[2]);
    }
    if (Vec == 2 && Dir == 1) {         //If X Axis moved right of center, adjust PWM for right motor
      Level[1] = ThrottleMath(L_Scale[1]); //by 90% of throttle and turn on right motor for throttle value
      Level[2] = L_Scale[2];               //to make turn left
      Forward(Level[1], Level[2]);
    }
    if (Vec == 2 && Dir == 0) {         //Same as above, except for reversing
      Level[1] = ThrottleMath(L_Scale[1]);
      Level[2]= L_Scale[2];
      Reverse(Level[1], Level[2]);
    }
    if (Vec == 0 && Dir == 0) {         //Same as above, except for reversing
      Level[1] = ThrottleMath(L_Scale[1]);
      Level[2] = L_Scale[2];
      Reverse(Level[1], Level[2]);
    }
    if (Vec == 2 && Dir == 2) {
      Level[1] = L_Scale[1];
      Level[2] = 0;
      Forward(Level[1], Level[2]);
    }
    if (Vec == 0 && Dir == 2) {
      Level[1] = 0;
      Level[2] = L_Scale[1];
      Forward(Level[1], Level[2]);
    }
  }
}

int GetDIR(int ypos) {                //Gets the direction from the joystick Y Axis (Forward/Reverse)
  //Directional vectors defined:
  //Direction = 0 - Reverse
  //Direction = 1 - Forward (Default)
  //Direction = 2 - Centered
  int Direction;
  if (ypos > (L_DEADZONE[3]) && ypos < (L_DEADZONE[4])) { //Check to see if centered
    Direction = 2;
    return Direction;                   //Pass direction back
  }
  if (ypos > (L_DEADZONE[4] + 1)) {                //Check if Y axis is being pushed downward
    Direction = 0;
    return Direction;                   //Pass direction back
  }
  Direction = 1;                        //Set direction to forward
  return Direction;                     //Pass direction back

}

int GetVector(int xpos) {               //Gets the direction from the joystick X Axis (Left/Right)
  //Directional vectors defined:
  //Vector = 0 - Left
  //Vector = 1 - Centered
  //Vector = 2 - Right
  int Vector;
  if (xpos > L_DEADZONE[1] && xpos < L_DEADZONE[2]) { //Check if stick centered
    Vector = 1;
    return Vector;
  }
  if (xpos < L_DEADZONE[1]) {                //Check to see if stick pushed left
    Vector = 0;
    return Vector;
  }
  else {                                //Check to see if stick is pushed right
    Vector = 2;
    return Vector;
  }
}

int ThrottleMath(int xval) {      //Math to Calculate Throttle for Turning
  int math = 0;
  int yval = ps2x.Analog(PSS_LY);
  math = abs(xval);
  math = math * THROTTLEREDUX;
  if (yval > 138 || yval < 118){
    math = 100 - math;
  }
  else {
    math = 90 - math;
  }
  return math;
}

void LCDDisplay(){
  unsigned long previousMillis = 0;        // will store last time LCD was updated
  const long interval = 1000;           // interval at which to update LCD (milliseconds)
  unsigned long currentMillis = millis();
  lcd.setCursor(0,0);
  lcd.print("Dir:");
  lcd.setCursor(0,1);
  lcd.print("L:   ");
  lcd.setCursor(6,1);
  lcd.print("R:   ");
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if (SERVOMODE == true){
      lcd.setCursor(15,0);
      lcd.print("H");
    }
    if (SERVOMODE == false){
      lcd.setCursor(14,0);
      lcd.print("  ");
    }
    if (BRAKE_TOGGLE == true){
      lcd.setCursor(4,0);
      lcd.print("Brake    ");
    }
    else if (FORWARD == true){
      lcd.setCursor(4,0);
      lcd.print("Forward  ");
    }
    else if (FORWARD == false){
      lcd.setCursor(4,0);
      lcd.print("Reverse  ");
    }
    lcd.setCursor(2, 1);
    lcd.print(POWERLEVEL[1]);
    lcd.setCursor(8, 1);
    lcd.print(POWERLEVEL[2]);
  }
}

void setup() {                          //Setup - Only runs when micro is powered up
  pwm.begin();
  pwm.setPWMFreq(50);                   // Analog servos run at ~50 Hz updates
  lcd.init();                           //Setup LCD
  lcd.backlight();
  DBH1.init(3,5,6,9,2,4,0,1);           //Define pins for motor controller
  Serial.begin(9600);                   //Enable Serial Port
  delay(5000);                          //Delay so PSX Controller can be detected
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, PRESSURES, RUMBLE); //Check to see if PSX Controller connected
  if (error == 0) {
    Serial.print("Found Controller, configured successful.");
  }
  else if (error == 1)
    Serial.println("No controller found, check wiring.");
  ps2x.read_gamepad();
  STARTUP = true;
  BRAKE_TOGGLE = true;
  Braking();              //Enable Brake during Calibration
  STARTUP = false;
  Serial.println("Calibrating Joystick...");
  Serial.println("Brake Enabled!");
  JoystickDeadzone();
  delay(2000);
  Serial.println("Press START to disable brake");
  Serial.println("Calibration Completed!");
  lcd.clear();                          //Clear the LCD to start
  LShoulder();
  RShoulder();
  DBH1.EnableBoth();
}

void loop() {
  ps2x.read_gamepad();
  if (ps2x.Button(PSB_R3)){
    SERVOMODE = true;
  }
  if (ps2x.Button(PSB_SQUARE)){
    SERVOMODE = false;
  }
  int joyxpos = ps2x.Analog(PSS_LX);
  int joyypos = ps2x.Analog(PSS_LY);
  LCDDisplay();
  Movement(GetDIR(joyypos), GetVector(joyxpos));
  LShoulder();
  RShoulder();
  ElbowMove();
}
