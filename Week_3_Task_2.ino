/*---Usable Code---*/


/*-----MPU 6050-----*/
#include <LiquidCrystal.h>//LCD
#include <Wire.h> //I2C communication
#include <MPU6050.h>//GYRO (DLPF)


//Motors -> Ultrasonic sensor referenced as head
#define IN1 13 //Right wheel
#define IN2 12 //Right Wheel
#define IN3 2 //Left wheel
#define IN4 1 //Left wheel
#define ENA 11 //PWM Port //Right wheels
#define ENB 3 //PWM port //Left wheels
// LCD
#define RS 8
#define EN 9
#define D4 4
#define D5 5
#define D6 6
#define D7 7

/*----------------------------------------------------------------------------------------------*/

//Motor speed
const int TARGET_RIGHT_SPEED = 150;
const int TARGET_LEFT_SPEED = 150;
//Declare MPU object
MPU6050 mpu;
// Madgwick filter object
Madgwick filter;
//Declare LCD object
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

/*----------------------------------------------------------------------------------------------*/
//Robot state
int robot_state = 0;
//Max ramp angle
float max_ramp_angle = 0;
//Start time
unsigned long lastTime = 0;
float dt;
//Angle
float yaw = 0; float pitch = 0; float roll = 0;
// Gyro offsets for calibration
float gyroXOffset = 0; float gyroYOffset = 0; float gyroZOffset = 0;
// Number of samples for calibration
const int CALIB_SAMPLES = 800;
/*----------------------------------------------------------------------------------------------*/


void setup() {
  // Start serial for debugging
  Wire.begin();
  // Initialize LCD
  lcd.begin(16, 2);
  // Initialize motors
  func_motorInit();
  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    //Lcd.print("MPU6050 connection failed!");
    Serial.print("MPU6050 connection failed!");
    while (1);
  }

  mpu.setDLPFMode(6); //Set DLPF mode
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250); // Set Gyro range
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // Set Accelerometer range
  //Calibrate Gyro
  func_calibrateGyro();
  lastTime = millis();
}

void loop() {
  func_process();
  func_print();

  //Case '0' (Before ramp)
  if ((robot_state == 0)){
    while(abs(pitch) < 3){
      func_print();

      func_process();
      func_moveForward(125,125);
    }

    robot_state = 1;
  }
  //Case '1' (Up the ramp)
  else if (robot_state == 1){
    func_moveForward(160, 160);

    //Exit to next minor case'
    if (abs(pitch) >= 27) {
      //Read Max ramp angle
      max_ramp_angle = abs(pitch);

      //Special while loop
      while (1) {
        //Update gyro values
        delay(5);
        func_process();
        //Read more accurately max ramp angle
        if (pitch > max_ramp_angle) {
          max_ramp_angle = -pitch;
        }
        //Ensure that it is properly on top of the ramp
        if (abs(pitch) <= 3.0) {
          delay(200);
          robot_state = 2;
          break;
        }
      }
    }
  }
  //Case '2' (On ramp)
  else if (robot_state == 2){
    //Stop for 4 seconds
    func_stop();
    delay(4000);

    //Determine initial yaw
    func_process();
    float initial_yaw = yaw;
    float delta_yaw = 0;
    //Spin 360 degrees (right)
    while(delta_yaw <= 330.0f){
      //Keep calculatin yaw
      delta_yaw = abs(initial_yaw - yaw);
      func_print();
      func_process();

      func_spin();
    }

    robot_state = 3;
  }
  //Case '3' (Going down ramp)
  else if((robot_state == 3)){
    func_moveForward(110,110);
    delay(1000);

    if (abs(pitch) <= 27) {
      //Special while loop
      while (1) {
        delay(5);
        func_process();
        if (abs(pitch) <= 3.0) {
          //Stop robot
          func_stop();
          //Print ramp angle
          lcd.clear();
          func_print_rampAngle();
          while(true){};
          break;
        }
      }
    }
  }
  //Escape case
  else{
    lcd.clear();
    lcd.print("Error!!!");
    func_stop();
    while(true){};
  }
}

void func_motorInit(){
  //Initialize motor pin
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
  //Initialize speed pin
  pinMode(ENA, OUTPUT); 
  pinMode(ENB, OUTPUT);
  analogWrite(ENA,0);
  analogWrite(ENB,0);
}

//Stop
void func_stop(){
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
  delayMicroseconds(10);
}

//Forward
void func_moveForward(int R, int L){
  //Set speed
  analogWrite(ENA,R);
  analogWrite(ENB,L);
  //Set direction
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  delayMicroseconds(10);
}

void func_spin(){
  //Set speed
  analogWrite(ENA,130);
  analogWrite(ENB,130);
  //Set direction
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  delayMicroseconds(10);
}

void func_process(){
  // Compute delta time
  unsigned long currentTime = millis();
  dt = ((currentTime - lastTime) / 1000.0);
  lastTime = currentTime;

  // Read raw sensor data
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  // Convert to proper units
  float gyroX = (gx / 131.0); // assuming ±250 deg/s
  float gyroY = (gy / 131.0);
  float gyroZ = (gz / 131.0);

  // Compute yaw angle (Degrees)
  if (abs(gyroZ) <= 0.7){ //Reduce noise
    gyroZ = 0;
  }
  else{
    yaw += ((gyroZ - gyroZOffset) * dt);
  }
  
  // Compute pitch angle (Degrees)
  if (abs(gyroY) <= 0.3){ //Reduce noise
    gyroY = 0;
  }
  else{
    pitch = ((gyroY - gyroYOffset) * dt);
  }
}

//Calibrate gyro
void func_calibrateGyro() {
  long sumX = 0, sumY = 0, sumZ = 0;
  int16_t gx, gy, gz;

  for (int i = 0; i < CALIB_SAMPLES; i++) {
    mpu.getRotation(&gx, &gy, &gz);
    sumX += gx; //Roll
    sumY += gy; //Pitch
    sumZ += gz; //Yaw
    delay(5);
  }

  gyroXOffset = sumX / (float)CALIB_SAMPLES / 131.0; //Roll
  gyroYOffset = sumY / (float)CALIB_SAMPLES / 131.0; //Pitch
  gyroZOffset = sumZ / (float)CALIB_SAMPLES / 131.0; //Yaw

}

void func_print(){
  // Print values
  lcd.print("Yaw: "); lcd.print(yaw);
  lcd.setCursor(0, 1);
  lcd.print("Pitch: "); lcd.print(pitch);
  lcd.setCursor(0, 0);
}

void func_print_rampAngle(){
  lcd.print("Ramp angle:");
  lcd.print(max_ramp_angle);
}
