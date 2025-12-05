#include <LiquidCrystal.h>

// RW grounded -> Only can write
#define D4 4
#define D5 5
#define D6 6
#define D7 7
#define RS 8 //Command or data mode
#define EN 9 //Signal LCD to read
// Ultrasonic sensor referenced as head
#define IN1 13 //Right wheel
#define IN2 12 //Right Wheel
#define IN3 2 //Left wheel
#define IN4 1 //Left wheel
#define ENA 11 //PWM Port //Right wheels
#define ENB 3 //PWM port //Left wheels
//Wheels speed
const int SPEED_RIGHT = 150;    //Range: 0-255 (PWM)
const int SPEED_LEFT = 150;    //Range: 0-255 (PWM)

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
long int start_time;

void setup() {
  // put your setup code here, to run once:
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); 
  pinMode(ENB, OUTPUT);
  func_motorInit();

  lcd.begin(16, 2); //All lcd pins auto configure as output
  lcd.print("Week 1: Move");
  lcd.setCursor(0, 1);
  lcd.print("Time: ");

  start_time = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  long int current_time = millis();
  int elapsed_time = (current_time - start_time);

  if (elapsed_time >= 10000)
  {
    lcd.clear();
    lcd.print("Movement Done!");
    func_stop();
    while (true){};
  }
  else
  {
    func_moveForward(SPEED_LEFT,SPEED_RIGHT);
    func_print(elapsed_time);
  }

}


//Reset initial state
void func_motorInit()
{
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
  analogWrite(ENA,0);
  analogWrite(ENB,0);
}
//Stop robo car
void func_stop()
{
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
  delay(10);
}
//Move robo car
void func_moveForward(const int SPEED_LEFT,const int SPEED_RIGHT)
{
  analogWrite(ENA,SPEED_RIGHT);
  analogWrite(ENB,SPEED_LEFT);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
}

void func_print(int elapsed_time)
{
    int seconds = elapsed_time / 1000;
    int milliseconds = elapsed_time % 1000;

    lcd.setCursor(6, 1);
    // Format: SS:MMM
    if (seconds < 10) 
      lcd.print("0");
    lcd.print(seconds);
    lcd.print(":");
    if (milliseconds < 100) 
      lcd.print("0");
    if (milliseconds < 10) 
      lcd.print("0");
    lcd.print(milliseconds);
}
