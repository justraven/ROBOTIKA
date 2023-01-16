#include <Arduino.h>
#include <Servo.h>
#include <LiquidCrystal.h>
#include <parsing.h>

#define N 500
#define period timesampling

//----- LCD PINs -----//
typedef struct{

  const uint8_t RS = 3;
  const uint8_t EN = 2;
  const uint8_t D4 = 4;
  const uint8_t D5 = 5;
  const uint8_t D6 = 6;
  const uint8_t D7 = 7;

}LCD_t;

//----- feedback -----//
typedef struct{

  float x = 0;
  float y = 0;

  float sum_x = 0;
  float sum_y = 0;

  float average_x = 0;
  float average_y = 0;

}feedback_t;

//----- PID Parameter -----//
typedef struct{

  float feedback = 0;
  float setPoint = 0;

  float KP = 0;
  float KI = 0;
  float KD = 0;

  float KPValue = 0;
  float KIValue = 0;
  float KDValue = 0;

  float error = 0;
  float lastError = 0;

  float totalError = 0;
  float deltaError = 0;

  int maxControl = 100;
  int minControl = -100;

  int controlSignal =0;

}PIDParameter_t;

//----- call function -----//
feedback_t feedback;
LCD_t lcdPin;
Servo servo[2];
LiquidCrystal lcd(lcdPin.RS, lcdPin.EN, lcdPin.D4, lcdPin.D5, lcdPin.D6, lcdPin.D7);
PIDParameter_t pid[2];

//----- map float -----//
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

unsigned long t = 0;
int period = 50;

int control_x = 1477;  // 86 degree ==> 1477 pulse width
int control_y = 1555;  // 100 degree ==> 1555 pulse width

void setup(){

  Serial.begin(9600);

  lcd.begin(16,2);

  servo[0].attach(10); // servo sumbu x
  servo[1].attach(9);  // servo sumbu y

  servo[0].writeMicroseconds(control_x); // x
  servo[1].writeMicroseconds(control_y); // y

  pid[0].KP = 3.0;
  pid[0].KI = 0.0001;
  pid[0].KD = 1.0;

  pid[1].KP = 3.0;
  pid[1].KI = 0.0001;
  pid[1].KD = 1.0;

  pid[0].setPoint = 1477;
  pid[1].setPoint = 1555;

  lcd.setCursor(4,0);
  lcd.print("KELOMPOK");
  lcd.setCursor(3,1);
  lcd.print("INDEPENDEN");

}

void loop(){

  if(millis() - t >= period){

    t += period;

    if(Serial.available() > 0){

      READ_DATA_UNTIL('\n');
      data.replace(',','.');
      parseString();

      feedback.x = DATA_STR(0).toFloat();
      feedback.y = DATA_STR(1).toFloat();

      feedback.sum_x = 0;
      feedback.sum_y = 0;

      for(int i = 0; i < N; i++){
        feedback.sum_x += feedback.x;
        feedback.sum_y += feedback.y;
      }

      feedback.average_x = feedback.sum_x / N;
      feedback.average_y = feedback.sum_y / N;

    }

    if(feedback.average_x !=0 || feedback.average_y !=0){

      pid[0].feedback = mapFloat(feedback.average_x,0,640,1000,2000);
      pid[1].feedback = mapFloat(feedback.average_y,0,480,2000,1000);

      pid[0].error = pid[0].setPoint - pid[0].feedback;
      pid[1].error = pid[1].setPoint - pid[1].feedback;

      pid[0].totalError += pid[0].error; //integral pid servo 1
      pid[1].totalError += pid[1].error; //integral pid servo 2
    
      if(pid[0].totalError >= pid[0].maxControl) //anti-wind-up servo 1
        pid[0].totalError = pid[0].maxControl;
      else if(pid[0].totalError <= pid[0].minControl)
        pid[0].totalError = pid[0].minControl;
      
      if(pid[1].totalError >= pid[1].maxControl) //anti-wind-up servo 2
        pid[1].totalError = pid[1].maxControl;
      else if(pid[1].totalError <= pid[1].minControl)
        pid[1].totalError = pid[1].minControl;
      
      pid[0].deltaError = pid[0].error - pid[0].lastError; //differential servo 1
      pid[1].deltaError = pid[1].error - pid[1].lastError; //differential servo 2

      pid[0].KPValue = (pid[0].KP / 100) * pid[0].error;
      pid[0].KIValue = (pid[0].KI / 100) * pid[0].totalError * timesampling;
      pid[0].KDValue = (pid[0].KD / timesampling) * pid[0].deltaError;

      pid[1].KPValue = (pid[1].KP / 100) * pid[1].error;
      pid[1].KIValue = (pid[1].KI / 100) * pid[1].totalError * timesampling;
      pid[1].KDValue = (pid[1].KD / timesampling) * pid[1].deltaError;

      pid[0].controlSignal = pid[0].KPValue + pid[0].KIValue + pid[0].KDValue;
      pid[1].controlSignal = pid[1].KPValue + pid[1].KIValue + pid[1].KDValue;

      if(pid[0].controlSignal >= pid[0].maxControl) //limiter pid servo 1
        pid[0].controlSignal = pid[0].maxControl;
      else if(pid[0].controlSignal <= pid[0].minControl)
        pid[0].controlSignal = pid[0].minControl;
      
      if(pid[1].controlSignal >= pid[1].maxControl) //limiter pid servo 1
        pid[1].controlSignal = pid[1].maxControl;
      else if(pid[1].controlSignal <= pid[1].minControl)
        pid[1].controlSignal = pid[1].minControl;
      
      control_x = control_x + pid[0].controlSignal;
      control_y = control_y + pid[1].controlSignal;

      if(control_x >= 2000)
        control_x = 2000;
      else if (control_x <= 1000)
        control_x = 1000;

      if(control_y >= 2000)
        control_y = 2000;
      else if (control_y <= 1000)
        control_y = 1000;
      

      servo[0].writeMicroseconds(mapFloat(control_x,1000,2000,2000,1000));
      servo[1].writeMicroseconds(control_y);

      pid[0].lastError = pid[0].error;
      pid[1].lastError = pid[1].error;

      lcd.setCursor(2,0);
      lcd.print("PV_X:");
      lcd.print(pid[0].feedback);
      lcd.setCursor(2,1);
      lcd.print("PV_Y:");
      lcd.print(pid[1].feedback);

    }
  }
}