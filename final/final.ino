#define Pin_Encoder_Right_A 2          //     E2A----------------2
#define Pin_Encoder_Right_B 3          //     E2B----------------3
#define Pin_Encoder_Left_A 20          //     E1A----------------20
#define Pin_Encoder_Left_B 21          //     E1B----------------21
//M1左輪   M2右輪

long theta_Right = 0, theta_Left = 0;
unsigned long currentMillis;
long previousMillis = 0;    // set up timers
float interval = 100;
int time = 0;

//直流馬達----------TB6612腳位----------ArduinoUNO腳位
//                             PWMA-----------------4
//                             AIN1--------------------6
//                             AIN2--------------------5
//                             STBY-------------------7
//                             PWMB-----------------8
//                             BIN1-------------------10
//                             BIN2-------------------9
//                     
//                             GND-------------------GND
//                             VM--------------------12V
//                             VCC-------------------5V
//                             GND------------------GND
# define PWMA 4
# define AIN1 6
# define AIN2 5
# define STBY 7
# define PWMB 8
# define BIN1 10
# define BIN2 9

//超音波感測器註解
# define TRIG 13
# define ECHO 12

long duration, cm, pre_cm;

//直流馬達----------TB6612腳位----------ArduinoUNO腳位
int PwmA, PwmB;

void initMotor(){
  //控制訊號初始化
  pinMode(AIN1, OUTPUT);//控制馬達A的方向，(AIN1, AIN2)=(1, 0)為正轉，(AIN1, AIN2)=(0, 1)為反轉
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);//控制馬達B的方向，(BIN1, BIN2)=(0, 1)為正轉，(BIN1, BIN2)=(1, 0)為反轉
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);//A馬達PWM
  pinMode(PWMB, OUTPUT);//B馬達PWM
  pinMode(STBY, OUTPUT);//TB6612致能,設置0則所有馬達停止,設置1才允許控制馬達

  //初始化TB6612馬達驅動模組
  digitalWrite(AIN1, 1);
  digitalWrite(AIN2, 0);
  digitalWrite(BIN1, 1);
  digitalWrite(BIN2, 0);
  digitalWrite(STBY, 1);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

void SetPWM(int motor, int pwm)
{
  //motor=1代表控制馬達A，pwm>=0則(AIN1, AIN2)=(1, 0)為正轉
  if(motor == 1 && pwm >= 0){
    analogWrite(PWMA, pwm);
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 0);
  }
  //motor=1代表控制馬達A，pwm<0則(AIN1, AIN2)=(0, 1)為反轉
  else if(motor == 1 && pwm < 0){
    analogWrite(PWMA, -pwm);
    digitalWrite(AIN1, 0);
    digitalWrite(AIN2, 1);
  }
  //motor=2代表控制馬達B，pwm>=0則(BIN1, BIN2)=(0, 1)為正轉
  else if(motor == 2 && pwm >=0){
    analogWrite(PWMB, pwm);
    digitalWrite(BIN1, 0);
    digitalWrite(BIN2, 1);
  }
  //motor=2代表控制馬達B，pwm<0則(BIN1, BIN2)=(1, 0)為反轉
  else if(motor ==2 && pwm <0){
    analogWrite(PWMB, -pwm);
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 0);
  }
}

//定義前進、後退、左轉、右轉、停止
//前進
void forward(int s1,int s2){
  SetPWM(1, s1);
  SetPWM(2, -s2);
}
//右轉
void right(int s1,int s2){
  SetPWM(1, s1);
  SetPWM(2, -s2);
}
//左轉
void left(int s1,int s2){
  SetPWM(1, s1);
  SetPWM(2, -s2);
}
//後退
void back(int s1,int s2){
  SetPWM(1, -s1);
  SetPWM(2, s2);
}
//停止
void stopp(){
  SetPWM(1, 0);
  SetPWM(2, 0);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, OUTPUT);
  initMotor();
  previousMillis = millis();
}

void loop() {
  if(Serial.available() > 0){
    String command = Serial.readStringUntil('\n'); //接收python傳來的訊息

    //接收到"n"代表新的一個迴圈開始，使用超音波感測器測量距離
    if(command == "n"){
      //measure distance
      digitalWrite(TRIG, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG, LOW);
      pinMode(ECHO, INPUT);
      duration = pulseIn(ECHO, HIGH);
      cm = duration*0.017;

      //若測出來距離小於55cm，則回傳"y"到python，表示55cm內有障礙物
      if(cm < 55){
        Serial.println("y");
      }
      //若沒有障礙物則回傳"n"
      else{
        Serial.println("n");
      }
    }

    //直走
    else if(command == "s"){
      forward(35, 35); //以左輪35右輪35的轉速向前走
      delay(300); //持續0.3秒
      Serial.println("forward"); //回傳"forward"自串到python
    }
    
    //左轉(數字越大轉幅越大)
    else if(command == "-1"){
      delay(30);
      forward(20, 30);
      delay(150);
      Serial.println("lean to left 1");
    }
    else if(command == "-2"){
      delay(30);
      forward(20, 35); 
      delay(100);
      Serial.println("lean to left 2");
    }
    else if(command == "-3"){
      delay(30);
      forward(20, 38); 
      delay(100);
      Serial.println("lean to left 3");
    }
    else if(command == "-4"){
      delay(70);
      forward(14, 35);
      delay(100);
      Serial.println("lean to left 4");
    }
    else if(command == "-5"){
      delay(70);
      forward(16, 43);
      delay(50);
      Serial.println("lean to left 5");
    }

    //右轉(數字越大轉幅越大)
    else if(command == "1"){
      delay(30);
      forward(30, 20);
      delay(150);
      Serial.println("lean to right 1");
    }
    else if(command == "2"){
      delay(30);
      forward(35, 20);
      delay(100);
      Serial.println("lean to right 2");
    }
    else if(command == "3"){
      delay(30);
      forward(38, 20);
      delay(100);
      Serial.println("lean to right 3");
    }
    else if(command == "4"){
      delay(73);
      forward(48, 20); 
      delay(100);
      Serial.println("lean to right 4");
    }
    else if(command == "5"){
      delay(80);
      forward(44, 17);
      delay(50);
      Serial.println("lean to right 5");
    }

    //偵測到左轉標示
    else if(command == "L"){
        //先向前走一段路後再左轉
        forward(25,25);
        delay(1500);
        forward(20, 63); 
        delay(2100);
        Serial.println("turn left");
        
        //進到最後一個部分
        while(1){
          //measure distance
          digitalWrite(TRIG, HIGH);
          delayMicroseconds(10);
          digitalWrite(TRIG, LOW);
          pinMode(ECHO, INPUT);
          duration = pulseIn(ECHO, HIGH);
          cm = duration*0.017;
          Serial.println(cm);
          //50cm之內有障礙物則轉彎避障
          if(cm < 50){
            forward(20, 40);
            delay(2000);
            forward(45, 40);
            delay(1000);
            break;
          }
          //若無則直走
          else{
            forward(40, 40);
            delay(500);
          }
        }
      }
    }

    //偵測到右轉標誌
    else if(command == "R"){
      //第一個右轉標誌的轉彎
      if(time == 0){
        forward(25,25); 
        delay(1300); 
        forward(57, 20); 
        delay(1500);
      }
      //第二、三個右轉標誌的轉彎
      else{
        forward(25,25); 
        delay(2000); 
        forward(65, 20); 
        delay(1800); 
      }
      Serial.println("turn right");
    }

    //停止
    else if(command == "p"){
      forward(35, 20);
      delay(2000);
      stopp();
      Serial.println("pause");
      delay(1500);
    }

    //結束程式
    else if(command == "q"){
      stopp();
      exit(0);
    }
  }
}
