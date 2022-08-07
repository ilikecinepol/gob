#include "MPU6050.h"                 // Подключение библиотеки MPU6050 
#include "GyverFilters.h"
#include <AutoPID.h>
 
MPU6050 GY521;                       // Создаем объект, символизирующий модуль датчика
int16_t ax, ay, az;                  // Переменные для хранения значений акселерометра
int16_t gx, gy, gz;                  // Переменные для хранения значений гироскоп

int enb = 9;                        // Объявление переменных драйвера двигателя
int ena = 10;
int in1 = 11;
int in2 = 12;
int in3 = 7;
int in4 = 8;
int alphaX0 = 0;
int alphaY0 = 0;
//pid settings and gains
int OUTPUT_MIN= 0;
int OUTPUT_MAX = 255;
float KP = 0.1;
float KI = 0.0;
int KD = 0;


int x_vel = 0;
int y_vel = 0;
int max_vel = 100;
int min_vel = 50;


// Рабочие переменные
int16_t f_ax;
int16_t f_ay;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);                  // Инициализация последовательного порта
  Serial.println("Initializing I2C devices..."); // Печать текста
  GY521.initialize();                   // Инициализация MPU
  delay(100);                           // Пауза

  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  delay(2000);
  GY521.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Чтение значений гироскопа и акселерометра
  alphaX0 = ax;
  alphaY0 = ay;
}


GKalman xFilter(400, 40, 0.5);
GKalman yFilter(1000, 100, 0.5);
void forward(int vel){
  analogWrite(enb, vel);    // y

digitalWrite(in3, 0);
digitalWrite(in4, 1);
}

void moving(int xvel, int yvel){
  if (xvel >= 0){
    xvel = map(xvel, 0, 255, min_vel, max_vel);
    digitalWrite(in1, 1);
    digitalWrite(in2, 0);
  }
  else{
    xvel = map(xvel, -1, -255, min_vel, max_vel);
    digitalWrite(in1, 0);
    digitalWrite(in2, 1);
    
  }
  if (yvel >= 0){
    yvel = map(yvel, 0, 255, min_vel, max_vel);
    digitalWrite(in3, 1);
    digitalWrite(in4, 0);
  }
  else{
    yvel = map(yvel, -1, -255, min_vel, max_vel);
    digitalWrite(in3, 0);
    digitalWrite(in4, 1);
    
  }
  
  analogWrite(ena, xvel);
  analogWrite(enb, yvel);
}


void loop() {
 GY521.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Чтение значений гироскопа и акселерометра
 
  // вывод значений в монитор
  Serial.print("a/g:\t");
  
  //Serial.print(ax); Serial.print("\t");
  //Serial.print(az); Serial.print("\t");
  //Serial.print(gx); Serial.println("\t");
  //Serial.print(gy); Serial.print("\t");
  //Serial.println(gz);
  
  f_ax = xFilter.filtered((int)ax);
  // fin_ax = map(f_ax, -15000, 15000, -90, 90);
  f_ay = yFilter.filtered((int)ay);
  //Serial.print(ax); Serial.println("\t");
  //Serial.print(f_ax); Serial.println("\t");
  delay(50); 
  
  // AutoPID myPID(f_ax, alphaX0, velocity, 0, 250, KP, KI, KD);
  x_vel = map((f_ax - alphaX0), -10000, 10000, -250, 250);
  y_vel = map((f_ay - alphaY0), -10000, 10000, -250, 250);
  moving(x_vel, y_vel);

  
  
  Serial.print(x_vel); Serial.println("\t");
  Serial.print(y_vel); Serial.println("\t");
}
