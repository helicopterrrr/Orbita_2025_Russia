#include <Servo.h>
#include <I2CDevice.h>
#include <LIS3MDL.h>
#include <LM75A.h>
#include <LSM6DS3.h>
#include <Logger.h>
#include <MS5611.h>
#include <Wire.h>
#include <SPI.h>
#include <STM32SD.h>

using namespace IntroStratLib;

#define SPI2_MISO PB14
#define SPI2_MOSI PB15
#define SPI2_SCK PD3
#define SPI2_SS PD4
#define I2C1_SCL PB6
#define I2C1_SDA PB7
#define I2C2_SCL PB6
#define I2C2_SDA PB7
#define USART_TX PA9
#define USART_RX PA10
#define SERVO_PIN PA4
#define START_BUTTON PB0
#define CONC_2 PB1
#define TEMPERATURE_1_TX PD8
#define TEMPERATURE_1_RX PD9
#define TEMPERATURE_2_TX PB13
#define TEMPERATURE_2_RX PB12
#define IVAN PB10
#define DELITEL PC3_C 
#define POT_SERVO PC0 

#ifndef SD_DETECT_PIN
#define SD_DETECT_PIN PB8
#endif

Servo servo;
HardwareSerial Serialx(USART_RX, USART_TX);
TwoWire Wirex(I2C1_SDA, I2C1_SCL);
TwoWire Wirex2(I2C2_SDA, I2C2_SCL);
SPIClass SPI_x(SPI2_MOSI, SPI2_MISO, SPI2_SCK);

MS5611 bar(Wirex, 0x77);
LSM6DS3 gacc(Wirex, 0x6A);
LIS3MDL mag(Wirex, 0x1C);
LM75A temp(Wirex, 0x4A);

File dataFile;

const int max_servo = 50; //сколько серве в одну сторону
struct Dater{
  unsigned long current_time;
  unsigned long time_last_experiment;
  float temperature[3];
  float pressure;
  float height;
  float height_last_experiment;
  float magn[3];
  float gyro[3];
  float accel[3];
  float servo;
  float voltage;
  float amperage;
  bool parking_position;
  bool startExperiment;
  bool breakdown;
};

void start_new_experiment(Dater * new_data){
  while(!(digitalRead(CONC_2) || new_data->servo >= (max_servo - new_data->servo))){
    servo.writeMicroseconds(1000); 
    delay(1600); //Подобрать время хода в одну сторону!!!! (1600 - один оборот)
    servo.writeMicroseconds(1500);
    delay(100);
  }
  servo.writeMicroseconds(2000); 
  delay(1600 * max_servo); //Подобрать время хода в одну сторону!!!! (1600 - один оборот)
  servo.writeMicroseconds(1500);
  delay(100);
  new_data->parking_position = true;
  new_data->servo = 0;
}

bool chance_to_fall(const Dater * data){
  return (digitalRead(CONC_2) || data->servo >= max_servo); //(или серво сделал 50 оборотов (или сколько там ему в одну сторону))
}

void first_check(Dater *data){
  servo.writeMicroseconds(1000); 
  delay(1600); //Подобрать время хода в одну сторону!!!! (1600 - один оборот)
  servo.writeMicroseconds(1500);
  delay(1000);
  servo.writeMicroseconds(2000);
  delay(1600);
  servo.writeMicroseconds(1500);
  delay(1000);
  sendUART("Electrods have just checked");
  saveSD("Electrods have just checked");
}

void get_telemetry(Dater *new_data){
  unsigned long current_time = millis();
  new_data->current_time = current_time;

  float temperature = temp.GetTemperature();
  new_data->temperature[0] = temperature;

  float pressure = bar.GetPressure();
  new_data->pressure = pressure;
  
  new_data->temperature[1] = bar.GetTemperature();

  float height = ((8.31 * (temperature + 273)) / (0.029 * 9.81)) * (log(760 / pressure));

  new_data->accel[0] = gacc.AX();
  new_data->accel[1] = gacc.AY();
  new_data->accel[2] = gacc.AZ();

  new_data->magn[0] = mag.MX();
  new_data->magn[1] = mag.MY();
  new_data->magn[2] = mag.MZ();
  
  new_data->gyro[0] = gacc.GX();
  new_data->gyro[1] = gacc.GY();
  new_data->gyro[2] = gacc.GZ();

}

String createStringToData(const Dater * data){
  String result = "";
  result += String(data->current_time, 10) + '\t';
  result += String(data->time_last_experiment, 10) + '\t';
  result += String(data->temperature[0], 2) + '\t';
  result += String(data->temperature[1], 2) + '\t';
  result += String(data->pressure, 2) + '\t';
  result += String(data->height, 2) + '\t';
  result += String(data->height_last_experiment) + '\t';
  result += String(data->magn[0], 2) + '\t';
  result += String(data->magn[1], 2) + '\t';
  result += String(data->magn[2], 2) + '\t';
  result += String(data->gyro[0], 2) + '\t';
  result += String(data->gyro[1], 2) + '\t';
  result += String(data->gyro[2], 2) + '\t';
  result += String(data->accel[0], 2) + '\t';
  result += String(data->accel[1], 2) + '\t';
  result += String(data->accel[2], 2) + '\t';
  result += String(data->voltage, 2) + '\t';
  result += String(data->amperage, 2) + '\t';
  result += String(data->servo, 2) + '\t';
  result += String(data->startExperiment);
  return result;
}

void saveSD(String text){
    dataFile = SD.open("datalog.txt", FA_OPEN_APPEND);
    if (dataFile){
        dataFile.println(text);
        dataFile.close();
    }
    else{
        Serialx.println("File not open");
    }
}

void sendUART(String text){
    Serialx.println(text);
}

void sendTextFormat(const Dater * data){
    String text = createStringToData(data);
    saveSD(text);
    sendUART(text);
}

void sendData(const Dater * data){
    sendTextFormat(data);
}

Dater new_data = { 0 };
unsigned long current_time = 0;
unsigned long time_from_last_experiment = 0;
unsigned long height_from_last_experiment = 0;

void setup() {
  SD.setDx(PC8, PC9, PC10, PC11);
  SD.setCMD(PD2);
  SD.setCK(PC12);
  Serialx.begin(115200, SERIAL_8E1);
  Serialx.print("Hello");
  pinMode(DELITEL, INPUT);
  pinMode(POT_SERVO, INPUT);
  pinMode(CONC_2, INPUT);
  pinMode(START_BUTTON, INPUT);
  Wirex.begin();
  Wirex2.begin();
  servo.attach(SERVO_PIN);
  bar.Init();
  gacc.InitAccel();
  gacc.InitGyro();
  mag.Init();
  temp.Init();
  while (!SD.begin());

  //заклинания для настройки таймера на определенную частоту шим
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(IVAN), PinMap_PWM);
  uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(IVAN), PinMap_PWM));
  HardwareTimer *MyTim = new HardwareTimer(Instance);
  MyTim->setPWM(channel, IVAN, 0, 0);

  get_telemetry(&new_data);

  new_data.servo = analogRead(POT_SERVO);

  first_check(&new_data);
  sendData(&new_data);
}

void loop() {
  get_telemetry(&new_data);
  time_from_last_experiment = new_data.current_time - new_data.time_last_experiment;
  height_from_last_experiment = new_data.height - new_data.height_last_experiment; 

  int reading = analogRead(TEMPERATURE_1_RX);
  float voltage = reading * 5;
  voltage /= 1024.0;
  float temperatureC = (voltage - 0.5) * 100 ;
  Serialx.print(analogRead(PD8));
  Serialx.print(' ');
  sendData(&new_data);
  Serialx.print(digitalRead(CONC_2));
  Serialx.print(' ');
  Serialx.print(digitalRead(START_BUTTON));
  Serialx.println(' ');
  delay(2000);
  /*
  if (time_from_last_experiment >= 4 * 60 * 1000 || height_from_last_experiment >= 1500){
      new_data.startExperiment = true;

      //это мы успешно (надеюсь) включаем плату Ивана, у нас будет 5 опорных точек напряжения (10, 8, 6, 4, 2 кВ) 
      TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(IVAN), PinMap_PWM);
      uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(IVAN), PinMap_PWM));
      HardwareTimer *MyTim = new HardwareTimer(Instance);

      MyTim->setPWM(channel, IVAN, 100000, 100); // 100КГц, 100% - коэффициент заполнения, Иван, какой нужен?
      new_data.voltage = 10; //первый прогон на 10кВ
      new_data.parking_position = false;

      while (!chance_to_fall(&new_data) && (analogRead(DELITEL) / new_data.amperage) < 2){ //2 - во сколько раз у нас возрастет ток
        get_telemetry(&new_data);

        servo.writeMicroseconds(1000); 
        delay(1600); 
        servo.writeMicroseconds(1500);
        delay(100);
        new_data.servo += 1; // увеличиваем количество оборотов сервы 
        
        if ((analogRead(DELITEL) / new_data.amperage) > 2){
          new_data.breakdown = 1;
          get_telemetry(&new_data);
          sendData(&new_data);
          MyTim->setPWM(channel, IVAN, 0, 0); // отключили напряжение
          start_new_experiment(); // уехали на исходную точку
          break;
        }

        if (chance_to_fall(&new_data)){
          get_telemetry(&new_data);
          sendData(&new_data);
          MyTim->setPWM(channel, IVAN, 0, 0); // отключили напряжение
          start_new_experiment(&new_data); // уехали на исходную точку
          break;
        }
        sendData(&new_data);
      }
      MyTim->setPWM(channel, IVAN, 100000, 100); // 100КГц, 100% - коэффициент заполнения, Иван, какой нужен?
      new_data.voltage = 8; 
      new_data.parking_position = false;

      while (!chance_to_fall(&new_data) && (analogRead(DELITEL) / new_data.amperage) < 2){ //2 - во сколько раз у нас возрастет ток
        get_telemetry(&new_data);

        servo.writeMicroseconds(1000); 
        delay(1600); 
        servo.writeMicroseconds(1500);
        delay(100);
        new_data.servo += 1; // увеличиваем количество оборотов сервы 
        
        if (analogRead(DELITEL) / new_data.amperage) > 2{
          new_data.breakdown = 1;
          get_telemetry(&new_data);
          sendData(&new_data);
          MyTim->setPWM(channel, IVAN, 0, 0); // отключили напряжение
          start_new_experiment(); // уехали на исходную точку
          break;
        }

        if (chance_to_fall(&new_data)){
          get_telemetry(&new_data);
          sendData(&new_data);
          MyTim->setPWM(channel, IVAN, 0, 0); // отключили напряжение
          start_new_experiment(); // уехали на исходную точку
          break;
        }
        sendData(&new_data);
      }

      MyTim->setPWM(channel, IVAN, 100000, 100); // 100КГц, 100% - коэффициент заполнения, Иван, какой нужен?
      new_data.voltage = 6; 
      new_data->parking_position = false;

      while (!chance_to_fall(&new_data) && (analogRead(DELITEL) / new_data.amperage) < 2){ //2 - во сколько раз у нас возрастет ток
        get_telemetry(&new_data);

        servo.writeMicroseconds(1000); 
        delay(1600); 
        servo.writeMicroseconds(1500);
        delay(100);
        new_data.servo += 1; // увеличиваем количество оборотов сервы 
        
        if (analogRead(DELITEL) / new_data.amperage) > 2{
          new_data.breakdown = 1;
          get_telemetry(&new_data);
          sendData(&new_data);
          MyTim->setPWM(channel, IVAN, 0, 0); // отключили напряжение
          start_new_experiment(); // уехали на исходную точку
          break;
        }

        if (chance_to_fall(&new_data)){
          get_telemetry(&new_data);
          sendData(&new_data);
          MyTim->setPWM(channel, IVAN, 0, 0); // отключили напряжение
          start_new_experiment(); // уехали на исходную точку
          break;
        }
        sendData(&new_data);
      }

      MyTim->setPWM(channel, IVAN, 100000, 100); // 100КГц, 100% - коэффициент заполнения, Иван, какой нужен?
      new_data.voltage = 4; 
      new_data->parking_position = false;

      while (!chance_to_fall(&new_data) && (analogRead(DELITEL) / new_data.amperage) < 2){ //2 - во сколько раз у нас возрастет ток
        get_telemetry(&new_data);

        servo.writeMicroseconds(1000); 
        delay(1600); 
        servo.writeMicroseconds(1500);
        delay(100);
        new_data.servo += 1; // увеличиваем количество оборотов сервы 
        
        if (analogRead(DELITEL) / new_data.amperage) > 2{
          new_data.breakdown = 1;
          get_telemetry(&new_data);
          sendData(&new_data);
          MyTim->setPWM(channel, IVAN, 0, 0); // отключили напряжение
          start_new_experiment(); // уехали на исходную точку
          break;
        }

        if (chance_to_fall(&new_data)){
          get_telemetry(&new_data);
          sendData(&new_data);
          MyTim->setPWM(channel, IVAN, 0, 0); // отключили напряжение
          start_new_experiment(); // уехали на исходную точку
          break;
        }
        sendData(&new_data);
      }

      MyTim->setPWM(channel, IVAN, 100000, 100); // 100КГц, 100% - коэффициент заполнения, Иван, какой нужен?
      new_data.voltage = 2; //первый прогон на 10кВ
      new_data->parking_position = false;

     while (!chance_to_fall(&new_data) && (analogRead(DELITEL) / new_data.amperage) < 2){ //2 - во сколько раз у нас возрастет ток
        get_telemetry(&new_data);

        servo.writeMicroseconds(1000); 
        delay(1600); 
        servo.writeMicroseconds(1500);
        delay(100);
        new_data.servo += 1; // увеличиваем количество оборотов сервы 
        
        if (analogRead(DELITEL) / new_data.amperage) > 2{
          new_data.breakdown = 1;
          get_telemetry(&new_data);
          sendData(&new_data);
          MyTim->setPWM(channel, IVAN, 0, 0); // отключили напряжение
          start_new_experiment(); // уехали на исходную точку
          break;
        }

        if (chance_to_fall(&new_data)){
          get_telemetry(&new_data);
          sendData(&new_data);
          MyTim->setPWM(channel, IVAN, 0, 0); // отключили напряжение
          start_new_experiment(); // уехали на исходную точку
          break;
        }
        sendData(&new_data);
      }
      new_data.time_last_experiment = millis();
  }
  */
}
