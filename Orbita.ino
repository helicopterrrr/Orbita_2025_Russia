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
#define TEMPERATURE_1 PC1
#define TEMPERATURE_2 PC2_C
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

const int max_servo = 61; //сколько серве в одну сторону
struct Dater{
  unsigned long current_time;
  unsigned long time_last_experiment;
  float temperature[4];
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
  bool meet_concev;
  bool start_working;
};

void start_new_experiment(Dater * new_data){
  new_data->breakdown = false;
  servo.writeMicroseconds(1500);
  delay(100);
  servo.writeMicroseconds(1000); 
  delay(1600); //Подобрать время хода в одну сторону!!!! (1600 - один оборот)
  servo.writeMicroseconds(1500);
  delay(100);
  new_data->parking_position = true;
  new_data->servo = 0;
  while(!chance_to_fall(new_data)){
    servo.writeMicroseconds(2000); 
    delay(1600); //Подобрать время хода в одну сторону!!!! (1600 - один оборот)
    servo.writeMicroseconds(1500);
    delay(10);
  }
}

void experiment(int voltage, Dater*new_data){
  new_data->startExperiment = true;

  servo.writeMicroseconds(1000);
  delay(6 * 1600);
  servo.writeMicroseconds(1500);
  delay(1000);
  //это мы успешно (надеюсь) включаем плату Ивана, у нас будет 5 опорных точек напряжения (10, 8, 6, 4, 2 кВ) 
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(IVAN), PinMap_PWM);
  uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(IVAN), PinMap_PWM));
  HardwareTimer *MyTim = new HardwareTimer(Instance);

  MyTim->setPWM(channel, IVAN, voltage, 100); // 100КГц, 100% - коэффициент заполнения, Иван, какой нужен?
  new_data->voltage = voltage; //первый прогон на 10кВ
  new_data->parking_position = false;

  while (!chance_to_fall(new_data)){ //  && (analogRead(DELITEL) / new_data->amperage) < 2 2 - во сколько раз у нас возрастет ток
    get_telemetry(new_data);

    servo.writeMicroseconds(2000); 
    delay(1600); 
    servo.writeMicroseconds(1500);
    delay(10);
    new_data->servo += 1; // увеличиваем количество оборотов сервы 

    /* 
    if ((analogRead(DELITEL) / new_data->amperage) > 2){
      new_data->breakdown = 1;
      get_telemetry(new_data);
      sendData(new_data);
      MyTim->setPWM(channel, IVAN, 0, 0); // отключили напряжение
      start_new_experiment(new_data); // уехали на исходную точку
      break;
    }
    */ 

    if (chance_to_fall(new_data)){
      Serialx.println("dhfhdjhfdjgjdfkgkj");
      get_telemetry(new_data);
      sendData(new_data);
      MyTim->setPWM(channel, IVAN, 0, 0); // отключили напряжение
      start_new_experiment(new_data); // уехали на исходную точку
      Serialx.println("dhfhdjhfdjgjdfkgkj");
      break;
    }
    new_data->amperage = analogRead(DELITEL);
    sendData(new_data);
    }
}
bool chance_to_fall(Dater * data){
  return (!digitalRead(CONC_2)); // (или серво сделал 50 оборотов (или сколько там ему в одну сторону))
}

void first_check(Dater * new_data){
  int one_turn = 0;
  while (!chance_to_fall(new_data)){
    servo.writeMicroseconds(2000); 
    delay(1600); //Подобрать время хода в одну сторону!!!! (1600 - один оборот)
    servo.writeMicroseconds(1500);
    delay(10);
    one_turn += 1;
  }

  servo.writeMicroseconds(1000);
  delay(1600 * 60);
  servo.writeMicroseconds(1500);
  delay(1000);

  while (!chance_to_fall(new_data)){
    servo.writeMicroseconds(2000); 
    delay(1600); //Подобрать время хода в одну сторону!!!! (1600 - один оборот)
    servo.writeMicroseconds(1500);
    delay(10);
    one_turn += 1;
  }
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
  int reading = analogRead(TEMPERATURE_1);
  float voltage = reading * 3.3;
  voltage /= 1024.0;
  float temperatureC = (voltage - 0.5) * 100 ;
  new_data->temperature[2] = temperatureC;

  reading = analogRead(TEMPERATURE_2);
  voltage = reading * 3.3;
  voltage /= 1024.0;
  temperatureC = (voltage - 0.5) * 100 ;
  new_data->temperature[3] = temperatureC;

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

  new_data->meet_concev = digitalRead(CONC_2);
}

String createStringToData(Dater * data){
  String result = "";
  result += String(data->current_time, 10) + '\t';
  result += String(data->time_last_experiment, 10) + '\t';
  result += String(data->temperature[0], 2) + '\t';
  result += String(data->temperature[1], 2) + '\t';
  result += String(data->temperature[2], 2) + '\t';
  result += String(data->temperature[3], 2) + '\t';
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
  result += String(data->breakdown, 2) + '\t';
  result += String(data->meet_concev, 2) + '\t';
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

void sendTextFormat(Dater * data){
    String text = createStringToData(data);
    saveSD(text);
    sendUART(text);
}

void sendData(Dater * data){
    sendTextFormat(data);
}

Dater new_data = { 0 };
unsigned long current_time = 0;
unsigned long time_from_last_experiment = 0;
unsigned long height_from_last_experiment = 0;

void setup() {
  pinMode(START_BUTTON, INPUT);
  SD.setDx(PC8, PC9, PC10, PC11);
  SD.setCMD(PD2);
  SD.setCK(PC12);
  Serialx.begin(115200, SERIAL_8E1);
  Serialx.print("Hello");
  pinMode(DELITEL, INPUT);
  pinMode(POT_SERVO, INPUT);
  pinMode(CONC_2, INPUT);
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

  new_data.servo = analogRead(POT_SERVO);
  while(!digitalRead(START_BUTTON)){
    get_telemetry(&new_data);
    sendData(&new_data);
    delay(2000);
  }
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(IVAN), PinMap_PWM);
  uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(IVAN), PinMap_PWM));
  HardwareTimer *MyTim = new HardwareTimer(Instance);
  MyTim->setPWM(channel, IVAN, 0, 0);
  //first_check(&new_data);
  sendData(&new_data);
}

void loop() {
  get_telemetry(&new_data);
  time_from_last_experiment = new_data.current_time - new_data.time_last_experiment;
  height_from_last_experiment = new_data.height - new_data.height_last_experiment; 
  sendData(&new_data);

  if (time_from_last_experiment >= 60 * 1000 || height_from_last_experiment >= 1500){
      new_data.startExperiment = true;
      get_telemetry(&new_data);
      sendData(&new_data);

      experiment(10000, &new_data);
      experiment(8000, &new_data);
      experiment(6000, &new_data);
      experiment(4000, &new_data);
      experiment(2000, &new_data);

      get_telemetry(&new_data);
      new_data.height_last_experiment = new_data.height;
      new_data.time_last_experiment = millis();
      new_data.startExperiment = false;
      sendData(&new_data);
  }
}
