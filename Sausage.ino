// -----------------------------------------------------------------------------------------------------
// Приготовление колбасы
//
// Режим 1
// Включается реле 1 и работает на поддержание по DHT22 t=60C
// Когда на щупе температура поднялась до 42C Поддержание DHT22 t=90C + Включается реле2 ДЫМ
// Когда на щупе поднялась температура до 60C реле 1 поддерживает t=80C Реле 2 ДЫМ выкл, включается реле3 для поддержания влажности 80%
// Когда на щупе t=72 Выключается реле1, реле2, реле3
//
// Режим 2
// Поддержание необходимой температуры и влажности
//
// Режим 3 Корректировка температур и времени приготовления
// ----------------------------------------------------------------------------------------------------
/*     Структура массива
 *     1. Температура на щупе, к которой стремимся
 *     2. Температура включения реле1
 *     3. Температура выключения реле1
 *     4. Реле2 (0-выключено, 1-включено)
 *     5. Температура включения реле3
 *     6. Температура выключения реле3
 *     {42,50,60,0,0,0}    До t42C Нагреватель1 (от50C до 60C) Реле2 ВЫКЛ Реле3 ВЫКЛ
 *     {60,80,90,1,0,0}    До t60C Нагреватель1 (от80C до 90C) Реле2 ВКЛ Реле3 ВЫКЛ
 *     {72,70,80,0,70,80}  До t72C Нагреватель1 (от70C до 80C) Реле2 ВЫКЛ Реле3 ВКЛ (от 70% до 80%)
 *     Stage - Стадия приготовления 0 - выкл, 1- Стадия1, 2 - Стадия2, 3- Стадия3
*/

//#include <LCD5110_Basic.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <avr/eeprom.h>
//#include "GyverButton.h"
#include "GyverTimers.h"
#include "GyverRelay.h"
#include "GyverEncoder.h"

#include <dht.h>
#define DHT22_PIN 4     // Сюда подключен DHT22
#define SENSOR1_PIN A0  // Сюда подключен Датчик щупа№1  Главный щуп
#define SENSOR2_PIN A1  // Сюда подключен Датчик щупа№2
#define SENSOR3_PIN A2  // Сюда подключен Датчик щупа№3
#define BTN_PIN 3   // кнопка   СТАРТ (BTN_PIN --- КНОПКА --- GND)
#define DS_PIN 2  // пин датчика температуры 18b20

#define HEATER_PIN    A3  // Реле нагревателя
#define SMOKE_PIN     A4  // Реле Дыма
#define HUMMIDITY_PIN A5  // Реле нагревателя для вланости
 
#define LED_ON  digitalWrite(13, HIGH)
#define LED_OFF digitalWrite(13, LOW)

#define ON  LOW
#define OFF HIGH

#define ENCODER_CLK 11
#define ENCODER_DT 12

//----------------------------------------------------------------------------- Термодатчик
#define B 3950 // B-коэффициент
#define SERIAL_R 240000 // сопротивление последовательного резистора, 120 кОм
#define THERMISTOR_R 100000 // номинальное сопротивления термистора, 100 кОм
#define NOMINAL_T 25 // номинальная температура (при которой TR = 100 кОм)
#define tempPin  A0
//--------------------------------------------------------------------------------------------

volatile unsigned char led_status=0,LedRefresh=1,DHT_read_data=1, Stage=0, Mode=0, StartCoocking=0,cClock=0, AnalogVal=0;
volatile unsigned char EncoderValue=0, EditSettings=0, Row=0, Col=0;
float DSTemp=0;

unsigned char Sensors[][6] = { {42,50,60,0,0,0 },      // До t=42C Реле1 t=60C, Реле2 ВЫКЛ, Реле 3ВЫКЛ
                               {60,80,90,1,0,0 },      // До t=60C Реле1 t=90C, Реле2 ВКЛ, Реле3 ВЫКЛ
                               {72,70,80,0,70,80},     // До t=72C Реле1 t=80C, Реле2 ВЫКЛ, Реле3 80%
                               {0,0,0,0,0,0},          // Всё ВЫКЛ
                               {0,28,35,0,55,85} };    // Поддержание температуры и влажности


dht DHT;
struct
{
    uint32_t total;
    uint32_t ok;
    uint32_t crc_error;
    uint32_t time_out;
    uint32_t connect;
    uint32_t ack_l;
    uint32_t ack_h;
    uint32_t unknown;
} stat = { 0,0,0,0,0,0,0,0}; 

//GButton butt1(BTN_PIN);
Encoder Encoder1(ENCODER_CLK, ENCODER_DT, BTN_PIN, TYPE2);

//LCD5110(SCK=9, MOSI=8, DC=7, RST=5, CS=6)
Adafruit_PCD8544 display = Adafruit_PCD8544(9, 8, 7, 6, 5);       // Инициализация дисплея

void setup() 
{
  pinMode(13, OUTPUT);           // будем мигать

  pinMode(HEATER_PIN,    OUTPUT);
  pinMode(SMOKE_PIN,     OUTPUT);
  pinMode(HUMMIDITY_PIN, OUTPUT);

  AllRelayOff();  // Выключить все реле
  analogRead(SENSOR1_PIN);  // Сбросить мусор из порта

  dallas_begin(DS_PIN); // инициализация DS18B20
  
  Serial.begin(115200);
  Timer1.setFrequency(2);               // Высокоточный таймер 1 для первого прерывания, частота - 2 Герца
  //Timer1.setPeriod(333333);           // то же самое! Частота 3 Гц это период 333 333 микросекунд
  //Timer1.setFrequencyFloat(4.22);     // Если нужна дробная частота в Гц  
  Timer1.enableISR();                   // Запускаем прерывание (по умолч. канал А)
//  cClock = eeprom_read_byte(0);
//  cClock++;
//  eeprom_write_byte(0,cClock);

    display.begin();
    display.clearDisplay();
    display.cp437(true);
    display.setContrast(56);

    if(eeprom_read_byte(1)==255)        // Если настройки не записаны в EEPROM, записать
    {
      WriteSettings();
    }
    else
    {
      ReadSettings();
    }
}


void loop() 
{
  Encoder1.tick();  // Обработка энкодера
  if(Encoder1.isSingle() )   // Один клик и режим приготовления выключен
  {
    if(EditSettings==0)     // Редактирование настроек выключено
    {
        if(StartCoocking==0)
        {
            Serial.print("Mode Change=");

            if(Mode==0)           // Переключить на режим поддержания
            {
              Mode=1;          
              Stage=4;
            } 
            else
            {
                Mode=0;    // Переключить на режим приготовления
                Stage=0;
            }

            Serial.print(Mode,1);  
            Serial.println("\t");
        }
    }
    else                                    //--------------------------  Режим редактирования настроек. Изменить указатель строки столбца
    {
      if(Row==4 && Col==5)
      {
        Row=0;
        Col=0;
      }
      else
      {
          if(Col<5)        Col=Col+1;
          else
          {
            if(Row<3)
            {
              Row=Row+1;
              Col=0;
            }
            if(Row==3)
            {
              Row=4;
              Col=0;
            }
          }
      }
    }
  }

  if ( Encoder1.isHolded() )                       // Удержание кнопки.
  {
      if(EditSettings==0)                       // Редактирование режимов выключено, можно отрабатывать приготовление
      {
        Serial.println("Coocking Change");       // проверка на удержание

        if(StartCoocking==0)  StartCoocking=1;  // Включить режим приготовления
        else                                    // Выключить режим приготовления
        {
          Stage=0;  //Стадия приготовления сброшена в начальную позицию
          StartCoocking=0;  
          AllRelayOff();
        }
        Serial.print(StartCoocking, 1);
        Serial.print("\t");
        Serial.print(Mode, 1);
        Serial.print("\t");
      }
  }

//--------------------------------------------------------------------------------- Режим редактирования настроек
    if(Encoder1.isDouble() && StartCoocking==0)
    {
        if(EditSettings==0)
        {
          EditSettings=1;
          Row=0;
          Col=0;
        }
        else
        {
          EditSettings=0;
          WriteSettings();            // ----------------------------------------- Записать настройки
        }
    }
  
  if(EditSettings==0)
  {
    DHT_Read();
    LedShow();
  }
  else
  {
    ChangeSettings();
    if(Encoder1.isRight())
    {
      if(Sensors[Row][Col]<100) Sensors[Row][Col]=Sensors[Row][Col]+1;
      else                      Sensors[Row][Col]=0;
      if(Col==3 && Sensors[Row][Col]>1) Sensors[Row][Col]=1;
    }
    if(Encoder1.isLeft())
    {
      if(Sensors[Row][Col]!=0)  Sensors[Row][Col]=Sensors[Row][Col]-1;
      else                      Sensors[Row][Col]=99;
      if(Col==3 && Sensors[Row][Col]>1) Sensors[Row][Col]=0;
    }
  }
}


void AllRelayOff()    // Выключить все реле
{
        StartCoocking=0;  
        Stage=0;
        Mode=0;
        digitalWrite(HEATER_PIN,    OFF);
        digitalWrite(SMOKE_PIN,     OFF);
        digitalWrite(HUMMIDITY_PIN, OFF);
}


// Прерывание А таймера 1
ISR(TIMER1_A) 
{  // пишем  в сериал
//  Serial.println("timer1");
     if(led_status==0)
     {
        led_status=1;
     }
     else
     {
        led_status=0;
     }
     LedRefresh=1; // Обновить экран
     DHT_read_data=1;   // Перечитать термодатчик
}



void WriteSettings()
{
  unsigned char i=0, j=0, StartPos=1;

        Serial.println("Write Settings ");
        for(i=0;i<5;i++)
        {
          for(j=0;j<6;j++)
          {
            if(Sensors[i][j]!=eeprom_read_byte(StartPos))     eeprom_write_byte(StartPos, Sensors[i][j]);
            StartPos=StartPos+1;
          }
        }

}

void ReadSettings()
{
  unsigned char i=0, j=0, StartPos=1;

        Serial.println("Read Settings ");
        for(i=0;i<5;i++)
        {
          for(j=0;j<6;j++)
          {
            Sensors[i][j]=eeprom_read_byte(StartPos);
            Serial.print(Sensors[i][j], 1);
            Serial.print(" ");
            StartPos=StartPos+1;
          }
          Serial.println("");
        }

}





void DHT_Read()
{
//  int AnalogVal=0;
  int TempDecimal=0, TempInt=0;

  if(DHT_read_data==1)
  {
    CalckTemp();
    DSTemp = dallas_getTemp(DS_PIN);          //--------------------------- Считать температуру DS18B20
    Serial.print(" 18b20=");
    Serial.print(DSTemp);
    Serial.print(" ");

      Serial.print(" Encoder=");
      Serial.print(EncoderValue,1);
      Serial.print(" ");

    DHT_read_data=0;
    DHT.read22(DHT22_PIN);
//    AnalogVal=analogRead(SENSOR1_PIN);
    AnalogVal=CalckTemp();                //----------------------- Расчёт температуры на аналоговом датчике

    DHT.temperature=DSTemp;               //---------------- Заменить температуру DHT температурой датчика ds18B20

    Serial.print(AnalogVal,1);
    Serial.print("\t");
    
    Serial.print(DHT.temperature, 1);
    Serial.print("C\t");
    
    Serial.print("[");                      // ----------------- Выделить градусы и десятую градусов
    TempInt=(DHT.temperature*10);
    TempDecimal=DHT.temperature;
    TempDecimal=TempDecimal*10;
    TempDecimal=TempInt-TempDecimal;
    TempInt=DHT.temperature;
    Serial.print((unsigned char)TempInt,1);
    Serial.print(" ");
    Serial.print((unsigned char)TempDecimal,1);
    Serial.print("]");

    Serial.print(DHT.humidity, 1);
    Serial.print("%");

//------------------------------------------------------------ Показать температутуру и влажность
    display.clearDisplay(); // очищаем дисплей
    display.setCursor(45, 10);
    display.setTextSize(2); // размер текста 2
    display.setTextColor(BLACK); // цвет текста темный
//    display.print((unsigned char) floor(DHT.temperature));
//    display.print((char)176);     // Знак градуса

    display.print((unsigned char) TempInt);
    display.setTextSize(1); // размер текста 2
    display.setCursor(71, 10);
    display.print((unsigned char) TempDecimal);


    display.setTextSize(2); // размер текста 2
    display.setCursor(45, 30);
    display.setTextColor(BLACK); // цвет текста темный
    display.print((unsigned char) DHT.humidity);
    display.print("%");

//    display.display();
//    display.setCursor(15, 0);
//    display.clearDisplay(); // очищаем дисплей
//----------------------

    if(StartCoocking==1)  // Запущен режим приготовления
    {
       Serial.print("Mode=");
       Serial.print(Mode, 1);    // Режим приготовления
       Serial.print("\t");

       Serial.print("(");
       Serial.print(Stage, 1);    // Цикл приготовления
       Serial.print(")\t");
       Serial.print(Sensors[Stage][0], 1);    // Целевая температура
       Serial.print("C\t");

       Serial.print(Sensors[Stage][1], 1);    // температура включения нагревателя
       Serial.print("-");
       Serial.print(Sensors[Stage][2], 1);    // температура выключения нагревателя

       Serial.print("C\t");
       Serial.print(Sensors[Stage][3], 1);    // Реле2 ДЫМ

       Serial.print("\t");
       Serial.print(Sensors[Stage][4], 1);    // Влажность включение нагревателя
       Serial.print("%-");
       Serial.print(Sensors[Stage][5], 1);    // Влажность выключние нагревателя
       Serial.print("%\t");

       if(Mode==0)         // Режим приготовления 1
       {
        //{42,60,0,0}
                                                                                            //---------------------- Нагреватель
          if(DHT.temperature<=Sensors[Stage][1] && Sensors[Stage][0]>0)   // Температура меньше, включить нагреватель
          {
              display.setCursor(35, 9);
              display.print((char)24);
              digitalWrite(HEATER_PIN, ON);
          }
          if(DHT.temperature>=Sensors[Stage][2])  digitalWrite(HEATER_PIN, OFF);                        // Температура меньше, выключить нагреватель
//          if(cClock>=5) digitalWrite(HEATER_PIN, OFF);

          if(Sensors[Stage][3]>0)                         // Включить дым                   //---------------------- Дым
          {
              display.setCursor(35, 24);
              display.print((char)126);
              digitalWrite(SMOKE_PIN, ON);
          }
          if(Sensors[Stage][3]==0)    digitalWrite(SMOKE_PIN,OFF);  // Выключить дым


                                                                                            //---------------------- Влажность
          if(DHT.humidity<=Sensors[Stage][4] && Sensors[Stage][0]>0)  // Влажность меньше, включить нагреватель
          {
              display.setCursor(35, 29);
              display.print((char)24);
              digitalWrite(HUMMIDITY_PIN, ON);
          }
          if(DHT.humidity>=Sensors[Stage][5]) digitalWrite(HUMMIDITY_PIN,OFF);                        // Влажность больше, выключить нагреватель

          if(AnalogVal>=Sensors[Stage][0])   Stage=Stage+1;    // Температура на щупе поднялась к нужному значению

          // Если все элементы массива пустые, то выключить всё. Закончили цикл приготовления
          if(Sensors[Stage][0]==0 && Sensors[Stage][2]==0 && Sensors[Stage][1]==0 &&
             Sensors[Stage][3]==0 && Sensors[Stage][4]==0 && Sensors[Stage][5]==0 )
          {
            StartCoocking=0;
            Stage=0;
            Serial.print("END OF STAGE");    // Конец цикла приготовления  ВЫКЛЮЧИТЬ ВСЁ
            AllRelayOff();
          }
       }
       else               // Режим приготовления 2
       {
          Serial.print("------------>");
          if(DHT.temperature<=Sensors[Stage][1] && Sensors[Stage][1]>0)   // Температура меньше, включить нагреватель
          {
              digitalWrite(HEATER_PIN, ON);   // Температура меньше, включить нагреватель
              display.setCursor(35, 9);
              display.print((char)24);
          }
          if(DHT.temperature>=Sensors[Stage][2])  digitalWrite(HEATER_PIN, OFF);                        // Температура меньше, выключить нагреватель
          if(Sensors[Stage][3]>0)             // Включить дым
          {
              digitalWrite(SMOKE_PIN, ON);
              display.setCursor(35, 24);
              display.print((char)126);
          }
          if(Sensors[Stage][3]==0)    digitalWrite(SMOKE_PIN,OFF);  // Включить дым
          if(DHT.humidity<=Sensors[Stage][4] && Sensors[Stage][0]>0)                                    // Влажность меньше, включить нагреватель
          {
              digitalWrite(HUMMIDITY_PIN, ON);
              display.setCursor(35, 29);
              display.print((char)24);
          }
          if(DHT.humidity>=Sensors[Stage][5]) digitalWrite(HUMMIDITY_PIN,OFF);                        // Влажность больше, выключить нагреватель
        
       }
    }
    Serial.println("");
//    LedShow();
    dallas_requestTemp(DS_PIN);   //------------------- Запросить температуру для следующей итерации
  }
}


void LedShow()
{
    if(LedRefresh=1)     // Обновить экран
    {
      LedRefresh=0;
      display.setCursor(3, 0);
      display.setTextSize(1); // размер текста 1
      display.setTextColor(BLACK); // цвет текста темный
      if(led_status==0)
      {
        LED_OFF;
        if(StartCoocking==1)  display.setTextColor(WHITE); // цвет текста темный
      }
      else
      {
        LED_ON;
      }
      if(Mode==1) display.print(utf8rus("  Термостат"));
      else        display.print(utf8rus("Приготовление"));
//------------------
      display.setTextColor(BLACK);
      if(StartCoocking==1)
      {
        if(Mode==0)
        {
          display.setCursor(0, 40);
          display.print(utf8rus("Шаг "));
          display.print((unsigned char)Stage+1);    // Цикл приготовления
        }
      }
//------------------
        display.setCursor(0, 10);
        display.print("t1=");
        display.print((unsigned char)AnalogVal);    // Температура датчика1
//        display.print((unsigned char)CalckTemp());    // Температура датчика1
        display.print((char)176);

        display.setCursor(0, 20);
        display.print("t2=");
//        display.print((unsigned char)analogRead(SENSOR2_PIN));    // Температура датчика2
        display.print((char)176);

        display.setCursor(0, 30);
        display.print("t3=");
//        display.print((unsigned char)analogRead(SENSOR3_PIN));    // Температура датчика3
        display.print((char)176);
/*
        display.setCursor(0, 40);
        display.print(DSTemp,1);    // Температура из DS18B20
        display.print((char)176);
*/
        display.display();
    }
}



void ChangeSettings()
{
    unsigned char StringPos=0, ShowRow, ShowCol;

    if(LedRefresh=1)     // Обновить экран
    {
      LedRefresh=0;
      display.clearDisplay();
      display.setCursor(3, 0);
      display.setTextSize(1); // размер текста 1
      if(led_status==0)
      {
        LED_OFF;
      }
      else
      {
        LED_ON;
      }
        display.print(utf8rus("  Настройка"));
        display.setCursor(4, 0);
        display.print(utf8rus("  Настройка"));

        for(ShowRow=0;ShowRow<5;ShowRow++)
        {
          if(ShowRow==3)  ShowRow=4;

           switch (ShowRow)
           {
              case 0:
                  StringPos=10;
                  break;
              case 1:
                  StringPos=20;
                  break;
              case 2:
                  StringPos=30;
                  break;
              case 4:
                  StringPos=40;
                  break;
           }
          for(ShowCol=0;ShowCol<6;ShowCol++)
          {
              switch (ShowCol)
              {
                case 0:
                      display.setCursor(0, StringPos);
                      break;
                case 1:
                      display.setCursor(17, StringPos);
                      break;
                case 2:
                      display.setCursor(30, StringPos);
                      break;
                case 3:
                      display.setCursor(47, StringPos);
                      break;
                case 4:
                      display.setCursor(59, StringPos);
                      break;
                case 5:
                      display.setCursor(72, StringPos);
                      break;
              }

              if(ShowRow==Row && ShowCol==Col)
              {
                  if(led_status==0)                display.setTextColor(WHITE); // цвет текста светлый
              }

              display.print((unsigned char)Sensors[ShowRow][ShowCol]);
              display.setTextColor(BLACK);
          }
        }
      display.display();
    }
}

//------------------------------------------------------------------------------- Расчёт температуры с аналогово датчика
float CalckTemp()
{
    int t = analogRead( tempPin );
    float tr = 1023.0 / t - 1;
    tr = SERIAL_R / tr;
    Serial.print(" R=");
    Serial.print(tr);
    Serial.print(", t=");

    float steinhart;
    steinhart = tr / THERMISTOR_R; // (R/Ro)
    steinhart = log(steinhart); // ln(R/Ro)
    steinhart /= B; // 1/B * ln(R/Ro)
    steinhart += 1.0 / (NOMINAL_T + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart; // Invert
    steinhart -= 273.15; 
    Serial.print(steinhart);
    return(steinhart);
}

// ------------------------------------------------------------- DS18B20
void dallas_begin(uint8_t pin) {
  pinMode(pin, INPUT);
  digitalWrite(pin, LOW);
}
void dallas_requestTemp(uint8_t pin) {
  if (oneWire_reset(pin)) return;
  oneWire_write(0xCC, pin);
  oneWire_write(0x44, pin);
}
float dallas_getTemp(uint8_t pin) {
  uint8_t data[2];
  if (oneWire_reset(pin)) return;
  oneWire_write(0xCC, pin);
  oneWire_write(0xBE, pin);
  data[0] = oneWire_read(pin);
  data[1] = oneWire_read(pin);
  float result = (float)((data[1] << 8) | data[0]) * 0.0625; //>
  return result;
}



// --------------------------------------------------------------- 1wire
boolean oneWire_reset(byte pin) {
  pinMode(pin, 1);
  delayMicroseconds(640);
  pinMode(pin, 0);
  delayMicroseconds(2);
  for (uint8_t c = 80; c; c--) {
    if (!digitalRead(pin)) {
      uint32_t tmr = micros();
      while (!digitalRead(pin)) {
        if (micros() - tmr > 200) return false;
      }
      return false;
    }
    delayMicroseconds(1);
  }
  return true;
}
void oneWire_write(uint8_t data, byte pin) {
  for (uint8_t p = 8; p; p--) {
    pinMode(pin, 1);
    if (data & 1) {
      delayMicroseconds(5);
      pinMode(pin, 0);
      delayMicroseconds(90);
    } else {
      delayMicroseconds(90);
      pinMode(pin, 0);
      delayMicroseconds(5);
    }
    data >>= 1;
  }
}
uint8_t oneWire_read(byte pin) {
  uint8_t data = 0;
  for (uint8_t p = 8; p; p--) {
    data >>= 1;
    pinMode(pin, 1);
    delayMicroseconds(2);
    pinMode(pin, 0);
    delayMicroseconds(8);
    bool dataBit = digitalRead(pin);
    delayMicroseconds(80);
    if (dataBit) data |= 0x80;
  }
  return data;
}
