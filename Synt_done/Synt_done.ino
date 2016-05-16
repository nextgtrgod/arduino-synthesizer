//Подключение необходимых библиотек
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
 
//Инициализация входов и выходов 
#define seqStep_number  A1 //идентификатор шага секвенсора
#define potSeq_speed    A2 //потенциометр управления скоростью секвенсора
#define potLFO_freq     A3 //потенциометр управления частотой модуляции
#define potLFO_amount   A4 //потенциометр управления шириной  модуляции
#define potSeq_length   A5 //потенциометр управления длиной секвенции
#define speakerPin 11 //выход звука
#define buttonPin  2  //кнопка переключения огибающей волны
#define buttonSine 5  //кнопка включения сигнала синусоидального сигнала
#define buttonSaw  6  //кнопка включения пилообразного сигнала
#define buttonTri  7  //кнопка включения треугольного сигнала
#define buttonSqr  8  //кнопка включения прямоугольного сигнала
#define buttonSeqToggle 3 //вкл|выкл. секвенсор (арпеджиатор)
#define buttonLFOToggle 4 //вкл|выкл. модулятор
 
//инициализация частоы прерывания 
#define INTERRUPT_PERIOD 512
#define FINT (F_CPU / INTERRUPT_PERIOD) // 32kHz
#define FS (FINT)
 
//предварительно вычисленная таблица одного периода 8-bit'ного вывода синусоидального сигнала
const unsigned char PROGMEM sinetable[256] = {
  128,131,134,137,140,143,146,149,152,156,159,162,165,168,171,174,
  176,179,182,185,188,191,193,196,199,201,204,206,209,211,213,216,
  218,220,222,224,226,228,230,232,234,236,237,239,240,242,243,245,
  246,247,248,249,250,251,252,252,253,254,254,255,255,255,255,255,
  255,255,255,255,255,255,254,254,253,252,252,251,250,249,248,247,
  246,245,243,242,240,239,237,236,234,232,230,228,226,224,222,220,
  218,216,213,211,209,206,204,201,199,196,193,191,188,185,182,179,
  176,174,171,168,165,162,159,156,152,149,146,143,140,137,134,131,
  128,124,121,118,115,112,109,106,103,99, 96, 93, 90, 87, 84, 81, 
  79, 76, 73, 70, 67, 64, 62, 59, 56, 54, 51, 49, 46, 44, 42, 39, 
  37, 35, 33, 31, 29, 27, 25, 23, 21, 19, 18, 16, 15, 13, 12, 10, 
  9,  8,  7,  6,  5,  4,  3,  3,  2,  1,  1,  0,  0,  0,  0,  0,  
  0,  0,  0,  0,  0,  0,  1,  1,  2,  3,  3,  4,  5,  6,  7,  8,  
  9,  10, 12, 13, 15, 16, 18, 19, 21, 23, 25, 27, 29, 31, 33, 35, 
  37, 39, 42, 44, 46, 49, 51, 54, 56, 59, 62, 64, 67, 70, 73, 76, 
  79, 81, 84, 87, 90, 93, 96, 99, 103,106,109,112,115,118,121,124
};
 
//таблица/массив выходной волны
unsigned char wavetable[256];

//таблица частот, соответсвующих хроматической гаммы октав 2-6 (+C7)
short int note_mas[61] = {65,  69,  73,  78,  82,  87,  92,  98,  104, 110, 117, 124,
                          131, 139, 147, 156, 165, 175, 185, 196, 208, 220, 233, 247, 
                          262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494,
                          523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988, 
              1046, 1109, 1175, 1244, 1318, 1397, 1480, 1568, 1661, 1760, 1865, 1975, 2093};

short int seq_mas[8] = {0, 0, 0, 0, 0, 0, 0, 0};  //по-умолчанию сдвигов нет
short int seq_mas_MIDI[8];

unsigned int frequencyCoef = 100;
bool soundEnabled = true;
bool soundPWM = false;  //ШИМ
bool soundOn = false;
 
int currentVoice = 0;   //индекс вида сигнала, по-умолчанию синусоида
int MIDI_note = 0; 
/*----------------------------------------------------
 Основные функции
 ----------------------------------------------------*/
 
// Эта функция вызывается с частотой сэмплирования, в ней выводится новый сэмпл
ISR(TIMER1_COMPA_vect)
{
  static unsigned int phase0;
  static unsigned int sig0;
  static unsigned char flag = 0;
  static unsigned int tempphase;
 
  if (soundPWM)
  {
    tempphase = phase0 + frequencyCoef;
    sig0 = wavetable[phase0>>8];
    phase0 = tempphase;
    OCR2A = sig0; // регистр, управляющий ШИМ
  } 
  else { //square wave 
    flag ^= 1;
    digitalWrite(speakerPin, flag);
  }
  readMIDI();
}      
 
 
 
void setupPWMSound()
{
  // Используем TIMER2 для вывода звука с помощью ШИМ
  // Регистр ASSR, бит EXCLK = 0 - отключить работу 
  // с внешним источником тактового сигнала, использовать встроенный
  ASSR &= ~(_BV(EXCLK) | _BV(AS2));
  
  // Установим режим быстрого ШИМ
  TCCR2A |= _BV(WGM21) | _BV(WGM20);
  TCCR2B &= ~_BV(WGM22);
  
  // Выведем сигнал ШИМ на выход OC2A, на Arduino это pin 11
  TCCR2A = (TCCR2A | _BV(COM2A1)) & ~_BV(COM2A0);
  
  // Выход OC2B не используется ШИМом
  TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));
  
  // Не делим тактовую частоту таймера, оставим 16МГц как есть
  TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);
  
  // Сначала не выводим ничего, запишем 0 в значение ширины импульса
  OCR2A = 0;
  
  // Настроим TIMER1, чтобы по прерыванию от него выводить новый сэмпл.
  cli();
  
  // Режим таймера "сбросить при совпадении" (CTC - (Clear Timer on Compare Match). 
  // Таймер сбросится, когда досчитает до числа в регистре OCR1A, и сгенерирует прерывание
  TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
  TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));
  
  // Не делим тактовую частоту таймера, оставим 16МГц как есть
  TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);
  
  // Установим сравнивающий регистр (OCR1A).
  // Зададим частоту семплирования
  // 16 МГц / 512 = 31.25 кГц.
  OCR1A = INTERRUPT_PERIOD;
  
  // Разрешим прерывание от TIMER1 по совпадению с регистром OCR1A
  TIMSK1 |= _BV(OCIE1A);
  // Разрешим прерывания
  sei();
  soundPWM = true;
}
 
void startSound()
{
  // Чтобы включить звук, разрешим прерывание от TIMER1 по совпадению с регистром OCR1A
  // TIMSK1.OCIE1A = 1
  // Чтобы никто нам не помешал, закроем на этот момент остальные прерывания
  cli();
  TIMSK1 |= _BV(OCIE1A);
  
  // Готово, разрешим прерывания
  sei();
  soundOn = true;
} 
 
void stopSound()
{
  cli();  
  // Отключение воспроизведение при прерывании сэмпла
  // Чтобы выключить звук, запретим прерывание от TIMER1.
  TIMSK1 &= ~_BV(OCIE1A);
  sei();
  soundOn = false;
}
 
void setFrequency(short int freq)
{
  // исходя из требуемой частоты звука вычислим, 
  // сколько семплов приходится на каждое значение звуковой таблицы
  if (soundPWM) {
    unsigned long templong = freq;
    frequencyCoef = templong * 65536 / FS;
  } 
  else {
    unsigned long periode = F_CPU/(2*freq);
    cli();
    OCR1A = periode;
  }
}
 
/*------------------------------------------------------------------------------------
 функции поределения нужной таблицы, вызов функции заполнения
 ------------------------------------------------------------------------------------*/
void loadVoice(int voice)
{
  if(soundOn) // воспроизведение звука включено
  {
    stopSound(); // выключить звук
  }
  
  switch (voice)
  {
  // sine
  case 0:
    sineWave();
    break;
  // sawtooth
  case 1:
    sawtoothWave();
    break;
  // triangle
  case 2:
    triangleWave();
    break;
  // square
  case 3:
    squareWave();
    break;
  }
  
  // инициализируем ШИМ
  if(!soundPWM) 
  {
    setupPWMSound();
  }
  startSound(); // включить звук
}


//-----------------ФУНКЦИИ-ЗАДАНИЯ-ТАБЛИЦ----------------------------------------------------- 
void sineWave()
{
  // функция, загружающая синус из flash-памяти
  for (int i = 0; i < 256; ++i) {
    wavetable[i] = pgm_read_byte_near(sinetable + i);
  }
}
 
void sawtoothWave()  // пилообразный сигнал
{
  for (int i = 0; i < 256; ++i) {
    wavetable[i] = i; 
  }
}
 
 
void triangleWave() // треугольный сигнал
{
  for (int i = 0; i < 128; ++i) {
    wavetable[i] = i * 2;
  }
  int value = 255;
  for (int i = 128; i < 256; ++i) {
    wavetable[i] = value;
    value -= 2;
  }
}
 
void squareWave()  // прямоугольный сигнал
{
  for (int i = 0; i < 128; ++i) {
    wavetable[i] = 255;
  }
  for (int i = 128; i < 256; ++i) {
    wavetable[i] = 0;
  }
}
 

/*-----------------------------------------------------------------------------------
 стандартные функции setup() и loop()
 -----------------------------------------------------------------------------------*/

unsigned char LFO_mod = 0;

//ф-ия управления MIDI---------------------------------------------------------------
byte commandByte;     //bbbbcccc
byte noteByte;        //0xxxxxxx
byte velocityByte;    //0xxxxxxx
byte noteOn = 144;    //1001cccc
byte mode;           //режим огибающей модулятора

void setup()
{ 
  Serial.begin(31250); //частота опроса порта для принятия MIDI-сообщений (кбит/с)
  pinMode(speakerPin, OUTPUT); 
  loadVoice(currentVoice);
} 


//-------------Опрос MIDI-порта-------------------------------------------------------
void readMIDI()
{
  do
    {
      if (Serial.available())
      {
      commandByte = Serial.read();  //последовательное считывания трех байт MIDI-сообщения
      noteByte = Serial.read();     //-
      velocityByte = Serial.read(); //-
  
      if (commandByte == noteOn)
      {
        if (velocityByte > 0)       
        {
          startSound();
          int n = (int(noteByte)-36);  //C2 начинается с №36 MIDI
           if ((n >= 0) && (n <= 61))  //"невыход" за границы массива
           {
            MIDI_note = n;
           } 
        }
      }
    }
  }
  while (Serial.available() > 2);//when at least three bytes available
}


//--------------Опрос портов (кнопок), задание режима---------------------------------
void readButtons()
{
  // 0 - sine wave
  // 1 - sawtooth wave
  // 2 - triangle wave
  // 3 - square wave   
  if (digitalRead(buttonSine)==0)   //синусоида
   {
     delay(10);
     if (digitalRead(buttonSine)==0)
     currentVoice = 0;
     loadVoice(currentVoice);
   }  
   if (digitalRead(buttonSaw)==0)    //пила
   {
     delay(10);
     if (digitalRead(buttonSaw)==0)
     currentVoice = 1;
     loadVoice(currentVoice);
   }   
    if (digitalRead(buttonTri)==0)   //треугольник
   {
     delay(10);
     if (digitalRead(buttonTri)==0)
     currentVoice = 2;
     loadVoice(currentVoice);
   } 
  
    if (digitalRead(buttonSqr)==0)   //квадрат
   {
     delay(10);
     if (digitalRead(buttonSqr)==0)
     currentVoice = 3;
     loadVoice(currentVoice);
   }        
    if (digitalRead(buttonPin)==0)  //LFO_mod
   {
     delay(10);
     if (digitalRead(buttonPin)==0)
     LFO_mod = (LFO_mod + 1) % 3;
   } 
}

void Synth()
{
  // Seq | LFO  0-кнопка нажата, 1--;
  //   1 |   1  Simple output
  //   0 |   1  w/Sequencer
  //   1 |   0  w/LFO filter
  //   0 |   0  Sequencer w/LFO filter
  
  
  //выбор режима
//--------------------------------------------------------------------------------------------  
  if ((digitalRead(buttonSeqToggle)==1) && (digitalRead(buttonLFOToggle)==1))  //простой режим вывода звука
  {
    
    setFrequency(note_mas[MIDI_note]);
    delay(50);
  }

//--------------------------------------------------------------------------------------------  
  if ((digitalRead(buttonSeqToggle)==0) && (digitalRead(buttonLFOToggle)==1))  //режим секвенсора
  {
    {
     int inputPotValue = analogRead(potSeq_speed);
     int Seq_speed = map(inputPotValue, 0, 1023, 62, 250); //120bpm - 1/32--1/8
     
     inputPotValue = analogRead(potSeq_length);
     int Seq_length = map(inputPotValue, 0, 1023, 1, 8);   //кол-во шагов секвенции
    
     
              
     for (int i=0; i<8; i++)
     {
       seq_mas[i] = seq_mas_MIDI[i] - seq_mas_MIDI[1];
     }
       
      for (int i=0; i<Seq_length; i++)
      {
        if (((MIDI_note + seq_mas[i])>=0) && ((MIDI_note + seq_mas[i])<=61))  //"невыход" за границы массива нот
        {
          setFrequency(note_mas[MIDI_note + seq_mas[i]]);
          delay(Seq_speed);
        }
      }  
    }
  }

//-----------------------------------------------------------------------------------------------  
  if ((digitalRead(buttonSeqToggle)==1) && (digitalRead(buttonLFOToggle)==0))  //режим модуляции
  {
    
    LFO(MIDI_note, 1000);
  }

//--------------------------------------------------------------------------------------------  
  if ((digitalRead(buttonSeqToggle)==0) && (digitalRead(buttonLFOToggle)==0))  //режим секвенсора с модуляцией звука
  {
     {
     int inputPotValue = analogRead(potSeq_speed);
     int Seq_speed = map(inputPotValue, 0, 1023, 62, 250); //120bpm - 1/32--1/8
     
     inputPotValue = analogRead(potSeq_length);
     int Seq_length = map(inputPotValue, 0, 1023, 1, 8);   //кол-во шагов секвенции
    
      
              
     for (int i=0; i<8; i++)
     {
       seq_mas[i] = seq_mas_MIDI[i] - seq_mas_MIDI[1];
     }
       
      for (int i=0; i<Seq_length; i++)
      {
        if (((MIDI_note + seq_mas[i])>=0) && ((MIDI_note + seq_mas[i])<=61))  //"невыход" за границы массива нот
        {
          LFO((MIDI_note + seq_mas[i]), Seq_speed);
        }
      }  
    }
  }
}


//-------------------------------ФУНКЦИЯ--LFO-------------------------------------------------------------
void LFO(int note, int duration)
{
  int LFO_freq =   analogRead(potLFO_freq);    //частота работы фильтра (1 - 10Гц)
  int LFO_amount = map(analogRead(potLFO_amount), 0, 1023, 0, 100);  //величина отклонения фильтра
  
  int freq = note_mas[note];
  int time = (map(LFO_freq, 0, 1023, 62, duration))/32; //частота работы осциллятора филлтра модуляции
    
    switch (LFO_mod)
   {
   
   //квадрат
   case 0:
    for (int i=0; i<16; i++)
    {
      setFrequency(freq + LFO_amount);
      delay(time);
    }
     for (int i=16; i<32; i++)
    {
      setFrequency(freq - LFO_amount);
      delay(time);
    }
    break;
   
   //треугольник
   case 1:  
   {
    int tempFreq = freq - LFO_amount;
    setFrequency(tempFreq);
    delay(time);
    
    for (int i=1; i<=16; i++)
    {
      setFrequency(tempFreq + int(((2*LFO_amount)/16)*i));
      delay(time); 
    }

     tempFreq = freq + LFO_amount;
     
     for (int i=1; i<16; i++)
    {
      setFrequency(tempFreq - int(((2*LFO_amount)/16)*i));
      delay(time);
    }
    break;
   }
   
   //пила
   case 2:  
    {
    int tempFreq = freq - LFO_amount;
    setFrequency(tempFreq);
    delay(time);
    
    for (int i=1; i<32; i++)
    {
     tempFreq += int((2*LFO_amount)/32);
     setFrequency(tempFreq);
     delay(time); 
    }
    break;
    }
  }
}


void fill_seq()   //процедура заполенения секвенции
{   
 if (digitalRead(buttonSeqToggle==1))    //если секвенсор не включен
 {
  
  if ((analogRead(seqStep_number)<=1023) && (analogRead(seqStep_number)>=1019))      //1
      {
        delay(10);
        if ((analogRead(seqStep_number)<=1023) && (analogRead(seqStep_number)>=1019)) 
        {
          for (int i=1; i<=7; i++)
           {
             seq_mas[i] = 0;               //обнулить массив значений смещения относительно основного тона
             seq_mas_MIDI[i] = MIDI_note;  //
           }
        }
      }  
    if ((analogRead(seqStep_number)<=1003) && (analogRead(seqStep_number)>=998))    //2
      {
        delay(10);
        if ((analogRead(seqStep_number)<=1003) && (analogRead(seqStep_number)>=998))
        {
          seq_mas_MIDI[1] = MIDI_note;
        }
      }
    if ((analogRead(seqStep_number)<=984) && (analogRead(seqStep_number)>=977))     //3
      {
        delay(10);
        if ((analogRead(seqStep_number)<=984) && (analogRead(seqStep_number)>=977))
        {
          seq_mas_MIDI[2] = MIDI_note;
        }
      }
    if ((analogRead(seqStep_number)<=962) && (analogRead(seqStep_number)>=957))    //4
      {
        delay(10);
        if ((analogRead(seqStep_number)<=962) && (analogRead(seqStep_number)>=957))
        {
          seq_mas_MIDI[3] = MIDI_note;
        }
      }
    if ((analogRead(seqStep_number)<=943) && (analogRead(seqStep_number)>=937))    //5
      {
        delay(10);
        if ((analogRead(seqStep_number)<=943) && (analogRead(seqStep_number)>=937))
        {
          seq_mas_MIDI[4] = MIDI_note;
        }
      }
    if ((analogRead(seqStep_number)<=925) && (analogRead(seqStep_number)>=919))   //6
      {
        delay(10);
        if ((analogRead(seqStep_number)<=925) && (analogRead(seqStep_number)>=919)) 
        {
          seq_mas_MIDI[5] = MIDI_note;
        }
      }
    if ((analogRead(seqStep_number)<=906) && (analogRead(seqStep_number)>=901))   //7
      {
        delay(10);
        if ((analogRead(seqStep_number)<=906) && (analogRead(seqStep_number)>=901))
        {
          seq_mas_MIDI[6] = MIDI_note;
        }
      }
    if ((analogRead(seqStep_number)<=890) && (analogRead(seqStep_number)>=884))   //8
      {
        delay(10);
        if ((analogRead(seqStep_number)<=890) && (analogRead(seqStep_number)>=884))
        {
          seq_mas_MIDI[7] = MIDI_note;
        }
      }
 }     
}

void loop() 
{

  do
    {
      if (Serial.available())
      {
      commandByte = Serial.read();  //последовательное считывания трех байт MIDI-сообщения
      noteByte = Serial.read();     //-
      velocityByte = Serial.read(); //-
  
      if (commandByte == noteOn)
      {
      }
      else 
      {
        stopSound();
      }
    }
  }
  while (Serial.available() > 2);//when at least three bytes available

  
  
  
 readButtons();  //опрос портов переключателей режимов 
 fill_seq();     //заполнение секвенции 
 Synth();        //синтез звука
}
