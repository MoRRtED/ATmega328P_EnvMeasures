#include <LiquidCrystal.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define DHTTYPE DHT11
#define DHTPIN A0

#define LCD_RS A5
#define LCD_EN A4
#define LCD_D4 9
#define LCD_D5 7
#define LCD_D6 6
#define LCD_D7 4

#define KV_DIG_PIN 8
#define LED_SOUND 0
#define KVPIN A1

#define ST_CP 5 //latchPin 
#define SH_CP A2 //clockPin 
#define DS 10 //dataPin 

// This defines the LCD wiring to the DIGITALpins
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
DHT dht(DHTPIN, DHTTYPE);

int speakerPin_Term = 8;
int length = 1;

#define LED_TOO_COLD A0
#define LED_PERFECT 07
#define LED_TOO_HOT A2

void pciSetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

int interrupt_count = 0;
boolean LEDStatus = false;

//PC interrupt handler
ISR (PCINT0_vect) // for D8
{
  if (PINB & _BV(PB0))
  {
    float sound_analog = analogRead (KVPIN) * (5.0 / 1023.0);
    float sound_level = abs(20 * log10(sound_analog / 5.0));
    float resist = sound_analog / 118;
    Serial.print ("Analog voltage :");
    Serial.print (sound_analog, 4);
    Serial.print ("V ,");
    Serial.print (sound_level, 2);
    Serial.println ("dB");
    if (sound_level >= 6.1 && sound_level <= 6.3)
    {
      /*if (LEDStatus)
        {
        Serial.println("LED OFF");
        LEDStatus = false;
        PORTB &= ~bit (2); // turn off D10
        }
        else
        {
        Serial.println("LED ON");
        LEDStatus = true;
        PORTB |= bit (2);  // turn on D10
        }*/
      Serial.println("LED ON");
      byte bitsToSend = 0;
      int numberToDisplay = 0;
      // set HIGH РІ to the necessary level
      if (LEDStatus)
      {
        bitWrite(bitsToSend, numberToDisplay, HIGH);
      }
      else
      {
        bitWrite(bitsToSend, numberToDisplay, LOW);        
      }
      // set trigger to LOW
      digitalWrite(ST_CP, LOW);
      // РїРµСЂРµРґР°РµРј РїРѕСЃР»РµРґРѕРІР°С‚РµР»СЊРЅРѕ РЅР° dataPin
      shiftOut(DS, SH_CP, MSBFIRST, bitsToSend);
      //"Р·Р°С‰РµР»РєРёРІР°РµРј" СЂРµРіРёСЃС‚СЂ, С‚РµРј СЃР°РјС‹Рј СѓСЃС‚Р°РЅР°РІР»РёРІР°СЏ Р·РЅР°С‡РµРЅРёСЏ РЅР° РІС‹С…РѕРґР°С…
      digitalWrite(ST_CP, HIGH);
      LEDStatus = !LEDStatus;     
    }
  }
}

bool isInterrupt_T1 = false;
//interrupt T1 handler
ISR(TIMER1_COMPA_vect)
{
  isInterrupt_T1 = true;
}

//PC interrupt handler
/*ISR (PCINT1_vect) // for A4
  {
  float sound_analog = analogRead (KVPIN) * (5.0 / 1023.0);
  Serial.print ("Analog voltage :");
  Serial.print (sound_analog, 4);
  Serial.println ("V");
  }*/

void timerOneSetup()
{
  cli();
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536) // set compare match register for 1hz increments
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10); // Set CS12 and CS10 bits for 1024 prescaler
  //TIMSK1 |= (1 << TOIE1); //set to overflow mode
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  sei();
}

void setup()
{
  timerOneSetup();
  pciSetup(KV_DIG_PIN);
  Serial.begin(9600);
  Serial.println("DHT11 test!");
  lcd.begin(16, 2);

  pinMode(KVPIN, INPUT);
  pinMode(KV_DIG_PIN, INPUT);
  //pinMode(LED_SOUND, OUTPUT);

  //pinMode(speakerPin_Term, OUTPUT);
  pinMode(DS, OUTPUT);
  pinMode(ST_CP, OUTPUT);
  pinMode(SH_CP, OUTPUT);

  dht.begin();
}

void loop()
{
  if (isInterrupt_T1)
  {
    getDataDHT11();
    isInterrupt_T1 = false;
  }
  //for test purpouse
      byte bitsToSend = 0;
      int numberToDisplay = 0;
      // set HIGH РІ to the necessary level
      if (LEDStatus)
      {
        bitWrite(bitsToSend, numberToDisplay, HIGH);
        lcd.setCursor(0, 0);
        lcd.print("turned on ");
         
      }
      else
      {
        bitWrite(bitsToSend, numberToDisplay, LOW);
        lcd.setCursor(0, 0);
        lcd.print("turned off ");
        
      }


   
      // set trigger to LOW
      digitalWrite(ST_CP, LOW);
      // РїРµСЂРµРґР°РµРј РїРѕСЃР»РµРґРѕРІР°С‚РµР»СЊРЅРѕ РЅР° dataPin
      shiftOut(DS, SH_CP, MSBFIRST, bitsToSend);
      //"Р·Р°С‰РµР»РєРёРІР°РµРј" СЂРµРіРёСЃС‚СЂ, С‚РµРј СЃР°РјС‹Рј СѓСЃС‚Р°РЅР°РІР»РёРІР°СЏ Р·РЅР°С‡РµРЅРёСЏ РЅР° РІС‹С…РѕРґР°С…
      digitalWrite(ST_CP, HIGH);
      LEDStatus = !LEDStatus;     
      delay(100);
}

void getDataDHT11()
{
  int h = dht.readHumidity();
  int t = dht.readTemperature();
  float f = dht.readTemperature(true);

  if (isnan(h) || isnan(t) || isnan(f))
  {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Update LCD Display
  lcd.setCursor(0, 0);
  lcd.print("Humidity ");
  lcd.print(h);
  lcd.print("%");
  lcd.setCursor(0, 1);
  lcd.print("Temperature ");
  lcd.print(t);
  lcd.println("*C");

  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print("%\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.println(" *C ");
}


