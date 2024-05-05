

///
/// @mainpage   Stepper41
///
/// @details   Description of the project
/// @n
/// @n
/// @n @a      Developed with [embedXcode+](https://embedXcode.weebly.com)
///
/// @author      Ruedi Heimlicher
/// @author      Ruedi Heimlicher
/// @date      06.05.2020 21:02
/// @version   n
///
/// @copyright   (c) Ruedi Heimlicher, 2020
/// @copyright   GNU General Public Licence
///
/// @see      ReadMe.txt for references
///

///
/// @file      Stepper32.ino
/// @brief      Main sketch
///
/// @details   <#details#>
/// @n @a      Developed with [embedXcode+](https://embedXcode.weebly.com)
///
/// @author      Ruedi Heimlicher
/// @author      Ruedi Heimlicher
/// @date      06.05.2020 21:02
/// @version   <#version#>
///
/// @copyright   (c) Ruedi Heimlicher, 2020
/// @copyright   GNU General Public Licence
///
/// @see      ReadMe.txt for references
/// @n
///


#include "Arduino.h"
#include <stdint.h>

#include <SD.h>
#include <SPI.h>

#include <Wire.h>

//#include <LiquidCrystal_I2C.h> // auch in Makefile angeben!!!


// von VS_RobotAuto_T
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <LiquidCrystal_I2C.h>
#include "main.h"
#include "display.h"
#include "font.h"

 #include "lcd.h"
#include "settings.h"

#include <ADC.h>
// https://registry.platformio.org/libraries/adafruit/Adafruit%20SSD1327/examples/ssd1327_test/ssd1327_test.ino

//#include <MUIU8g2.h>


#define OLED_RESET   -1
#define OLED_CS      14
#define OLED_DC      12
#define OLED_CLK     13
#define OLED_MOSI    11


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);



#define TASTEOK            1
#define AKTIONOK           2
#define UPDATEOK           3

uint16_t  cursorpos[8][8]={}; // Aktueller screen: werte fuer page und darauf liegende col fuer den cursor
uint16_t  posregister[8][8]; // Aktueller screen: werte fuer page und daraufliegende col fuer Menueintraege (hex). geladen aus progmem


const int chipSelect = 14;

// https://registry.platformio.org/libraries/fmalpartida/LiquidCrystal/examples/HelloWorld_i2c/HelloWorld_i2c.pde
//LiquidCrystal_I2C lcd(0x38); 
LiquidCrystal_I2C lcd(0x27,20,4);

#include "bresenham.h"


#include "U8x8lib.h"

// Set parameters

// Include application, user and local libraries
// !!! Help http://bit.ly/2CL22Qp

// Define structures and classes

// Define variables and constants
uint8_t loopLED;
#define USB_DATENBREITE 64

#define TEST 1

int8_t r;

// USB
volatile uint8_t inbuffer[USB_DATENBREITE] = {};
volatile uint8_t outbuffer[USB_DATENBREITE] = {};
volatile uint16_t usb_recv_counter = 0;
volatile uint16_t cnc_recv_counter = 0;
// end USB

elapsedMillis sinceblink;
elapsedMillis sincelcd;
elapsedMillis sinceusb;

elapsedMillis sinceload; // Zeitdauer der Anzeige des Potentialwertes

elapsedMicros sincelaststep;

elapsedMillis sincelastthread;


elapsedMicros sincelastimpuls;
uint16_t cncdelaycounter = 0;


// Prototypes

uint8_t buffer[USB_DATENBREITE] = {};
static volatile uint8_t sendbuffer[USB_DATENBREITE] = {};

// Ringbuffer
uint8_t CNCDaten[RINGBUFFERTIEFE][USB_DATENBREITE];
uint8_t CDCStringArray[RINGBUFFERTIEFE];

uint16_t abschnittnummer = 0;
uint16_t endposition = 0xFFFF;
uint16_t ladeposition = 0;

// volatile uint16_t          globalaktuelleladeposition = 0;
uint16_t aktuelleladeposition = 0;
volatile uint8_t ringbufferstatus = 0x00;

uint8_t aktuellelage = 0; 

uint16_t Abschnitte = 0;
uint16_t AbschnittCounter = 0;
volatile uint8_t liniencounter = 0;
// end Ringbuffer
volatile uint16_t steps = 0;

volatile uint16_t korrekturintervallx = 0;
volatile uint16_t korrekturintervally = 0;

volatile uint16_t korrekturintervallcounterx = 0;
volatile uint16_t korrekturintervallcountery = 0;

volatile uint16_t korrekturcounterx = 0;
volatile uint16_t korrekturcountery = 0;

volatile uint8_t vorzeichen = 0;

uint8_t motorsteps = 48;
uint8_t micro = 1;

volatile uint16_t loadtime = 0;

volatile uint8_t timer0startwert = TIMER0_STARTWERT;

volatile uint16_t timer2Counter = 0;
volatile uint8_t cncstatus = 0x00;
volatile uint8_t sendstatus = 0x00;

volatile uint8_t usbstatus = 0x00;
static volatile uint8_t motorstatus = 0x00;
static volatile uint8_t anschlagstatus = 0x00;

volatile uint8_t taskstatus = 0x00; // Anzeige running
#define TASK    1
#define RUNNING    2

// bresenham

volatile uint8_t bresenhamstatus = 0x00; // relevanter motor, in Abschnittladen:bres gesetzt

volatile uint16_t bresenham_errAB = 0; // error A,B
volatile uint16_t bresenham_e2AB = 0;  // check A,B

volatile uint16_t bresenham_errCD = 0;
volatile uint16_t bresenham_e2CD = 0;

volatile uint16_t StepStartA = 0; // startwert am Anfang des Abschnittes
volatile uint16_t StepStartC = 0;

// Seite A
volatile int16_t xA, yA, tA, dxA, dyA, incxA, incyA, pdxA, pdyA, ddxA, ddyA, deltaslowdirectionA, deltafastdirectionA, errA;

volatile uint16_t deltafastdelayA = 0; // aktueller delay
volatile uint16_t bres_delayA = 0;     // steps fuer fastdirection
volatile uint16_t bres_counterA = 0;   // zaehler fuer fastdirection

uint16_t stepdurA = 0;
uint16_t stepdurB = 0;
uint16_t stepdurC = 0;
uint16_t stepdurD = 0;

// Seite B
volatile int16_t xB, yB, tB, dxB, dyB, incxB, incyB, pdxB, pdyB, ddxB, ddyB, deltaslowdirectionB, deltafastdirectionB, errB;

volatile uint16_t deltafastdelayB = 0; // aktueller delay
volatile uint16_t bres_delayB = 0;     // steps fuer fastdirection
volatile uint16_t bres_counterB = 0;   // zaehler fuer fastdirection

volatile uint8_t timerstatus = 0;

volatile uint8_t status = 0;

volatile uint8_t pfeilstatus = 0;
volatile uint8_t tastaturstatus = 0;

volatile uint8_t PWM = 0;
static volatile uint8_t pwmposition = 0;
static volatile uint8_t pwmdivider = 0;

// CNC

volatile uint16_t CounterA = 0; // Zaehler fuer Delay von Motor A
volatile uint16_t CounterB = 0; // Zaehler fuer Delay von Motor B
volatile uint16_t CounterC = 0; // Zaehler fuer Delay von Motor C
volatile uint16_t CounterD = 0; // Zaehler fuer Delay von Motor D

volatile uint32_t DelayA = 24; // Delay von Motor A
volatile uint32_t DelayB = 24; // Delay von Motor B
volatile uint32_t DelayC = 24; // Delay von Motor C
volatile uint32_t DelayD = 24; // Delay von Motor D

volatile uint32_t StepCounterA = 0; // Zaehler fuer Schritte von Motor A
volatile uint32_t StepCounterB = 0; // Zaehler fuer Schritte von Motor B
volatile uint32_t StepCounterC = 0; // Zaehler fuer Schritte von Motor C
volatile uint32_t StepCounterD = 0; // Zaehler fuer Schritte von Motor D

volatile uint8_t richtung = 0;
volatile uint8_t homestatus = 0;

volatile uint8_t parallelcounter = 0;
volatile uint8_t parallelstatus = 0; // Status des Thread

volatile uint16_t timerintervall = TIMERINTERVALL;
volatile uint16_t timerintervall_SLOW = 0; // Intervall klein
volatile uint16_t timerintervall_FAST = 0; // Intervall gross

// Ramp

volatile uint16_t ramptimerintervall = TIMERINTERVALL;

volatile uint8_t rampstatus = 0;
// volatile uint8_t           RampZeit = RAMPZEIT;
// volatile uint8_t           RampFaktor = RAMPFAKTOR;
volatile uint32_t rampstepstart = 0; // Stepcounter am Anfang
// volatile uint32_t          ramptimercounter=0;  // laufender counter  fuer Rampanpassung
// volatile uint32_t          //ramptimerdelay = 100;  // Takt fuer Rampanpassung
uint8_t rampschritt = 2;
volatile uint16_t rampbreite = 10; // anzahl Schritte der Ramp. Wird beim Start bestimmt und fuer das Ende verwendet

volatile uint32_t rampendstep = 0; // Beginn der Endramp. Wird in Abschnittladen bestimmt

uint8_t richtungstatus = 0;
uint8_t oldrichtungstatus = 0;
#define AXNEG 0
#define AYNEG 1
#define BXNEG 4
#define BYNEG 5

int16_t lastdax = 0; // letzte Werte fuer schritte x, y. Fuer berechnung gradient
int16_t lastday = 0;

// Create an IntervalTimer object
IntervalTimer delayTimer;


uint16_t errarray[1024];
uint16_t errpos = 0;
// Utilities

// OLED display
canal_struct canaldata;
canal_struct indata;
//file:///Users/ruediheimlicher/Documents/Elektronik/ESP-32/OLED/In-Depth:%20Interface%20OLED%20Display%20Module%20with%20ESP8266%20NodeMCU.webarchive
// Ganssle


// ADC
#define TASTATURPIN  A9

uint16_t tastenwert = 0;
uint8_t Taste = 0;
uint16_t tastaturcounter = 0;
uint16_t prellcounter = 0;


uint8_t tastencounter = 0;
uint8_t analogtastaturstatus = 0;
#define TASTE_OFF  0
#define TASTE_ON  1
uint16_t TastenStatus=0;
uint16_t Tastenprellen=0x1F;
uint8_t oldTaste = 0;
uint8_t            pfeiltastecode = 0;
// IntervalTimer for Tastatur
IntervalTimer tastaturTimer;
IntervalTimer kanalimpulsTimer;
volatile uint16_t tastaturimpulscounter = 0; // PWM fuer Tastaturimpulse
volatile uint8_t tastaturmotorport = 0xFF; // aktiver Port bei Tastendruck
volatile uint8_t tastaturstep = 0xFF; // aktiver Port bei Tastendruck

// von Mill35
volatile uint16_t           pfeilimpulsdauer = 0;
volatile uint16_t           pfeilrampcounter = 0;
volatile uint16_t           pfeilrampdelay = 0;
volatile uint16_t            endimpulsdauer = ENDIMPULSDAUER;
volatile uint16_t            rampimpulsdauer = TASTENSTARTIMPULSDAUER;




uint8_t manright [64] = {128, 37, 0, 0, 20, 0, 0, 0, 128, 37, 0, 0, 20, 0, 0, 0, 194, 3, 0, 0, 0, 1, 1, 48, 240, 48, 1, 0, 0, 0, 0, 17, 3, 0, 128, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};



uint8_t manup [64] = {0, 0, 128, 37, 0, 0, 20, 0, 0, 0, 128, 37, 0, 0, 20, 0, 194, 3, 0, 0, 0, 2, 1, 48, 240, 48, 1, 0, 0, 0, 0, 17, 3, 0, 128, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t manleft [64] = {128, 165, 0, 0, 20, 0, 0, 0, 128, 165, 0, 0, 20, 0, 0, 0, 194, 3, 0, 0, 0, 1, 1, 48, 240, 48, 1, 0, 0, 0, 0, 17, 3, 0, 128, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t mandown [64] = {0, 0, 128, 165, 0, 0, 20, 0, 0, 0, 128, 165, 0, 0, 20, 0, 194, 3, 0, 0, 0, 2, 1, 48, 240, 48, 1, 0, 0, 0, 0, 17, 3, 0, 128, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// Functions

// bresenham
int sgn(int x)
{
   if (x > 0)
      return +1;
   if (x < 0)
      return -1;
   return 0;
}

void OSZIA_LO(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_A, LOW);
}

void OSZIA_HI(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_A, HIGH);
}

void OSZIA_TOGG(void)
{
   if (TEST)
      digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
}

void OSZIB_LO(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_B, LOW);
}

void OSZIB_HI(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_B, HIGH);
}
void OSZIB_TOGG(void)
{
   if (TEST)
      digitalWrite(OSZI_PULS_B, !digitalRead(OSZI_PULS_B));
}

void OSZIC_LO(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_C, LOW);
}

void OSZIC_HI(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_C, HIGH);
}
void OSZI_D_LO(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_D, LOW);
}

void OSZI_D_HI(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_D, HIGH);
}

void startTimer2(void)
{
   timerstatus |= (1 << TIMER_ON);
}

void stopTimer2(void)
{
   timerstatus &= ~(1 << TIMER_ON);
}

void delaytimerfunction(void) // 1us ohne ramp
{
   if (timerstatus & (1 << TIMER_ON))
   {
      // OSZIA_LO(); // 100us

      if (bres_delayA)
      {
         bres_delayA -= 1;
      }

      if (bres_delayB)
      {
         bres_delayB -= 1;
      }
   }
}

void kanalimpulsfunktion(void)
{
   OSZIB_HI();
   digitalWriteFast(tastaturstep,HIGH);
   kanalimpulsTimer.end();
}


void tastaturtimerFunktion() // TASTENSTARTIMPULSDAUER
{
   OSZIB_LO();
   kanalimpulsTimer.begin(kanalimpulsfunktion,IMPULSBREITE); // neuer Kanalimpuls
   digitalWriteFast(tastaturstep,LOW);
   if(rampimpulsdauer > (TASTENENDIMPULSDAUER + RAMPDELAY))
   {
      rampimpulsdauer -= RAMPDELAY;
      tastaturTimer.update(rampimpulsdauer);
   }

}





// MARK: mark AbschnittLaden_bres
uint8_t AbschnittLaden_bres(uint8_t *AbschnittDaten) // 22us
{
   //OSZIA_LO();
   // stopTimer2();
   //  lcd_gotoxy(15,0);
   //  lcd_puts("    ");

   uint8_t returnwert = 0;
   
   /*
    Reihenfolge der Daten:
    0    schritteax lb
    1    schritteax hb
    2    schritteay lb
    3    schritteay hb

    4    delayax lb
    5    delayax hb
    6    delayay lb
    7    delayay hb

    8    schrittebx lb
    9    schrittebx hb
    10    schritteby lb
    11    schritteby hb

    12    delaybx lb
    13    delaybx hb
    14    delayby lb
    15    delayby hb


    16   (8)    code
    17   (9)    position // Beschreibung der Lage im Schnittpolygon:first, last, ...
    18   (10)   indexh     // Nummer des Abschnitts
    19   (11)   indexl

    20     pwm

    21   motorstatus // relevanter Motor fuer Abschnitt

    22   zoomfaktor

    25   steps
    26   micro

    */
   taskstatus |= (1<<TASK);
   taskstatus |= (1<<RUNNING);
   motorsteps = AbschnittDaten[25];

   micro = AbschnittDaten[26];

   uint16_t index = (AbschnittDaten[18] << 8) | AbschnittDaten[19];

   if (AbschnittDaten[35] == 1)
   {
      // // Serial.printf("+++ +++ +++ \t\t\t index: %d AbschnittLaden_bres WENDEPUNKT \n",index);
         //  rampstatus |=(1<<RAMPOKBIT);
   }

   // pwm-rate
   PWM = AbschnittDaten[20];
   // // Serial.printf("AbschnittLaden_bres steps: %d micro: %d PWM: %d\n",steps,micro,PWM);
   // // Serial.printf("AbschnittLaden_bres start \n");
   analogWrite(DC_PWM, PWM);

   // // Serial.printf("AbschnittLaden_bres AbschnittDaten eingang index: %d\n", index);

   /*
   for(uint8_t i=0;i<27;i++) // 5 us ohne printf, 10ms mit printf
   {
      // // Serial.printf("%d \t",AbschnittDaten[i]);
   }
   // // Serial.printf("\n");

 //  // Serial.printf("AbschnittLaden_bres steps: %d micro: %d\n",motorsteps,micro);

   //   lage = AbschnittDaten[9]; // Start: 1, innerhalb: 0, Ende: 2
    */

   int lage = 0;
   lage = AbschnittDaten[17]; // Start: 1, innerhalb: 0, Ende: 2
                              // // Serial.printf("******* *********   AbschnittLaden_bres lage: %d\n",lage);
                              // // Serial.printf("AbschnittLaden_bres lage: %d\n",lage);
   if (lage & 0x01)           // erstes Element
   {
      returnwert = 1;
   }
   if (lage & 0x02) // letztes Element
   {
      returnwert = 2;
   }

   richtung = 0;

   // Motor A
   digitalWriteFast(MA_EN, LOW); // Pololu ON

   uint8_t dataL = 0;
   uint8_t dataH = 0;

   uint8_t delayL = 0;
   uint8_t delayH = 0;

   dataL = AbschnittDaten[0];
   dataH = AbschnittDaten[1];

   // lcd_gotoxy(17,0);
   int8_t vz = 1;
   if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
   {
      richtung |= (1 << RICHTUNG_A); // Rueckwarts
      digitalWriteFast(MA_RI, LOW);  // PIN fuer Treiber stellen
      vz = -1;
      // lcd_putc('r');
   }
   else
   {
      richtung &= ~(1 << RICHTUNG_A);
      digitalWriteFast(MA_RI, HIGH);
      // lcd_putc('v');   // Vorwaerts
   }

   dataH &= (0x7F);                     // bit 8 entfernen
   StepCounterA = dataL | (dataH << 8); //

   // int16_t newdax =  StepCounterA * vz;
   StepCounterA *= micro;
   StepStartA = StepCounterA;

   delayL = AbschnittDaten[4];
   delayH = AbschnittDaten[5];

   DelayA = delayL | (delayH << 8);

   CounterA = DelayA;

   // Motor B
   // CounterB=0;
   digitalWriteFast(MB_EN, LOW); // Pololu ON
   dataL = AbschnittDaten[2];
   dataH = AbschnittDaten[3];
   // lcd_gotoxy(19,1);
   vz = 1;
   if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
   {
      richtung |= (1 << RICHTUNG_B); // Rueckwarts
      digitalWriteFast(MB_RI, LOW);  // lcd_putc('r');
      vz = -1;
   }
   else
   {
      richtung &= ~(1 << RICHTUNG_B);
      digitalWriteFast(MB_RI, HIGH);
   }

   dataH &= (0x7F);

   StepCounterB = dataL | (dataH << 8);
   // int16_t newday = StepCounterB * vz;

   StepCounterB *= micro;

   DelayB = (AbschnittDaten[7] << 8) | AbschnittDaten[6];

   // // Serial.printf("\nAbschnittLaden_bres index: %d StepCounterA  : %d DelayA: %d StepCounterB: %d DelayB: %d\n",index,StepCounterA, DelayA, StepCounterB, DelayB);

   CounterB = DelayB;

   // Motor C
   digitalWriteFast(MC_EN, LOW); // Pololu ON
   // CounterC=0;
   dataL = 0;
   dataH = 0;

   delayL = 0;
   delayH = 0;

   dataL = AbschnittDaten[8];
   dataH = AbschnittDaten[9];
   // // Serial.printf("AbschnittLaden_bres C datah: %d\n",dataH);
   // richtung=0;
   if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
   {
      richtung |= (1 << RICHTUNG_C); // Rueckwarts
      digitalWriteFast(MC_RI, LOW);
      // // Serial.printf("AbschnittLaden_bres C negativ\n");
   }
   else
   {
      richtung &= ~(1 << RICHTUNG_C);
      digitalWriteFast(MA_RI, HIGH);
      // // Serial.printf("AbschnittLaden_bres C positiv\n");
   }

   dataH &= (0x7F);
   //   StepCounterC = dataH;      // HByte
   //   StepCounterC <<= 8;      // shift 8
   //   StepCounterC += dataL;   // +LByte

   StepCounterC = dataL | (dataH << 8);
   StepCounterC *= micro;

   StepStartC = StepCounterC;
   delayL = AbschnittDaten[12];
   delayH = AbschnittDaten[13];

   //   DelayC = delayH;
   //   DelayC <<=8;
   //   DelayC += delayL;

   DelayC = delayL | (delayH << 8);

   CounterC = DelayC;

   // Motor D
   //digitalWriteFast(MD_EN, LOW); // CounterD=0;
   dataL = 0;
   dataH = 0;

   delayL = 0;
   delayH = 0;

   dataL = AbschnittDaten[10];
   dataH = AbschnittDaten[11];
   // // Serial.printf("AbschnittLaden_bres D datah: %d\n",dataH);
   if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
   {
      richtung |= (1 << RICHTUNG_D); // Rueckwarts
      //digitalWriteFast(MD_RI, LOW);
      // // Serial.printf("AbschnittLaden_bres D negativ\n");
      // lcd_putc('r');
   }
   else
   {
      richtung &= ~(1 << RICHTUNG_D);
      //digitalWriteFast(MD_RI, HIGH);
      // // Serial.printf("AbschnittLaden_bres D positiv\n");
   }

   dataH &= (0x7F);

   StepCounterD = dataL | (dataH << 8);
   StepCounterD *= micro;

   delayL = AbschnittDaten[14];
   delayH = AbschnittDaten[15];

   DelayD = delayL | (delayH << 8);

   // // Serial.printf("AbschnittLaden_bres StepCounterD: %d DelayD: %d\n",StepCounterD,DelayD);
   CounterD = DelayD;

   //  ****
   //  Bresenham
   //  ***
   // // Serial.printf("AbschnittLaden_bres vor bresenham: StepCounterA: %d StepCounterB: %d\n",StepCounterA,StepCounterB);
   deltafastdirectionA = 0;
   deltaslowdirectionA = 0;
   deltafastdirectionB = 0;
   deltaslowdirectionB = 0;
   deltafastdelayA = 0;
   deltafastdelayB = 0;

   // bresenham Seite A

   // relevanten Motor setzen
   if (StepCounterA > StepCounterB)
   {
      // incx = 1
      pdxA = 1; // /pd. ist Parallelschritt, nur eine Richtung, x ist Hauptrichtung
      pdyA = 0; //
      ddxA = 1; // dd. ist Diagonalschritt , beide Richtungen
      ddyA = 1;
      deltaslowdirectionA = StepCounterB;
      deltafastdirectionA = StepCounterA;
      deltafastdelayA = DelayA;
      //     // Serial.printf("AbschnittLaden_bres  A > B\n");
   }
   else
   {
      //
      pdxA = 0; // pd. ist Parallelschritt, nur eine Richtung. y ist Hauptrichtung
      pdyA = 1;

      ddxA = 1; // dd. ist Diagonalschritt , beide Richtungen
      ddyA = 1;
      deltaslowdirectionA = StepCounterA;
      deltafastdirectionA = StepCounterB;
      deltafastdelayA = DelayB;
      //     // Serial.printf("AbschnittLaden_bres  A < B\n");
   }
   // aktuelle Werte einsetzen
   bres_delayA = deltafastdelayA;       // aktueller delay in fastdir
   bres_counterA = deltafastdirectionA; // aktueller counter fuer steps

   if (rampstatus & (1 << RAMPOKBIT))
   {
      //// Serial.printf("AbschnittLaden_bres index: %d set RAMPSTARTBIT\n", index);
      rampstatus |= (1 << RAMPSTARTBIT); // Ramp an Start
      errpos = 0;
      ramptimerintervall += (ramptimerintervall / 4 * 3);
      delayTimer.update(ramptimerintervall);
   }

   xA = StepCounterA; //
   yA = StepCounterB;

   errA = deltafastdirectionA / 2;

   // // Serial.printf("AbschnittLaden_bres deltafastdirectionA: %d deltaslowdirectionA: %d  deltafastdelayA: %d errA: %d bres_counterA: %d bres_delayA: %d\n",deltafastdirectionA,deltaslowdirectionA, deltafastdelayA,errA,bres_counterA,bres_delayA);

   // bresenham Seite B

   // relevanten Motor setzen
   if (StepCounterC > StepCounterD)
   {
      //
      pdxB = 1;
      pdyB = 0;
      ddxB = 1;
      ddyB = 1;
      deltaslowdirectionB = StepCounterD;
      deltafastdirectionB = StepCounterC;
      deltafastdelayB = DelayC;
      // // Serial.printf("AbschnittLaden_bres  C > D\n");
   }
   else
   {
      //
      pdxB = 0;
      pdyB = 1;
      ddxB = 1;
      ddyB = 1;
      deltaslowdirectionB = StepCounterC;
      deltafastdirectionB = StepCounterD;
      deltafastdelayB = DelayD;
      // // Serial.printf("AbschnittLaden_bres  C < D\n");
   }
   // aktuelle Werte einsetzen
   bres_delayB = deltafastdelayB;       // aktueller delay in fastdir
   bres_counterB = deltafastdirectionB; // aktueller counter fuer steps

   xB = StepCounterC; //
   yB = StepCounterD;

   errB = deltafastdirectionB / 2;

   {

      timerintervall_FAST = TIMERINTERVALL;
      //  OSZIB_LO();
   }

   // motorstatus: welcher Motor ist relevant
   motorstatus = AbschnittDaten[21];

   // richtung change
   ////#pragma mark Richtung change

   // rampstatus |=(1<<RAMPOKBIT);

   // startTimer2();

   // // Serial.printf("\nAbschnittLaden_bres end aktuellelage: %d \n",returnwert);
   //OSZIA_HI();
   tastaturstatus = 0 ;
   return returnwert;
}

void AnschlagVonMotor(const uint8_t motor)
{
   // return;
   // lcd_gotoxy(0,1);
   // lcd_putc('A');
   // lcd_gotoxy(2+2*motor,1);
   // lcd_puthex(motor);
   // // Serial.printf("*** AnschlagvonMotor: %d\n",motor);

   uint8_t endPin = END_A0_PIN;
   uint8_t endBit = 0;
   switch (motor)
   {
   case 0:
   {
      endPin = END_A0_PIN;
      endBit = motor;
   }
   break;
   case 1:
   {
      endPin = END_B0_PIN;
      endBit = motor;
   }
   break;
   case 2:
   {
      endPin = END_C0_PIN;
      endBit = motor;
   }
   break;
   case 3:
   {
      // endPin = END_D0;
   }
   break;

   } // switch motor

   if (digitalRead(endPin)) // Eingang ist HI, Schlitten nicht am Anschlag A0
   {
      // if (anschlagstatus &(1<< endPin)) // Schlitten war, aber ist nicht mehr am Anschlag
      if (anschlagstatus & (1 << motor)) // Schlitten war, aber ist nicht mehr am Anschlag

      {
         anschlagstatus &= ~(1 << motor); // Bit fuer Anschlag A0 zuruecksetzen
      }
   }
   else // Schlitten bewegte sich auf Anschlag zu und ist am Anschlag A0
   {
      //   AnschlagVonMotor(0); // Bewegung anhalten
      // Strom OFF
      analogWrite(DC_PWM, 0);
      // if (richtung & (1<<(RICHTUNG_A + motor))) // Richtung ist auf Anschlag A0 zu
      if (richtung & (1 << (RICHTUNG_A + motor))) // Richtung ist auf Anschlag A0 zu   (RICHTUNG_A ist 0)

      {
         if (!(anschlagstatus & (1 << (END_A0 + motor))))
         {
            // cli();
            // Serial.printf("\t*** Motor %d ist am anschlag angekommen\n", motor);
            anschlagstatus |= (1 << (END_A0 + motor)); // Bit fuer Anschlag A0+motor setzen (END_A0 ist 4)

            if (cncstatus & (1 << GO_HOME)) // nur eigene Seite abstellen
            {
               // Paralleler Schlitten gleichzeitig am Anschlag?

               // Serial.printf("*** Anschlag Home motor %d\n", motor);
               // lcd_putc('B');
               //  code:
               sendbuffer[0] = 0xB5 + motor;

               if (motor < 2) // Stepperport 1
               {
                  // Serial.printf("Stepperport 1\n");
                  // STEPPERPORT_1 |= (1<<(MA_EN + motor));     // Motor 0,1 OFF
                  digitalWriteFast(MA_EN, HIGH);
                  digitalWriteFast(MB_EN, HIGH);

                  // STEPPERPORT_2 |= (1<<(MA_EN + motor + 2)); // Paralleler Motor 2,3 OFF
                  //              StepCounterA=0;
                  //              StepCounterB=0;
                  xA = 0;
                  yA = 0;
                  xB = 0;
                  yB = 0;

                  bres_counterA = 0;
                  bres_delayA = 0;

                  bres_counterB = 0;
                  bres_delayB = 0;
                  deltafastdirectionA = 0;
                  deltafastdelayA = 0;
                  deltafastdirectionB = 0;
                  deltafastdelayB = 0;

                  //               CounterA=0xFFFF;
                  //              CounterB=0xFFFF;
               }
               else // Stepperport 2
               {
                  // Serial.printf("Stepperport 2\n");
                  //    STEPPERPORT_2 |= (1<<(MA_EN + motor));     // Motor 2,3 OFF
                  //digitalWriteFast(MC_EN, HIGH);
                  //digitalWriteFast(MD_EN, HIGH); // Paralleler Motor 0,1 OFF
                  StepCounterC = 0;
                  StepCounterD = 0;
                  xA = 0;
                  yA = 0;
                  xB = 0;
                  yB = 0;

                  bres_counterA = 0;
                  bres_delayA = 0;

                  bres_counterB = 0;
                  bres_delayB = 0;
                  deltafastdirectionB = 0;
                  deltafastdelayB = 0;

                  //               CounterC=0xFFFF;
                  //               CounterD=0xFFFF;
               }
               // cncstatus &= ~(1<<GO_HOME);
               /*
               digitalWriteFast(MA_EN,HIGH);
               digitalWriteFast(MB_EN,HIGH);
               digitalWriteFast(MC_EN,HIGH);
               digitalWriteFast(MD_EN,HIGH);
                */

            }    // home
            else // beide Seiten abstellen
            {
               // Serial.printf("CCC\n");
               cncstatus = 0;
               sendbuffer[0] = 0xA5 + motor;

               if (motor < 2) // Stepperport 1
               {
                  // STEPPERPORT_1 |= (1<<(MA_EN + motor));     // Motor 0,1 OFF
                  digitalWriteFast(MA_EN, HIGH);
                  // STEPPERPORT_2 |= (1<<(MA_EN + motor + 2)); // Paralleler Motor 2,3 OFF
                  digitalWriteFast(MA_EN + 2, HIGH);
               }
               else // Stepperport 2
               {
                  // STEPPERPORT_2 |= (1<<(MA_EN + motor));     // Motor 2,3 OFF
                  digitalWriteFast(MA_EN + motor, HIGH);
                  // STEPPERPORT_1 |= (1<<(MA_EN + motor - 2)); // Paralleler Motor 0,1 OFF
                  digitalWriteFast(MA_EN + motor + 4, HIGH);
               }

               // Alles abstellen
               StepCounterA = 0;
               StepCounterB = 0;
               StepCounterC = 0;
               StepCounterD = 0;

               // xA = 0;
               // yA = 0;
               bres_counterA = 0;
               bres_delayA = 0;

               bres_counterB = 0;
               bres_delayB = 0;

               /*
                CounterA=0xFFFF;
                CounterB=0xFFFF;
                CounterC=0xFFFF;
                CounterD=0xFFFF;
                */

               ladeposition = 0;
               motorstatus = 0;

               digitalWriteFast(MA_EN, HIGH);
               digitalWriteFast(MB_EN, HIGH);
               digitalWriteFast(MC_EN, HIGH);
               //digitalWriteFast(MD_EN, HIGH);
            }

            sendbuffer[5] = (abschnittnummer & 0xFF00) >> 8;
            ;
            sendbuffer[6] = abschnittnummer & 0x00FF;

            sendbuffer[7] = (ladeposition & 0xFF00) >> 8;
            sendbuffer[8] = ladeposition & 0x00FF;

            //
            sendbuffer[22] = cncstatus;

            // Serial.printf("*** Anschlag Home motor %d code: %d cncstatus: %d\n", motor, sendbuffer[0], cncstatus);

            // Serial.printf("E\n");
            uint8_t senderfolg = usb_rawhid_send((void *)sendbuffer, 10);
             // Serial.printf("*** Anschlag Home motor senderfolg: %d\n",senderfolg);
            for (uint8_t i = 0; i < 32; i++) // 5 us ohne printf, 10ms mit printf
            {
               // // Serial.printf("%d \t",sendbuffer[i]);
            }
            // // Serial.printf("\n");

            richtung &= ~(1 << (RICHTUNG_A + motor)); // Richtung umschalten

            interrupts();
         } // NOT END_A0
         else
         {
         }
      }
      else
      {
         if (!(anschlagstatus & (1 << (END_A0 + motor))))
         {
            anschlagstatus &= ~(1 << (END_A0 + motor)); // Bit fuer Anschlag B0 zuruecksetzen
         }
      }
      // // Serial.printf("End Anschlag AbschnittCounter %d\n",AbschnittCounter);
   }
}

void setPixel(int px, int py)
{
   // Serial.printf("px: %d py: %d\n", px, py);
}

void plot_line(int x0, int y0, int x1, int y1)
{
   int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
   int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
   int err = dx + dy, e2; /* error value e_xy */

   for (;;)
   { /* loop */
      setPixel(x0, y0);
      if (x0 == x1 && y0 == y1)
         break;
      e2 = 2 * err;
      if (e2 >= dy)
      {
         err += dy;
         x0 += sx;
      } /* e_xy+e_x > 0 */
      if (e2 <= dx)
      {
         err += dx;
         y0 += sy;
      } /* e_xy+e_y < 0 */
   }
}

// gpio_MCP23S17     mcp0(10,0x20);//instance 0 (address A0,A1,A2 tied to 0)
// LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x20
// delay(1000);
//  Add setup code

void thread_func(int inc)
{
   /*
   // lcd.setCursor(0,0);
   // lcd.print("A: ");

   // lcd.setCursor(0,1);
   // lcd.print("A:");
   // lcd.setCursor(6,1);
   // lcd.print("B:");

   // lcd.setCursor(5,0);
   // lcd.print("PWM:");
*/
/*
   while (1)
   {
      if (parallelstatus & (1 << THREAD_COUNT_BIT))
      {
         parallelcounter += 1;

         // lcd.setCursor(0,1);
         // String s = "D";
         // lcd.setCursor(13,0);

         uint16_t rest = Abschnitte - AbschnittCounter;
         // lcd.print(String(rest));

         //  s.append(rest);
         
         s.append("n ");

         s.append(StepCounterA);
         s.append("\n");

         // lcd.print(s);
        
         
         //// lcd.print(String(parallelcounter));
         // lcd.setCursor(9,0);
         // lcd.print(String(PWM));
         // lcd.setCursor(2,1);
         // lcd.print(String(StepCounterA));
         // lcd.setCursor(8,1);
         // lcd.print(String(StepCounterB));
          
         parallelstatus &= ~(1 << THREAD_COUNT_BIT);
      }
   }
   */
   /*
   if (sincelastthread >= 500)
   {
      sincelastthread = 0;
      parallelcounter += 2;
 //     // lcd.setCursor(12,0);
  //    // lcd.print(String(parallelcounter));
   }
   */
}

ADC *adc = new ADC(); // adc object;

uint8_t Tastenwahl(uint16_t Tastaturwert)
{
   
   if (Tastaturwert < TASTE1)
      return 1;
   if (Tastaturwert < TASTE2)
      return 2;
   if (Tastaturwert < TASTE3)
      return 3;
   if (Tastaturwert < TASTE4)
      return 4;
   if (Tastaturwert < TASTE5)
      return 5;
   if (Tastaturwert < TASTE6)
      return 6;
   if (Tastaturwert < TASTE7)
      return 7;
   if (Tastaturwert < TASTE8)
      return 8;
   if (Tastaturwert < TASTE9)
      return 9;
   if (Tastaturwert < TASTEL)
      return 10;
   if (Tastaturwert < TASTE0)
      return 0;
   if (Tastaturwert < TASTER)
      return 12;
   
   return 0;
}
 // tastenwahl

 // von Mill32






uint16_t readTastatur(void)
{
   uint16_t adctastenwert = adc->adc0->analogRead(TASTATURPIN);
   if (adctastenwert > 10)
   {
      //// Serial.printf("readTastatur adctastenwert: %d\n",adctastenwert);
      return adctastenwert;
   }
   return 0;
}


void tastenfunktion(uint16_t Tastenwert)
{
   
   if (Tastenwert>23) // ca Minimalwert der Matrix
   {
      //         wdt_reset();
      
        tastaturcounter++;
        
      if (tastaturcounter>=40)   //   Prellen
      {
         
         tastaturcounter=0x00;
         if (analogtastaturstatus & (1<<TASTE_ON)) // Taste schon gedrueckt
         {
            //();
         }
         else // Taste neu gedrückt
         {
            OSZIA_LO();
            analogtastaturstatus |= (1<<TASTE_ON); // nur einmal
            
            Taste=Tastenwahl(Tastenwert);
            // Serial.printf("Tastenwert: %d Taste: %d \n",Taste,Tastenwert);
            tastaturcounter=0;
            Tastenwert=0x00;
            
            
            
            switch (Taste)
            {
               case 0://
               { 
                  // Serial.printf("Taste 0\n");
                  break;
                  // Blinken auf C2
                  
               }
                  break;
                  
                  
               case 1: 
               {
                  // Serial.printf("Taste 1\n");
                  //motor_C.setTargetAbs(800);
                  digitalWriteFast(MC_EN,LOW);
                  //controller.move(motor_C);
                  
               }
                  break;
                  
               case 2:     // up                             //  
               {
                  //uint8_t lage = AbschnittLaden_TS(pfeil4);
                  if (pfeiltastecode == 0)
                  {
                     OSZIB_LO();
                     pfeiltastecode = 2;
                     pfeilimpulsdauer = TASTENSTARTIMPULSDAUER;
                     endimpulsdauer = TASTENENDIMPULSDAUER;
                     
                     tastaturstep = MB_STEP;

                     digitalWriteFast(MB_EN,LOW);
                     digitalWriteFast(MB_RI,HIGH);

                     //digitalWriteFast(MB_STEP,LOW);



                  }
               }break;
                  
               case 3:   //
               {
                  //uint8_t lage = AbschnittLaden_TS(drillup);
                  if (pfeiltastecode == 0)
                  {
                     pfeiltastecode = 22;
                     pfeilimpulsdauer = TASTENSTARTIMPULSDAUER;
                     endimpulsdauer = TASTENENDIMPULSDAUER;
                  }
               }break;
                  
               case 4:   // left
               {
                  //uint8_t lage = AbschnittLaden_TS(pfeil3);
                  // Serial.printf("Tastatur  left\n");
                  if (pfeiltastecode == 0)
                  {
                     pfeiltastecode = 3;
                     pfeilimpulsdauer = TASTENSTARTIMPULSDAUER+20;
                     pfeilrampcounter = 0;
                     endimpulsdauer = TASTENENDIMPULSDAUER;
                     tastaturstep = MA_STEP;

                     digitalWriteFast(MA_EN,LOW);
                     digitalWriteFast(MA_RI,LOW);
                     
                     //digitalWriteFast(MA_STEP,LOW);

                  }
                } break; // case Vortag
                  
                  
               case 5:                        // Ebene tiefer
               {
                  // Serial.printf("Taste 5\n");
                  
                  
                 }            break;
                  
               case 6: // right
               {
                  //uint8_t lage = AbschnittLaden_TS(pfeil1);
                  if (pfeiltastecode == 0)
                  {
                     
                     pfeiltastecode = 1;
                     pfeilimpulsdauer = TASTENSTARTIMPULSDAUER;
                     endimpulsdauer = TASTENENDIMPULSDAUER;
                     tastaturstep = MA_STEP;

                     digitalWriteFast(MA_RI,HIGH);
                     digitalWriteFast(MA_EN,LOW);

                     //digitalWriteFast(MA_STEP,LOW);
                  }
                   
               } break; // case Folgetag
                  
                  /*
               case 7:
               {
                  // Serial.printf("\nTaste 7 \n");
                  uint32_t pos_A = motor_A.getPosition();
                  uint32_t pos_B = motor_B.getPosition();
                  // Serial.printf("pos A: %d pos B: %d\n",pos_A, pos_B);
                  
                  //motor_A.setTargetRel(-pos_A);
                  //motor_B.setTargetRel(-pos_B);
                  motor_A.setTargetAbs(0);
                  motor_B.setTargetAbs(0);
                  
                  digitalWriteFast(MA_EN,LOW);
                  digitalWriteFast(MB_EN,LOW);
                  controller.move(motor_A, motor_B);
                  
               }
                  break;
                  */
                  
               case 8:    // down                                //Menu rueckwaertsschalten
               {
                  //uint8_t lage = AbschnittLaden_TS(pfeil2);
                  if (pfeiltastecode == 0)
                  {
                     pfeiltastecode = 4;
                     pfeilimpulsdauer = TASTENSTARTIMPULSDAUER;
                     endimpulsdauer = TASTENENDIMPULSDAUER;

                     tastaturstep = MB_STEP;

                     digitalWriteFast(MB_EN,LOW);
                     digitalWriteFast(MB_RI,LOW);

                     //digitalWriteFast(MB_STEP,LOW);
                  }
                  
                    
               }
                  break;
                  
               case 9:
               {
                  //uint8_t lage = AbschnittLaden_TS(drilldown);
                  if (pfeiltastecode == 0)
                  {
                     pfeiltastecode = 24;
                     pfeilimpulsdauer = TASTENSTARTIMPULSDAUER;
                     endimpulsdauer = TASTENENDIMPULSDAUER;
                  }
                  
               }
                  
                  break;
                  /*
               case 10: // set Nullpunkt
               {
                  // Serial.printf("Taste 10\n");
                  break;
                  uint32_t pos_A = motor_A.getPosition();
                  
                  uint32_t pos_B = motor_B.getPosition();
                  // Serial.printf("vor reset: pos A: %d pos B: %d\n",pos_A, pos_B);
                  
                  motor_A.setPosition(0);
                  motor_B.setPosition(0);
                  
                  pos_A = motor_A.getPosition();
                  pos_B = motor_B.getPosition();
                  // Serial.printf("nach reset: pos A: %d pos B: %d\n",pos_A, pos_B);
                  
                  //motor_C.setTargetAbs(0);
                  //digitalWriteFast(MC_EN,LOW);
                  //controller.move(motor_C);
                  
               }break;
               */
              /*
               case 12:// 
               {
                  //STOP
                  
                  stopTask(0);
                  
                  //Taste=99;
                  
                  //lcd_clr_line(1);
                  //lcd_gotoxy(0,1);
                  //lcd_puts("Taste 12\0");
                  //delay_ms(100);
                  //lcd_clr_line(1);
                  switch (Menu_Ebene & 0xF0)
                  {
                     case 0x00:
                     {
                        
                     }break;
                        
                     case 0x10:
                     {
                        Menu_Ebene = 0x00;                     //Ebene 0
                        
                     }break;
                     case 0x20:
                     {
                        Menu_Ebene = 0x10;                     //   Ebene 1
                        Menu_Ebene &= 0xF0;                     //   Bits 7 - 4 behalten, Bits 3-0 loeschen
                        
                     }break;
                        
                        
                  }//switch MenuEbene
                  
                  
               }break;
                */  
                  
            }//switch Taste
            OSZIB_HI();
            // Tastaturtimer starten
            if (pfeiltastecode > 0)
            {
                //OSZIA_HI();
               tastaturimpulscounter = 0;
               tastaturTimer.begin(tastaturtimerFunktion,TASTENSTARTIMPULSDAUER);
               rampimpulsdauer = TASTENSTARTIMPULSDAUER;

            }
         }
         OSZIA_HI(); 
         
      }
     
   }
   else 
   {
      if (analogtastaturstatus & (1<<TASTE_ON))
      {
         //
         pfeiltastecode = 0;
         endimpulsdauer = ENDIMPULSDAUER;
         //A0_ISR();  
            digitalWriteFast(MA_EN,HIGH);
            digitalWriteFast(MB_EN,HIGH);
            digitalWriteFast(MC_EN,HIGH);
            digitalWriteFast(MA_STEP,HIGH);
            digitalWriteFast(MB_STEP,HIGH);
            digitalWriteFast(MC_STEP,HIGH);

    
         analogtastaturstatus &= ~(1<<TASTE_ON);
         
      }
   }
   
}



void stopCNC(void)
{

   // Serial.printf("F1 reset\n");
   uint8_t i = 0, k = 0;
   for (k = 0; k < RINGBUFFERTIEFE; k++)
   {
      for (i = 0; i < USB_DATENBREITE; i++)
      {
         CNCDaten[k][i] = 0;
      }
   }

   ringbufferstatus = 0;
   motorstatus = 0;
   anschlagstatus = 0;
   tastaturstatus = 0xF0 ;
   cncstatus = 0;
   ladeposition = 0;
   endposition = 0xFFFF;
   abschnittnummer = 0;
   rampstatus = 0;

   bres_counterA = 0;
   bres_counterB = 0;
   bres_delayA = 0; 
   bres_delayB = 0; 

   StepCounterA = 0;
   StepCounterB = 0;


   AbschnittCounter = 0;
   PWM = 0;
   // digitalWriteFast(DC_PWM,LOW);
   analogWrite(DC_PWM, 0);
   korrekturcounterx = 0;
   korrekturcountery = 0;

   xA = 0;
   yA = 0;

   xB = 0;
   yB = 0;

   digitalWriteFast(MA_EN, HIGH);
   digitalWriteFast(MB_EN, HIGH);

   digitalWriteFast(MA_EN, HIGH);
   digitalWriteFast(MB_EN, HIGH);
   
   digitalWriteFast(MA_STEP, HIGH);
   digitalWriteFast(MB_STEP, HIGH);
   digitalWriteFast(MC_STEP, HIGH);
   //digitalWriteFast(MD_STEP, HIGH);

   for (k=0;k<RINGBUFFERTIEFE;k++)
   {
      for(i=0;i<USB_DATENBREITE;i++)
      {
         CNCDaten[k][i]=0;  
      }
   }
    
   pfeiltastecode = 0;
   endimpulsdauer = ENDIMPULSDAUER;
   
   ladeposition=0;
   endposition=0xFFFF;
   
   AbschnittCounter=0;
   PWM = 0;
   
     
   sendbuffer[0] = 0xF2;
   usb_rawhid_send((void*)sendbuffer, 0);
   sendbuffer[0] = 0x00;
   // Serial.printf("F1 reset end\n");
}

void stopTask(uint8_t emergency) // reset
{
   // Serial.printf("stoptask\n");
   ringbufferstatus = 0;
   motorstatus=0;
   anschlagstatus = 0;
   cncstatus = 0;
   sendbuffer[0]=0xE1;
   
   sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
   sendbuffer[6]=abschnittnummer & 0x00FF;
   
   sendbuffer[8]=ladeposition & 0x00FF;
   sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
  // RawHID.send(sendbuffer, 50);
  uint8_t senderfolg = usb_rawhid_send((void *)sendbuffer, 10);

   sendbuffer[0]=0x00;
   sendbuffer[5]=0x00;
   sendbuffer[6]=0x00;
   sendbuffer[8]=0x00;
   
   
   uint8_t i=0, k=0;
   for (k=0;k<RINGBUFFERTIEFE;k++)
   {
      for(i=0;i<USB_DATENBREITE;i++)
      {
         CNCDaten[k][i]=0;  
      }
   }
    
   pfeiltastecode = 0;
   endimpulsdauer = ENDIMPULSDAUER;
   
   ladeposition=0;
   endposition=0xFFFF;
   
   AbschnittCounter=0;
   PWM = 0;
   
     
   taskstatus &= ~(1<<TASK);
   
   digitalWriteFast(MA_EN,HIGH);
   digitalWriteFast(MB_EN,HIGH);
   digitalWriteFast(MC_EN,HIGH);
   
   digitalWriteFast(MA_STEP,HIGH);
   digitalWriteFast(MB_STEP,HIGH);
   digitalWriteFast(MC_STEP,HIGH);
   
   
}

void setup()
{
   Serial.begin(115200);
   pinMode(LOOPLED, OUTPUT);

// https://registry.platformio.org/libraries/pedvide/Teensy_ADC/examples/analogRead/analogRead.ino
   pinMode(TASTATURPIN , INPUT);
   adc->adc0->setAveraging(4); // set number of averages
   adc->adc0->setResolution(8);
   adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
   adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);
   pinMode(DC_PWM, OUTPUT);
   // digitalWriteFast(DC_PWM, HIGH); // OFF
   digitalWriteFast(DC_PWM, 1);

   //   pinMode(STROM, OUTPUT);
   //   digitalWriteFast(STROM, LOW); // LO, OFF

   // init Pins
   // Stepper A
   pinMode(MA_STEP, OUTPUT); //
   pinMode(MA_RI, OUTPUT);   //
   pinMode(MA_EN, OUTPUT);   //

   digitalWriteFast(MA_STEP, HIGH); // HI
   digitalWriteFast(MA_RI, HIGH);   // HI
   digitalWriteFast(MA_EN, HIGH);   // HI

   // Stepper B
   pinMode(MB_STEP, OUTPUT); // HI
   pinMode(MB_RI, OUTPUT);   // HI
   pinMode(MB_EN, OUTPUT);   // HI

   digitalWriteFast(MB_STEP, HIGH); // HI
   digitalWriteFast(MB_RI, HIGH);   // HI
   digitalWriteFast(MB_EN, HIGH);   // HI

   // Stepper C
   pinMode(MC_STEP, OUTPUT); // HI
   pinMode(MC_RI, OUTPUT);   // HI
   pinMode(MC_EN, OUTPUT);   // HI

   digitalWriteFast(MC_STEP, HIGH); // HI
   digitalWriteFast(MC_RI, HIGH);   // HI
   digitalWriteFast(MC_EN, HIGH);   // HI

   // Stepper D

   //pinMode(MD_STEP, OUTPUT); // HI
   //pinMode(MD_RI, OUTPUT);   // HI
   //pinMode(MD_EN, OUTPUT);   // HI

   //digitalWriteFast(MD_STEP, HIGH); // HI
   //digitalWriteFast(MD_RI, HIGH);   // HI
   //digitalWriteFast(MD_EN, HIGH);   // HI

   pinMode(END_A0_PIN, INPUT); //
   pinMode(END_B0_PIN, INPUT); //
   pinMode(END_C0_PIN, INPUT); //
   //pinMode(END_D0_PIN, INPUT); //

   pinMode(END_A0_PIN, INPUT_PULLUP); // HI
   pinMode(END_B0_PIN, INPUT_PULLUP); //
   pinMode(END_C0_PIN, INPUT_PULLUP); //
   //pinMode(END_D0_PIN, INPUT_PULLUP); //

   if (TEST)
   {
      pinMode(OSZI_PULS_A, OUTPUT);
      digitalWriteFast(OSZI_PULS_A, HIGH);

      pinMode(OSZI_PULS_B, OUTPUT);
      digitalWriteFast(OSZI_PULS_B, HIGH);
      pinMode(OSZI_PULS_C, OUTPUT);
      digitalWriteFast(OSZI_PULS_C, HIGH);
      //pinMode(OSZI_PULS_D, OUTPUT);
      //digitalWriteFast(OSZI_PULS_D, HIGH);
   }


  // initialize the OLED object

  // https://forum.pjrc.com/index.php?threads/teensy-3-2-ssd1306-hardware-spi.70260/
  //if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) 
   
   
  

/*
// Waveshare OLED
 if ( ! OLED_display.begin(0x3D) ) {
     Serial.println("Unable to initialize OLED");
     while (1) yield();
  }
 OLED_display.clearDisplay();
  OLED_display.display();



// draw rectangles
  testdrawrect();
  OLED_display.display();
  delay(1000);
  OLED_display.clearDisplay();
*/
/*
lcd.begin(20,4);               // initialize the lcd 
   delay(100);
   // lcd.init();
   delay(100);
   // lcd.backlight();
     lcd.home ();                   // go home
  lcd.print("Hello, ARDUINO ");  
*/

  // rampstatus |= (1 << RAMPOKBIT);

   //// lcd.setCursor(0,0);
   //// lcd.print("hallo");
   delayTimer.begin(delaytimerfunction, timerintervall);
   delayTimer.priority(0);

   //   threads.addThread(thread_func, 1);

   // lcd.setCursor(0,0);
   // lcd.print("CNC");
   //   // lcd.setCursor(0,1);
   //   // lcd.print("PWM:");
   /*
   // lcd.setCursor(0,1);
   // lcd.print("A:");
   // lcd.setCursor(6,1);
   // lcd.print("B:");

   // lcd.setCursor(5,0);
   // lcd.print("PWM:");
*/

 
/*
Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(BUILTIN_SDCARD)) 
  {
    Serial.println("Card failed, or not present");
    while (1) {
      // No SD card, so don't do anything more - stay stuck here
    }
  }
  Serial.println("card initialized.");

   Serial.println("CNC_Mill_Slave_32_bres\n");
  */
 //tastaturstatus = 0xF0;

  // von Mill32
 pfeilimpulsdauer = TASTENSTARTIMPULSDAUER;
 taskstatus = 0;
}

// Add loop code
void loop()
{
   //   Serial.println(steps);
   //   threads.delay(1000);
//tastaturimpulscounter++;
// von Mill35


   if (sinceblink > 1000)
   {
      //OSZIA_LO();

      // PWM = 55;
      // analogWrite(DC_PWM, PWM);
      // scanI2C(100000);
      loopLED++;
      sinceblink = 0;

      //      // lcd.setCursor(0,1);
      //      // lcd.print(String(loopLED));

      if (digitalRead(LOOPLED) == 1)
      {
         // OLED



      //if (tastaturstatus == 0xF0) // Tastaturwerte weitergeben
      {
         sendbuffer[0] = 0xB8;
         sendbuffer[59] = tastaturcounter++;
         sendbuffer[57] = tastenwert;
         sendbuffer[58] = Taste;

         //uint8_t senderfolg = usb_rawhid_send((void *)sendbuffer, 10);
      }
         // OLED
         //// Serial.printf("LED ON\n");
         digitalWriteFast(LOOPLED, 0);
         /*
          //// Serial.printf("blink\t %d\n",loopLED);
          // lcd.setCursor(0,0);
          // lcd.print("hallo");
          // lcd.print(String(timer2Counter));
          // lcd.setCursor(10,0);
          // lcd.print(String(usb_recv_counter));

          // lcd.setCursor(16,0);
          // lcd.print(String(abschnittnummer));
          // lcd.setCursor(0,1);
          // lcd.print(String(CounterA&0xFF));
          // lcd.setCursor(4,1);
          // lcd.print(String(CounterB&0xFF));
          */
         /*
         if (digitalRead(END_A0_PIN)) // Eingang ist HI, Schlitten nicht am Anschlag A0
         {
            //// Serial.printf("Anschlag Motor A OFF\n");
         }
          else
          {
             // Serial.printf("Anschlag Motor A ON\n");
          }

         if (digitalRead(END_B0_PIN)) // Eingang ist HI, Schlitten nicht am Anschlag A0
         {
            //// Serial.printf("Anschlag Motor B OFF\n");
         }
         else
         {
            // Serial.printf("Anschlag Motor B ON\n");
         }
*/

      }
      else
      {
         digitalWriteFast(LOOPLED, 1);
      }
      parallelcounter += 2;
      //      lcd.setCursor(14,0);
      //      lcd.print(String(parallelcounter));
   } // sinceblink

   if (sincelaststep > 50) // 60 us
   {
      
         if (analogtastaturstatus & (1<<TASTE_ON)) // Taste gedrueckt
         {
            OSZIC_LO();
             
         }
         else 
         { 
            tastaturTimer.end();
            /*
            digitalWriteFast(MA_EN,HIGH);
            digitalWriteFast(MB_EN,HIGH);
            digitalWriteFast(MC_EN,HIGH);
            digitalWriteFast(MA_STEP,HIGH);
            digitalWriteFast(MB_STEP,HIGH);
            digitalWriteFast(MC_STEP,HIGH);
            */
            OSZIC_HI();
         }
  
      // // Serial.printf("sincelaststep\n");
      sincelaststep = 0;


    
      if(taskstatus & (1<<TASK))
      {
         //OSZIB_LO();
      }
      else 
      {
         
         if (taskstatus & (1<<RUNNING))
         {
            //OSZIB_HI();
            taskstatus &= ~(1<<RUNNING);
            //
      
         }
         else 
         {
            
            //
          
         
         {
           
         }
            tastencounter++;
            if (tastencounter > 10)
            {
               //OSZIA_LO();
               // Tastatur lesen
               tastenwert = readTastatur(); //adc->adc0->analogRead(TASTATURPIN);
               //tastenwert = 99;

               // repetition erkennen
               oldTaste = Taste; 
               //uint8_t newTaste = Tastenwahl(tastenwert);
               // if((oldTaste == 0) || (newTaste != oldTaste)) // andere Taste
               {
 
                  //Taste = newTaste;
                  tastenfunktion(tastenwert);

               }
               
                  
               }
            
            //OSZIA_HI();
         }



      }
      
      //Taste = 0;
   
   }// sincelaststep > 500
 
   ////#pragma mark start_(usb
   
   r = usb_rawhid_recv((void *)buffer, 0); // 1.5us

   if (r > 0) //
   {
      //OSZIB_LO();
      noInterrupts();
      uint8_t code = 0x00;
      code = buffer[16];

      usb_recv_counter++;
      uint8_t device = buffer[32];

   
      // sendbuffer[63]=0; // nur bei home auf 1 setzen
      sendbuffer[24] = buffer[32];
      switch (code)
      {
         case 0xA4:
         {
            // Serial.printf("A4 clear\n");
         }
         break;

         ////#pragma mark A5
         case 0xA5: //
         {
            //        // Serial.printf("A5\n");
           
            if (StepCounterA)
            {
               uint8_t vorzeichenx = buffer[4];

               uint16_t dx = buffer[0] | ((buffer[1] << 8) & 0x7F);

           
            }
            if (StepCounterB)
            {
               uint8_t vorzeicheny = buffer[12];
               uint16_t dy = buffer[8] | ((buffer[9] << 8) & 0x7F);
               // StepCounterB += dy;
               //           // Serial.printf("setStepCounter dy: %d vorzeichen: %d\n",dy,vorzeicheny);
            }
            AbschnittLaden_bres(buffer);
            taskstatus |= (1<<TASK);
         }
         break;
         ////#pragma mark B1
         case 0xB1:
         {
            // Serial.printf("B1 PCB\n");
            uint8_t indexh = buffer[26];
            uint8_t indexl = buffer[27];

            abschnittnummer = indexh << 8;
            abschnittnummer += indexl;
            // Serial.printf("B1 abschnittnummer: %d\n", abschnittnummer);

            AbschnittLaden_bres(buffer);
            if (abschnittnummer == 0)
            {
            }

            sendbuffer[7] = (ladeposition & 0xFF00) >> 8;
            sendbuffer[8] = ladeposition & 0x00FF;

            sendbuffer[10] = (endposition & 0xFF00) >> 8;
            sendbuffer[11] = (endposition & 0x00FF);

            sendbuffer[0] = 0xB2;
            //          usb_rawhid_send((void*)sendbuffer, 0);
         }
         break;
         ////#pragma mark B3
         case 0xB3: // sendTextdaten
         {
            // Serial.printf("*B3 Joystick*\n");
            uint8_t i = 0;
            for (i = 0; i < 33; i++) // 5 us ohne printf, 10ms mit printf
            {
               // Serial.printf("%d \t", buffer[i]);
            }
            // Serial.printf("\n");
            sendbuffer[24] = buffer[32];

            uint8_t indexh = buffer[26];
            uint8_t indexl = buffer[27];
            // Serial.printf("indexh: %d indexl: %d\n", indexh, indexl);
            abschnittnummer = indexh << 8;
            abschnittnummer += indexl;
            // Serial.printf("abschnittnummer: *%d*\n", abschnittnummer);
            sendbuffer[5] = (abschnittnummer & 0xFF00) >> 8;
         
            sendbuffer[6] = abschnittnummer & 0x00FF;

            // Lage:

            uint8_t lage = buffer[25];

            //// Serial.printf("B3 abschnittnummer: %d\tbuffer25 lage: %d \t device: %d\n", abschnittnummer, lage, buffer[32]);
            //             // Serial.printf("count: %d\n",buffer[22]);
            if (abschnittnummer == 0) // Start
            {
               // noInterrupts();
               PWM = buffer[29];
               //              lcd.print(String(PWM));

               ladeposition = 0;
               // globalaktuelleladeposition = 0;
               aktuelleladeposition = 0;
               endposition = 0xFFFF;
               cncstatus = 0;
               sendstatus = 0;
               motorstatus = 0;
               korrekturcounterx = 0;
               korrekturcountery = 0;
               ringbufferstatus = 0x00;
               anschlagstatus = 0;
               ringbufferstatus |= (1 << FIRSTBIT);
               ringbufferstatus |= (1 << STARTBIT);
               AbschnittCounter = 0;
               sendbuffer[5] = (abschnittnummer & 0xFF00) >> 8;
               sendbuffer[6] = abschnittnummer & 0x00FF;

               sendbuffer[14] = (TIMERINTERVALL & 0xFF00) >> 8;
               sendbuffer[15] = (TIMERINTERVALL & 0x00FF);
               sendbuffer[8] = ladeposition;
               sendbuffer[0] = 0xD1;
               // Abschnitt 0 melden
               //      usb_rawhid_send((void*)sendbuffer, 0);

            }
            else // Abschnittnummer > 0
            {
               // Ablauf schon gestartert
               //          // Serial.printf("  -----                  B3 Ablauf gestartet, abschnittnummer: %d\n",abschnittnummer);
            }

            // lage im Ablauf:
            // 1: erster Abschnitt
            // 2: letzter Abschnitt
            // 0: innerer Abschnitt

            if (buffer[25] & 0x02) // letzter Abschnitt
            {
               //              // Serial.printf("------------------------  last abschnitt\n");
               ringbufferstatus |= (1 << LASTBIT);     // letzter Abschnitt
               if (ringbufferstatus & (1 << FIRSTBIT)) // nur ein Abschnitt
               {
                  // endposition setzen
                  //                // Serial.printf("------------------------  erster ist letzter Abschnitt\n");
                  endposition = abschnittnummer; // erster ist letzter Abschnitt

                  //               // Serial.printf("------------------------  nur ein abschnitt endposition: %d   * ringbufferstatus: %d\n",endposition, ringbufferstatus);
               }
            }

            uint8_t pos = (abschnittnummer);

            pos &= 0x03; // 2 bit // Beschraenkung des index auf Buffertiefe
            //              // Serial.printf("default: load  CNC-Daten. abschnittnummer: %d endposition: %d pos: %d\n",abschnittnummer,endposition,pos);
            // if (abschnittnummer>8)



            // Daten laden in ringbuffer an Position pos
            //          uint8_t i=0;
         
            // // Serial.printf("\n");
            // OSZIA_LO();
            //               // Serial.printf("default: abschnittnummer: %d pos: %d\n",abschnittnummer,pos);
            for (i = 0; i < 64; i++) // 5 us ohne printf, 10ms mit printf
            {
               //                 // Serial.printf("%d \t",buffer[i]);
               CNCDaten[pos][i] = buffer[i];
            }
            interrupts();
         } // B3
         break;

         ////#pragma mark B5
      case 0xB5: // PCB neu
      {
         // Serial.printf("B5\n");
         /*
          digitalWriteFast(MA_EN,HIGH);
         digitalWriteFast(MB_EN,HIGH);
         
         digitalWriteFast(MC_EN,HIGH);
         digitalWriteFast(MD_EN,HIGH);
         */
         sendbuffer[0] = 0xB6;
         usb_rawhid_send((void *)sendbuffer, 0);
         uint8_t i = 0;
         for (i = 0; i < 33; i++) // 5 us ohne printf, 10ms mit printf
         {
            //                 // Serial.printf("%d \t",buffer[i]);
            CNCDaten[0][i] = buffer[i];
         }
         sendbuffer[24] = buffer[32];

         uint8_t indexh = buffer[26];
         uint8_t indexl = buffer[27];
         //   // Serial.printf("indexh: %d indexl: %d\n",indexh,indexl);
         abschnittnummer = indexh << 8;
         abschnittnummer += indexl;
         //   // Serial.printf("abschnittnummer: *%d*\n",abschnittnummer);
         sendbuffer[5] = (abschnittnummer & 0xFF00) >> 8;
         ;
         sendbuffer[6] = abschnittnummer & 0x00FF;

         if (abschnittnummer == 0) // Start
         {
            // noInterrupts();
            // Serial.printf("B5 abschnittnummer 0 \tbuffer25 lage: %d \t buffer32 device: %d\n", buffer[25], buffer[32]);
            //             // Serial.printf("count: %d\n",buffer[22]);
            PWM = buffer[29];
            //              lcd.print(String(PWM));

            ladeposition = 0;
            // globalaktuelleladeposition = 0;
            aktuelleladeposition = 0;
            endposition = 0xFFFF;
            cncstatus = 0;
            sendstatus = 0;
            motorstatus = 0;
            korrekturcounterx = 0;
            korrekturcountery = 0;
            ringbufferstatus = 0x00;
            anschlagstatus = 0;
            ringbufferstatus |= (1 << FIRSTBIT);
            ringbufferstatus |= (1 << STARTBIT);
            AbschnittCounter = 0;
            // sendbuffer[8]= versionintl;
            // sendbuffer[8]= versioninth;
            sendbuffer[5] = (abschnittnummer & 0xFF00) >> 8;
            ;
            sendbuffer[6] = abschnittnummer & 0x00FF;

            // lcd_gotoxy(0,0);
            sendbuffer[14] = (TIMERINTERVALL & 0xFF00) >> 8;
            sendbuffer[15] = (TIMERINTERVALL & 0x00FF);
            sendbuffer[8] = ladeposition;
            {
               sendbuffer[0] = 0xB6;
            }
            sendbuffer[22] = cncstatus;
         }
         if (buffer[25] & 0x02) // letzter Abschnitt
         {
            // Serial.printf("------------------------ B5  last abschnitt\n");
            ringbufferstatus |= (1 << LASTBIT);     // letzter Abschnitt
            if (ringbufferstatus & (1 << FIRSTBIT)) // nur ein Abschnitt
            {
               // endposition setzen
               //                     // Serial.printf("------------------------  erster ist letzter Abschnitt\n");
               endposition = abschnittnummer; // erster ist letzter Abschnitt

               //                      // Serial.printf("------------------------  nur ein abschnitt endposition: %d   * ringbufferstatus: %d\n",endposition, ringbufferstatus);
            }
         }
      }
      break;

         ////#pragma mark C0 Pfeiltaste
      case 0xC0: // mousedown
      {
         //// Serial.printf("case C0\n");
         sendbuffer[24] = buffer[32];

         // Abschnittnummer bestimmen
         uint8_t indexh = buffer[18];
         uint8_t indexl = buffer[19];
         uint8_t position = buffer[17];
         //// Serial.printf("C0 position: %d\n", position);
         abschnittnummer = indexh << 8;
         abschnittnummer += indexl;
         sendbuffer[0] = 0xC2;
         uint8_t lage = buffer[25];
         uint8_t richtung = buffer[31];
         // // Serial.printf("\n****************************************\n");
         // // Serial.printf("C0 Abschnitt lage: %d abschnittnummer: %d richtung: %d\n",lage,abschnittnummer, richtung);
         // // Serial.printf("****************************************\n");
         ladeposition = 0;
         endposition = 0xFFFF;
         cncstatus = 0;
         motorstatus = 0;
         ringbufferstatus = 0x00;
         anschlagstatus = 0;
         ringbufferstatus |= (1 << FIRSTBIT);
         ringbufferstatus |= (1 << STARTBIT);
         ringbufferstatus |= (1 << LASTBIT);
         AbschnittCounter = 0;
         endposition = abschnittnummer;
         // Daten vom buffer in CNCDaten laden
         {
            uint8_t pos = (abschnittnummer);
            pos &= 0x03; // 2 bit // Beschraenkung des index auf Buffertiefe
            // if (abschnittnummer>8)
            {
               // lcd_putint1(pos);
            }
            uint8_t i = 0;
            for (i = 0; i < USB_DATENBREITE; i++)
            {
               CNCDaten[pos][i] = buffer[i];
            }
         }
         taskstatus |= (1<<TASK);
         sendbuffer[0] = 0xC2;
         uint8_t senderfolg = usb_rawhid_send((void *)sendbuffer, 10);
         startTimer2();

      }
      break;

      case 0xC2: // mouseup
      {
         //// Serial.printf("case C2\n");
         uint8_t richtung = buffer[31];
         //// Serial.printf("richtung: %d\n", richtung);
         //// Serial.printf("StepCounterA: %d StepCounterB: %d StepCounterC: %d StepCounterD: %d \n", StepCounterA, StepCounterB, StepCounterC, StepCounterD);
         StepCounterA = 0;
         StepCounterB = 0;
         StepCounterC = 0;
         StepCounterD = 0;
         digitalWriteFast(MA_STEP, HIGH);
         digitalWriteFast(MB_STEP, HIGH);
         digitalWriteFast(MC_STEP, HIGH);
         //digitalWriteFast(MD_STEP, HIGH);

         digitalWriteFast(MA_EN, HIGH);
         digitalWriteFast(MB_EN, HIGH);
         digitalWriteFast(MC_EN, HIGH);
         //digitalWriteFast(MD_EN, HIGH);
 
         xA = 0;
         yA = 0;
         bres_counterA = 0;
         bres_delayA = 0;

         xB = 0;
         yB = 0;

         bres_counterB = 0;
         bres_delayB = 0;

         ringbufferstatus = 0;

         //ringbufferstatus |= (1 << STARTBIT);
         //ringbufferstatus |= (1 << FIRSTBIT); // Experiment 240312
         // > in 0xC0
         cncstatus = 0;
         motorstatus = 0;
        
         taskstatus &= ~(1<<TASK);

         analogtastaturstatus &= ~(1<<TASTE_ON);sendbuffer[0] = 0xC2;
         uint8_t senderfolg = usb_rawhid_send((void *)sendbuffer, 10);

      }
      break;

      case 0xCA:
      {
         // Serial.printf("case CA\n");
      }
      break;

      case 0xE0: // Man: Alles stoppen
      {
         // Serial.printf("E0 Stop\n");
         ringbufferstatus = 0;
         motorstatus = 0;
         anschlagstatus = 0;
         cncstatus = 0;
         sendbuffer[0] = 0xE1;
         tastaturstatus = 0xF0 ;
         sendbuffer[5] = (abschnittnummer & 0xFF00) >> 8;
         
         sendbuffer[6] = abschnittnummer & 0x00FF;

         sendbuffer[8] = ladeposition & 0x00FF;
         // sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
         sendbuffer[22] = cncstatus;

         usb_rawhid_send((void *)sendbuffer, 0);

         sendbuffer[0] = 0x00;
         sendbuffer[5] = 0x00;
         sendbuffer[6] = 0x00;
         sendbuffer[8] = 0x00;

         ladeposition = 0;
         sendbuffer[8] = ladeposition;
         endposition = 0xFFFF;

         AbschnittCounter = 0;
         PWM = sendbuffer[29];
         // digitalWriteFast(DC_PWM,HIGH);

         analogWrite(DC_PWM, 0);

         StepCounterA = 0;
         StepCounterB = 0;
         StepCounterC = 0;
         StepCounterD = 0;

         CounterA = 0;
         CounterB = 0;
         CounterC = 0;
         CounterD = 0;

         /*
         digitalWriteFast(MA_EN,HIGH);
         digitalWriteFast(MB_EN,HIGH);
         digitalWriteFast(MC_EN,HIGH);
         digitalWriteFast(MD_EN,HIGH);
          */
         // lcd.setCursor(0,1);
         // lcd.print("HALT");

         // lcd_gotoxy(0,1);
         // lcd_puts("HALT\0");
         // Serial.printf("E0 Stop END\n");
      }
      break;

         ////#pragma mark E2 DC
      case 0xE2: // DC_PWM ON_OFF: Temperatur Schneiddraht setzen
      {

         PWM = buffer[20];
         // Serial.printf("E2 setPWM: %d\n", PWM);
         analogWrite(DC_PWM, PWM);
         // analogWrite(9,PWM);

         parallelstatus |= (1 << THREAD_COUNT_BIT);

         sendbuffer[0] = 0xE3;
         //          usb_rawhid_send((void*)sendbuffer, 0);
         // sendbuffer[0]=0x00;
         // sendbuffer[5]=0x00;
         // sendbuffer[8]=0x00;
      }
      break;

      case 0xE4: // Stepperstrom ON_OFF
      {
         // Serial.printf("E4 ON\n");
         if (buffer[8])
         {
            // CMD_PORT |= (1<<STROM); // ON
            digitalWriteFast(STROM, HIGH);
            PWM = buffer[29];
         }
         else
         {
            // CMD_PORT &= ~(1<<STROM); // OFF
            digitalWriteFast(STROM, LOW);
            PWM = 0;
         }

         if (PWM == 0)
         {
            // CMD_PORT &= ~(1<<DC_PWM);
            digitalWriteFast(DC_PWM, LOW);
         }

         sendbuffer[0] = 0xE5;
         //          usb_rawhid_send((void*)sendbuffer, 0);
         // sendbuffer[0]=0x00;
         // sendbuffer[5]=0x00;
         // sendbuffer[6]=0x00;
      }
      break;

         ////#pragma mark F1 reset
      case 0xF1: // reset
      {
         // Serial.printf("F1 reset\n");
         uint8_t i = 0, k = 0;
         for (k = 0; k < RINGBUFFERTIEFE; k++)
         {
            for (i = 0; i < USB_DATENBREITE; i++)
            {
               CNCDaten[k][i] = 0;
            }
         }

         ringbufferstatus = 0;
         motorstatus = 0;
         anschlagstatus = 0;
         tastaturstatus = 0xF0 ;
         cncstatus = 0;
         ladeposition = 0;
         endposition = 0xFFFF;

         AbschnittCounter = 0;
         PWM = 0;
         // digitalWriteFast(DC_PWM,LOW);
         analogWrite(DC_PWM, 0);
         korrekturcounterx = 0;
         korrekturcountery = 0;

         xA = 0;
         yA = 0;

         xB = 0;
         yB = 0;

         digitalWriteFast(MA_EN, HIGH);
         digitalWriteFast(MB_EN, HIGH);

         digitalWriteFast(MA_EN, HIGH);
         digitalWriteFast(MB_EN, HIGH);
         
         digitalWriteFast(MA_STEP, HIGH);
         digitalWriteFast(MB_STEP, HIGH);
         digitalWriteFast(MC_STEP, HIGH);
         //digitalWriteFast(MD_STEP, HIGH);


         sendbuffer[0] = 0xF2;
                  usb_rawhid_send((void*)sendbuffer, 0);
         sendbuffer[0] = 0x00;
         // Serial.printf("F1 reset end\n");
      }
      break;

         // MARK: F0
      case 0xF0: // cncstatus fuer go_home setzen
      {
         tastaturstatus = 0 ;
         // Serial.printf("F0 home \n");
         //  gohome();
         //  break;

         // Strom OFF
         analogWrite(DC_PWM, 0);

         // abschnittnummer = 0; // diff 220520

         ladeposition = 0;
         endposition = 0xFFFF;
         cncstatus = 0;
         motorstatus |= 0;
         ringbufferstatus = 0x00;
         anschlagstatus = 0;
         ringbufferstatus |= (1 << FIRSTBIT);
         ringbufferstatus |= (1 << STARTBIT); // diff 220520, Start
         ringbufferstatus |= (1 << LASTBIT);
         uint8_t lage = buffer[17];
         AbschnittCounter = 0;
         // sendbuffer[8]= versionintl;
         // sendbuffer[8]= versioninth;

         sendbuffer[0] = 0xF1;

         cncstatus |= (1 << GO_HOME); // Bit fuer go_home setzen
         sendbuffer[63] = 1;
         sendbuffer[22] = cncstatus;

         // Daten vom buffer in CNCDaten laden
         {
            uint8_t pos = 0;
            pos &= 0x03; // 2 bit // Beschraenkung des index auf Buffertiefe
            // if (abschnittnummer>8)
            {
               // lcd_putint1(pos);
            }
            uint8_t i = 0;
            for (i = 0; i < USB_DATENBREITE; i++)
            {
               if (i < 5)
               {
                  //  lcd_puthex(buffer[i]);
               }
               CNCDaten[pos][i] = buffer[i];
            }
         }
         startTimer2();

         // F0 melden
         //            usb_rawhid_send((void*)sendbuffer, 0);

         sei();
      }
      break;

         // #pragma mark default
      default: // CNC
      {
         // // Serial.printf("---  default   usb_recv_counter %d\t \nringbufferstatus: %02X position(buffer17): %02X device: %d \n",usb_recv_counter,ringbufferstatus, buffer[17],device);
         //  Abschnittnummer bestimmen

         sendbuffer[24] = buffer[32];

         // Abschnittnummer bestimmen
         uint8_t indexh = buffer[18];
         uint8_t indexl = buffer[19];

         uint16_t index = indexl | (indexh >> 8);

         uint8_t position = buffer[17];
         // // Serial.printf("default index: %d position: %d\n",index,position);

         abschnittnummer = indexh << 8;
         abschnittnummer += indexl;
         sendbuffer[0] = 0x33;
         sendbuffer[5] = abschnittnummer;
         sendbuffer[6] = buffer[16];
         // Lage:

         //    uint8_t lage = buffer[25];
         // lage im Ablauf:
         // 1: erster Abschnitt
         // 2: letzter Abschnitt
         // 0: innerer Abschnitt

         // OSZIA_LO();
         //             // Serial.printf("\n\ndefault abschnittnummer: %d  lage: %d code: %d\n",abschnittnummer,lage,code); // 50 us
         //             for(int i=0;i<33;i++)
         {
            //              // Serial.printf("%d\t",buffer[i]);
         }
         //           // Serial.printf("\n");

         // // Serial.printf("\n****************************************\n");
         // // Serial.printf("default Abschnitt lage: %d abschnittnummer: %d\n",lage,abschnittnummer);
         // // Serial.printf("****************************************\n");

         //              usb_rawhid_send((void*)sendbuffer, 0); // nicht jedes Paket melden

         if (abschnittnummer == 0)
         {
            // anschlagstatus &= ~(1<< END_A0); // 220518 diff
            //   lcd_clr_line(1);
            cli();
            /*
             uint8_t i=0,k=0;
             for (k=0;k<RINGBUFFERTIEFE;k++)
             {
             for(i=0;i<USB_DATENBREITE;i++)
             {
             CNCDaten[k][i]=0;
             }
             }
             */
            // CNCDaten = {};

            ladeposition = 0;
            endposition = 0xFFFF;
            cncstatus = 0;
            motorstatus = 0;
            ringbufferstatus = 0x00;
            anschlagstatus = 0;
            ringbufferstatus |= (1 << FIRSTBIT);
            AbschnittCounter = 0;
            // sendbuffer[8]= versionintl;
            // sendbuffer[8]= versioninth;
            sendbuffer[5] = 0x00;

            // in teensy3.2: timerintervall
            //                 sendbuffer[8] = (TIMERINTERVALL & 0xFF00)>>8;
            //                 sendbuffer[9] = (TIMERINTERVALL & 0x00FF);
            sendbuffer[0] = 0xD1;

            //         usb_rawhid_send((void*)sendbuffer, 0);
            startTimer2();
            sei();
         }
         else
         {
         }

         //             if (buffer[9]& 0x02)// letzter Abschnitt

         if (buffer[17] & 0x02) // letzter Abschnitt
         {
            // Serial.printf("default last Abschnitt: %d\n", buffer[17]);
            ringbufferstatus |= (1 << LASTBIT);
            if (ringbufferstatus & (1 << FIRSTBIT)) // nur ein Abschnitt
            {
               // // Serial.printf("default last Abschnitt: nur ein Abschnitt.\n");
               endposition = abschnittnummer; // erster ist letzter Abschnitt
            }
         }

         // Daten vom buffer in CNCDaten laden
         {
            uint8_t pos = (abschnittnummer);
            pos &= 0x03; // 2 bit // Beschraenkung des index auf Buffertiefe
            // if (abschnittnummer>8)
            {
               // lcd_putint1(pos);
            }
            uint8_t i = 0;
            for (i = 0; i < USB_DATENBREITE; i++)
            {
               CNCDaten[pos][i] = buffer[i];
            }
         }

         // Erster Abschnitt, mehrere Abschnitte: naechsten Abschnitt laden
         if ((abschnittnummer == 0) && (endposition))
         {
            {
               // lcd_putc('*');
               //  Version zurueckmelden

               // versionl=VERSION & 0xFF;
               // versionh=((VERSION >> 8) & 0xFF);

               sendbuffer[5] = abschnittnummer;
               sendbuffer[6] = ladeposition;
               sendbuffer[0] = 0xAF;
               usb_rawhid_send((void *)sendbuffer, 0);
               tastaturstatus = 0xF0 ;
               sei();
            }
         }

         ringbufferstatus &= ~(1 << FIRSTBIT);

         // Ringbuffer ist voll oder  letzter Abschnitt schon erreicht

         if ((abschnittnummer == 1) || ((abschnittnummer == 0) && (ringbufferstatus & (1 << LASTBIT))))
         {
            {
               ringbufferstatus &= ~(1 << LASTBIT);
               ringbufferstatus |= (1 << STARTBIT);
            }
         }
      }
      break; // default

      } // switch code
      interrupts();
      code = 0;

      //OSZIB_HI();
   } // r > 0
   /**   End USB-routinen   ***********************/

   ////#pragma mark CNC-routinen
   /*>
    #define STARTBIT   2       // Buffer ist geladen
    #define RINGBUFFERBIT 3    // Ringbuffer wird verwendet
    #define LASTBIT   4         // Letzter Abschnitt  ist geladen
    #define ENDBIT   5          // Letzter Abschnitt  ist abgearbeitet
    #define STOPBIT   6        // Ablauf stoppen
    #define FIRSTBIT   7

    */
   /*   Start CNC-routinen   ********************** */
   if (ringbufferstatus & (1 << STARTBIT)) // Buffer ist in Ringbuffer geladen, Schnittdaten von Abschnitt 0 laden
   {
      // noInterrupts();

      //     // Serial.printf("\n\n                 Abschnitt 0 laden ringbufferstatus: %d\n",ringbufferstatus);
      ringbufferstatus &= ~(1 << STARTBIT); // Startbit entfernen
      ladeposition = 0;                     // laufender Zaehler fuer Ringbuffer, gefiltert mit Ringbuffertiefe
      AbschnittCounter = 0;
      richtungstatus = 0; // neubeginn, set back
      oldrichtungstatus = 0;
      // Abschnitt 0 laden
      uint8_t l = sizeof(CNCDaten[ladeposition]);
      uint8_t micro = CNCDaten[ladeposition][26];

      ////#pragma mark default Ersten Abschnitt laden

      // // Serial.printf("+++ Ersten Abschnitt laden AbschnittLaden_bres len: %d ringbufferstatus: %d micro: %d \n",l,ringbufferstatus, micro);
      //     uint8_t lage=AbschnittLaden_bres(CNCDaten[ladeposition]); // erster Wert im Ringbuffer

      uint8_t lage = AbschnittLaden_bres(CNCDaten[ladeposition]); // erster Wert im Ringbuffer

      // Gradient
      int8_t vz = 1;                           // vorzeichen
      uint8_t axh = CNCDaten[ladeposition][1]; // hi byte
      if (axh & 0x80)                          // bit8, vz
      {
         vz = -1;
      }
      lastdax = (CNCDaten[ladeposition][0] | ((CNCDaten[ladeposition][1] & 0x7F) << 8)) * vz;

      vz = 1;
      uint8_t ayh = CNCDaten[ladeposition][3]; // hi byte
      if (ayh & 0x80)                          // bit8, vz
      {
         vz = -1;
      }
      lastday = (CNCDaten[ladeposition][2] | ((CNCDaten[ladeposition][3] & 0x7F) << 8)) * vz;

      // // Serial.printf("\n+++ \nErster Abschnitt lastdax: %d lastday: %d \n",lastdax,lastday);

      // // Serial.printf("+++ Erster Abschnitt lage nach AbschnittLaden_bres: %d\n",lage);
      ladeposition++;
      if (lage == 2) // nur ein Abschnitt
      {
         // // Serial.printf("Abschnitt 0 laden nur 1 Abschnitt cncstatus: %d\n", cncstatus);
         ringbufferstatus |= (1 << ENDBIT); // unbenutzt
         ringbufferstatus |= (1 << LASTBIT);
      }
      AbschnittCounter += 1;

      interrupts();

      //      startTimer2();
      //      // // Serial.printf("motorstatus: %d\n",motorstatus);
      // Serial.printf("+++   +++   +++    Erster Abschnitt end\n");
   } // 1 << STARTBIT: Ersten Abschnitt laden



   // sendbuffer[0]=0;
   ////#pragma mark Anschlag

   // Anschlagsituation abfragen
   ////#pragma mark Anschlag   Motor A
   // ********************
   // * Anschlag Motor A *
   // ********************
   // AnschlagVonMotor(0);

   if (digitalRead(END_A0_PIN)) // Eingang ist HI, Schlitten nicht am Anschlag A0
   {

      if (anschlagstatus & (1 << END_A0)) // Schlitten war, aber ist nicht mehr am Anschlag
      {
         anschlagstatus &= ~(1 << END_A0); // Bit fuer Anschlag A0 zuruecksetzen
      }
   }
   else // Schlitten bewegte sich auf Anschlag zu und ist am Anschlag A0
   {
      // // Serial.printf("Anschlag Motor A\n");
      AnschlagVonMotor(0); // Bewegung anhalten
   }

   // #pragma mark Anschlag   Motor B
   //  **************************************
   //  * Anschlag Motor B *
   //  **************************************
   // AnschlagVonMotor(1);

   if (digitalRead(END_B0_PIN)) // Schlitten nicht am Anschlag B0
   {
      if (anschlagstatus & (1 << END_B0))
      {
         anschlagstatus &= ~(1 << END_B0); // Bit fuer Anschlag B0 zuruecksetzen
      }
   }
   else // Schlitten bewegte sich auf Anschlag zu und ist am Anschlag B0
   {
      // // Serial.printf("Anschlag Motor B\n");
      AnschlagVonMotor(1);
   } // end Anschlag B0

   // End Anschlag B

   // #pragma mark Anschlag   Motor C
   //  ********************
   //  * Anschlag Motor C *
   //  ********************
   // AnschlagVonMotor(2);

   // Anschlag C0
   if (digitalRead(END_C0_PIN)) // Eingang ist HI, Schlitten nicht am Anschlag C0
   {
      if (anschlagstatus & (1 << END_C0))
      {
         anschlagstatus &= ~(1 << END_C0); // Bit fuer Anschlag C0 zuruecksetzen
      }
   }
   else // Schlitten bewegte sich auf Anschlag zu und ist am Anschlag C0
   {
      // Serial.printf("Motor C an C0\n");
      AnschlagVonMotor(2);
   }
/*
   // #pragma mark Anschlag   Motor D
   //  ***************
   //  * Anschlag Motor D *
   //  ***************
   // AnschlagVonMotor(3);

   // Anschlag D0
   if (digitalRead(END_D0_PIN)) // Schlitten nicht am Anschlag D0
   {
      if (anschlagstatus & (1 << END_D0))
      {
         anschlagstatus &= ~(1 << END_D0); // Bit fuer Anschlag D0 zuruecksetzen
      }
   }
   else // Schlitten bewegte sich auf Anschlag zu und ist am Anschlag D0
   {
      // Serial.printf("Motor D an D0\n");
      AnschlagVonMotor(3);
   }
*/
   // #pragma mark Motor A B
   // // Serial.printf("deltafastdirectionA: %d deltafastdirectionB: %d  \n",deltafastdirectionA,deltafastdirectionB);

   // **************************************
   // * Motor A,B *
   // **************************************
   noInterrupts();

   if (deltafastdirectionA > 0) // Bewegung auf Seite A vorhanden
   {
      // // Serial.printf("abschnittnummer: %d richtungstatus: %d\n",abschnittnummer,richtungstatus);
      //  Es hat noch Steps, bres_delayA ist abgezaehlt (bres_delayA bestimmt Impulsabstand fuer Steps)
      if ((bres_counterA > 0) && (bres_delayA == 0) && ((!(anschlagstatus & (1 << END_A0))) && (!(anschlagstatus & (1 << END_B0)))))
      {
         // start ramp

         if (rampstatus & (1 << RAMPSTARTBIT))
         {
            if (ramptimerintervall > timerintervall_FAST) // noch nicht auf max speed
            {
                if (rampstatus & (1 << RAMPOKBIT))
               {
                  ramptimerintervall -= RAMPSCHRITT;

                  delayTimer.update(ramptimerintervall);
                  // rampbreite++;
               }
            }
            else // max
            {
               // OSZIB_HI();
               // errarray[errpos++] = 1000;
               rampstatus &= ~(1 << RAMPSTARTBIT);
               rampendstep = rampstepstart - max(StepCounterA, StepCounterB);
               rampstatus |= (1 << RAMPENDBIT);
               rampstatus |= (1 << RAMPEND0BIT);
               // // Serial.printf("end rampstepstart: %d rampendstep: %d ramptimerintervall: %d timerintervall: %d\n",rampstepstart,rampendstep, ramptimerintervall,timerintervall);
               // // Serial.printf("end ramp\n");
               rampstatus &= ~(1 << RAMPOKBIT);
            }
         } //  RAMPSTARTBIT



         // end ramp

         //      noInterrupts();
         //
         // Aktualisierung Fehlerterm
         //OSZIC_LO();
         errA -= deltaslowdirectionA;

         bres_counterA -= 1; // steps abarbeiten

         if (bres_counterA < 4)
         {
            // // Serial.printf("bres_counterA: %d xA: %d yA: %d\n",bres_counterA, xA, yA);
         }
         if (errA < 0)
         {
            // Fehlerterm wieder positiv (>=0) machen
            errA += deltafastdirectionA;
            // Schritt in langsame Richtung, Diagonalschritt
            xA -= ddxA;
            yA -= ddyA;
            if (xA>=0)
            {
               if (ddxA && (xA>=0)) // Motor A soll steppen
               {
                  digitalWriteFast(MA_STEP, LOW);
                  stepdurA = STEPDUR;
               }
            }
            if (yA>=0)
            {
               if (ddyA && (yA>=0)) // Motor B soll steppen
               {
                  digitalWriteFast(MB_STEP, LOW);
                  stepdurB = STEPDUR;
               }
            }
            // Impuls A und B starten
            // // Serial.printf("Motor A diagonal\t");
         }
         else
         {
            // Schritt in schnelle Richtung, Parallelschritt
            if (xA>=0) // noch Schritte da
            {
               xA -= pdxA;
            }
            if (yA>=0)
            {
               yA -= pdyA;
            }

            if (xA>=0) // noch Schritte Motor A
            {
               if (pdxA && (xA>=0)) // Motor A soll steppen
               {
                  digitalWriteFast(MA_STEP, LOW);
                  stepdurA = STEPDUR;
               }
            }
            if (yA>=0) // noch Schritte Motor B
            {
               if (pdyA && (yA>=0)) // Motor B soll steppen
               {
                  digitalWriteFast(MB_STEP, LOW);
                  stepdurB = STEPDUR;
               }
            }

            // // Serial.printf("Motor A parallel\t");
         }
         bres_delayA = deltafastdelayA;
         // CounterA zuruecksetzen fuer neuen Impuls

         // Wenn StepCounterA jetzt nach decrement abgelaufen und relevant: next Datenpaket abrufen
         if ((bres_counterA == 0)) // relevanter counter abgelaufen
         {
            // // Serial.printf("Motor AB bres_counterA ist null\n");
            if ((abschnittnummer == endposition)) // Ablauf fertig
            {
               // // Serial.printf("*** *** *** *** *** *** Motor AB abschnittnummer==endposition xA: %d yA: %d cncstatus: %d\n",xA, yA, cncstatus);
               if (cncstatus & (1 << GO_HOME))
               {
                  homestatus |= (1 << COUNT_A);
               }

               //        cli();
               // // Serial.printf("Motor A endpos > BD\n");
               ringbufferstatus = 0;
               // home:
               motorstatus &= ~(1 << COUNT_A);
               motorstatus = 0;

               sendbuffer[0] = 0xBD;
               sendbuffer[5] = abschnittnummer;
               sendbuffer[6] = ladeposition;
               //    sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
               sendbuffer[22] = cncstatus;
               // // Serial.printf("*** *** *** *** *** BD 1\n");
               usb_rawhid_send((void *)sendbuffer, 0);
               ladeposition = 0;

               analogWrite(DC_PWM, 0);
               cncstatus = 0;

               digitalWriteFast(MA_EN, HIGH);

               taskstatus &= ~(1<<TASK);
               
               /*
               for (uint16_t i=0;i<255;i++)
               {

                  // // Serial.printf("%d\t%d \n",i,errarray[i]);
               }
               */
               //      sei();
            }
            else
            {
               aktuellelage = 0; // Lage innerhalb der Abschnittserie: Start: 1, Innerhalb: 0, Ende: 2

               uint8_t aktuelleladeposition = (ladeposition & 0x00FF); // 8 bit
               aktuelleladeposition &= 0x03;

               // aktuellen Abschnitt laden
               //_delay_us(5);

               // // Serial.printf("axh: %d \t",CNCDaten[aktuelleladeposition][1]);
               uint8_t axh = CNCDaten[aktuelleladeposition][1];
               // // Serial.printf("axh: %d \t",axh);
               if (axh < 128)
               {
                  // // Serial.printf("richtung x positiv\n");
               }
               else
               {
                  // // Serial.printf("richtung x positiv\n");
               }

               //     // Serial.printf("Motor AB: aktuellelage code vor: %d\nAbschnittdaten vor Funktion: \n",CNCDaten[aktuelleladeposition][17]);
               for (uint8_t i = 0; i < 27; i++) // 5 us ohne printf, 10ms mit printf
               {
                  //  // Serial.printf("%d \t",CNCDaten[aktuelleladeposition][i]);
               }
               // // Serial.printf("\n");

               aktuellelage = AbschnittLaden_bres(CNCDaten[aktuelleladeposition]);

               // // Serial.printf("deltafastdirectionA: %d Motor AB: ladeposition: %d aktuellelage: %d ",deltafastdirectionA, ladeposition,aktuellelage);
               if (aktuellelage == 2) // war letzter Abschnitt
               {
                  // // Serial.printf("Motor AB:  war letzter Abschnitt xA: %d yA: %d abschnittnummer: %d\n",xA, yA, abschnittnummer);

                  endposition = abschnittnummer; // letzter Abschnitt

                  // Neu: letzten Abschnitt melden
                  sendbuffer[0] = 0xD0;
                  sendbuffer[5] = abschnittnummer;
                  sendbuffer[6] = ladeposition;
                  // sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
                  sendbuffer[22] = cncstatus;
                  usb_rawhid_send((void *)sendbuffer, 0);
                  tastaturstatus = 0xF0 ;
                  //taskstatus &= ~(1<<TASK);
                  // sei();
               }
               else
               {
                  OSZI_D_LO();
                  // neuen Abschnitt abrufen
                  // // Serial.printf("Motor AB neuen Abschnitt abrufen xA: %d yA: %d abschnittnummer: %d\n",xA, yA, abschnittnummer);
                  sendbuffer[5] = abschnittnummer;
                  sendbuffer[6] = ladeposition;
                  // sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
                  sendbuffer[22] = cncstatus;
                  // TODO : ev.  A0 setzen
                  sendbuffer[0] = 0xA1;
                  usb_rawhid_send((void *)sendbuffer, 0);
                  OSZI_D_HI();
               }

               ladeposition++;

               AbschnittCounter++;
            }
         }

         //       interrupts();
         //OSZIC_HI();
      }
      else // if (bres_counterA == 0)
      {
         // OSZI_D_LO();
         if (digitalReadFast(MA_STEP) == 0) // 100 ns
         {
            if (stepdurA)
            {
               stepdurA--;
            }
            // // Serial.printf("step A beenden\n");
            if (stepdurA == 0)
            {
               digitalWriteFast(MA_STEP, HIGH);
            }
         }
         if (digitalReadFast(MB_STEP) == 0) // 100 ns
         {
            if (stepdurB)
            {
               stepdurB--;
            }
            if (stepdurB == 0)
            {
               digitalWriteFast(MB_STEP, HIGH);
            }
         }
         if ((xA == 0))
         {
            if (digitalReadFast(MA_EN) == 0)
            {
               // Motoren ausschalten
               // // Serial.printf("Motor A ausschalten\n");
               // digitalWriteFast(MA_EN,HIGH);
            }
         }
         if ((yA == 0))
         {
            if (digitalReadFast(MB_EN) == 0)
            {
               // Motoren ausschalten
               // // Serial.printf("Motor B ausschalten\n");
               // digitalWriteFast(MB_EN,HIGH);
            }
         }

         // OSZI_D_HI();
         //      interrupts();
      }

   } // if deltafastdirectionA > 0

   // //

   // Begin Motor C D
   // #pragma mark Motor C D
   //  **************************************
   //  * Motor C D *
   //  **************************************

   // Es hat noch Steps, CounterC ist abgezaehlt (bres_delayB bestimmt Impulsabstand fuer Steps)
   if (deltafastdirectionB > 0)

   // // Serial.printf("C bres_counterB: %d bres_delayB %d \n",bres_counterB,bres_delayB);
   {
      if ((bres_counterB > 0) && (bres_delayB == 0) && ((!(anschlagstatus & (1 << END_C0))) && (!(anschlagstatus & (1 << END_D0)))))
      {
         // // Serial.printf("D %d ",bres_counterB);
         //   noInterrupts();


         // bres
         // Aktualisierung Fehlerterm
         errB -= deltaslowdirectionB;

         bres_counterB -= 1; // steps abarbeiten

         if (bres_counterB < 4)
         {
            // // Serial.printf("bres_counterB: %d xB: %d yB: %d\n",bres_counterB, xB, yB);
         }

         if (errB < 0)
         {
            // Fehlerterm wieder positiv (>=0) machen
            errB += deltafastdirectionB;
            // Schritt in langsame Richtung, Diagonalschritt
            if (xB >=0)
            {
               xB -= ddxB;
            }
            if (yB >=0)
            {
               yB -= ddyB;
            }
            if (xB >=0)
            {
               if (ddxB && (xB >=0)) // Motor C soll steppen
               {
                  digitalWriteFast(MC_STEP, LOW);
                  stepdurC = STEPDUR;
               }
            }
            if (yB >=0)
            {
               /**/
               if (ddyB && (yB >=0)) // Motor D soll steppen
               {
                  //digitalWriteFast(MD_STEP, LOW);
                  stepdurD = STEPDUR;
               }
            }
            // Impuls A und B starten

            // // Serial.printf("Motor CD diagonal\t");
         }
         else
         {
            // Schritt in schnelle Richtung, Parallelschritt
            xB -= pdxB;
            yB -= pdyB;

            if (xB >=0)
            {
               if (pdxB && (xB >=0)) // Motor C soll steppen
               {
                  digitalWriteFast(MC_STEP, LOW);
                  stepdurC = STEPDUR;
               }
            }
            if (yB >=0)
            {
               if (pdyB && (yB >=0)) // Motor D soll steppen
               {
                  //digitalWriteFast(MD_STEP, LOW);
                  stepdurC = STEPDUR;
               }
            }

            // // Serial.printf("Motor CD parallel\t");
         }
         bres_delayB = deltafastdelayB;

         // end bres>=0
         //  Impuls starten
         //   digitalWriteFast(MC_STEP,LOW);
         //   CounterC=DelayC;                     // CounterC zuruecksetzen fuer neuen Impuls

         // StepCounterC--;

         // Wenn StepCounterC abgelaufen und relevant: next Datenpaket abrufen
         if (bres_counterB == 0) // StepCounterC abgelaufen
         {
            // // Serial.printf("Motor BC bres_counterB ist null\n");
            //    sendstatus |= (1<<COUNT_C);

            if (abschnittnummer == endposition)
            {
               // // Serial.printf("*** *** *** *** *** *** Motor CD abschnittnummer==endposition xB: %d yB: %d cncstatus: %d\n",xB, yB, cncstatus);

               if (cncstatus & (1 << GO_HOME))
               {
                  homestatus |= (1 << COUNT_C);
               }

               cli()
                   // // Serial.printf("\nMotor C endpos > BD\n");
                   ringbufferstatus = 0;
               // home:
               motorstatus &= ~(1 << COUNT_C);
               motorstatus = 0;

               sendbuffer[0] = 0xBD;
               sendbuffer[5] = abschnittnummer;
               sendbuffer[6] = ladeposition;
               sendbuffer[22] = cncstatus;

               sendbuffer[19] = motorstatus;
               cncstatus = 0;
               motorstatus = 0;

               digitalWriteFast(MB_EN, HIGH);
               // // Serial.printf("*** *** *** *** BD 2\n");
               //
               usb_rawhid_send((void *)sendbuffer, 0);
               ringbufferstatus = 0;

               ladeposition = 0;
               analogWrite(DC_PWM, 0);

               taskstatus &= ~(1<<TASK);
            }
            else
            {
               uint8_t aktuellelage = 0; // Lage innerhalb der Abschnittserie: Start: 1, Innerhalb: 0, Ende: 2
               {
                  aktuelleladeposition = (ladeposition & 0x00FF);
                  aktuelleladeposition &= 0x03;

                  /*
                  // // Serial.printf("Motor CD: aktuellelage code vor: %d\nAbschnittdaten vor Funktion: \n",CNCDaten[aktuelleladeposition][17]);
                  for(uint8_t i=0;i<27;i++) // 5 us ohne printf, 10ms mit printf
                  {
                     // // Serial.printf("%d \t",CNCDaten[aktuelleladeposition][i]);
                  }
                  // // Serial.printf("\n");
                  */
                  // aktuellen Abschnitt laden
                  aktuellelage = AbschnittLaden_bres(CNCDaten[aktuelleladeposition]);

                  if (aktuellelage == 2) // war letzter Abschnitt
                  {
                     // // Serial.printf("Motor CD:  war letzter Abschnitt xB: %d yB: %d abschnittnummer: %d\n",xB, yB, abschnittnummer);

                     endposition = abschnittnummer; // letzter Abschnitt
                     // Neu: letzten Abschnitt melden
                     sendbuffer[0] = 0xD0;
                     sendbuffer[5] = abschnittnummer;
                     sendbuffer[6] = ladeposition;
                     sendbuffer[22] = cncstatus;
                     usb_rawhid_send((void *)sendbuffer, 0);
                     // sei();
                  }
                  else
                  {
                     // neuen Abschnitt abrufen
                     // // Serial.printf("Motor CD neuen Abschnitt abrufen xB: %d yB: %d abschnittnummer: %d\n",xB, yB, abschnittnummer);

                     sendbuffer[5] = abschnittnummer;
                     sendbuffer[6] = ladeposition;
                     sendbuffer[22] = cncstatus;

                     //      sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
                     sendbuffer[0] = 0xA1;
                     usb_rawhid_send((void *)sendbuffer, 0);
                     taskstatus &= ~(1<<TASK);
                  }

                  ladeposition++;
               }

               AbschnittCounter++;
            }
         }

         //      interrupts();
      }
      else
      {
         // // Serial.printf("F");
         // STEPPERPORT_2 |= (1<<MC_STEP);               // Impuls an Motor C HI -> OFF
         if (digitalReadFast(MC_STEP) == 0) // 100 ns
         {
            // // Serial.printf("step beenden\n");
            if (stepdurC)
            {
               stepdurC--;
            }
            if (stepdurC == 0)
            {
               digitalWriteFast(MC_STEP, HIGH);
            }
         }
         /*
         if (digitalReadFast(MD_STEP) == 0) // 100 ns
         {
            if (stepdurD)
            {
               stepdurD--;
            }
            if (stepdurD == 0)
            {
               digitalWriteFast(MD_STEP, HIGH);
            }
         }
         */
         if ((xB == 0))
         {
            if (digitalReadFast(MC_EN) == 0)
            {
               // Motoren ausschalten
               // // Serial.printf("Motor C ausschalten\n");
               digitalWriteFast(MC_EN, HIGH);
            }
         }
         /*
         if ((yB == 0))
         {
            if (digitalReadFast(MD_EN) == 0)
            {
               // Motoren ausschalten
               // // Serial.printf("Motor D ausschalten\n");
               digitalWriteFast(MD_EN, HIGH);
            }
         }
         */
      }
   }
   // // Serial.printf("G\n");
   interrupts();

   // #pragma mark sendstatus
   // if (sendstatus >= 3)
   sendstatus = 0;

   interrupts();
   // End Motor D

   /**   End CNC-routinen   ***********************/
}
