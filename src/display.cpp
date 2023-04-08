//
//  display.cpp
//  OLED
//
//  Created by Ruedi Heimlicher on 01.03.2023.
//
//
#include <stdint.h>
#include "display.h"
#include "text.h"
#include "font.h"
#include "main.h"

#define linetab 12


extern uint16_t lxmittel;
extern uint16_t lymittel;
extern uint8_t Taste;
extern uint16_t stopsekunde;
extern uint16_t stopminute;
extern uint8_t sendesekunde;
extern uint8_t sendeminute;

extern  uint8_t       curr_setting;
//extern uint8_t       curr_model; // aktuelles modell

uint8_t char_x;
uint8_t char_y;
extern uint16_t  posregister[8][8]; // Aktueller screen: werte fuer page und daraufliegende col fuer Menueintraege (hex). geladen aus progmem

extern uint16_t  cursorpos[8][8]; // Aktueller screen: werte fuer page und daraufliegende col fuer cursor (hex). geladen aus progmem


extern canal_struct canaldata;
extern canal_struct indata;

extern float battspannungmittel;
extern float battspannunginterpol;
extern float spannungvolt;
extern float slavespannungvolt;
uint8_t cursortab[10] = {cursortab0,cursortab1,cursortab2,cursortab3,cursortab4,cursortab5,cursortab6,cursortab7,cursortab0,cursortab0};
uint8_t itemtab[10] = {itemtab0,itemtab1,itemtab2,itemtab3,itemtab4,itemtab5,itemtab6,itemtab7,itemtab0,itemtab0};


// https://forum.arduino.cc/t/clearing-a-line-on-oled-adafruit-ssd1306/369985/8
void clearblock(uint8_t startx, uint8_t starty, uint8_t rangex,uint8_t rangey)
{
for (uint8_t y=starty; y<=starty+rangey; y++)
      {
       for (uint8_t x=startx; x<startx + rangex; x++)
       {
        display.drawPixel(x, y, BLACK); 
       }
      }

}

void clearline(uint8_t starty)
{
for (uint8_t y=starty; y<=starty+6; y++)
      {
       for (uint8_t x=0; x<127; x++)
       {
        display.drawPixel(x, y, BLACK); 
       }
      }

}

void resetRegister(void)
{
  //Serial.printf("resetRegister Start\n");
   uint8_t i=0,k=0;
   for(i=0;i<8;i++)
   {
      //Serial.printf("resetRegister i: %d\n",i);
      for (k=0;k<8;k++)
      {
        //Serial.printf("resetRegister k: %d\n",k);
         posregister[i][k]=0xFFFF;
      }
   }
   Serial.printf("resetRegister End\n");
}


void sethomescreen(void)
{
  resetRegister();
  posregister[0][0] = 0   | (2 << 10); // Name Modell
  posregister[0][1] = 98  | (1 << 10);// Laufzeit
   blink_cursorpos=0xFFFF;
   posregister[0][0] = itemtab[5] | (1 << 10);// Laufzeit Anzeige
   
   
   posregister[1][0] = (0) | (32 << 10); // Text Motorzeit
   posregister[1][1] = (0) | (42 << 10); // Anzeige Motorzeit
   
   
   posregister[2][0] = (58) | (32 << 10); // Text Stoppuhr
   posregister[2][1] = (58) | (42 << 10); // Anzeige Stoppuhr
   Serial.printf("posregister 2 0: %d posregister 2 1: %d\n",posregister[2][0], posregister[2][1]);
   
   posregister[3][0] = (60+DISPLAY_OFFSET) | (0x07 << 10); // Text Akku
   posregister[3][1] = (84+DISPLAY_OFFSET) | (0x08 << 10); // Anzeige Akku

   posregister[4][0] = (0+DISPLAY_OFFSET) | (2 << 10); // Name Modell
   posregister[4][1] = (80+DISPLAY_OFFSET) | (3 << 10); // Text Setting
   posregister[4][2] = (100+DISPLAY_OFFSET) | (3 << 10); // Anzeige Setting

   cursorpos[0][1] = cursortab[0] |    (8 << 10); //  cursorpos fuer Menu

  

  char_x = DISPLAY_OFFSET;
  char_y = 0*linetab;

  display.setCursor(char_x,char_y);
    //display.setCursor(0,0);
  display.print(TitelTable[0]);
  //clearline(10);
  char_y = 1*linetab;
  display.setCursor(char_x,char_y);
  //display.print(ModelTable[0]); // Name Modell
  display_write_str(ModelTable[curr_model],2);

  
  display.display();
}// sethomescreen


uint8_t refresh_screen(void)
{
   display.clearDisplay();
   uint8_t fehler=0;
   uint16_t cursorposition = cursorpos[curr_cursorzeile][curr_cursorspalte];
   fehler=1;
   //Serial.printf("****************  refresh_screen: %d\n",curr_screen);
  // switch curr_screen
    switch (curr_screen)
   {
         
      case HOMESCREEN: // homescreen
      {
          //Serial.printf("refresh_screen: homescreen\n");
          refreshhomescreen();
      }break;

      case SETTINGSCREEN: // Settingscreen
      {
          //Serial.printf("refresh_screen: settingscreen\n");
          refreshsettingscreen();    

      }break;
   } // end switch

return 0;
}

void refreshhomescreen(void)
{
  // laufzeit
  char_x = posregister[0][1] & 0x00FF;
  char_y = (posregister[0][1] & 0xFF00)>>10;
  clearblock(char_x,char_y,68,8);
  display.setCursor(char_x,char_y);
  display_write_laufzeit(sendesekunde,sendeminute);

  char_x = DISPLAY_OFFSET;
  char_y = 0*linetab;

  //clearline(10);
  display.setCursor(char_x,char_y);
  display.print(TitelTable[0]);
  //clearblock(0,20,68,8);
  char_y = 1*linetab;
  display.setCursor(char_x,char_y);
  display_write_str(ModelTable[curr_model],2);
  

  // Stoppuhrtext schreiben
  char_x = (posregister[1][0] & 0x00FF);
  char_y = (posregister[1][0] & 0xFF00)>> 10;
  //Serial.printf("charx: %d chary: %d\n",char_x, char_y);
  display.setCursor(char_x,char_y);
  display_write_str(TitelTable[2],1);

  // Stoppzeit schreiben
  char_y= (posregister[1][1] & 0xFF00)>> 10;
  char_x = (posregister[1][1] & 0x00FF);
  clearblock(char_x,char_y,62,20);
  display.setCursor(char_x,char_y);
  display_write_stopzeit(stopsekunde,stopminute, 1);
  
  
  clearblock(18,55,100,8);
  display.setCursor(0,57);
  putint12(lxmittel);
  

  display.setCursor(30,57);
  putint(canaldata.x);


  display.setCursor(52,57);
  putint12(lymittel);

  display.setCursor(82,57);
  putint(canaldata.y);
 
  display.setCursor(62,57);
  //display_write_str("x:",1);
  //display.print(spannungvolt);

  display.setCursor(92,57);
  //display_write_str("y:",1);
  //display.print(indata.y);


 /*
  clearblock(0,32,68,8);
  display.setCursor(0, 32);
  display.print("ly:");
  display.print(lymittel);
*/
  //pfeilvollrechts(30,50,1);
  //display.print(0x03,0x00);
  //display.print('*');
  //clearblock(0,48,60,8);
  display.setCursor(48,32);
  //display.print(canaldata.y);
  
/*
  clearblock(90,0,10,8);
  display.setCursor(90,0);
  display.print(Taste);
  //display.setCursor(110,0);
  */
  //uint16_t levelprozent = canaldata.x*100/255;
/*
  // Levelmeter
  uint8_t h = 44;
  float max = 4.2;
  
  float level = (spannungvolt-2)/(max-2)*(h);
  uint8_t levelint = uint8_t(level);
  //Serial.printf("spannungvolt: %2.2f level: %2.2f levelint: %d\n",spannungvolt,level,levelint);
  display.setCursor(112,12);
  display.print("T");
  drawlevelmeter(112, 20,4,h,levelint);
  
  // Slave
   float slavelevel = 2;
  if(slavespannungvolt > 2)
  {
    slavelevel = (slavespannungvolt-2)/(max-2)*(h);
  }
  
  uint8_t slavelevelint = uint8_t(slavelevel);
  //Serial.printf("slavespannungvolt: %2.2f slavelevel: %2.2f slavelevelint: %d\n",slavespannungvolt,slavelevel,slavelevelint);
  display.setCursor(120,12);
  display.print("R");
  drawlevelmeter(120, 20,4,h,slavelevelint);
  */
  display.display();

}// refreshhomescreen

void setsettingscreen(void)
{
  {
    Serial.printf("setsettingscreen mit homescreen\n");
    // von sethomescreen
  resetRegister();

  // Zeile 0
  posregister[0][0] = 0   | (2 << 10); // Name Modell
  posregister[0][1] = itemtab[5] | (1 << 10);// Laufzeit
   blink_cursorpos=0xFFFF;
   posregister[0][0] = itemtab[0] | (1* ZEILENABSTAND << 10);// Laufzeit Anzeige
   
   // Zeile 1
   posregister[0][2] =  itemtab[5] |    (1 * ZEILENABSTAND << 10); // settingtext
   posregister[0][3] =  itemtab[7] |    (1 * ZEILENABSTAND << 10); // settingnummer


   // Zeile 2 
   posregister[1][0] =  itemtab[0] |    (2 * ZEILENABSTAND << 10); // Kanaltext
   
   // Zeile 3
   posregister[2][0] =  itemtab[0] |    (3 * ZEILENABSTAND << 10); // mixtext
   posregister[2][1] =  itemtab[5] |    (3 * ZEILENABSTAND << 10); // mixcode


   posregister[3][0] =  itemtab[0] |    (4 * ZEILENABSTAND << 10); // Zuteilungtext
 
   posregister[4][0] =  itemtab[0] |    (7 * ZEILENABSTAND << 10); // Zuteilungtext
   
   cursorpos[0][0] = cursortab[0] |    (1 * ZEILENABSTAND << 10); // modellcursor lo: tab hi: page
   // cursorpos fuer model zeile/colonne
   
   cursorpos[0][1] = cursortab[5] |    (1 * ZEILENABSTAND << 10); //  cursorpos fuer set nr
   
   cursorpos[1][0] = cursortab[0] |    (3 * ZEILENABSTAND << 10);  // cursorpos fuer kanal
   
   cursorpos[2][0] = cursortab[0] |    (5 * ZEILENABSTAND << 10);  // cursorpos fuer mix
   cursorpos[2][1] = cursortab[5] |    (5 * ZEILENABSTAND << 10);  // cursorpos fuer mixcode

   cursorpos[3][0] = cursortab[0] |    (7 * ZEILENABSTAND << 10);  // cursorpos fuer zuteilung
   cursorpos[4][0] = cursortab[0] |    (7 * ZEILENABSTAND << 10);  // cursorpos fuer zuteilung
  
  

  char_x = 0;
  char_y = 0;

  display.setCursor(char_x,char_y);
    //display.setCursor(0,0);
  display.print(SettingTable[0]);
  // laufzeit aktualisieren
    char_x = posregister[0][1] & 0x00FF;
    char_y = (posregister[0][1] & 0xFF00)>>10;
    clearblock(char_x,char_y,68,8);
    display.setCursor(char_x,char_y);
    display_write_laufzeit(sendesekunde,sendeminute);
  //Modellname
  char_y= (posregister[0][0] & 0xFF00)>> 10;
  char_x = posregister[0][0] & 0x00FF;
  display.setCursor(char_x,char_y);
  display_write_str(ModelTable[curr_model],1);

   char_x = cursorpos[0][0] & 0x00FF;
   char_y = ((cursorpos[0][0] & 0xFF00)>> 10) ;
   display.setCursor(char_x,char_y);
  pfeilvollrechts(char_x,char_y,1);

 // Zeile 1 Set 
   char_y= (posregister[0][2] & 0xFF00)>> 10;
   char_x = posregister[0][2] & 0x00FF;
   display.setCursor(char_x,char_y);
   display_write_str(SettingTable[2],1); // Set


// Zeile 2 Kanal
   char_y= (posregister[1][0] & 0xFF00)>> 10;
   char_x = posregister[1][0] & 0x00FF;
   display.setCursor(char_x,char_y);
   display_write_str(SettingTable[3],1);

   // Zeile 3 Mix-Zeile
   char_y= (posregister[2][0] & 0xFF00)>> 10;
   char_x = posregister[2][0] & 0x00FF;
   display.setCursor(char_x,char_y);
   
   display_write_str(SettingTable[4],1);


/*
  char_x = 50;
  display.setCursor(char_x,char_y);
  //display.print(ModelTable[0]); // Name Modell
  display_write_str(ModelTable[curr_model],1);
*/
  char_x = posregister[0][1] & 0x00FF;
  char_y = (posregister[0][1] & 0xFF00)>>10;
  clearblock(char_x,char_y,68,8);
  display.setCursor(char_x,char_y);
  display_write_laufzeit(sendesekunde,sendeminute);

  char_y= (posregister[0][0] & 0xFF00)>> 10;
  char_x = posregister[0][0] & 0x00FF;

   display_write_str(ModelTable[curr_model],1);

   char_x = cursorpos[0][0] & 0x00FF;
   char_y = ((cursorpos[0][0] & 0xFF00)>> 10) + 20;
  pfeilvollrechts(char_x,char_y,1);

  

  
  display.display();
  Serial.printf("setsettingscreen mit homescreen end\n");
  return;

  }




  //Serial.printf("setsettingscreen start\n");
 
   
  resetRegister();
 //Serial.printf("setsettingscreen nach resetregister\n");
   blink_cursorpos=0xFFFF;
  
  
   // 2. Zeile
   posregister[0][0] =  itemtab[0] |    (2 << 10); //Modellname
   posregister[0][1] =  itemtab[0] |    (98 << 10); //

   posregister[0][2] =  itemtab[5] |    (3 << 10); // settingtext
   posregister[0][3] =  itemtab[7] |    (3 << 10); // settingnummer
   
   posregister[1][0] =  itemtab[0] |    (4 << 10); // Kanaltext
   
   posregister[2][0] =  itemtab[0] |    (5 << 10); // mixtext

   posregister[3][0] =  itemtab[0] |    (6 << 10); // Zuteilungtext
 
   posregister[4][0] =  itemtab[0] |    (7 << 10); // Zuteilungtext
   
   cursorpos[0][0] = cursortab[0] |    (2 << 10); // modellcursor lo: tab hi: page
   // cursorpos fuer model zeile/colonne
   
   cursorpos[0][1] = cursortab[5] |    (3 << 10); //  cursorpos fuer setting
   cursorpos[1][0] = cursortab[0] |    (4 << 10);  // cursorpos fuer kanal
   cursorpos[2][0] = cursortab[0] |    (5 << 10);  // cursorpos fuer mix
   cursorpos[3][0] = cursortab[0] |    (6 << 10);  // cursorpos fuer zuteilung
   cursorpos[4][0] = cursortab[0] |    (7 << 10);  // cursorpos fuer zuteilung
  
   

   char_x=itemtab[0];
   char_y = 1;
   char_height_mul = 1;
   char_width_mul = 1;
   display_write_str(SettingTable[0],1);
   char_height_mul = 2;
   char_width_mul = 1;
   
   // Modell Name
   
   //display_write_prop_str(char_y,char_x,0,menubuffer,2);
    // laufzeit
  char_x = posregister[0][1] & 0x00FF;
  char_y = (posregister[0][1] & 0xFF00)>>10;
  clearblock(char_x,char_y,68,8);
  display.setCursor(char_x,char_y);
  display_write_laufzeit(sendesekunde,sendeminute);


   char_y= (posregister[0][0] & 0xFF00)>> 10;
   char_x = posregister[0][0] & 0x00FF;

   display_write_str(ModelTable[curr_model],1);
   char_height_mul = 2;
   char_width_mul = 1;

   char_y= (cursorpos[0][0] & 0xFF00)>> 10;
   char_x = cursorpos[0][0] & 0x00FF;
   //display_write_symbol(pfeilvollrechts);
   
   // 2. Zeile Set mit Nummer
   char_y= (posregister[0][2] & 0xFF00)>> 10;
   char_x = posregister[0][2] & 0x00FF;
   
   char_height_mul = 1;
   display_write_str(SettingTable[2],2);
   

   // Kanal-Zeile
   char_y= (posregister[1][0] & 0xFF00)>> 10;
   char_x = posregister[1][0] & 0x00FF;
   //char_x=0;
   display_write_str(SettingTable[3],2);
   
   char_y= (posregister[1][1] & 0xFF00)>> 10;
   char_x = posregister[1][1] & 0x00FF;
//   display_write_int(curr_kanal,2);

   
   // Mix-Zeile
   char_y= (posregister[2][0] & 0xFF00)>> 10;
   char_x = posregister[2][0] & 0x00FF;
   //char_x=0;
   display_write_str(SettingTable[4],2);
   
   // Zuteilung-Zeile
   char_y= (posregister[3][0] & 0xFF00)>> 10;
   char_x = posregister[3][0] & 0x00FF;
   //char_x=0;
   display_write_str(SettingTable[5],2);
 
   // Output-Zeile
   char_y= (posregister[4][0] & 0xFF00)>> 10;
   char_x = posregister[4][0] & 0x00FF;
   //char_x=0;
   display_write_str(SettingTable[6],2);
   
   //Serial.printf("setsettingscreen end\n");
} // setSettingscreen

void refreshsettingscreen(void)
{
  {
    //Serial.printf("refreshsettingscreen mit homescreen\n");
    char_x = 0;
    char_y = 0;

    display.setCursor(char_x,char_y);
    //'Settings' schreiben'
    display.print(SettingTable[0]);

    // laufzeit aktualisieren
    char_x = posregister[0][1] & 0x00FF;
    char_y = (posregister[0][1] & 0xFF00)>>10;
    clearblock(char_x,char_y,68,8);
    display.setCursor(char_x,char_y);
    display_write_laufzeit(sendesekunde,sendeminute);

    char_y= (posregister[0][0] & 0xFF00)>> 10;
    char_x = posregister[0][0] & 0x00FF;
    display.setCursor(char_x,char_y);
    display_write_str(ModelTable[curr_model],1);
    char_x = cursorpos[0][0] & 0x00FF;
    char_y = ((cursorpos[0][0] & 0xFF00)>> 10) ;
    display.setCursor(char_x,char_y);
    pfeilvollrechts(char_x,char_y,1);

   // Zeile 1 Set Nummer
   char_y= (posregister[0][2] & 0xFF00)>> 10;
   char_x = posregister[0][2] & 0x00FF;
  display.setCursor(char_x,char_y);
   display_write_str(SettingTable[2],1);

 // Zeile 1 Set Nummer
   char_y= (posregister[0][3] & 0xFF00)>> 10;
   char_x = posregister[0][3] & 0x00FF;
  display.setCursor(char_x,char_y);
   puthex(curr_setting);



   // Zeile 2 Kanal
   char_y= (posregister[1][0] & 0xFF00)>> 10;
   char_x = posregister[1][0] & 0x00FF;
   display.setCursor(char_x,char_y);
   display_write_str(SettingTable[3],1);


// Zeile 3 Mix-Zeile
   char_y= (posregister[2][0] & 0xFF00)>> 10;
   char_x = posregister[2][0] & 0x00FF;
   display.setCursor(char_x,char_y);
   display_write_str(SettingTable[4],1);

   char_y= (posregister[2][1] & 0xFF00)>> 10;
   char_x = posregister[2][1] & 0x00FF;
   display.setCursor(char_x,char_y);
   display_write_str(SettingcodeTable[0],1);


    //Serial.printf("refreshsettingscreen mit homescreen end\n");
  }
  display.display();
return;



  //resetRegister();
  blink_cursorpos=0xFFFF;

  //Serial.printf("refreshsettingscreen start\n");
  posregister[0][0] = 0   | (2 << 10); // Name Modell
  posregister[0][1] = 98  | (1 << 10);// Laufzeit
 
   posregister[0][2] =  itemtab[5] |    (3 << 10); // settingtext
   posregister[0][3] =  itemtab[7] |    (3 << 10); // settingnummer
   
   posregister[1][0] =  itemtab[0] |    (32 << 10); // Kanaltext
   
   posregister[2][0] =  itemtab[0] |    (42 << 10); // mixtext

   posregister[3][0] =  itemtab[0] |    (6 << 10); // Zuteilungtext
 
   posregister[4][0] =  itemtab[0] |    (7 << 10); // Zuteilungtext
   
   cursorpos[0][0] = cursortab[0] |    (2 << 10); // modellcursor lo: tab hi: page
   // cursorpos fuer model zeile/colonne
   
   cursorpos[0][1] = cursortab[5] |    (3 << 10); //  cursorpos fuer setting
   cursorpos[1][0] = cursortab[0] |    (4 << 10);  // cursorpos fuer kanal
   cursorpos[2][0] = cursortab[0] |    (5 << 10);  // cursorpos fuer mix
   cursorpos[3][0] = cursortab[0] |    (6 << 10);  // cursorpos fuer zuteilung
   cursorpos[4][0] = cursortab[0] |    (7 << 10);  // cursorpos fuer zuteilung
  
   

   char_x=itemtab[0];
   char_y = 1;
   char_height_mul = 1;
   char_width_mul = 1;
   display_write_str(SettingTable[0],1);
   char_height_mul = 2;
   char_width_mul = 1;
   
   // Modell Name
   
   //display_write_prop_str(char_y,char_x,0,menubuffer,2);
   
   char_y= (posregister[0][0] & 0xFF00)>> 10;
   char_x = posregister[0][0] & 0x00FF;

   display_write_str(ModelTable[curr_model],1);
   char_height_mul = 2;
   char_width_mul = 1;

   char_y= (cursorpos[0][0] & 0xFF00)>> 10;
   char_x = cursorpos[0][0] & 0x00FF;
   //display_write_symbol(pfeilvollrechts);
   
   // 2. Zeile Set mit Nummer
   char_y= (posregister[0][2] & 0xFF00)>> 10;
   char_x = posregister[0][2] & 0x00FF;
   
   char_height_mul = 1;
   display_write_str(SettingTable[2],2);
   

   // Kanal-Zeile
   char_y= (posregister[1][0] & 0xFF00)>> 10;
   char_x = posregister[1][0] & 0x00FF;
   //char_x=0;
   display_write_str(SettingTable[3],2);
   
   char_y= (posregister[1][1] & 0xFF00)>> 10;
   char_x = posregister[1][1] & 0x00FF;
//   display_write_int(curr_kanal,2);

   
   // Mix-Zeile
   char_y= (posregister[2][0] & 0xFF00)>> 10;
   char_x = posregister[2][0] & 0x00FF;
   //char_x=0;
   display_write_str(SettingTable[4],2);
   
   // Zuteilung-Zeile
   char_y= (posregister[3][0] & 0xFF00)>> 10;
   char_x = posregister[3][0] & 0x00FF;
   //char_x=0;
   display_write_str(SettingTable[5],2);
 
   // Output-Zeile
   char_y= (posregister[4][0] & 0xFF00)>> 10;
   char_x = posregister[4][0] & 0x00FF;
   //char_x=0;
   display_write_str(SettingTable[6],2);
   display.display();
   //Serial.printf("refreshsettingscreen end\n");
}

void display_write_str(const char *str, uint8_t prop)
{
  if(prop == 2)
  {
    display.setTextSize(2);
    display.write(str);
    display.setTextSize(1);
  }
  else
  {
    display.setTextSize(1);
    display.write(str);
  }
      

}

void display_write_byte(unsigned char data) 
{
      display.print(data);
 
 }

void display_go_to (unsigned char x, unsigned char y)
{
   
	display_write_byte(DISPLAY_PAGE_ADDRESS | ((y) & 0x0F));  
	display_write_byte(DISPLAY_COL_ADDRESS_MSB | ((x>>4) & 0x0F));
	display_write_byte(DISPLAY_COL_ADDRESS_LSB | ((x) & 0x0F));
	
}


void display_write_symbol(const char* symbol)
{
   unsigned char col,page,tmp1,tmp2,tmp3,tmp4,counter;
   const char* pointer = symbol;
   uint8_t symboldelay = 2;
   uint8_t count = 0;
   col=(char_x+DISPLAY_OFFSET);
   page=char_y;
   display.print(symbol);
      return;

	for(col=(char_x+DISPLAY_OFFSET);col<(char_x+(FONT_WIDTH*char_width_mul)+DISPLAY_OFFSET);col=col+char_width_mul)
	{
		for (page=char_y;page<(char_y+((FONT_HEIGHT/8)*char_height_mul));page = page +char_height_mul)
		{
			tmp1 = (symbol[count++]);
			
			if (char_height_mul > 1)
			{
				tmp2 = (tmp1&0xf0)>>4;
				tmp1 = tmp1 & 0x0f;
				
				tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
				tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
				
				if (char_height_mul>2)
				{
					tmp3 = tmp2;
					tmp2 = (tmp1&0xf0)>>4;
					tmp1 = tmp1 & 0x0f;
               
					tmp1 = ((tmp1&0x01)*3)+(((tmp1&0x02)<<1)*3)+(((tmp1&0x04)<<2)*3)+(((tmp1&0x08)<<3)*3);
					tmp2 = ((tmp2&0x01)*3)+(((tmp2&0x02)<<1)*3)+(((tmp2&0x04)<<2)*3)+(((tmp2&0x08)<<3)*3);
					
               
					tmp4 = (tmp3&0xf0)>>4;
					tmp3 = tmp3 & 0x0f;
               
					tmp3 = ((tmp3&0x01)*3)+(((tmp3&0x02)<<1)*3)+(((tmp3&0x04)<<2)*3)+(((tmp3&0x08)<<3)*3);
					tmp4 = ((tmp4&0x01)*3)+(((tmp4&0x02)<<1)*3)+(((tmp4&0x04)<<2)*3)+(((tmp4&0x08)<<3)*3);
					
					display_go_to(col,page+1);
					for(counter = 0;counter<char_width_mul;counter++)
					{
						display_write_byte(tmp3);
					}
					
					display_go_to(col,page+2);
					for(counter = 0;counter<char_width_mul;counter++)
					{
						display_write_byte(tmp4);
					}
				}
				display_go_to(col,page);
				
				for(counter = 0;counter<char_width_mul;counter++)
				{
					display_write_byte(tmp2);
				}
			}
			
			display_go_to(col,page-1);
			for(counter = 0;counter<char_width_mul;counter++)
			{
				display_write_byte(tmp1);
			}
		}
	}
	
	if (char_x < (128 + DISPLAY_OFFSET))
	{
		char_x = char_x + (FONT_WIDTH*char_width_mul);
	}
	return;
}

void display_setcursorblink(uint8_t zeit)
{
   uint16_t cursorposition = cursorpos[curr_cursorzeile][curr_cursorspalte];
   char_y= (cursorposition & 0xFF00)>> 10;
   char_x = cursorposition & 0x00FF;
      if (zeit%2) // gerade
      {
        pfeilvollrechts(char_x, char_y, 1);
         //display_write_symbol(pfeilwegrechts);
      }
      else
      {
        pfeilvollrechts(char_x, char_y, 0);
         //display_write_symbol(pfeilvollrechts);
      }

   
}

void drawverticalrect(void) 
{
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2; i+=2) {
    display.drawRect(i, i, display.width()-2*i, display.height()-2*i, WHITE);
    display.display(); // Update screen with each newly-drawn rectangle
    delay(1);
  }

  
}

void drawlevelmeter(uint8_t x,uint8_t y,uint8_t w,uint8_t h, uint8_t level)
{
  /*
  level in Volt
  Beginn bei 2V
  Marke bei 2.5V 11px
  */
    uint8_t mark = 11;
    display.drawRect(x, y, w, h, WHITE);
    uint16_t anzeige = level; //*h/100;
    display.fillRect(x+1,y+1,w-2,h-anzeige-2,0); // oberen teil leeren
    display.fillRect(x,y+h-anzeige+1,w,anzeige,WHITE);

    display.drawPixel(x-1,y+h-mark,WHITE);
    display.drawLine(x+1,y+h-mark,x+w-2,y+h-mark,0);
    display.drawPixel(x+w,y+h-mark,WHITE);


    //display.display(); // Update screen with each newly-drawn rectangle
}

void pfeilvollrechts(uint8_t x, uint8_t y, uint8_t full)
{
  if(full)
  {
    display.fillTriangle(x, y, x+4, y+4, x, y+8, WHITE);
  }
  else
  {
    display.fillTriangle(x, y, x+4, y+4, x, y+8, BLACK);
  }
}

void putint(uint8_t zahl)
{
  char string[4];
  int8_t i;                             // schleifenzähler
  int8_t leer = 0; 
  if(zahl < 100)
  {
    if (zahl < 10)
    {
      leer = 2;
    }
    else
    {
       leer = 1;
    }
  }
  string[3]='\0';                       // String Terminator
  for(i=2; i>=0; i--) 
  {
    if (i < leer)
    {
      string[i]=' '; 
    }
    else
    {
    string[i]=(zahl % 10) +'0';         // Modulo rechnen, dann den ASCII-Code von '0' addieren
    }
    zahl /= 10;
  }
  display.print(string);
}

void putint2(uint8_t zahl)
{
  char string[3];
  int8_t i;                             // schleifenzähler
  int8_t leer = 0; 
  
    if (zahl < 10)
    {
      leer = 2;
    }
    else
    {
       leer = 1;
    }
  
  string[2]='\0';                       // String Terminator
  for(i=1; i>=0; i--) 
  {
    
    if (i < leer)
    {
      string[i]=' '; 
    }
    else
    {
    string[i]=(zahl % 10) +'0';         // Modulo rechnen, dann den ASCII-Code von '0' addieren
    }
    zahl /= 10;

  }
  display.print(string);
}

void putint12(uint16_t zahl)
{
  char string[5];
  int8_t i;                             // schleifenzähler
  int8_t leer = 0; 
  if(zahl < 100)
  {
    if (zahl < 10)
    {
      leer = 2;
    }
    else
    {
       leer = 1;
    }
  }
  string[4]='\0';                       // String Terminator
  for(i=3; i>=0; i--) 
  {
    if (i < leer)
    {
      string[i]=' '; 
    }
    else
    {
    string[i]=(zahl % 10) +'0';         // Modulo rechnen, dann den ASCII-Code von '0' addieren
    }
    zahl /= 10;
  }
  display.print(string);
}

void puthex(uint8_t zahl)
{
  char string[3];
   uint8_t l,h;                             // schleifenzähler
   
   string[2]='\0';                       // String Terminator
   l=(zahl % 16);
   if (l<10)
   string[1]=l +'0';  
   else
   {
      l%=10;
      string[1]=l + 'A'; 
      
   }
   zahl /=16;
   h= zahl % 16;
   if (h<10)
   string[0]=h +'0';  
   else
   {
      h%=10;
      string[0]=h + 'A'; 
   }
  display.print(string); 
}


void display_write_stopzeit(uint8_t sekunde,uint8_t minute,uint8_t prop)
{
  
   
   char tempbuffer[6]={};
   tempbuffer[0] =minute/10+'0';
   tempbuffer[1] =minute%10+'0';
   tempbuffer[2] =':';
   tempbuffer[3] =sekunde/10+'0';
   tempbuffer[4] =sekunde%10+'0';
   tempbuffer[5] = '\0';
   display.setTextSize(2);
   //display.setFont(&FreeSans9pt7b);
   display.print(tempbuffer);
   display.setTextSize(1);
   
}
void display_write_laufzeit(uint8_t sekunde,uint8_t minute)
{
  display.setTextSize(1);
  
   
   char tempbuffer[6]={};
   tempbuffer[0] =minute/10+'0';
   tempbuffer[1] =minute%10+'0';
   tempbuffer[2] =':';
   tempbuffer[3] =sekunde/10+'0';
   tempbuffer[4] =sekunde%10+'0';
   tempbuffer[5] = '\0';
   display.setTextSize(1);
   //display.setFont(&FreeSans9pt7b);
   display.print(tempbuffer);
   
   
}