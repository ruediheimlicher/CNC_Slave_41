//
//  display.h
//  DOG_LCD
//
//  Created by Ruedi Heimlicher on 01.03.2023.
//
//
#include <stdint.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>



extern Adafruit_SSD1306 display;

#define DISPLAY_OFFSET 2

#define HOMESCREEN      0
#define SETTINGSCREEN   1
#define KANALSCREEN     2
#define LEVELSCREEN     3
#define EXPOSCREEN      4
#define MIXSCREEN       5


#define cursortab0 2
#define cursortab1 26
#define cursortab2 46
#define cursortab3 54
#define cursortab4 66
#define cursortab5 78
#define cursortab6 90
#define cursortab7 100

#define itemtab0  10
#define itemtab1  34
#define itemtab2  50
#define itemtab3  62
#define itemtab4  74
#define itemtab5  88
#define itemtab6  110
#define itemtab7  118


#define FONT_WIDTH 	6


#define DISPLAY_OFFSET		2
#define DISPLAY_COL_ADDRESS_LSB		0x00  
#define DISPLAY_PAGE_ADDRESS		0xB0
#define DISPLAY_COL_ADDRESS_MSB		0x10

#define ZEILENABSTAND 10


extern  uint8_t       curr_levelarray[8];
extern  uint8_t       curr_expoarray[8];
extern  uint8_t       curr_mixarray[8];
extern  uint8_t       curr_funktionarray[8];
extern  uint8_t       curr_statusarray[8];
extern  uint8_t       curr_ausgangarray[8];

extern  uint8_t       curr_devicearray[8];

extern  int8_t        curr_trimmungarray[8];

extern  uint8_t       curr_mixstatusarray[8];
extern  uint8_t       curr_mixkanalarray[8];

extern  uint8_t       curr_screen;
extern  uint8_t       curr_page; // aktuelle page
extern  uint8_t       curr_col; // aktuelle colonne
extern uint8_t       curr_model; // aktuelles modell

extern  uint8_t       curr_kanal; // aktueller kanal

extern  uint8_t       curr_richtung; // aktuelle richtung
extern  uint8_t       curr_impuls; // aktueller impuls

extern  uint8_t       eepromsavestatus;
extern  uint8_t       programmstatus;
extern  uint8_t       servostatus;

extern  uint8_t       curr_setting;
extern  uint8_t       curr_cursorzeile; // aktuelle zeile des cursors
extern  uint8_t       curr_cursorspalte; // aktuelle colonne des cursors

extern  uint8_t       last_cursorzeile; // letzte zeile des cursors
extern  uint8_t       last_cursorspalte; // letzte colonne des cursors
extern  uint16_t      blink_cursorpos;


void clearblock(uint8_t startx, uint8_t starty, uint8_t rangex,uint8_t rangey);
void clearline(uint8_t starty);


// 
extern  uint16_t              updatecounter; // Zaehler fuer Einschalten


extern uint8_t char_height_mul;
extern uint8_t char_width_mul;


uint8_t refresh_screen(void);
void sethomescreen(void);
void refreshhomescreen(void);
void resetRegister(void);
void setsettingscreen(void);
void refreshsettingscreen(void);
void setausgangscreen(void);
void setcanalscreen(void);
void settrimmscreen(void);
void setmixscreen(void);
void setsavescreen(void);
void display_setcursorblink(uint8_t zeit);
void display_write_str(const char *str, uint8_t prop);
void display_write_byte(unsigned char data);
void display_write_symbol(const char* symbol);
void display_write_stopzeit(uint8_t sekunde,uint8_t minute,uint8_t prop);
void display_write_laufzeit(uint8_t sekunde,uint8_t minute);
void drawverticalrect(void);

void putint(uint8_t zahl);
void putint2(uint8_t zahl);
void putint12(uint16_t zahl);
void puthex(uint8_t zahl);

void drawlevelmeter(uint8_t x,uint8_t y,uint8_t w,uint8_t h, uint8_t level);

void pfeilvollrechts(uint8_t x, uint8_t y, uint8_t full);