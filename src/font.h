
	#include <inttypes.h>

		#define FONT_WIDTH 	6
		#define FONT_HEIGHT 8  //8, 16, 32, 64, 128



/*
const char  pfeilvollrechtsbig[]={0x00,0x00,0xFE,0x7C,0x38,0x10};
const char  pfeilvollrechts[]={0x00,0x00,0x7F,0x3E,0x1C,0x08};

const char  pfeilvollrechtsklein[]={0x00,0x00,0x7C,0x38,0x10,0x00};


const char  pfeilleerrechts[]={0x00,0x00,0x7F,0x22,0x14,0x08};
//const char  pfeilleerrechts[]={0x00,0x00,0x82,0x44,0x28,0x10};
const char  richtungdown[]={0x00,0x04, 0x0C, 0x1C, 0x0C, 0x04};
const char  richtungup[]={0x00,0x20, 0x30, 0x38, 0x30, 0x20};
const char  richtungright[]={0x00,0x3E, 0x3E, 0x1C, 0x1C, 0x08};
const char  richtungleft[]={0x00,0x08, 0x1C, 0x1C, 0x3E, 0x3E};

const char  proprichtungdown[]= {10, 0x00, 0x04, 0x0C, 0x1C, 0x3F, 0x7F, 0x3F, 0x1C, 0x0C, 0x04};
const char  proprichtungup[]=   {10, 0x00, 0x10, 0x18, 0x1C, 0x7E, 0x7F, 0x7E, 0x1C, 0x18, 0x10};
const char  proprichtungright[]={10, 0x00, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x7F, 0x3E, 0x1C, 0x08};
const char  proprichtungleft[]= {10, 0x00, 0x08, 0x1C, 0x3E, 0x7F, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C};

const char  fragezeichen[]= {6,0x00,0x02,0x01,0x59,0x09,0x06};
*/


const char  pfeilwegrechts[]={0x00,0x00,0x00,0x00,0x00,0x00};


const char  diaga[]={0x08,0x02,0x04,0x08,0x10,0x20,0x40,0x80};


const char  testchar[] = {0x77, 0x1C, 0x17, 0x2E, 0x6A, 0x3E, 0x2B, 0x3A};
//	#if defined FONT2



const char  pitch[]=      {6,0x00, 0x80, 0xC7, 0xFF, 0xC7, 0x80, 0x00, 0x00, 0x00};
const char  schieber[]=   {9,0x40 ,0x4C ,0x7C ,0x4C ,0x40 ,0x40 ,0x40 ,0x40 ,0x40};
const char  schalter[]=   {8,0x00 ,0x40 ,0x40 ,0x40 ,0x60 ,0x50 ,0x4C ,0x4C, 0x00};

const char  steuertyp[3][10] =
{
   {6,0x00, 0x80, 0xC7, 0xFF, 0xC7, 0x80, 0x00, 0x00, 0x00},
   {9,0x40 ,0x4C ,0x7C ,0x4C ,0x40 ,0x40 ,0x40 ,0x40 ,0x40},
   {8,0x00 ,0x40 ,0x40 ,0x40 ,0x60 ,0x50 ,0x4C ,0x4C, 0x00}
};
