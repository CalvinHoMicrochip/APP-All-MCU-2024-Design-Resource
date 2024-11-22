
#include "OLED128x64.h"
#include "OLED_FONTs.c"


#define ReverseScreen

volatile unsigned char TxBuffer[16] ;

//extern void ReadEncoder(void);

void OLED_Init(void)
{
    
    DELAY_milliseconds(50); // MCC Delay added by Calvin - Must import DELAY foundation service 
    // __delay_us(50);  Original statement Source 
#ifndef ReverseScreen
    OLEDWrCmd(0xAE);      // display off

    OLEDWrCmd(0xA8);      // multiplex ratio mode set
    OLEDWrCmd(0x3F);      // duty = 1/64

    OLEDWrCmd(0xD3);      // set display offset
    OLEDWrCmd(0x00);      // mapping display start line to one of COM0-63

    OLEDWrCmd(0xB0);      // set page address (B0H - B7H))
    OLEDWrCmd(0x00);      // set lower column address
    OLEDWrCmd(0x10);      // set higher column address

//    OLEDWrCmd(0x20);    //set addressing mode
//    OLEDWrCmd(0x00); 			//set horizontal addressing mode 
    
    OLEDWrCmd(0x40);      //set display start line (40H - 7FH)

    OLEDWrCmd(0x81);      // contrast control mode set
    OLEDWrCmd(0xFF);      // set eight bits of data to the contrast data register

    OLEDWrCmd(0xA0);      // Set Segment Re-map: (A0H - A1H)

    OLEDWrCmd(0xC0);      // Set Common Output Scan Direction: (C0H - C8H)

    OLEDWrCmd(0xA6);      // Set Normal/Reverse Display: (A6H -A7H)

    OLEDWrCmd(0xD5);      // Divide Ratio/Oscillator Frequency Mode Set
    OLEDWrCmd(0x50);      // Divide Ratio/Oscillator Frequency Data Set: (00H - FFH)

    OLEDWrCmd(0xD9);      // Pre-charge Period Mode Set
    OLEDWrCmd(0xf1);      // Pre-charge Period Adjust: (A3 - A0), Dis-charge Period Adjust: (A7 - A4)

    OLEDWrCmd(0xDA);      // Common Pads Hardware Configuration Mode Set
    OLEDWrCmd(0x12);      // Sequential/Alternative Mode Set: (02H - 12H)

    OLEDWrCmd(0xDB);      // VCOM Deselect Level Mode Set
    OLEDWrCmd(0x35);      // VCOM Deselect Level Data Set: (00H - FFH)

//    OLEDWrCmd(Deactivate_Scroll_Cmd);    // 0x2E

//    OLEDWrCmd(0x8D);      // 0x8D set charge pump enable
//    OLEDWrCmd(0x14);      // 0x14   

    OLEDWrCmd(0xAF);      // Display OFF/ON: (AEH - AFH)
#else
    OLEDWrCmd(0xAE);   //display off
	OLEDWrCmd(0x20);	//Set Memory Addressing Mode	
	OLEDWrCmd(0x10);	//00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	OLEDWrCmd(0xB0);	//Set Page Start Address for Page Addressing Mode,0-7
	OLEDWrCmd(0xC8);	//Set COM Output Scan Direction
	OLEDWrCmd(0x00);    //---set low column address
	OLEDWrCmd(0x10);    //---set high column address
	OLEDWrCmd(0x40);    //--set start line address
	OLEDWrCmd(0x81);    //--set contrast control register
	OLEDWrCmd(0x7F);
	OLEDWrCmd(0xA1);    //--set segment re-map 0 to 127
	OLEDWrCmd(0xA6);    //--set normal display
	OLEDWrCmd(0xA8);    //--set multiplex ratio(1 to 64)
	OLEDWrCmd(0x3F);    //
	OLEDWrCmd(0xA4);    //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	OLEDWrCmd(0xD3);    //-set display offset
    
	OLEDWrCmd(0x00);    //-not offset
	OLEDWrCmd(0xD5);    //--set display clock divide ratio/oscillator frequency
	OLEDWrCmd(0xF0);    //--set divide ratio
	OLEDWrCmd(0xD9);    //--set pre-charge period
	OLEDWrCmd(0x22);    //
	OLEDWrCmd(0xDA);    //--set com pins hardware configuration
	OLEDWrCmd(0x12);
	OLEDWrCmd(0xDB);    //--set vcomh
	OLEDWrCmd(0x20);    //0x20,0.77xVcc
	OLEDWrCmd(0x8D);    //--set DC-DC enable
	OLEDWrCmd(0x14);    //
	OLEDWrCmd(0xAF);    //--turn on oled panel 
#endif    
}

/********** copy from LQ12864 **********/

/******** display 16*16 matrix, coordinate(x,y) y range 0~7 *********/
void OLED_Put16x16Ch(uint8_t x, uint8_t y, uint8_t index)
{
	unsigned char     i, j;
    
    i= index;                       // point to character which will be put to LCD
	OLED_Set_Pos(x, y);
	for(j=0; j<16; j++)              // upper part of Chinese character
	{
		OLEDWrDat(Font16x16[i][j]);
	}
	OLED_Set_Pos(x, (unsigned char)(y+1) );
	for(j=16; j<32; j++)            // lower part of Chinese character
	{
		OLEDWrDat(Font16x16[i][j]);
	} 	  	
}

/*********************set OLED display location************************************/
void OLED_Set_Pos(uint8_t x, uint8_t y) 
{ 
	OLEDWrCmd((uint8_t) (0xb0+y) );
	OLEDWrCmd((uint8_t)(((x&0xf0)>>4)|0x10));
	OLEDWrCmd((uint8_t)((x&0x0f)|0x01));
}

/********** display 6*8 ASCII character at coordinate(x,y), y range 0~7 ***********/
void OLED_Put6x8Str(uint8_t x, uint8_t y, const uint8_t ch[])
{
	uint8_t c, i, j;
    
    j = 0;
	while (ch[j]!='\0')
	{
		c = (uint8_t) (ch[j] - 0x20);
		if(x>126) {
            x=0;
            y++;
        }
		OLED_Set_Pos(x,y);
		for(i=0; i<6; i++)
            OLEDWrDat(Font6x8[c][i]);
		x += 6;
		j++;
	}
}

/******* display 8*16 ASCII character at coordinate(x,y), y range 0~7 *******/
void OLED_Put8x16Str(uint8_t x, uint8_t y, const uint8_t ch[])
{
	uint8_t c, i, j;
    
    j = 0;
	while (ch[j]!='\0')
	{
		c = (uint8_t) (ch[j] - 0x20);    // ASCII not start from 0, it start from 32 (20H) to 126 (7EH)
		if(x>120) {
            x=0;
            y++;
        }
		OLED_Set_Pos(x, y);             // upper part of the ASCII character
		for(i=0; i<8; i++) {
            OLEDWrDat(Font8x16[c][i]);
//            ReadEncoder();
        }
		OLED_Set_Pos(x, (uint8_t) (y+1));
		for(i=8; i<16; i++) {            // lower part of the ASCII character
            OLEDWrDat(Font8x16[c][i]);
//            ReadEncoder();
        }    
		x += 8; // 8x16 font so x move 8 for next character
		j++;    // next character
	}
}

/******* display 8*16 ASCII character at coordinate(x,y), y range 0~7 *******/
void OLED_Put8x16ASCII(uint8_t x, uint8_t y, uint8_t no, uint8_t data[])
{
	uint8_t cIndex, i, j;
    
    j = 0;
	while (j < no)
	{
		cIndex = (uint8_t) (data[j] - 0x20);    // ASCII not start from 0, it start from 32 (20H) to 126 (7EH)
		if(x>120) {
            x=0;
            y++;
        }
		OLED_Set_Pos(x, y);             // upper part of the ASCII character
		for(i=0; i<8; i++) {
            OLEDWrDat(Font8x16[cIndex][i]);
//            ReadEncoder();
        }
		OLED_Set_Pos(x, (uint8_t) (y+1));
		for(i=8; i<16; i++) {            // lower part of the ASCII character
            OLEDWrDat(Font8x16[cIndex][i]);
//            ReadEncoder();
        }
		x += 8; // 8x16 font so x move 8 for next character
		j++;    // next data byte
	}
}


void OLED_CLS(void)
{
	uint8_t     x, y;
    
	for(y=0; y<8; y++)
	{
		OLEDWrCmd((uint8_t) (0xb0+y));
		OLEDWrCmd(0x00);
		OLEDWrCmd(0x10);
		for(x=0;x<X_WIDTH;x++)
            OLEDWrDat(0);
	}
}

/***** display BMP picture, initial coordinate(x,y), x range 0~127, y range 0~7 *****/
void Draw_BMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t BMP[])
{
	uint16_t j;
	uint8_t x, y;

//    if(y1%8 == 0)   y = y1 >> 3;    //y1/8;      
//    else            y= (y1 >> 3) + 1; //y1/8+1;
    j = 0;
	for(y=y0; y<y1; y++)
	{
		OLED_Set_Pos(x0,y);
        for(x=x0; x<x1; x++) {      
	    	OLEDWrDat(BMP[j++]);
	    }
	}
}

/********************* End of LQ12864 ******************/

void  OLEDWrCmd(uint8_t command)
{
       TxBuffer[0] = OLED_Command_Mode;
       TxBuffer[1] = command;
        I2C1_Host.Write(OLED_ADDRESS, TxBuffer, 2);
        while (I2C1_Host.IsBusy()) ;    
}

void  OLEDWrDat(uint8_t data)
{
       TxBuffer[0] = OLED_Data_Mode;
       TxBuffer[1] = data;
        I2C1_Host.Write(OLED_ADDRESS, TxBuffer, 2);
        while (I2C1_Host.IsBusy()) ;    
}

void displayOn(void)
{
    OLEDWrCmd(0xAF);        //display on
}

// Turns display off.
void displayOff(void)
{
    OLEDWrCmd(0xAE);		//display off
}

// the most basic function, set a single pixel
//void drawPixel(uint8_t x, uint8_t y, uint8_t color) 
//{
//    if ((x < 0) || (x >= 128) || (y < 0) || (y >= 64))
//        return;
//
//    // x is which column
//    if (color == WHITE) 
//        BMP_buffer[x+ (y/8)*128] |= _BV((y%8));  
//    else
//        BMP_buffer[x+ (y/8)*128] &= ~_BV((y%8)); 
//}

