 /*
 * MAIN Generated Driver File
 * 
 * @file main.c
 * 
 * @defgroup main MAIN
 * 
 * @brief This is the generated driver implementation file for the MAIN driver.
 *
 * @version MAIN Driver Version 1.0.2
 *
 * @version Package Version: 3.1.2
*/

/*
? [2024] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/
#include "mcc_generated_files/system/system.h"
#include "mcc_generated_files/timer/delay.h"
#include "OLED128x64.h"
#include "bme280.h"

#define VEML7700_ADDR   0x10 
#define MCP9808_ADDR    0x1f
#define BME280_ADDR     0x76
#define BMI323_ADDR     0x68 
#define KXTJ3_ADDR      0x0e
#define MCP79411_ADDR   0x6f

#define DEFAULT_STANDBY_TIME        BME280_STANDBY_HALFMS
#define DEFAULT_FILTER_COEFF        BME280_FILTER_COEFF_OFF
#define DEFAULT_TEMP_OSRS           BME280_OVERSAMP_X1
#define DEFAULT_PRESS_OSRS          BME280_OVERSAMP_X1
#define DEFAULT_HUM_OSRS            BME280_OVERSAMP_X1
#define DEFAULT_SENSOR_MODE         BME280_FORCED_MODE

uint8_t  WS2812_Buffer[6] = {32,00,00,00,32,00};
uint8_t  WS2812_State = 0;

volatile bool           SW3_Released = true ;
volatile bool           LED_Shift = 1 ;
volatile bool           SW_Detected = false ;

unsigned char           ASCII_Buffer[20];
volatile unsigned char  I2C_Wbuffer[16];   
volatile unsigned char  I2C_Rbuffer[16] ;
unsigned int            LightingValue = 0 ;
unsigned int            MCP9808Value = 0 ;
unsigned int            ADCC_Result ;

float   BME280_Temp ;
float   BME280_Pressure ;
float   BME280_Humidity ;

union
{
    char   ByteVal[2];
    int    WordValue ;
}KXTJ3_XOUT;

union
{
    char   ByteVal[2];
    int    WordValue ;
}KXTJ3_YOUT;

union
{
    char   ByteVal[2];
    int    WordValue ;
}KXTJ3_ZOUT;


uint8_t  APP_2024_StateMachine = 0 ;

void    WS2812_Status_Update(void);
void    WS2812_Data_Update(void);
void    Mode0_Update(void);
void    Mode1_Update(void);
void    Mode2_Update(void);
void    WeatherClick_readSensors(void);
void    WeatherStation_initialize(void);


/*
    Main application
*/

int main(void)
{
    SYSTEM_Initialize();
    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts 
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global Interrupts 
    // Use the following macros to: 

    // Enable the Global Interrupts 
    INTERRUPT_GlobalInterruptEnable(); 

    // Disable the Global Interrupts 
    //INTERRUPT_GlobalInterruptDisable(); 
        DELAY_milliseconds(200);
        OLED_Init();      
        OLED_CLS(); 
        OLED_Put8x16Str(0,0,"PIC18F57Q43 M:0 ");
        OLED_Put8x16Str(0,2,"MCP9800:        ");
        OLED_Put8x16Str(0,4,"                ");        
        OLED_Put8x16Str(0,6,"                ");            
        
        // --------------------------------------------------------------------------------------
        // Doing On-Board Peripheral testing and initialize
        
                I2C_Wbuffer[0] = 0xd0 ;     // Read ID information from BME280
                I2C1_Host.WriteRead(BME280_ADDR,I2C_Wbuffer,1,I2C_Rbuffer,1);
                while ( I2C1_Host.IsBusy()) ;
                
                if (I2C_Rbuffer[0] != 0x60)
                {
                    OLED_Put8x16Str(0,2,"BME280 Fail");
                    while (1) ;
                }

        // Activate Lighting Sensor
            I2C_Wbuffer[0] = 0x00 ;
            I2C_Wbuffer[1] = 0x00 ;
            I2C_Wbuffer[2] = 0x18 ;
            I2C1_Host.Write(VEML7700_ADDR,I2C_Wbuffer , 3) ;                
            while ( I2C1_Host.IsBusy()) ;
                
        // Initialize MCP9808
                I2C_Wbuffer[0] = 0x01 ;
                I2C_Wbuffer[1] = 0x00 ;
                I2C_Wbuffer[2] = 0x04 ;
                I2C1_Host.Write(MCP9808_ADDR,I2C_Wbuffer,2);
                while ( I2C1_Host.IsBusy()) ;
        
                I2C_Wbuffer[0] = 0x02 ;
                I2C_Wbuffer[1] = 0x05 ;
                I2C_Wbuffer[2] = 0xa0 ;
                I2C1_Host.Write(MCP9808_ADDR,I2C_Wbuffer,3);
                while ( I2C1_Host.IsBusy()) ;
                
                I2C_Wbuffer[0] = 0x03 ;
                I2C_Wbuffer[1] = 0x00 ;
                I2C_Wbuffer[2] = 0x10 ;
                I2C1_Host.Write(MCP9808_ADDR,I2C_Wbuffer,3);
                while ( I2C1_Host.IsBusy()) ;   
                
                I2C_Wbuffer[0] = 0x04 ;
                I2C_Wbuffer[1] = 0x05 ;
                I2C_Wbuffer[2] = 0xa0 ;
                I2C1_Host.Write(MCP9808_ADDR,I2C_Wbuffer,3);
                while ( I2C1_Host.IsBusy()) ;             
                
        // Initialize KXTJ3
                I2C_Wbuffer[0] = 0x1b ;
                I2C_Wbuffer[1] = 0x80 ;
                I2C1_Host.Write(KXTJ3_ADDR,I2C_Wbuffer,2);
                while ( I2C1_Host.IsBusy()) ;                          
        // --------------------------------------------------------------------------------------
        // Finished On-Board Peripheral testing and initialize   
        // --------------------------------------------------------------------------------------
                
    WeatherStation_initialize();            
    SPI1_Host_Open(HOST_CONFIG);
    SPI1CON0bits.BMODE = 1 ;
    SPI1CON2bits.TXR = 1 ;
    SPI1CON2bits.RXR = 0 ;
    
    while(1)
    {
        DELAY_milliseconds(100);
        WS2812_Status_Update();
        WS2812_Data_Update();
        
        if (SWA_GetValue() && SWB_GetValue() && SWC_GetValue() && SWD_GetValue() && SWCNT_GetValue())
        {
            if  (SW_Detected == true)
            {
                LED1_SetLow();
                LED2_SetLow();
                LED3_SetLow();
                LED4_SetLow();
                SW_Detected = false ;
            }
                LED1_Toggle();
                LED2_Toggle();
                LED3_Toggle();
                LED4_Toggle() ;
            }
            
            if (!SWA_GetValue())    
                {
                    LED1_Toggle();
                    LED2_SetLow();
                    LED3_SetLow();
                    LED4_SetLow() ;     
                    SW_Detected = true ;
                }
            if (!SWB_GetValue())    
                {
                    LED1_SetLow();
                    LED2_Toggle();
                    LED3_SetLow();
                    LED4_SetLow() ;   
                    SW_Detected = true ;
                }
            if (!SWC_GetValue())    
                {
                    LED1_SetLow();
                    LED2_SetLow();
                    LED3_Toggle();
                    LED4_SetLow() ;   
                    SW_Detected = true ;
                }                
            if (!SWD_GetValue())   
                {
                    LED1_SetLow();
                    LED2_SetLow();
                    LED3_SetLow();
                    LED4_Toggle() ;          
                    SW_Detected = true ;
                }                
            if (!SWCNT_GetValue())  
                {
                    LED1_SetHigh();
                    LED2_SetHigh();
                    LED3_SetHigh();
                    LED4_SetHigh() ;    
                    SW_Detected = true ;
                }               
            
        
        
        if (!SW3_GetValue() && (SW3_Released == true)) 
        {
            SW3_Released = false;
            
            if (APP_2024_StateMachine ==2)
                    APP_2024_StateMachine = 0 ;
                else
                    APP_2024_StateMachine ++ ;
            
            switch (APP_2024_StateMachine)
            {
                case    0:
                    OLED_Put8x16Str(0,0,"PIC18F57Q43 M:0 ");                  
                    OLED_Put8x16Str(0,2,"                ");
                    OLED_Put8x16Str(0,4,"                ");        
                    OLED_Put8x16Str(0,6,"                ");                         
                    break ;
                case    1:
                    OLED_Put8x16Str(0,0,"PIC18F57Q43 M:1 ");                        
                    OLED_Put8x16Str(0,2,"MCP9800:        ");
                    OLED_Put8x16Str(0,4,"ADC:            ");        
                    OLED_Put8x16Str(0,6,"Lighting:       ");                             
                    break ;
                case    2:
                    OLED_Put8x16Str(0,0,"PIC18F57Q43 M:2 ");                    
                    OLED_Put8x16Str(0,2,"X =             ");
                    OLED_Put8x16Str(0,4,"Y =             ");        
                    OLED_Put8x16Str(0,6,"Z =             ");                         
                    break ;
            }
        }
        
        if (SW3_GetValue() && (SW3_Released == false))
                SW3_Released = true ;
        
        switch (APP_2024_StateMachine)
        {
            case 0:
                Mode0_Update();
                break ;
            case 1:
                Mode1_Update();
                break ;
            case 2:
                Mode2_Update();
                break ;
        }
    }    
}

void    Mode0_Update(void)
{               
           WeatherClick_readSensors();
            BME280_Temp = BME280_getTemperature();
            BME280_Pressure = BME280_getPressure();
            BME280_Humidity = BME280_getHumidity();
            
                sprintf (ASCII_Buffer,"Temp= %5.2f C", BME280_Temp) ;
                OLED_Put8x16Str(0,2,(uint8_t*)ASCII_Buffer) ;      
                    sprintf (ASCII_Buffer,"Hum.=%5.2f %c", BME280_Humidity,'%') ;
                    OLED_Put8x16Str(0,4,(uint8_t*)ASCII_Buffer) ;      
                        sprintf (ASCII_Buffer,"Pre.=%5.2f Kpa", BME280_Pressure) ;
                        OLED_Put8x16Str(0,6,(uint8_t*)ASCII_Buffer) ; 
                          
}

void    Mode1_Update(void)
{
                 // ---------------
                // -- Read MCP9808 
                // ---------------                
                I2C_Wbuffer[0] = 5 ;
                I2C1_Host.WriteRead(MCP9808_ADDR,I2C_Wbuffer,1,I2C_Rbuffer,2);
                while ( I2C1_Host.IsBusy()) ;
                MCP9808Value = (unsigned int)(I2C_Rbuffer[0] * 256) + I2C_Rbuffer[1];   
                MCP9808Value  = MCP9808Value >> 4 ;
                MCP9808Value &= 0x00ff ;    
                
                // ----------------------------------
                // -- Read VEML7700 to LightingValue
                // ----------------------------------                
                I2C_Wbuffer[0] = 4 ;
                I2C1_Host.WriteRead(VEML7700_ADDR,I2C_Wbuffer,1,I2C_Rbuffer,2);
                while ( I2C1_Host.IsBusy()) ;                               
                LightingValue = I2C_Rbuffer[0] + (I2C_Rbuffer[1]) * 256;
                
                ADCC_Result = ADCC_GetSingleConversion(channel_ANB0);

                
                // Display MCP9800 Temperature
                sprintf (ASCII_Buffer,"%3d",MCP9808Value) ;
                OLED_Put8x16ASCII(96,2,3,(uint8_t*)ASCII_Buffer) ;   
                // Display VEML7700 Lighting Sensor RAW Data
                sprintf (ASCII_Buffer,"%4d",LightingValue) ;
                OLED_Put8x16ASCII(88,6,4,(uint8_t*)ASCII_Buffer) ;   
                
                // Display VR input result
                sprintf (ASCII_Buffer,"%4d",ADCC_Result) ;
                OLED_Put8x16ASCII(88,4,4,(uint8_t*)ASCII_Buffer) ;   
                                
                
}

void    Mode2_Update(void)
{
                // -------------
                // -- Read KXTJ3 to KXTJ3_XOUT, KXTJ3_YOUT , KXTJ3_ZOUT
                // -------------
                I2C_Wbuffer[0] = 0x6 ;
                I2C1_Host.WriteRead(KXTJ3_ADDR,I2C_Wbuffer,1,I2C_Rbuffer,6);
                while ( I2C1_Host.IsBusy()) ;
                KXTJ3_XOUT.ByteVal[0] = I2C_Rbuffer[0];
                KXTJ3_XOUT.ByteVal[1] = I2C_Rbuffer[1];     
                    KXTJ3_YOUT.ByteVal[0] = I2C_Rbuffer[2];
                    KXTJ3_YOUT.ByteVal[1] = I2C_Rbuffer[3];      
                        KXTJ3_ZOUT.ByteVal[0] = I2C_Rbuffer[4];
                        KXTJ3_ZOUT.ByteVal[1] = I2C_Rbuffer[5];     
                        
                sprintf (ASCII_Buffer,"%3d",KXTJ3_XOUT.WordValue/256) ;
                OLED_Put8x16ASCII(40,2,3,(uint8_t*)ASCII_Buffer) ;  
                sprintf (ASCII_Buffer,"%3d",KXTJ3_YOUT.WordValue/256) ;
                OLED_Put8x16ASCII(40,4,3,(uint8_t*)ASCII_Buffer) ;  
                sprintf (ASCII_Buffer,"%3d",KXTJ3_ZOUT.WordValue/256) ;
                OLED_Put8x16ASCII(40,6,3,(uint8_t*)ASCII_Buffer) ;                  
            
                
}

void    WS2812_Status_Update(void)
{
    switch  (WS2812_State)
    {
        case 0:
            WS2812_Buffer[0] = 16;
            WS2812_Buffer[1] = 00;
            WS2812_Buffer[2] = 00;
            WS2812_Buffer[3] = 00;
            WS2812_Buffer[4] = 16;
            WS2812_Buffer[5] = 00;        
            WS2812_State = 1;
            break;
        case 1:
            WS2812_Buffer[0] = 00;
            WS2812_Buffer[1] = 16;
            WS2812_Buffer[2] = 00;
            WS2812_Buffer[3] = 16;
            WS2812_Buffer[4] = 00;
            WS2812_Buffer[5] = 00;                   
            WS2812_State = 0;
            break;
        case 2:
            WS2812_Buffer[0] = 00;
            WS2812_Buffer[1] = 00;
            WS2812_Buffer[2] = 32;
            WS2812_Buffer[3] = 00;
            WS2812_Buffer[4] = 00;
            WS2812_Buffer[5] = 32;                   
            WS2812_State = 3 ;
            break;
        case 3:
            WS2812_Buffer[0] = 32;
            WS2812_Buffer[1] = 00;
            WS2812_Buffer[2] = 00;
            WS2812_Buffer[3] = 32;
            WS2812_Buffer[4] = 00;
            WS2812_Buffer[5] = 00;                   
            WS2812_State = 0 ;
            break;
    }
}
void    WS2812_Data_Update(void)
{
            while(!SPI1_IsTxReady()) ;
            SPI1TXB = WS2812_Buffer[0];
             while(!SPI1_IsTxReady()) ;
            SPI1TXB = WS2812_Buffer[1];   
            while(!SPI1_IsTxReady()) ;
            SPI1TXB = WS2812_Buffer[2];  
            while(!SPI1_IsTxReady()) ;
            SPI1TXB = WS2812_Buffer[3];
             while(!SPI1_IsTxReady()) ;
            SPI1TXB = WS2812_Buffer[4];   
            while(!SPI1_IsTxReady()) ;
            SPI1TXB = WS2812_Buffer[5];                
    
}

void WeatherClick_readSensors(void) {
    if (DEFAULT_SENSOR_MODE == BME280_FORCED_MODE) {
        BME280_startForcedSensing();
    }
    BME280_readMeasurements();
}

void WeatherStation_initialize(void) 
{
    BME280_reset();
    DELAY_milliseconds(50);
    BME280_readFactoryCalibrationParams();
    BME280_config(BME280_STANDBY_HALFMS, BME280_FILTER_COEFF_OFF);
    
    // 設定為 FORCED Mode
    // BME280_ctrl_meas(BME280_OVERSAMP_X1, BME280_OVERSAMP_X1, BME280_FORCED_MODE);
    
    // 設定為 NORMAL Mode 
    BME280_ctrl_meas(BME280_OVERSAMP_X1, BME280_OVERSAMP_X1, BME280_NORMAL_MODE);
    BME280_ctrl_hum(BME280_OVERSAMP_X1);
    BME280_initializeSensor();
}