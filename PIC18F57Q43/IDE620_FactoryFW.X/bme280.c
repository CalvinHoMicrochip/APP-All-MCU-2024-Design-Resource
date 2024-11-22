/*
 * File:   bme280.c
 * Author: C16783
 *
 * Created on May 13, 2020, 11:56 AM
 */

#include "bme280.h"

    bme280_config_t bme280_config;
    uint8_t bme280_ctrl_hum;
    bme280_ctrl_meas_t bme280_ctrl_meas;
    bme280_calibration_param_t calibParam;
    long adc_T, adc_H, adc_P, t_fine;
    char str_temp[30];
    char str_press[30];
    char str_hum[30];
    char str_light[30];
    
// #include "mcc_generated_files/system/system.h"
extern unsigned char  I2C_Wbuffer[16];   
extern unsigned char  I2C_Rbuffer[16] ;

/**
  Section: Driver APIs
 */

uint8_t BME280_getID(void) {
    // TWI0_WriteRead(uint16_t address, uint8_t *writeData, size_t writeLength, uint8_t *readData, size_t readLength);
    I2C_Wbuffer[0] = BME280_ID_REG ;
    I2C1_Host.WriteRead(BME280_ADDR,I2C_Wbuffer,1,I2C_Rbuffer,1);      // Read One Byte from Address : BME280_ID_REG
    while (I2C1_Host.IsBusy()) ;
    return I2C_Rbuffer[0];
}

void BME280_reset(void) {
    // i2c_write1ByteRegister(BME280_ADDR, BME280_RESET_REG, BME280_SOFT_RESET);
    I2C_Wbuffer[0] = BME280_RESET_REG ;
    I2C_Wbuffer[1] = BME280_SOFT_RESET ;
    I2C1_Host.Write(BME280_ADDR,I2C_Wbuffer , 2) ;  
    while (I2C1_Host.IsBusy()) ;
}

void BME280_sleep(void) {
    bme280_ctrl_meas.mode = BME280_SLEEP_MODE;
    // i2c_write1ByteRegister(BME280_ADDR, BME280_CTRL_MEAS_REG, bme280_ctrl_meas.ctrlMeasReg);
    I2C_Wbuffer[0] = BME280_CTRL_MEAS_REG ;
    I2C_Wbuffer[1] = bme280_ctrl_meas.ctrlMeasReg ;
    I2C1_Host.Write(BME280_ADDR,I2C_Wbuffer , 2) ;  
    while (I2C1_Host.IsBusy()) ;
}

void BME280_readFactoryCalibrationParams(void) {
    uint8_t paramBuff[24];
    // i2c_readDataBlock(BME280_ADDR, BME280_CALIB_DT1_LSB_REG, paramBuff, 24);
    I2C_Wbuffer[0] = BME280_CALIB_DT1_LSB_REG ;
    I2C1_Host.WriteRead(BME280_ADDR,I2C_Wbuffer,1, paramBuff, 24);      // Read Calibration data 
    while (I2C1_Host.IsBusy()) ;
    calibParam.dig_T1 = (((uint16_t) paramBuff[1]) << 8) + paramBuff[0];
    calibParam.dig_T2 = (((int) paramBuff[3]) << 8) + paramBuff[2];
    calibParam.dig_T3 = (((int) paramBuff[5]) << 8) + paramBuff[4];
    calibParam.dig_P1 = (((uint16_t) paramBuff[7]) << 8) + paramBuff[6];
    calibParam.dig_P2 = (((int) paramBuff[9]) << 8) + paramBuff[8];
    calibParam.dig_P3 = (((int) paramBuff[11]) << 8) + paramBuff[10];
    calibParam.dig_P4 = (((int) paramBuff[13]) << 8) + paramBuff[12];
    calibParam.dig_P5 = (((int) paramBuff[15]) << 8) + paramBuff[14];
    calibParam.dig_P6 = (((int) paramBuff[17]) << 8) + paramBuff[16];
    calibParam.dig_P7 = (((int) paramBuff[19]) << 8) + paramBuff[18];
    calibParam.dig_P8 = (((int) paramBuff[21]) << 8) + paramBuff[20];
    calibParam.dig_P9 = (((int) paramBuff[23]) << 8) + paramBuff[22];

    I2C_Wbuffer[0] = BME280_CALIB_DH1_REG ;
    I2C1_Host.WriteRead(BME280_ADDR,I2C_Wbuffer,1, paramBuff, 1);  
    while (I2C1_Host.IsBusy()) ;
    //calibParam.dig_H1 = (uint8_t) i2c_read1ByteRegister(BME280_ADDR, BME280_CALIB_DH1_REG);
    calibParam.dig_H1 = paramBuff[0];
    
    // i2c_readDataBlock(BME280_ADDR, BME280_CALIB_DH2_LSB_REG, paramBuff, 7);
    I2C_Wbuffer[0] = BME280_CALIB_DH2_LSB_REG ;
    I2C1_Host.WriteRead(BME280_ADDR,I2C_Wbuffer,1, paramBuff, 7);  
    while (I2C1_Host.IsBusy()) ;
    calibParam.dig_H2 = (((int) paramBuff[1]) << 8) + paramBuff[0];
    calibParam.dig_H3 = (uint8_t) paramBuff[2];
    calibParam.dig_H4 = (((int) paramBuff[3]) << 4) | (paramBuff[4] & 0xF);
    calibParam.dig_H5 = (((int) paramBuff[5]) << 4) | (paramBuff[4] >> 4);
    calibParam.dig_H6 = (short) paramBuff[6];
}

void BME280_config(uint8_t sbtime, uint8_t coeff) {
    bme280_config.t_sb = sbtime; // Set standby time;
    bme280_config.filter = coeff; // Set filter coefficient;
}

void BME280_ctrl_meas(uint8_t osrs_T, uint8_t osrs_P, uint8_t mode) {
    bme280_ctrl_meas.osrs_T = osrs_T; // Set oversampling temperature;
    bme280_ctrl_meas.osrs_P = osrs_P; // Set oversampling pressure;
    bme280_ctrl_meas.mode = mode; // Set sensor mode;
}

void BME280_ctrl_hum(uint8_t osrs_H) {
    bme280_ctrl_hum = osrs_H; // Set oversampling humidity;
}

void BME280_initializeSensor(void) {
//    i2c_write1ByteRegister(BME280_ADDR, BME280_CONFIG_REG, bme280_config.configReg);
    I2C_Wbuffer[0] = BME280_CONFIG_REG ;
    I2C_Wbuffer[1] = bme280_config.configReg ;
    I2C1_Host.Write(BME280_ADDR,I2C_Wbuffer,2);  
    while (I2C1_Host.IsBusy()) ;
//    i2c_write1ByteRegister(BME280_ADDR, BME280_CTRL_HUM_REG, bme280_ctrl_hum);
    I2C_Wbuffer[0] = BME280_CTRL_HUM_REG ;
    I2C_Wbuffer[1] = bme280_ctrl_hum;
    I2C1_Host.Write(BME280_ADDR,I2C_Wbuffer,2);    
    while (I2C1_Host.IsBusy()) ;
//    i2c_write1ByteRegister(BME280_ADDR, BME280_CTRL_MEAS_REG, bme280_ctrl_meas.ctrlMeasReg);
    I2C_Wbuffer[0] = BME280_CTRL_MEAS_REG ;
    I2C_Wbuffer[1] = bme280_ctrl_meas.ctrlMeasReg ;
    I2C1_Host.Write(BME280_ADDR,I2C_Wbuffer,2);   
    while (I2C1_Host.IsBusy()) ;
    
}

void BME280_startForcedSensing(void) {
    bme280_ctrl_meas.mode = BME280_FORCED_MODE;
    // i2c_write1ByteRegister(BME280_ADDR, BME280_CTRL_MEAS_REG, bme280_ctrl_meas.ctrlMeasReg);
    I2C_Wbuffer[0] = BME280_CTRL_MEAS_REG ;
    I2C_Wbuffer[1] = bme280_ctrl_meas.ctrlMeasReg ;
    I2C1_Host.Write(BME280_ADDR,I2C_Wbuffer,2);   
    while (I2C1_Host.IsBusy()) ;
}

void BME280_readMeasurements(void) {
    uint8_t sensorData[BME280_DATA_FRAME_SIZE];

    // i2c_readDataBlock(BME280_ADDR, BME280_PRESS_MSB_REG, sensorData, BME280_DATA_FRAME_SIZE);
    I2C_Wbuffer[0] = BME280_PRESS_MSB_REG ;
    I2C1_Host.WriteRead(BME280_ADDR,I2C_Wbuffer,1, sensorData, BME280_DATA_FRAME_SIZE);      // Read Calibration data 
    while (I2C1_Host.IsBusy()) ;
    adc_H = ((uint32_t) sensorData[BME280_HUM_MSB] << 8) |
            sensorData[BME280_HUM_LSB];

    adc_T = ((uint32_t) sensorData[BME280_TEMP_MSB] << 12) |
            (((uint32_t) sensorData[BME280_TEMP_LSB] << 4) |
            ((uint32_t) sensorData[BME280_TEMP_XLSB] >> 4));

    adc_P = ((uint32_t) sensorData[BME280_PRESS_MSB] << 12) |
            (((uint32_t) sensorData[BME280_PRESS_LSB] << 4) |
            ((uint32_t) sensorData[BME280_PRESS_XLSB] >> 4));
}

float BME280_getTemperature(void) {
    float temperature = (float) BME280_compensateTemperature() / 100;
    return temperature;
}

float BME280_getPressure(void) {
    float pressure = (float) BME280_compensatePressure() / 1000;
    // 在海平面的平均氣壓約為101.325千帕斯卡
    // pressure = pressure * 0.295301;
    return pressure;
}

float BME280_getHumidity(void) {
    float humidity = (float) BME280_compensateHumidity() / 1024;
    return humidity;
}

/* 
 * Returns temperature in DegC, resolution is 0.01 DegC. 
 * Output value of "5123" equals 51.23 DegC.  
 */
static uint32_t BME280_compensateTemperature(void) 
{
    long tempV1, tempV2, t;

    tempV1 = ((((adc_T >> 3) - ((long) calibParam.dig_T1 << 1))) * ((long) calibParam.dig_T2)) >> 11;
    tempV2 = (((((adc_T >> 4) - ((long) calibParam.dig_T1)) * ((adc_T >> 4) - ((long) calibParam.dig_T1))) >> 12)*((long) calibParam.dig_T3)) >> 14;
    t_fine = tempV1 + tempV2;
    t = (t_fine * 5 + 128) >> 8;

    return t;
}

/* 
 * Returns pressure in Pa as unsigned 32 bit integer. 
 * Output value of "96386" equals 96386 Pa = 96.386 kPa 
 */
static uint32_t BME280_compensatePressure(void) 
{
    long pressV1, pressV2;
    uint32_t p;

    pressV1 = (((long) t_fine) >> 1) - (long) 64000;
    pressV2 = (((pressV1 >> 2) * (pressV1 >> 2)) >> 11) * ((long) calibParam.dig_P6);
    pressV2 = pressV2 + ((pressV1 * ((long) calibParam.dig_P5)) << 1);
    pressV2 = (pressV2 >> 2)+(((long) calibParam.dig_P4) << 16);
    pressV1 = (((calibParam.dig_P3 * (((pressV1 >> 2) * (pressV1 >> 2)) >> 13)) >> 3) +
            ((((long) calibParam.dig_P2) * pressV1) >> 1)) >> 18;
    pressV1 = ((((32768 + pressV1))*((long) calibParam.dig_P1)) >> 15);

    if (pressV1 == 0) {
        // avoid exception caused by division by zero
        return 0;
    }

    p = (((uint32_t) (((long) 1048576) - adc_P)-(pressV2 >> 12)))*3125;
    if (p < 0x80000000) {
        p = (p << 1) / ((uint32_t) pressV1);
    } else {
        p = (p / (uint32_t) pressV1) * 2;
    }

    pressV1 = (((long) calibParam.dig_P9) * ((long) (((p >> 3) * (p >> 3)) >> 13))) >> 12;
    pressV2 = (((long) (p >> 2)) * ((long) calibParam.dig_P8)) >> 13;
    p = (uint32_t) ((long) p + ((pressV1 + pressV2 + calibParam.dig_P7) >> 4));

    return p;
}

static uint32_t BME280_compensateHumidity(void) 
{
    long humV;
    uint32_t h;

    humV = (t_fine - ((long) 76800));
    humV = (((((adc_H << 14) - (((long) calibParam.dig_H4) << 20) - (((long) calibParam.dig_H5) * humV)) +
            ((long) 16384)) >> 15) * (((((((humV * ((long) calibParam.dig_H6)) >> 10) *
            (((humV * ((long) calibParam.dig_H3)) >> 11) + ((long) 32768))) >> 10) +
            ((long) 2097152)) * ((long) calibParam.dig_H2) + 8192) >> 14));
    humV = (humV - (((((humV >> 15) * (humV >> 15)) >> 7) * ((long) calibParam.dig_H1)) >> 4));
    humV = (humV < 0 ? 0 : humV);
    humV = (humV > 419430400 ? 419430400 : humV);

    h = (uint32_t) (humV >> 12);
    return h;
}


