#ifndef LIGHT_SENSOR_H
#define LIGHT_SENSOR_H

#include <stdint.h>

int  LightSensor_Init(const char *spidev, int channel, double vref_v);
int  LightSensor_ReadRaw(uint16_t *raw12);
int  LightSensor_ReadVolts(double *volts);
int  LightSensor_ReadVoltsAvg(int n, double *volts_avg);
void LightSensor_Close(void);


#endif 