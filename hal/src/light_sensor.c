#define _POSIX_C_SOURCE 200809L
#include "hal/light_sensor.h"

#include <linux/spi/spidev.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

static int      s_fd     = -1; // file descriptior
static int      s_ch     = 0;   // chip channel 0    
static double   s_vref   = 3.3;   // reference voltage to ADC
static uint32_t s_speed  = 1000000; // 1 MHz spi freq 

static struct spi_ioc_transfer g_tr_tmpl = {
    .len           = 3,
    .speed_hz      = 1000000,   
    .bits_per_word = 8,
    .cs_change     = 0,
    .delay_usecs   = 0,
};

static int mcp3208_xfer(int ch, uint16_t *out12) 
{
    if (s_fd < 0 || ch < 0 || ch > 7 || !out12) 
    {
        return -1;
    }
    uint8_t tx[3] = {0};
    uint8_t rx[3] = {0};
    tx[0] = 0x06 | ((ch & 0x04) >> 2);
    tx[1] = (uint8_t)((ch & 0x03) << 6);
    tx[2] = 0x00;

    
    struct spi_ioc_transfer tr = g_tr_tmpl;
    tr.tx_buf =(uintptr_t)tx;                
    tr.rx_buf= (uintptr_t)rx;
    tr.speed_hz= s_speed;                   

    if (ioctl(s_fd, SPI_IOC_MESSAGE(1), &tr) < 1) 
    {
        return -1;
    }

    *out12 = ((uint16_t)(rx[1] & 0x0F) << 8) | rx[2];
    return 0;
}

int LightSensor_Init(const char *spidev, int channel, double vref_v) {
    if (!spidev || channel < 0 || channel > 7 || vref_v <= 0.0) 
    {
        errno = EINVAL; return -1;
    }

    int fd = open(spidev, O_RDWR | O_CLOEXEC);
    if (fd < 0) return -1;

    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) { close(fd); return -1; }
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) { close(fd); return -1; }
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &s_speed) < 0) { close(fd); return -1; }

    s_fd = fd;
    s_ch = channel;
    s_vref = vref_v;
    return 0;
}

int LightSensor_ReadRaw(uint16_t *raw12) 
{
    return mcp3208_xfer(s_ch, raw12);
}

int LightSensor_ReadVolts(double *volts) 
{
    if (!volts) 
    { 
        errno = EINVAL; 
        return -1; 
    }
    uint16_t r;
    int rc = mcp3208_xfer(s_ch, &r);
    if (rc < 0) 
    {
        return rc;
    }
    *volts = (double)r * (s_vref / 4096.0);
    return 0;
}

int LightSensor_ReadVoltsAvg(int n, double *volts_avg) 
{
    if (!volts_avg || n <= 0) 
    { 
        errno = EINVAL; 
        return -1; 
    }
    double acc = 0.0;
    for (int i = 0; i < n; ++i) 
    {
        double v;
        if (LightSensor_ReadVolts(&v) < 0) 
        {
            return -1;
        }
        acc += v;
        struct timespec ts = {.tv_sec = 0, .tv_nsec = 200000};
        nanosleep(&ts, NULL);
    }
    *volts_avg = acc / n;
    return 0;
}

void LightSensor_Close(void) {
    if (s_fd >= 0) { close(s_fd); s_fd = -1; }
}
