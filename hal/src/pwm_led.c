#define _POSIX_C_SOURCE 200809L
#include "hal/pwm_led.h"

#include <linux/spi/spidev.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include<stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>
#include <limits.h>

#ifndef PATH_MAX
#define PATH_MAX 4096  
#endif

#ifndef MAX_FREQ
#define MAX_FREQ 500
#endif


#ifndef PWM_PATH
#define PWM_PATH "/dev/hat/pwm/GPIO12"
#endif



static char path[PATH_MAX];
static int current_freq= -1;



static int write_to_file(const char *p, const char *value)
{
    FILE *file = fopen(p, "w");
    if (!file) 
    {
        return -1;
    }

    if (fputs(value, file) == EOF) 
    { 
        fclose(file);
        return -1;
    }
    if (fclose(file) != 0) 
    { 
        return -1;
    }
    return 0;
}


static int write_i(const char *path, int v){
    char buf[32];
    snprintf(buf, sizeof(buf), "%d", v);
    return write_to_file(path, buf);
}

static int write_u64(const char *path, unsigned long long v){
    char buf[48];
    snprintf(buf, sizeof(buf), "%llu", v);
    return write_to_file(path, buf);
}





bool Led_init(const char *pwm_dir){

    const char *src;

    if (pwm_dir != NULL && pwm_dir[0] != '\0')
    {
        src = pwm_dir;                
    }
    else
    {
        src = PWM_PATH;               
    }
    snprintf(path, sizeof(path), "%s", src);

    current_freq = -1;                
    return true;

}

bool Led_set_hz(int freq){

    char period[PATH_MAX+32];
    char duty[PATH_MAX+32];
    char enable[PATH_MAX+32];

    unsigned long long period_ns;
    unsigned long long duty_ns;

    bool result =  true;

    if(freq < 0)
    {
        freq = 0;
    }
    if(freq > MAX_FREQ)
    {
        freq = MAX_FREQ;
    }

    if(freq == current_freq)
    {
        return true;
    }
    
    snprintf(period,     sizeof period,     "%s/period",     path);
    snprintf(duty, sizeof duty, "%s/duty_cycle", path);
    snprintf(enable,     sizeof enable,     "%s/enable",     path);


    if (freq == 0)
    {
        if (write_i(enable, 0) < 0)
        {
            result = false;
        }
        else
        {
          current_freq = 0;
      result = true;
        }
    }
    else
    {
            period_ns = 1000000000ULL / (unsigned long long)freq;
            duty_ns   = period_ns / 2ULL;
            if (duty_ns >= period_ns)
            {
                duty_ns = period_ns - 1ULL;

            } 
            if (write_i(enable, 0) < 0)
            {
                result = false;
            }
            else if (write_u64(period, period_ns) < 0)
            {
                result = false;
            }
            else if (write_u64(duty, duty_ns) < 0)
            {
                result = false;
            }
            else if (write_i(enable, 1) < 0)
            {
                result = false;
            }
            else
            {
                current_freq = freq;
            }
        
    }
    return result;
}



int  Led_get_hz(void){

    return current_freq;

}

bool Led_off(void){

    char enable[PATH_MAX+32];
    snprintf(enable, sizeof enable, "%s/enable", path);
    if (write_i(enable, 0) < 0)
    {
        return false;

    } 
    current_freq = 0;         
    return true;

}


void Led_shutdown(void){

     (void)Led_off();
}

bool LED_set_bright(int duty_c)
{
    char period[PATH_MAX+32];
    char duty_cycle[PATH_MAX+32];
    char enable[PATH_MAX+32];
    FILE *file = NULL;
    unsigned long long period_ns = 0ULL;
    unsigned long long duty_ns   = 0ULL;

    if (duty_c < 0)
    {
        duty_c = 0;
    }

    if (duty_c > 100)
    {
        duty_c = 100;
    }
     

    snprintf(period,sizeof period,     "%s/period",path);
    snprintf(duty_cycle, sizeof duty_cycle, "%s/duty_cycle",path);
    snprintf(enable,sizeof enable,     "%s/enable", path);

    file = fopen(period, "r");
    if (file && fscanf(file, "%llu", &period_ns) == 1)
    {
        fclose(file);
    }
     else
    {
        if (file) fclose(file);
        period_ns = 0ULL;
    }

    if (period_ns == 0ULL) 
    {
        period_ns = 100000000ULL; 
        if (write_i(enable, 0) < 0)
        {
            return false; 
        }   
        if (write_u64(period, period_ns)  < 0)
        {
            return false; 
        }   
       
    }

    duty_ns = (period_ns * (unsigned long long)duty_c) / 100ULL;
    if (duty_ns >= period_ns && period_ns > 0ULL)
    {
        duty_ns = period_ns - 1ULL; 
    } 

    if (write_u64(duty_cycle, duty_ns) < 0)
    {
        return false;
    }
    
    (void)write_i(enable, 1);
    return true;
}
