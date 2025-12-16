#define _POSIX_C_SOURCE 200809L
#include "sampler.h"
#include "hal/light_sensor.h"
#include "hal/pwm_led.h"
#include "hal/encoder.h"
#include "periodTimer.h"



#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/timerfd.h>
#include <time.h>

#define MAX_SAMPLES 2000

static pthread_t sample_thread;

static bool sample_running = false;
static bool sample_average =  false;

static int  sample_file_descriptor =  -1;


static pthread_mutex_t lock =  PTHREAD_MUTEX_INITIALIZER;

static double current_samples[MAX_SAMPLES];
static double history_samples[MAX_SAMPLES];

static int c_number_samples = 0;
static int h_number_samples = 0;
static long long total_samples = 0;

static double average = 0.0;

static int timer (void){

    int result =0 ;
    int descriptor =  timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC);
    if (descriptor < 0) 
    {
        result = -1;
    }

    else
    {
        struct itimerspec ts;
        ts.it_value.tv_sec = 0;
        ts.it_interval.tv_sec = 0;
        ts.it_value.tv_nsec = 1000000;
        ts.it_interval.tv_nsec= 1000000;

        if(timerfd_settime(descriptor, 0, &ts, NULL) < 0)
        {
            close(descriptor);
            result = -1;
        }
        else result = descriptor;

    }
    return result;

}

static void average_update(double value){
    
    if(!sample_average)
    {
        average =  value;
        sample_average = true;
    }
    else
    {
        average =  0.999*average + (0.001*value);


    }



}

static bool sample_locked(void)
{
    if (c_number_samples >= MAX_SAMPLES) 
    {
        return false;
    }

    double v = 0.0;
    if (LightSensor_ReadVolts(&v) != 0)
    {
        return false;
    }

    current_samples[c_number_samples++] = v;
    total_samples++;
    average_update(v);
    Period_markEvent(PERIOD_EVENT_SAMPLE_LIGHT);
    return true;
}


static void *sample_worker(void *arg)
{
    (void)arg;
    while (true)
     {
        uint64_t ticks = 0;
        ssize_t n = read(sample_file_descriptor, &ticks, sizeof ticks);
        if (n < 0) 
        {
            if (errno == EINTR) continue;   
            break;                          
        }
        if (n != (ssize_t)sizeof ticks || ticks == 0) 
        {
            continue;                       
        }

        pthread_mutex_lock(&lock);
        int space = MAX_SAMPLES - c_number_samples;
        int take  = (int)ticks;
        if (take > space) take = space;     

        for (int i = 0; i < take; i++) 
        {
            if (!sample_locked()) 
            {
                break;
            }
        }
        pthread_mutex_unlock(&lock);
    }
    return NULL;
}

void Sampler_init(void)
{

    if(sample_running)
    {
        return;
    }

    sample_file_descriptor = timer();
    if (sample_file_descriptor < 0)
    {
        return;
    }

    sample_running =  true;
    if (pthread_create(&sample_thread, NULL, sample_worker, NULL) != 0)
    {
        sample_running = false;
        close(sample_file_descriptor);
        sample_file_descriptor = -1;
    }
}
void Sampler_cleanup(void)
{
    if(!sample_running)
    {
        return;
    }
    sample_running =  false;
    if(sample_file_descriptor != -1)
    {
        close(sample_file_descriptor);
        sample_file_descriptor  =-1;
    }
    pthread_join(sample_thread, NULL);
    pthread_mutex_lock(&lock);
    c_number_samples = 0;
    h_number_samples = 0;
    total_samples    = 0;
    average          = 0.0;
    sample_average   = false;
    pthread_mutex_unlock(&lock);
}

void Sampler_moveCurrentDataToHistory(void)
{
    pthread_mutex_lock(&lock);
    if (c_number_samples > 0)
    {
        memcpy(history_samples, current_samples, (size_t)c_number_samples * sizeof(double));
        h_number_samples = c_number_samples;
    } 
    else 
    {
        h_number_samples = 0;
    }
    c_number_samples = 0; 
    pthread_mutex_unlock(&lock);



}

int Sampler_getHistorySize(void)
{

    pthread_mutex_lock(&lock);
    int n =  h_number_samples;
    pthread_mutex_unlock(&lock);
    return n;
}

double* Sampler_getHistory(int *size)
{
    if (!size) return NULL;

    pthread_mutex_lock(&lock);
    int n = h_number_samples;
    double *out = NULL;
    if (n > 0) 
    {
        out = (double*)malloc((size_t)n * sizeof(double));
        if (out)
        {
            memcpy(out, history_samples, (size_t)n * sizeof(double));
        }
        else
        {
            n=0;
        }
    }
    pthread_mutex_unlock(&lock);

    *size = n;
    return out; 
}

double Sampler_getAverageReading(void)
{

    pthread_mutex_lock(&lock);
    double a;
    if(sample_average)
    {
        a =  average;
    }
    else
    {
        a =  0.0;
    
    }
    pthread_mutex_unlock(&lock);
    return a;
}

long long Sampler_getNumSamplesTaken(void)
{


    pthread_mutex_lock(&lock);
    long long t = total_samples;
    pthread_mutex_unlock(&lock);
    return t;
}





