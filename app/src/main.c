#define _POSIX_C_SOURCE 200809L

#include "sampler.h"
#include "hal/light_sensor.h"
#include "hal/pwm_led.h"
#include "hal/encoder.h"
#include "dip_detector.h"
#include "periodTimer.h"
#include "udp.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <signal.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <stdatomic.h>




#ifndef LED_PWM_DIR
#define LED_PWM_DIR "/dev/hat/pwm/GPIO12"
#endif

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int _) { (void)_; g_stop = 1; }

static void sleep_ms(int ms) {
    struct timespec ts = { ms/1000, (long)(ms%1000) * 1000000L };
    nanosleep(&ts, NULL);
}

static inline int clampi(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void stats(const double *x, int n, double *out_mean, double *out_min, double *out_max) {
    if (!x || n <= 0) 
    { 
        *out_mean = 0.0; 
        *out_min = 0.0; 
        *out_max = 0.0; 
        return; 
    }

    double s = 0.0, mn = x[0], mx = x[0];
    for (int i = 0; i < n; i++) 
    {
        double v = x[i];
        s += v; if (v < mn) mn = v; if (v > mx) mx = v;
    }
    *out_mean = s / (double)n; 
    *out_min = mn; 
    *out_max = mx;
}


static void print_line1(int n, int cur_hz, double avg, int dips)
{
    Period_statistics_t ps = {0};
    Period_getStatisticsAndClear(PERIOD_EVENT_SAMPLE_LIGHT, &ps);
    printf("#Smpl/s = %4d Flash @ %3dHz avg = %5.3fV dips = %3d "
           "Smpl ms[%6.3f, %6.3f] avg %6.3f/%4d\n",
           n, cur_hz, avg, dips,
           ps.minPeriodInMs, ps.maxPeriodInMs,
           ps.avgPeriodInMs, ps.numSamples);
}

static void print_line2_samples(const double *x, int n)
{
    if (!x || n <= 0) 
    { 
        puts(" (no samples)"); 
        return; 
    }

    int show = (n < 10) ? n : 10;
    putchar(' ');
    for (int i = 0; i < show; i++) 
    {
        int idx;
        if (show == 1) 
        {
            idx = 0;
        } 
        else 
        {
        
            double pos = (double)i * (double)(n - 1) / (double)(show - 1);
            idx = (int)llround(pos);
            if (idx < 0) idx = 0;
            if (idx >= n) idx = n - 1;
        }
        printf("%3d:%0.3f%s", idx, x[idx], (i + 1 < show) ? " " : "\n");
    }
}


int main(int argc, char **argv)
{

     atomic_bool udp_exit = false; 
    if (argc < 4)
    {
        fprintf(stderr,
"Usage: %s <spidev> <adc_channel> <vref_volts> [options]\n"
"Options:\n"
"  --print-samples                  Show sample list each window (ignored by spec; we always print 10)\n"
"  --chip=<gpiochip>                Encoder chip (e.g., gpiochip2)\n"
"  --a=<line> --b=<line>            Encoder A/B lines (defaults: 7/8)\n"
"  --edges=<N>                      Edges per detent (default: 4)\n"
"  --fmin=<N> --fmax=<N>            Freq clamp (default: 0..500 Hz)\n"
"  --start-hz=<N>                   Start freq (default: 10 Hz)\n"
"  --duty=<P>                       Duty percent 0..100 (default: 50)\n"
"  --step=<K>                       Hz per detent (default: 1)\n"
"  --dip-trig=<V>                   Trigger delta (V below EMA)\n"
"  --dip-rel=<V>                    Release delta (V below EMA)\n"
"  --dip-width=<N>                  Min width (samples)\n"
"  --dip-gap=<N>                    Min gap (samples)\n",
            argv[0]);
        return 2;
    }

    const char *spidev = argv[1];
    int         adc_ch = atoi(argv[2]);
    double      vref   = atof(argv[3]);

    const char *enc_chip = "gpiochip2";
    int enc_a = 7, enc_b = 8, enc_edges = 4;
    int fmin = 0, fmax = 500;
    int cur_hz = 10, duty = 50, step_hz = 1;
    const int poll_ms = 10;

    DipConfig dip = {
        .trigger_delta = 0.10,
        .release_delta = 0.07,
        .min_width     = 2,
        .min_gap       = 1
    };

    for (int i = 4; i < argc; i++) 
    {
        if      (!strncmp(argv[i], "--chip=", 7))          enc_chip = argv[i] + 7;
        else if (!strncmp(argv[i], "--a=", 4))             enc_a = atoi(argv[i] + 4);
        else if (!strncmp(argv[i], "--b=", 4))             enc_b = atoi(argv[i] + 4);
        else if (!strncmp(argv[i], "--edges=", 8))         enc_edges = atoi(argv[i] + 8);
        else if (!strncmp(argv[i], "--fmin=", 7))          fmin = atoi(argv[i] + 7);
        else if (!strncmp(argv[i], "--fmax=", 7))          fmax = atoi(argv[i] + 7);
        else if (!strncmp(argv[i], "--start-hz=", 11))     cur_hz = atoi(argv[i] + 11);
        else if (!strncmp(argv[i], "--duty=", 7))          duty = atoi(argv[i] + 7);
        else if (!strncmp(argv[i], "--step=", 7))          step_hz = atoi(argv[i] + 7);
        else if (!strncmp(argv[i], "--dip-trig=", 11))     dip.trigger_delta = atof(argv[i] + 11);
        else if (!strncmp(argv[i], "--dip-rel=", 10))      dip.release_delta = atof(argv[i] + 10);
        else if (!strncmp(argv[i], "--dip-width=", 12))    dip.min_width = atoi(argv[i] + 12);
        else if (!strncmp(argv[i], "--dip-gap=", 10))      dip.min_gap = atoi(argv[i] + 10);
        else fprintf(stderr, "WARN: unknown arg ignored: %s\n", argv[i]);
    }

    signal(SIGINT, on_sigint);
    signal(SIGTERM, on_sigint);

    /* Timing module */
    Period_init();

    // LED init
    if (!Led_init(LED_PWM_DIR)) 
    {
        fprintf(stderr, "Led_init(%s) failed\n", LED_PWM_DIR);
        Period_cleanup();
        return 2;
    }
    duty   = clampi(duty, 0, 100);
    cur_hz = clampi(cur_hz, fmin, fmax);
    if (!LED_set_bright(duty)) 
    {
        fprintf(stderr, "LED_set_bright(%d) failed\n", duty);
        Led_shutdown();
        Period_cleanup();
        return 2;
    }
    if (cur_hz == 0) 
    {
        Led_off();
        printf("LED: OFF (0 Hz)\n");
    } 
    else 
    {
        if (!Led_set_hz(cur_hz)) fprintf(stderr, "Led_set_hz(%d) failed\n", cur_hz);
        printf("LED: %d Hz @ %d%% (start)\n", cur_hz, duty);
    }

    // Encoder init
    if (!Enc_init(enc_chip, enc_a, enc_b, enc_edges)) 
    {
        fprintf(stderr, "Enc_init(%s, A=%d, B=%d, edges=%d) failed\n",enc_chip, enc_a, enc_b, enc_edges);
        Led_off(); Led_shutdown();
        Period_cleanup();
        return 3;
    }

    // Sensor + sampler
    if (LightSensor_Init(spidev, adc_ch, vref) != 0) 
    {
        fprintf(stderr, "LightSensor_Init failed for %s ch%d (vref=%.3f)\n", spidev, adc_ch, vref);
    }
    Sampler_init();
    sleep_ms(600);
    Sampler_moveCurrentDataToHistory();


    if (!udp_start(12345, &udp_exit))
    {
        fprintf(stderr, "udp_start failed on port 12345\n");
        Sampler_cleanup();
        LightSensor_Close();
        Enc_shutdown();
        Led_off(); Led_shutdown();
        Period_cleanup();
        return 4;
    }

    puts("Rotate encoder to change LED frequency. Ctrl+C to stop.");

    while (!g_stop && !atomic_load(&udp_exit))
     {
        int elapsed = 0, pending = 0;
        while (elapsed < 1000 && !g_stop && !atomic_load(&udp_exit))
        {
            int dir = Enc_get_direction(poll_ms);
            if (dir == CW || dir == CCW) pending += dir;
            while ((dir = Enc_get_direction(0)) != 0)
            {
                if (dir == CW || dir == CCW) pending += dir;
                else break;
            }
            if (pending)
            {
                int next = clampi(cur_hz + pending * step_hz, fmin, fmax);
                pending = 0;
                if (next != cur_hz)
                {
                    if (next == 0)
                     {
                        Led_off();
                        printf("LED: OFF (0 Hz)\n");
                    }
                    else if (!Led_set_hz(next))
                    {
                         fprintf(stderr, "Led_set_hz(%d) failed\n", next);
                    }
                    else printf("LED: %d Hz at %d%%\n", next, duty);
                    cur_hz = next;
                }
            }
            elapsed += poll_ms;
        }

        Sampler_moveCurrentDataToHistory();
        Period_markEvent(PERIOD_EVENT_MARK_SECOND);

        int n = 0;
        double *hist = Sampler_getHistory(&n);
        double avg   = Sampler_getAverageReading();

        int dips = Dip_count(hist, n, avg, &dip);

        print_line1(n, cur_hz, avg, dips);
        print_line2_samples(hist, n);
        fflush(stdout);

        free(hist);
    }
    
    udp_stop();
    Sampler_cleanup();
    LightSensor_Close();
    Enc_shutdown();
    Led_off();
    Led_shutdown();
    Period_cleanup();
    puts("Done.");
    return 0;
}






/*
on host

//if remmoving 

sudo rm -rf -- /home/ali/ensc351/public/as2

sudo mkdir -p /home/ali/ensc351/public/as2
sudo cp -a /home/ali/ensc351/work/as2/cmake_starter \
           /home/ali/ensc351/public/as2/
sudo chown -R ali:ali /home/ali/ensc351/public/as2/cmake_starter


   on beagle



chip=/sys/class/pwm/pwmchip3; [ -d "$chip/pwm1" ] || echo 1 | sudo tee "$chip/export" >/dev/null; sudo mkdir -p /dev/hat/pwm; sudo ln -sfn "$chip/pwm1" /dev/hat/pwm/GPIO12; PWM=/dev/hat/pwm/GPIO12; echo "PWM -> $(readlink -f "$PWM")"; echo 0 | sudo tee "$PWM/enable" >/dev/null; echo 1000000 | sudo tee "$PWM/period" >/dev/null; echo 500000 | sudo tee "$PWM/duty_cycle" >/dev/null; echo 1 | sudo tee "$PWM/enable" >/dev/null; sleep 1; echo 0 | sudo tee "$PWM/enable" >/dev/null




 ./mountNFS.sh

cd /mnt/remote/as2/cmake_starter
rm -rf build
cmake -S . -B build          
cmake --build build -j    



sudo ./build/light_sampler \
  /dev/spidev0.1 0 3.300 \
  --chip=gpiochip2 --a=7 --b=8 --edges=4 \
  --start-hz=10 --duty=50 --step=1 \
  --dip-trig=0.10 --dip-rel=0.07 --dip-width=2 --dip-gap=1


netcat -u 192.168.7.2 12345


//to run the python program

python3 /home/ali/Downloads/as2UdpGui.py






   





*/