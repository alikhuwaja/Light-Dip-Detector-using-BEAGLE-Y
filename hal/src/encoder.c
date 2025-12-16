#define _POSIX_C_SOURCE 200809L
#include <gpiod.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include "hal/encoder.h"


#define CCW (-1)
#define CW (1)
#define NONE (0)
#define ENC_DEFAULT_CHIP "gpiochip0"
#define ENC_DEFAULT_EDGES_PER_DETENT 4
#define GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP 0
#define SAMPLE_US  500
#define DBLREAD_US 200
#define SETTLE_US  800

static struct gpiod_chip *s_chip = NULL;
static struct gpiod_line *s_a = NULL;
static struct gpiod_line *s_b = NULL;

static unsigned previous_bits = 0;
static int edge_accum = 0;
static int s_edges_per_detent = ENC_DEFAULT_EDGES_PER_DETENT;

static inline uint64_t now_ms(void) {
    struct timespec ts; 
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}
static inline void sleep_us(long us) {
    struct timespec ts = { 0, us * 1000L }; 
    nanosleep(&ts, NULL);
}

static int read_current_state(void) {
    for (int i = 0; i < 3; ++i) 
    {
        int a1 = gpiod_line_get_value(s_a);
        int b1 = gpiod_line_get_value(s_b);
        if (a1 < 0 || b1 < 0) 
        {
            return -1;
        }
        sleep_us(DBLREAD_US);
        int a2 = gpiod_line_get_value(s_a);
        int b2 = gpiod_line_get_value(s_b);
        if (a2 < 0 || b2 < 0) 
        {
            return -1;
        }
        if (a1 == a2 && b1 == b2)
        {
            return ((unsigned)a2 << 1) | (unsigned)b2;
        } 
    }
    int a = gpiod_line_get_value(s_a);
    int b = gpiod_line_get_value(s_b);
    if (a < 0 || b < 0) return -1;
    return ((unsigned)a << 1) | (unsigned)b;
}

static const int8_t TRANS[16] = 
{
     0, +1, -1,  0,
    -1,  0,  0, +1,
    +1,  0,  0, -1,
     0, -1, +1,  0
};
static inline int step_from_states(unsigned prev2b, unsigned cur2b) {
    return TRANS[((prev2b & 0x3) << 2) | (cur2b & 0x3)];
}

bool Enc_init(const char *chip, int a, int b, int edges_per_detent)
{
    const char *chip_name;
    if (chip != NULL && chip[0] != '\0') 
    {
        chip_name = chip;
    }
    else 
    {
        chip_name = ENC_DEFAULT_CHIP;
    }

    if (edges_per_detent > 0) 
    {
        s_edges_per_detent = edges_per_detent;
    }
    else 
    {
        s_edges_per_detent = ENC_DEFAULT_EDGES_PER_DETENT;
    }


    s_chip = gpiod_chip_open_by_name(chip_name);
    if (!s_chip) 
    {
        return false;
    }

    s_a = gpiod_chip_get_line(s_chip, a);
    s_b = gpiod_chip_get_line(s_chip, b);
    if (!s_a || !s_b) 
    {
        Enc_shutdown(); 
        return false; 
    }

    if (gpiod_line_request_input_flags(s_a, "encoderA", GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP) < 0) 
    { 
        Enc_shutdown(); 
        return false; 
    }
    if (gpiod_line_request_input_flags(s_b, "encoderB", GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP) < 0) 
    { 
        Enc_shutdown(); 
        return false; 
    }

    int st = read_current_state();
    if (st < 0) 
    { 
        Enc_shutdown(); 
        return false; 
    }
    previous_bits = (unsigned)st;
    edge_accum = 0;
    return true;
}

int Enc_get_direction(int timeout_ms)
{
    if (!s_a || !s_b) return -1;

    uint64_t deadline = now_ms() + (timeout_ms > 0 ? (uint64_t)timeout_ms : 0);

    do 
    {
        int st = read_current_state();
        if (st < 0) return -1;

        unsigned cur2b = (unsigned)st;
        int step = step_from_states(previous_bits, cur2b);
        if (step != 0) 
        {
            previous_bits = cur2b;
            edge_accum += step;

            int reached = 0;
            if (edge_accum >= s_edges_per_detent) reached = +1;
            else if (edge_accum <= -s_edges_per_detent) reached = -1;

            if (reached) 
            {
                if (cur2b == 0 || cur2b == 3) 
                {
                    edge_accum = 0;
                    sleep_us(SETTLE_US);
                    return reached;
                }
                else 
                {
                    edge_accum = (reached > 0) ? s_edges_per_detent : -s_edges_per_detent;
                }
            }
        }

        if (timeout_ms == 0) break;
        sleep_us(SAMPLE_US);
    } 
    while (now_ms() < deadline);

    return 0;
}

void Enc_shutdown(void)
{
    if (s_a) 
    { 
        gpiod_line_release(s_a); 
        s_a = NULL; 
    }
    if (s_b) 
    { 
        gpiod_line_release(s_b); 
        s_b = NULL; 
    }
    if (s_chip) 
    { 
        gpiod_chip_close(s_chip); 
        s_chip = NULL; 
    }
    previous_bits = 0; edge_accum = 0;
}
