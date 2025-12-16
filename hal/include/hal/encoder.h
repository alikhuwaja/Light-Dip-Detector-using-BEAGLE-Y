#ifndef ENCODER_H
#define ENCODER_H

#include <stdbool.h>

#define CCW (-1)
#define CW (1)
#define NONE (0)

#define ENC_DEFAULT_CHIP "gpiochip0"
#define ENC_DEFAULT_EDGES_PER_DETENT 4             
#define ENC_DEFAULT_DEBOUNCE_ns (2L*1000*1000L)


bool Enc_init(const char *chip, int a, int b, int edges_per_detent);
int Enc_get_direction(int timeout_ms); // either 1, 0 or -1
void Enc_shutdown(void);

#endif