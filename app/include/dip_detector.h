#ifndef DIP_DETECTOR_H
#define DIP_DETECTOR_H
#include <stdbool.h>

typedef struct {
    double trigger_delta;   // volts below  average to trigger a dip 
    double release_delta;   // volts below average to release (e.g., 0.07) hysteresis
    int    min_width;       // require at least this many consecutive samples below trigger 
    int    min_gap;         // after a dip ends, require this many samples above release before allowing another
} DipConfig;


int Dip_count(const double *x, int n, double ema, const DipConfig *cfg);

static inline DipConfig Dip_default(void) 
{
    DipConfig c = { .trigger_delta = 0.10, .release_delta = 0.07, .min_width = 2, .min_gap = 1 };
    return c;
}

#endif
