#include "dip_detector.h"

int Dip_count(const double *x, int n, double ave, const DipConfig *cfg)
{
    if (!x || n <= 0 || !cfg) return 0;

    double trig = ave - cfg->trigger_delta;
    double rel  = ave - cfg->release_delta;

    enum { ABOVE, BELOW_WAIT, BELOW_OK, GAP } 
    state = ABOVE;
    int run = 0; 
    int gap = 0;
    int dips = 0;

    for (int i = 0; i < n; ++i) 
    {
        double v = x[i];

        if(state == ABOVE) 
        {
            if (v < trig) 
            {
                state = BELOW_WAIT;
                run = 1;
            }
        } 
        else if (state == BELOW_WAIT) 
        {
            if (v < trig) 
            {
                ++ run;
                if (run >= cfg->min_width) 
                {
                    ++dips;
                    state = BELOW_OK;
                }
            } 
            else 
            {
                run = 0;
                state = ABOVE;
            }
        }
        else if (state == BELOW_OK) 
        {
            if (v >= rel) 
            {
                if (cfg->min_gap > 0) 
                {
                    gap = cfg->min_gap;
                }
                else 
                {
                    gap = 0;
                }

                if (gap > 0) 
                {
                    state = GAP;
                }
                else 
                {
                    state = ABOVE;
                }
            }

        } 
        else if (state == GAP) 
        {
            gap -= 1;
            if (gap <= 0) 
            {
                state = ABOVE;
            }
        }


    }
    return dips;
}
