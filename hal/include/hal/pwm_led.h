#ifndef PWM_LED_H
#define PWM_LED_H

#include <stdbool.h>

bool Led_init(const char *pwm_dir);
bool Led_set_hz(int hz);
bool LED_set_bright(int duty_c);
int  Led_get_hz(void);
bool Led_off(void);
void Led_shutdown(void);


#endif


/*chip=/sys/class/pwm/pwmchip3
[ -d $chip/pwm1 ] || echo 1 | sudo tee $chip/export
cd $chip/pwm1

sudo mkdir -p /dev/hat/pwm
sudo ln -sfn /sys/class/pwm/pwmchip3/pwm1 /dev/hat/pwm/GPIO12
cd /dev/hat/pwm/GPIO12
*/