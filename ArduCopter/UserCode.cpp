#include "Copter.h"
#include <px4_config.h>
#include <AP_HAL/AP_HAL.h>

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    hal.console->printf("\n\n\n\nsibal\n\n\n\n");
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    hal.console->printf("\n\n\n\nsibal\n\n\n\n");
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
    hal.console->printf("\n\n\n\nsibal\n\n\n\n");
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
    hal.console->printf("\n\n\n\nsibal\n\n\n\n");
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
    hal.console->printf("\n\n\n\nsibal\n\n\n\n");
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
    hal.console->printf("\n\n\n\nsibal\n\n\n\n");
}
#endif
