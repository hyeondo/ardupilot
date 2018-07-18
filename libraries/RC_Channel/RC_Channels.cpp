/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       RC_Channels.cpp - class containing an array of RC_Channel objects
 *
 */

#include <stdlib.h>
#include <cmath>

#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;

#include <AP_Math/AP_Math.h>

#include "RC_Channel.h"

RC_Channel *RC_Channels::channels;

const AP_Param::GroupInfo RC_Channels::var_info[] = {
    // @Group: 1_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[0], "1_",  1, RC_Channels, RC_Channel),

    // @Group: 2_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[1], "2_",  2, RC_Channels, RC_Channel),

    // @Group: 3_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[2], "3_",  3, RC_Channels, RC_Channel),

    // @Group: 4_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[3], "4_",  4, RC_Channels, RC_Channel),

    // @Group: 5_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[4], "5_",  5, RC_Channels, RC_Channel),

    // @Group: 6_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[5], "6_",  6, RC_Channels, RC_Channel),

    // @Group: 7_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[6], "7_",  7, RC_Channels, RC_Channel),

    // @Group: 8_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[7], "8_",  8, RC_Channels, RC_Channel),

    // @Group: 9_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[8], "9_",  9, RC_Channels, RC_Channel),

    // @Group: 10_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[9], "10_", 10, RC_Channels, RC_Channel),

    // @Group: 11_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[10], "11_", 11, RC_Channels, RC_Channel),

    // @Group: 12_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[11], "12_", 12, RC_Channels, RC_Channel),

    // @Group: 13_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[12], "13_", 13, RC_Channels, RC_Channel),

    // @Group: 14_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[13], "14_", 14, RC_Channels, RC_Channel),

    // @Group: 15_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[14], "15_", 15, RC_Channels, RC_Channel),

    // @Group: 16_
    // @Path: RC_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[15], "16_", 16, RC_Channels, RC_Channel),
    
    AP_GROUPEND
};


/*
  channels group object constructor
 */
RC_Channels::RC_Channels(void)
{
    channels = obj_channels;
    
    // set defaults from the parameter table
    AP_Param::setup_object_defaults(this, var_info);

    // setup ch_in on channels
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        channels[i].ch_in = i;
    }
}

uint16_t RC_Channels::get_radio_in(const uint8_t chan)
{
    if (chan >= NUM_RC_CHANNELS) {
        return 0;
    }
    return channels[chan].get_radio_in();
}

uint8_t RC_Channels::get_radio_in(uint16_t *chans, const uint8_t num_channels)
{
    uint8_t read_channels = MIN(num_channels, NUM_RC_CHANNELS);
    for (uint8_t i = 0; i < read_channels; i++) {
        chans[i] = channels[i].get_radio_in();
    }

    // clear any excess channels we couldn't read
    if (read_channels < num_channels) {
        memset(&chans[NUM_RC_CHANNELS], 0, sizeof(uint16_t) * (num_channels - read_channels));
    }

    return read_channels;
}


bool
RC_Channels::read_target_position(void){
    static bool isSet = false;
    static int buff_len;
    static char buf[20];

    if (!isSet){
        isSet = true;
        hal.uartE->begin(9600);
    }
    if (isSet){
        buff_len = hal.uartE->available();
        if(buff_len > 0){
            for(int i = 0; i<buff_len; i ++){
                buf[i] = hal.uartE->read();
            }
            hal.console->printf("\n\n atoi : %d  uartE value : %s\n\n", (int)atoi("43"), buf);

        }
    }

    return true;
}

bool
RC_Channels::read_input(void)
{
    static int pitch_ch = 1; //ch2
    static int yaw_ch = 3; //ch4
    static int enable_sw = 5; //ch6

    static int roll_ch = 0; //ch1
    // static int mode_sw = 4; //ch5 not using

    // for logging
    static int j = 0;
    j++;
    // read_target_position();

    if (!hal.rcin->new_input()) {
        return false;
    }

    if( channels[enable_sw].read() > 1499){
        for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
            if(i == pitch_ch){
                channels[pitch_ch].set_pwm((channels[pitch_ch].radio_min.get() + channels[pitch_ch].radio_max.get())/2);
            }else if(i == yaw_ch){
                if(j>100){
                    read_target_position();
                    j = 0;
                }
                // if(j>100){
                //     hal.console->printf("\n\npitch mid = %d\n",(channels[pitch_ch].radio_min.get() + channels[pitch_ch].radio_max.get())/2);
                //     hal.console->printf("yaw mid = %d\n\n",(channels[yaw_ch].radio_min.get() + channels[yaw_ch].radio_max.get())/2);
                //     j = 0;
                // }
                channels[yaw_ch].set_pwm((channels[yaw_ch].radio_min.get() + channels[yaw_ch].radio_max.get())/2);
            }else if(i == roll_ch){
                channels[roll_ch].set_pwm((channels[roll_ch].radio_min.get() + channels[roll_ch].radio_max.get())/2);
            }
            else{
                channels[i].set_pwm(channels[i].read());    
            }
        }
    }else{
        //disable normal mode
        for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
            channels[i].set_pwm(channels[i].read());
        }
    }

    return true;
}

uint8_t RC_Channels::get_valid_channel_count(void)
{
    return MIN(NUM_RC_CHANNELS, hal.rcin->num_channels());
}

int16_t RC_Channels::get_receiver_rssi(void)
{
    return hal.rcin->get_rssi();
}

void RC_Channels::clear_overrides(void)
{
    hal.rcin->clear_overrides();
}

bool RC_Channels::set_override(const uint8_t chan, const int16_t value)
{
    if (chan < NUM_RC_CHANNELS) {
        return hal.rcin->set_override(chan, value);
    }
    return false;
}

bool RC_Channels::set_overrides(int16_t *overrides, const uint8_t len)
{
    return hal.rcin->set_overrides(overrides, len);
}

bool RC_Channels::receiver_bind(const int dsmMode)
{
    return hal.rcin->rc_bind(dsmMode);
}
