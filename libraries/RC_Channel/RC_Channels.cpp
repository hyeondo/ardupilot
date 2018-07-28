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
RC_Channels::pid_control(uint32_t curTime,float* error_x,float* error_y,int* yaw_val, int* pitch_val){
    //error_x : yaw
    //error_y : pitch

    static uint32_t prevTime = 0;

    static float pitch_Kp = 50;
    static float pitch_Ki = 0.112;
    static float pitch_Kd = 6.5;
    static float pitch_acc_error = 0; //for acc
    static float pitch_prev_error = 0; //for diff

    static float yaw_Kp = 15;
    static float yaw_Ki = 0.012;
    static float yaw_Kd = 1.4;
    static float yaw_acc_error = 0;
    static float yaw_prev_error = 0;

    float error_pitch = *error_y;
    float error_yaw = *error_x;
    
    if( prevTime == 0){
        //처음 1회
        prevTime = curTime;
        return false;
    }else{
        if(curTime-prevTime > 500){
            pitch_acc_error = 0;
            pitch_prev_error = 0;
            yaw_acc_error = 0;
            yaw_prev_error = 0;
            prevTime = curTime;
            //pid time out --
            return false;
        }
        float dt = (curTime-prevTime);
        dt = dt/1000;

        if(pitch_prev_error == 0){
            pitch_prev_error = error_pitch;
        }
        if(yaw_prev_error == 0){
            yaw_prev_error = error_yaw;
        }

        pitch_acc_error += error_pitch * dt;
        *pitch_val = (float)(pitch_Kp * error_pitch) + (float)( pitch_Ki * pitch_acc_error ) + (float)( pitch_Kd * ((error_pitch - pitch_prev_error)/dt));
        // hal.console->printf(" curTime : %d  \n",curTime);
        // hal.console->printf(" prevTime : %d  \n",prevTime);
        // hal.console->printf(" dt : %f  \n",dt);
        // hal.console->printf(" control_value_ p : %f  \n", (pitch_Kp * error_pitch));
        // hal.console->printf(" control_value_ i : %f  \n", ( pitch_Ki * pitch_acc_error));
        // hal.console->printf(" control_value_ d : %f  \n", ( pitch_Kd * ((error_pitch - pitch_prev_error)/dt)));
        // hal.console->printf(" control_value_ pid : %d  \n", *pitch_val);
        pitch_prev_error = error_pitch;

        yaw_acc_error += error_yaw * dt;
        *yaw_val = (float)(yaw_Kp * error_yaw) + (float)(yaw_Ki * yaw_acc_error) + (float)(yaw_Kd * ((error_yaw - yaw_prev_error)/dt));
        yaw_prev_error = error_yaw;
        prevTime = curTime;
        return true;
    }
}

bool
RC_Channels::read_target_position(float* error_x,float* error_y){
    static bool isSet = false;
    static int mode = 0; //mode == 1(x), mode == 2(y)
    static char diff_y[10] = {0};
    static char diff_x[10] = {0};
    static int idx = 0;
    bool isUpdated = false;
    int buff_len;

    if (!isSet){
        isSet = true;
        hal.uartE->begin(9600);
    }

    buff_len = hal.uartE->available();
    if(buff_len > 0){
        for(int i = 0; i<buff_len;i++){
            char buf;
            buf = hal.uartE->read();
            if(buf == 'z'){
                //TODO : 오브젝트 인식 실패시 
                idx = 0;
                mode = 0;
                for(int k = 0; k<10; k++) diff_y[k] = 0;
                for(int k = 0; k<10; k++) diff_x[k] = 0;
                // return false;
            }
            if(buf == 'x'){
                if(mode == 2){
                    //diff_x input fisish
                    //hal.console->printf("\n\n diff_x : %d  \n\n", (int)atoi(diff_x));
                    float temp = (int)atoi(diff_y);
                    *error_y = temp / 100;
                }
                mode = 1;
                idx = 0;
                for(int k = 0; k<10; k++) diff_y[k] = 0;
                for(int k = 0; k<10; k++) diff_x[k] = 0;
                isUpdated = true;
                // return true;
            }else if(buf == 'y'){
                if(mode == 1){
                    //diff_y input fisish
                    //hal.console->printf("\n\n diff_y : %d  \n\n", (int)atoi(diff_y));
                    float temp = (int)atoi(diff_x);
                    *error_x = temp/100;
                }
                mode = 2;
                idx = 0;
                for(int k = 0; k<10; k++) diff_y[k] = 0;
                for(int k = 0; k<10; k++) diff_x[k] = 0;
                isUpdated = true;
                // return true;
            }else if(idx > 9){
                //방어코드
                idx = 0;
                mode = 0;
                for(int k = 0; k<10; k++) diff_y[k] = 0;
                for(int k = 0; k<10; k++) diff_x[k] = 0;
                // return false;
            }else if ( (buf >= '0' && buf <= '9') || buf == '+' || buf == '-'){
                if(mode == 1){
                    diff_x[idx] = buf;
                    idx++;
                }else if(mode == 2){
                    diff_y[idx] = buf;
                    idx++;
                }
                // return false;
            }else{
                //방어코드
                idx = 0;
                mode = 0;
                for(int k = 0; k<10; k++) diff_y[k] = 0;
                for(int k = 0; k<10; k++) diff_x[k] = 0;
                // return false;
            }
        }
        return isUpdated;
    }else{
        return false;
    }

}

bool
RC_Channels::read_input(void)
{
    static uint32_t lastReceiveTime = 0;
    static int pitch_ch = 1; //ch2
    static int yaw_ch = 3; //ch4
    static int enable_sw = 5; //ch6
    static int roll_ch = 0; //ch1

    static float error_pitch = 0; //y
    static float error_yaw = 0; //x

    static int control_pitch_val = 0; //y
    static int control_yaw_val = 0; //x

    // for logging
    static int j = 0;

    if (!hal.rcin->new_input()) {
        return false;
    }
    

    if( channels[enable_sw].read() > 1499){
        //enable tracking mode
        j++;
        if(j>5){
            j = 0;
            if(read_target_position(&error_yaw,&error_pitch) == true){
                // hal.console->printf(" \n\nerror_value_yaw : %f  \n", error_yaw);
                // hal.console->printf(" error_value_pitch : %f  \n\n", error_pitch);
                lastReceiveTime = AP_HAL::millis();
                if(pid_control(lastReceiveTime,&error_yaw,&error_pitch,&control_yaw_val,&control_pitch_val) == true){
                     if (control_pitch_val > 300){
                        control_pitch_val = 300;
                    }else if (control_pitch_val < - 300){
                        control_pitch_val = -300;
                    }

                    if (control_yaw_val > 150){
                        control_yaw_val = 150;
                    }else if (control_yaw_val < - 150){
                        control_yaw_val = -150;
                    }

                    // hal.console->printf(" PID Control value: \n");
                    // hal.console->printf(" control_yaw_val : %d  \n", control_yaw_val);
                    // hal.console->printf(" control_pitch_val : %d  \n\n", control_pitch_val);
                }else{
                    control_pitch_val = 0;
                    control_yaw_val = 0;
                    // hal.console->printf("\n\n Timeout from PID: \n\n  \n");
                }

                //파라미터에서 1500을 중앙으로 설정해놓았음
                for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
                    if(i == pitch_ch){
                        channels[pitch_ch].set_pwm(1500);
                        // channels[pitch_ch].set_pwm(((channels[pitch_ch].radio_min.get() + channels[pitch_ch].radio_max.get())/2) + control_pitch_val);
                    }else if(i == yaw_ch){
                        channels[yaw_ch].set_pwm(1500);
                        // channels[yaw_ch].set_pwm(((channels[yaw_ch].radio_min.get() + channels[yaw_ch].radio_max.get())/2) + control_yaw_val);
                    }else if(i == roll_ch){
                        channels[roll_ch].set_pwm(1500);
                        // channels[roll_ch].set_pwm((channels[roll_ch].radio_min.get() + channels[roll_ch].radio_max.get())/2);
                    }
                    else{
                        channels[i].set_pwm(channels[i].read());    
                    }
                }

            }else{
                //check time out
                if(lastReceiveTime == 0){
                    lastReceiveTime = AP_HAL::millis();
                }else{
                    if((AP_HAL::millis() - lastReceiveTime) > 2000){
                        //2sec
                        lastReceiveTime = AP_HAL::millis();
                        error_yaw = 0;
                        error_pitch = 0;
                        // hal.console->printf("\n\n Timeout from Serial: %d  \n", lastReceiveTime);
                        // hal.console->printf(" error_value_yaw : %f  \n", error_yaw);
                        // hal.console->printf(" error_value_pitch : %f  \n\n", error_pitch);
                    }
                }

                for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
                    if(i == pitch_ch){
                        channels[pitch_ch].set_pwm((channels[pitch_ch].radio_min.get() + channels[pitch_ch].radio_max.get())/2);
                    }else if(i == yaw_ch){
                        channels[yaw_ch].set_pwm((channels[yaw_ch].radio_min.get() + channels[yaw_ch].radio_max.get())/2);
                    }else if(i == roll_ch){
                        channels[roll_ch].set_pwm((channels[roll_ch].radio_min.get() + channels[roll_ch].radio_max.get())/2);
                    }
                    else{
                        channels[i].set_pwm(channels[i].read());    
                    }
                }

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
