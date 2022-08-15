/*
 * com_bug_with_looping.h
 *
 *  Created on: Nov 8, 2018
 *      Author: knmcguire
 */

#ifndef SRC_LIB_WALLFOLLOWING_MULTIRANGER_ONBOARD_GRADIENT_BUG_WITH_LOOPING_H_
#define SRC_LIB_WALLFOLLOWING_MULTIRANGER_ONBOARD_GRADIENT_BUG_WITH_LOOPING_H_
#include <stdint.h>
#include <stdbool.h>
void init_SGBA_controller(float new_ref_distance_from_wall, float max_speed_ref,
                                       float begin_wanted_heading);
int SGBA_controller(float *vel_x, float *vel_y, float *vel_w, float *rssi_angle, int *state_wallfollowing,
                                 float front_range, float left_range, float right_range, float back_range,
                                 float current_heading, float current_pos_x, float current_pos_y, uint8_t rssi_beacon,
                                 uint8_t rssi_inter, float rssi_angle_inter, bool priority, bool outbound);

#endif /* SRC_LIB_WALLFOLLOWING_MULTIRANGER_ONBOARD_COM_BUG_WITH_LOOPING_H_ */
