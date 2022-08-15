/*
 * wallfolllowing_with_avoid.h
 *
 *  Created on: Nov 12, 2018
 *      Author: knmcguire
 */

#ifndef SRC_LIB_WALLFOLLOWING_MULTIRANGER_ONBOARD_WALLFOLLOWING_WITH_AVOID_H_
#define SRC_LIB_WALLFOLLOWING_MULTIRANGER_ONBOARD_WALLFOLLOWING_WITH_AVOID_H_

void init_wall_follower_and_avoid_controller(float new_ref_distance_from_wall, float max_speed_ref,
    float starting_local_direction);
int wall_follower_and_avoid_controller(float *vel_x, float *vel_y, float *vel_w, float front_range, float left_range,
                                       float right_range,  float current_heading, float pos_x, float pos_y, uint8_t rssi_other_drone);
#endif /* SRC_LIB_WALLFOLLOWING_MULTIRANGER_ONBOARD_WALLFOLLOWING_WITH_AVOID_H_ */
