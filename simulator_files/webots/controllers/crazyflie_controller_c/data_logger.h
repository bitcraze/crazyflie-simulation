#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

void data_logger_init(const char *sensor_file, const char *pose_file);

void data_logger_log_sensor(double timestamp,
                            double range_front, double range_left,
                            double range_back, double range_right,
                            double gyro_x, double gyro_y, double gyro_z,
                            double accel_x, double accel_y, double accel_z);

void data_logger_log_pose(double timestamp,
                          double x, double y, double z,
                          double roll, double pitch, double yaw);

void data_logger_close(void);

#endif
