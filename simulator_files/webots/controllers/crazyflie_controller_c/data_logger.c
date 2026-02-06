#include "data_logger.h"

#include <stdio.h>

static FILE *sensor_fp = NULL;
static FILE *pose_fp = NULL;
static int sensor_count = 0;
static int pose_count = 0;

void data_logger_init(const char *sensor_file, const char *pose_file) {
  sensor_fp = fopen(sensor_file, "w");
  pose_fp = fopen(pose_file, "w");
  if (sensor_fp)
    fprintf(sensor_fp, "[\n");
  if (pose_fp)
    fprintf(pose_fp, "[\n");
  sensor_count = 0;
  pose_count = 0;
}

void data_logger_log_sensor(double timestamp,
                            double range_front, double range_left,
                            double range_back, double range_right,
                            double gyro_x, double gyro_y, double gyro_z,
                            double accel_x, double accel_y, double accel_z) {
  if (!sensor_fp)
    return;
  if (sensor_count > 0)
    fprintf(sensor_fp, ",\n");
  fprintf(sensor_fp,
          "  {\"timestamp\": %.4f, "
          "\"range_front\": %.1f, \"range_left\": %.1f, "
          "\"range_back\": %.1f, \"range_right\": %.1f, "
          "\"gyro_x\": %.6f, \"gyro_y\": %.6f, \"gyro_z\": %.6f, "
          "\"accel_x\": %.6f, \"accel_y\": %.6f, \"accel_z\": %.6f}",
          timestamp, range_front, range_left, range_back, range_right,
          gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z);
  sensor_count++;
}

void data_logger_log_pose(double timestamp,
                          double x, double y, double z,
                          double roll, double pitch, double yaw) {
  if (!pose_fp)
    return;
  if (pose_count > 0)
    fprintf(pose_fp, ",\n");
  fprintf(pose_fp,
          "  {\"timestamp\": %.4f, "
          "\"x\": %.6f, \"y\": %.6f, \"z\": %.6f, "
          "\"roll\": %.6f, \"pitch\": %.6f, \"yaw\": %.6f}",
          timestamp, x, y, z, roll, pitch, yaw);
  pose_count++;
}

void data_logger_close(void) {
  if (sensor_fp) {
    fprintf(sensor_fp, "\n]\n");
    fclose(sensor_fp);
    sensor_fp = NULL;
  }
  if (pose_fp) {
    fprintf(pose_fp, "\n]\n");
    fclose(pose_fp);
    pose_fp = NULL;
  }
}
