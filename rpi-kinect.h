#define KINECT_MOTOR_STOPPED        0x0
#define KINECT_MOTOR_REACHED_LIMITS 0x1
#define KINECT_MOTOR_MOVING         0x4

typedef struct kinect_sensor_values {
  uint16_t ux;
  uint16_t uy;
  uint16_t uz;
  uint8_t positive_angle_degrees;
  uint8_t status_code;
} kinect_sensor_values;
