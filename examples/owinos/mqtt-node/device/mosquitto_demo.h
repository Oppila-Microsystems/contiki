#ifndef MOSQUITTO_DEMO_H_
#define MOSQUITTO_DEMO_H_
/*---------------------------------------------------------------------------*/
#include "mqtt-sensors.h"
/*---------------------------------------------------------------------------*/
enum {
  ANALOG_SENSOR_LDR = 0,
  DIGITAL_SENSOR_TEMP,
  DIGITAL_SENSOR_PRES,
  DIGITAL_SENSOR_X_AXIS,
  DIGITAL_SENSOR_Y_AXIS,
  DIGITAL_SENSOR_Z_AXIS,
};
enum {
        MOSQUITTO_DEMO_COMMAND = 0,
};

/* Sensor process events */
extern process_event_t mosquitto_demo_sensors_data_event;
extern process_event_t mosquitto_demo_sensors_alarm_event;
/*---------------------------------------------------------------------------*/
extern sensor_values_t mosquitto_demo_sensors;
/*---------------------------------------------------------------------------*/
extern command_values_t mosquitto_demo_commands;

/* PUBLISH strings */
#define DEFAULT_PUBLISH_EVENT_LDR     	"ldr"
#define DEFAULT_PUBLISH_ALARM_LDR     	"alarm_ldr"
#define DEFAULT_PUBLISH_EVENT_TEMP      "temperature"
#define DEFAULT_PUBLISH_EVENT_PRES      "pressure"
#define DEFAULT_PUBLISH_ALARM_TEMP      "alarm_temperature"
#define DEFAULT_PUBLISH_ALARM_PRES      "alarm_pressure"
#define DEFAULT_PUBLISH_EVENT_X_AXIS	"xaxis"
#define DEFAULT_PUBLISH_ALARM_X_AXIS    "alarm_x-axis"
#define DEFAULT_PUBLISH_EVENT_Y_AXIS    "yaxis"
#define DEFAULT_PUBLISH_ALARM_Y_AXIS    "alarm_y-axis"
#define DEFAULT_PUBLISH_EVENT_Z_AXIS    "zaxis"
#define DEFAULT_PUBLISH_ALARM_Z_AXIS    "alarm_zaxis"

/* SUBSCRIBE strings */
#define DEFAULT_SUBSCRIBE_CFG_LDRTHR    "ldr_thresh"
#define DEFAULT_SUBSCRIBE_CFG_TEMPTHR   "temperature_thresh"
#define DEFAULT_SUBSCRIBE_CFG_X_AXISHR  "xaxis_thresh"
#define DEFAULT_SUBSCRIBE_CFG_Y_AXISHR  "yaxis_thresh"
#define DEFAULT_SUBSCRIBE_CFG_Z_AXISHR  "zaxis_thresh"
#define DEFAULT_SUBSCRIBE_CFG_PRESTHR	"pressure_thresh"

/* Minimum and maximum values for the sensors */
#define DEFAULT_CC2538_LDR_MIN		(-150)
#define DEFAULT_CC2538_LDR_MAX          1500

#define DEFAULT_ADXL345_X_AXIS_MIN	(-10000)
#define DEFAULT_ADXL345_X_AXIS_MAX      10000
#define DEFAULT_ADXL345_Y_AXIS_MIN      (-1000)
#define DEFAULT_ADXL345_Y_AXIS_MAX      1000
#define DEFAULT_ADXL345_Z_AXIS_MIN      (-1000)
#define DEFAULT_ADXL345_Z_AXIS_MAX      1000

#define DEFAULT_BMP180_PRES_MIN		3000
#define DEFAULT_BMP180_PRES_MAX         11000
#define DEFAULT_BMP180_TEMP_MIN         (-2000)
#define DEFAULT_BMP180_TEMP_MAX         12000

#define DEFAULT_WRONG_VALUE             (-3000)

#define DEFAULT_LDR_THRESH		DEFAULT_CC2538_LDR_MAX
#define DEFAULT_ADXL345_X_AXIS_THRESH   DEFAULT_ADXL345_X_AXIS_MAX
#define DEFAULT_ADXL345_Y_AXIS_THRESH   DEFAULT_ADXL345_Y_AXIS_MAX
#define DEFAULT_ADXL345_Z_AXIS_THRESH	DEFAULT_ADXL345_Z_AXIS_MAX

#define DEFAULT_LDR_THRESL		DEFAULT_CC2538_LDR_MIN
#define DEFAULT_ADXL345_X_AXIS_THRESL   DEFAULT_ADXL345_X_AXIS_MIN
#define DEFAULT_ADXL345_Y_AXIS_THRESL   DEFAULT_ADXL345_Y_AXIS_MIN
#define DEFAULT_ADXL345_Z_AXIS_THRESL   DEFAULT_ADXL345_Z_AXIS_MIN


#define DEFAULT_TEMP_THRESH             3000
#define DEFAULT_PRES_THRESH             10500
#define DEFAULT_TEMP_THRESL		DEFAULT_BMP180_TEMP_MIN
#define DEFAULT_PRES_THRESL		DEFAULT_BMP180_PRES_MIN

#define DEFAULT_COMMAND_EVENT_RELAY     "relay_toggle"
#define DEFAULT_CMD_STRING              DEFAULT_COMMAND_EVENT_RELAY

#define DEFAULT_CONF_ALARM_TIME         80
/*---------------------------------------------------------------------------*/
#endif /* MOSQUITTO_DEMO_H_ */
/** @} */



