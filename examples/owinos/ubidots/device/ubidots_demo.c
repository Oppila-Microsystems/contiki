#include "contiki.h"
#include "sys/etimer.h"
#include "dev/adc-owinos.h"
#include "dev/adxl345.h"
#include "dev/relay.h"
#include "dev/i2c.h"
#include "dev/bmp180.h"
#include "ubidots_demo.h"
#include "mqtt-res.h"

#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
/*---------------------------------------------------------------------------*/
#if DEBUG_APP
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
sensor_values_t ubidots_demo_sensors;
command_values_t ubidots_demo_commands;
/*---------------------------------------------------------------------------*/
process_event_t ubidots_demo_sensors_data_event;
process_event_t ubidots_demo_sensors_alarm_event;
/*---------------------------------------------------------------------------*/
PROCESS(ubidots_demo_sensors_process, "Agriculture sensor process");
/*---------------------------------------------------------------------------*/
static int
activate_actuator(int arg)
{
  if(!arg) {
    relay.value(RELAY_ON);
  } else {
    relay.value(RELAY_OFF);
  }
  process_poll(&ubidots_demo_sensors_process);
  return 0;
}
/*---------------------------------------------------------------------------*/
static void
poll_sensors(void)
{
  /* Poll the temperature and humidity sensor */
  
  ubidots_demo_sensors.sensor[ANALOG_SENSOR_LDR].value = adc_owinos.value(OWINOS_SENSORS_ADC1);

  SENSORS_ACTIVATE(adxl345);
  ubidots_demo_sensors.sensor[DIGITAL_SENSOR_X_AXIS].value = adxl345.value(X_AXIS);
  ubidots_demo_sensors.sensor[DIGITAL_SENSOR_Y_AXIS].value = adxl345.value(Y_AXIS);
  ubidots_demo_sensors.sensor[DIGITAL_SENSOR_Z_AXIS].value = adxl345.value(Z_AXIS);
  SENSORS_DEACTIVATE(adxl345);

  SENSORS_ACTIVATE(bmp180);
  ubidots_demo_sensors.sensor[DIGITAL_SENSOR_PRES].value = bmp180.value(BMP180_READ_PRESSURE);
  ubidots_demo_sensors.sensor[DIGITAL_SENSOR_TEMP].value = bmp180.value(BMP180_READ_TEMP);
  SENSORS_DEACTIVATE(bmp180);

  /* Check the sensor values and publish alarms if required, else send the data
   * to any subscriber
   */
  mqtt_sensor_check(&ubidots_demo_sensors, ubidots_demo_sensors_alarm_event,
                    ubidots_demo_sensors_data_event);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ubidots_demo_sensors_process, ev, data)
{
  static struct etimer et;

  /* This is where our process start */
  PROCESS_BEGIN();

  /* Load sensor defaults */
  ubidots_demo_sensors.num = 0;

  adc_owinos.configure(SENSORS_HW_INIT, OWINOS_SENSORS_ADC1);

  mqtt_sensor_register(&ubidots_demo_sensors, ANALOG_SENSOR_LDR,
                       DEFAULT_WRONG_VALUE, DEFAULT_PUBLISH_EVENT_LDR,
                       NULL, DEFAULT_SUBSCRIBE_CFG_LDRTHR, DEFAULT_CC2538_LDR_MIN,
                       DEFAULT_CC2538_LDR_MAX, DEFAULT_LDR_THRESH,
                       DEFAULT_LDR_THRESL, 100);
 
  mqtt_sensor_register(&ubidots_demo_sensors, DIGITAL_SENSOR_X_AXIS,
                       DEFAULT_WRONG_VALUE, DEFAULT_PUBLISH_EVENT_X_AXIS,
                       NULL, DEFAULT_SUBSCRIBE_CFG_X_AXISHR,
                       DEFAULT_ADXL345_X_AXIS_MIN, DEFAULT_ADXL345_X_AXIS_MAX,
                       DEFAULT_ADXL345_X_AXIS_THRESH, DEFAULT_ADXL345_X_AXIS_THRESL,0);

  mqtt_sensor_register(&ubidots_demo_sensors, DIGITAL_SENSOR_Y_AXIS,
                       DEFAULT_WRONG_VALUE, DEFAULT_PUBLISH_EVENT_Y_AXIS,
                       NULL, DEFAULT_SUBSCRIBE_CFG_Y_AXISHR,
                       DEFAULT_ADXL345_Y_AXIS_MIN, DEFAULT_ADXL345_Y_AXIS_MAX,
                       DEFAULT_ADXL345_Y_AXIS_THRESH, DEFAULT_ADXL345_Y_AXIS_THRESL, 0);

  mqtt_sensor_register(&ubidots_demo_sensors, DIGITAL_SENSOR_Z_AXIS,
                       DEFAULT_WRONG_VALUE, DEFAULT_PUBLISH_EVENT_Z_AXIS,
                       NULL, DEFAULT_SUBSCRIBE_CFG_Z_AXISHR,
                       DEFAULT_ADXL345_Z_AXIS_MIN, DEFAULT_ADXL345_Z_AXIS_MAX,
                       DEFAULT_ADXL345_Z_AXIS_THRESH, DEFAULT_ADXL345_Z_AXIS_THRESL, 0);

  mqtt_sensor_register(&ubidots_demo_sensors, DIGITAL_SENSOR_PRES,
                       DEFAULT_BMP180_PRES_MIN, DEFAULT_PUBLISH_EVENT_PRES,
                       DEFAULT_PUBLISH_ALARM_PRES, DEFAULT_SUBSCRIBE_CFG_PRESTHR,
                       DEFAULT_BMP180_PRES_MIN, DEFAULT_BMP180_PRES_MAX,
                       DEFAULT_PRES_THRESH, DEFAULT_PRES_THRESL, 10);

  mqtt_sensor_register(&ubidots_demo_sensors, DIGITAL_SENSOR_TEMP,
                       DEFAULT_BMP180_TEMP_MIN, DEFAULT_PUBLISH_EVENT_TEMP,
                       DEFAULT_PUBLISH_ALARM_TEMP, DEFAULT_SUBSCRIBE_CFG_TEMPTHR,
                       DEFAULT_BMP180_TEMP_MIN, DEFAULT_BMP180_TEMP_MAX,
                       DEFAULT_TEMP_THRESH, DEFAULT_TEMP_THRESL, 10);

  /* Sanity check */
  if(ubidots_demo_sensors.num != DEFAULT_SENSORS_NUM) {
    printf("Ubidots sensors: error! number of sensors mismatch\n");
    PROCESS_EXIT();
  }

  /* Load commands default */
  ubidots_demo_commands.num = 1;
  memcpy(ubidots_demo_commands.command[UBIDOTS_DEMO_COMMAND].command_name,
         DEFAULT_COMMAND_EVENT_RELAY, strlen(DEFAULT_COMMAND_EVENT_RELAY));
  ubidots_demo_commands.command[UBIDOTS_DEMO_COMMAND].cmd = activate_actuator;

  if(ubidots_demo_commands.num != DEFAULT_COMMANDS_NUM) {
    printf("Ubidots sensors: error! number of commands mismatch\n");
    PROCESS_EXIT();
  }

  /* Get an event ID for our events */
  ubidots_demo_sensors_data_event = process_alloc_event();
  ubidots_demo_sensors_alarm_event = process_alloc_event();

  /* Activate Relay for GPIO activation */
  SENSORS_ACTIVATE(relay);
  activate_actuator(0);

  /* Start the periodic process */
  etimer_set(&et, DEFAULT_SAMPLING_INTERVAL);

  while(1) {

    PROCESS_YIELD();

    if(ev == PROCESS_EVENT_TIMER && data == &et) {
      poll_sensors();
      etimer_reset(&et);
    } else if(ev == sensors_stop_event) {
      PRINTF("Ubidots: sensor readings paused\n");
      etimer_stop(&et);
    } else if(ev == sensors_restart_event) {
      PRINTF("Ubidots: sensor readings enabled\n");
      etimer_reset(&et);
    }
  }

  PROCESS_END();
}

