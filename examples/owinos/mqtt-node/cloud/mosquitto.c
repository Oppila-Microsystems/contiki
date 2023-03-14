/*
 * Copyright (c) 2016, Antonio Lignan - antonio.lignan@gmail.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "lib/random.h"
#include "net/rpl/rpl.h"
#include "net/ip/uip.h"
#include "dev/leds.h"
#include "sys/etimer.h"
#include "dev/sys-ctrl.h"
#include "mqtt-client.h"
#include "mqtt-sensors.h"
#include "mqtt-res.h"
#include "mosquitto.h"
#include "pwm.h"
#include "sys/clock.h"

#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
/*---------------------------------------------------------------------------*/
#if DEBUG_PLATFORM
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
/* Pub topic types */
enum {
  PUB_TOPIC_RAW,
  PUB_TOPIC_STRING
};
/*---------------------------------------------------------------------------*/
#define MAX_KEY_LENGTH 20
#define MAX_VALUE_LENGTH 20
#define MAX_ENTRIES 3
/*---------------------------------------------------------------------------*/
#if DEFAULT_SENSORS_NUM
static char *buf_ptr;
static char app_buffer[APP_BUFFER_SIZE];
/*---------------------------------------------------------------------------*/
/* Topic placeholders */
static char data_topic[CONFIG_PUB_TOPIC_LEN];
#endif
static char cmd_topic[CONFIG_SUB_CMD_TOPIC_LEN];
/*---------------------------------------------------------------------------*/
PROCESS(mosquitto_process, "Mosquitto MQTT process");
/*---------------------------------------------------------------------------*/
/* Include there the processes to include */
PROCESS_NAME(mqtt_res_process);
PROCESS_NAME(SENSORS_NAME(MQTT_SENSORS, _sensors_process));
#if DEFAULT_SENSORS_NUM
PROCESS_NAME(mqtt_sensors_process);
/*---------------------------------------------------------------------------*/
static struct etimer alarm_expired;
/*---------------------------------------------------------------------------*/
static int
add_pub_topic(uint16_t length, char *meaning, char *value,
              uint8_t type, uint8_t first, uint8_t more)
{
  int len = 0;
  int pos = 0;
  char *topic = "\"%s\":%s";

  if((buf_ptr == NULL) || (length <= 0)){
    PRINTF("Mosquitto: null buffer or lenght less than zero\n");
    return -1;
  }

  if(first) {
    len = snprintf(buf_ptr, length, "%s", "{");
    pos = len;
    buf_ptr += len;
  }

  if(type == PUB_TOPIC_STRING) {
    topic = "\"%s\":\"%s\"";
  }

  len = snprintf(buf_ptr, (length - pos), topic, meaning, value);

  if(len < 0 || pos >= length) {
    PRINTF("Mosquitto: Buffer too short. Have %d, need %d + \\0\n", length, len);
    return -1;
  }

  pos += len;
  buf_ptr += len;

  if(more) {
    len = snprintf(buf_ptr, (length - pos), "%s", ",");
  } else {
    len = snprintf(buf_ptr, (length - pos), "%s", "}");
  }

  pos += len;
  buf_ptr += len;

  return pos;
}
/*---------------------------------------------------------------------------*/
void
publish_alarm(sensor_val_t *sensor)
{
  uint16_t aux_int, aux_res;

  if(etimer_expired(&alarm_expired)) {

    /* Clear buffer */
    memset(app_buffer, 0, APP_BUFFER_SIZE);

    PRINTF("Mosquitto: Alarm! %s --> %u\n", sensor->alarm_name, sensor->value);
    aux_int = sensor->value;
    aux_res = sensor->value;

    if(sensor->pres > 0) {
      aux_int /= sensor->pres;
      aux_res %= sensor->pres;
    } else {
      aux_res = 0;
    }

    snprintf(app_buffer, APP_BUFFER_SIZE, aux_res > 9 ? "{\"%s\":%d.%02u}" :
             "{\"%s\":%d.%01u0}", sensor->alarm_name, aux_int, aux_res);

    publish((uint8_t *)app_buffer, data_topic, strlen(app_buffer));

    /* Schedule the timer to prevent flooding the broker with the same event */
    etimer_set(&alarm_expired, (CLOCK_SECOND * DEFAULT_ALARM_TIME));
  }
}
/*---------------------------------------------------------------------------*/
void
publish_event(sensor_values_t *msg)
{
  char aux[64];
  int len = 0;
  uint8_t i;
  int16_t aux_int, aux_res;
  int remain = APP_BUFFER_SIZE;

  /* Clear buffer */
  memset(app_buffer, 0, APP_BUFFER_SIZE);

  /* Use the buf_ptr as pointer to the actual application buffer */
  buf_ptr = app_buffer;

  /* Mosquitto does not support string as payload content, default values such as
   * parent address, etc are not sent
   */
  mqtt_res_uptime(aux, sizeof(aux));
  len = add_pub_topic(remain, DEFAULT_PUBLISH_EVENT_UPTIME,
                      aux, PUB_TOPIC_RAW, 1, 1);
  remain =- len;

  mqtt_res_u16_addr(aux, sizeof(aux));

  len = add_pub_topic(remain, DEFAULT_PUBLISH_EVENT_ID, aux,
                      PUB_TOPIC_RAW, 0, 1);
  remain =- len;

  /* Include the sensor values, if `sensor_name` lenght is zero discard */
  for(i=0; i < msg->num; i++) {
    if(strlen(msg->sensor[i].sensor_name)) {
      memset(aux, 0, sizeof(aux));

      aux_int = msg->sensor[i].value;
      aux_res = msg->sensor[i].value;

      if(msg->sensor[i].pres > 0) {
        aux_int /= msg->sensor[i].pres;
        aux_res %= msg->sensor[i].pres;
      } else {
        aux_res = 0;
      }

      snprintf(aux, sizeof(aux), aux_res > 9 ? "%d.%02u" : "%d.%01u0",
               aux_int, aux_res);
      len = add_pub_topic(remain, msg->sensor[i].sensor_name,
                          aux, PUB_TOPIC_RAW, 0, 1);
      remain =- len;
    }
  }

  /* The last value to be sent, the `more` argument should be zero */
  mqtt_res_parent_rssi(aux, sizeof(aux));
  len = add_pub_topic(remain, DEFAULT_PUBLISH_EVENT_RSSI,
                      aux, PUB_TOPIC_RAW, 0, 0);

  PRINTF("Mosquitto: publish %s (%u)\n", app_buffer, strlen(app_buffer));
  publish((uint8_t *)app_buffer, data_topic, strlen(app_buffer));
}
#endif /* DEFAULT_SENSORS_NUM */
/*---------------------------------------------------------------------------*/
/* This function handler receives publications to which we are subscribed */
static void
mosquitto_pub_handler(const char *topic, uint16_t topic_len, const uint8_t *chunk,
            uint16_t chunk_len)
{
	int i,j=1;
	int entry = 0;
	char keys[MAX_ENTRIES][MAX_KEY_LENGTH];
	char values[MAX_ENTRIES][MAX_VALUE_LENGTH];
       
        char *token = strtok(chunk, "{\":,}");
	while (token != NULL) {
		if (strcmp(token, "CONT1") == 0) {
            		strcpy(keys[entry], token);
            		token = strtok(NULL, "{\":,}");
            		snprintf(values[entry], MAX_VALUE_LENGTH, "%s", (token));
    			if(strcmp(values[entry], "1")==0){
				PRINTF("entered if as value is one\n");
				leds_on(CMD_LED);
			}else if(strcmp(values[entry], "0")==0){
				PRINTF("entered else if as value is zero\n");
				leds_off(CMD_LED);
			}else {
       				PRINTF("Mosquitto: invalid command argument (expected boolean)!\n");
      			}
            		entry++;
       		} 
		else if (strcmp(token, "CONT2") == 0) {
            		strcpy(keys[entry], token);
            		token = strtok(NULL, "{\":,}");
            		snprintf(values[entry], MAX_VALUE_LENGTH, "%d", atoi(token));
			PRINTF("pwm value %ls\n", values[entry]);
			if(values[entry]!= "NULL")
			{
				for(j=values[entry];j<100;j++)
				{
				pwm_enable(16000,j,0,1,0);
				pwm_start(1,0,0,4);
				PRINTF("entering if and the value is----------///// %ls\n", values[entry]);
				clock_delay_usec(24464);
				}
			}
			else
				PRINTF("entering else if and the value is %ls\n", values[entry]);
            		entry++;
        	}
		else if (strcmp(token, "CONT3") == 0) {
	            strcpy(keys[entry], token);
        	    token = strtok(NULL, "{\":,}");
            	    snprintf(values[entry], MAX_VALUE_LENGTH, "%d", atoi(token));
           	     entry++;
        	}
		token = strtok(NULL, "{\":,}");		
	}
	
	PRINTF("{");
	for (int i = 0; i < entry; i++) {
        PRINTF("\"%s\": \"%s\"", keys[i], values[i]);
        if (i < entry - 1) {
            PRINTF(", ");
        }
    }
	
    PRINTF("}\n");

}
/*---------------------------------------------------------------------------*/
static void
init_platform(void)
{
  /* Register the publish callback handler */
  MQTT_PUB_REGISTER_HANDLER(mosquitto_pub_handler);

  /* Create client id */
  mqtt_res_client_id(conf.client_id, DEFAULT_IP_ADDR_STR_LEN);
#if DEFAULT_SENSORS_NUM
  /* Create topics, use only the last 12 bytes of the client ID */
  snprintf(data_topic, CONFIG_PUB_TOPIC_LEN, "%s/%s", DEFAULT_TOPIC_STR,
           &conf.client_id[strlen(conf.client_id) - MOSQUITTO_LABEL_LEN]);
#endif
  snprintf(cmd_topic, CONFIG_SUB_CMD_TOPIC_LEN, "%s/%s%s", DEFAULT_TOPIC_LONG,
           &conf.client_id[strlen(conf.client_id) - MOSQUITTO_LABEL_LEN],
           DEFAULT_CMD_STRING);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(mosquitto_process, ev, data)
{
  PROCESS_BEGIN();

  /* Initialize platform-specific */
  init_platform();

  printf("\nMosquitto process started\n");
  printf("  Client ID:    %s\n", conf.client_id);
#if DEFAULT_SENSORS_NUM
  printf("  Data topic:   %s\n", data_topic);
#endif
  printf("  Cmd topic:    %s\n\n", cmd_topic);

  while(1) {

    PROCESS_YIELD();

    if(ev == mqtt_client_event_connected) {

      /* Start the MQTT resource process */
      process_start(&mqtt_res_process, NULL);

      /* Subscribe to topics (MQTT driver only supports 1 topic at the moment */
      subscribe(cmd_topic);

      /* Start the mqtt-sensors process */
      process_start(&mqtt_sensors_process, NULL);

      /* Enable the sensor */
      process_start(&SENSORS_NAME(MQTT_SENSORS, _sensors_process), NULL);
    }

    if(ev == mqtt_client_event_disconnected) {
      /* We are not connected, disable the sensors */
      process_exit(&SENSORS_NAME(MQTT_SENSORS, _sensors_process));
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/** @} */

