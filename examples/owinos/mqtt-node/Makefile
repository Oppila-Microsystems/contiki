DEFINES+=PROJECT_CONF_H=\"project-conf.h\"

all: mqtt-client

MQTT_PLATFORM ?= mosquitto

MQTT_SENSORS  ?= mosquitto_demo

PROJECT_SOURCEFILES += mqtt-sensors.c mqtt-res.c

CLOUD_RESOURCES_DIR = ./cloud
DEVICE_RESOURCES_DIR = ./device
INTERNALS_RESOURCES_DIR = ./internals
PROJECTDIRS += $(CLOUD_RESOURCES_DIR) $(DEVICE_RESOURCES_DIR) $(INTERNALS_RESOURCES_DIR)

# Platform checks

ifeq ($(MQTT_PLATFORM), mosquitto)
  PROJECT_SOURCEFILES += mosquitto.c
endif

# Sensor checks

ifeq ($(MQTT_SENSORS), mosquitto_demo)
  PROJECT_SOURCEFILES += mosquitto_demo.c
  CONTIKI_TARGET_SOURCEFILES += adc-owinos.c relay.c
  CONTIKI_TARGET_SOURCEFILES += adxl345.c bmp180.c 
  CFLAGS += -DMQTT_SENSORS="mosquitto_demo"
  CFLAGS += -DDEFAULT_SENSORS_NUM=6
  CFLAGS += -DDEFAULT_COMMANDS_NUM=1
endif


WITH_LLSEC ?= 1
CFLAGS += -DWITH_LLSEC_ENABLED=$(WITH_LLSEC)

WITH_IP64 = 1

CFLAGS += -DMQTT_PLATFORM=$(MQTT_PLATFORM)

APPS += mqtt

# MQTT internals
PROJECT_SOURCEFILES += mqtt-sensors.c mqtt-res.c

#CFLAGS += -DWITH_WEBSERVER=$(WITH_WEBSERVER)

CONTIKI = ../../../
include $(CONTIKI)/Makefile.include
