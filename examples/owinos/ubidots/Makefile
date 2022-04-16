DEFINES+=PROJECT_CONF_H=\"project-conf.h\"

all: mqtt-client

MQTT_PLATFORM ?= ubidots

MQTT_SENSORS  ?= ubidots_demo

PROJECT_SOURCEFILES += httpd-simple.c mqtt-sensors.c mqtt-res.c

CLOUD_RESOURCES_DIR = ./cloud
DEVICE_RESOURCES_DIR = ./device
INTERNALS_RESOURCES_DIR = ./internals
PROJECTDIRS += $(CLOUD_RESOURCES_DIR) $(DEVICE_RESOURCES_DIR) $(INTERNALS_RESOURCES_DIR)


ifeq ($(MQTT_PLATFORM), ubidots)
  PROJECT_SOURCEFILES += ubidots.c
endif

ifeq ($(MQTT_SENSORS), ubidots_demo)
  PROJECT_SOURCEFILES += ubidots_demo.c
  CONTIKI_TARGET_SOURCEFILES += adc-owinos.c relay.c
  CONTIKI_TARGET_SOURCEFILES += adxl345.c bmp180.c 
  CFLAGS += -DMQTT_SENSORS="ubidots_demo"
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

# Alternatively includes the webserver to configure credentials
WITH_WEBSERVER ?= 1
ifeq ($(WITH_WEBSERVER), 1)
  PROJECT_SOURCEFILES += httpd-simple.c
endif

CFLAGS += -DWITH_WEBSERVER=$(WITH_WEBSERVER)

CONTIKI = ../../../
include $(CONTIKI)/Makefile.include
