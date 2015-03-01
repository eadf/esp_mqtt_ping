/* main.c -- MQTT client example
 *
 * Copyright (c) 2014-2015, Tuan PM <tuanpm at live dot com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of Redis nor the names of its contributors may be used
 * to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "ets_sys.h"
#include "osapi.h"
#include "mqtt.h"
#include "wifi.h"
#include "config.h"
#include "debug.h"
#include "gpio.h"
#include "user_interface.h"
#include "mem.h"
#include "stdout/stdout.h"
#include "ping/ping.h"
#include "user_config.h"

MQTT_Client mqttClient;
volatile static bool mqttIsIdle = true;
#define NUMBER_OF_SENSORS 3
static Ping_Data pingSetting[NUMBER_OF_SENSORS];
static volatile os_timer_t loop_timer;
static char pingTopic[25];    // the MQTT topic we send to
static char pingMessage[100]; // the MQTT message we send

static void setup(void);
static void loop(void);

void ICACHE_FLASH_ATTR
wifiConnectCb(uint8_t status) {

  os_sprintf(pingTopic, "/%0x/ping", system_get_chip_id());
  if (status == STATION_GOT_IP) {
    MQTT_Connect(&mqttClient);
  } else {
    MQTT_Disconnect(&mqttClient);
  }
}

/**
 * a callback that tells us if mqtt is busy or not
 */
void ICACHE_FLASH_ATTR
mqttPublishedCb(uint32_t *args) {
  mqttIsIdle = true;
}

/**
 * This is the main user program loop
 */
void ICACHE_FLASH_ATTR
loop(void) {
  static float oldDistance[3] = {-2.0, -2.0, -2.0}; // ping_pingDistance() only returns positive values (if any)
  static float maxDistance = 1000; // 1 meter
  static uint8_t currentSensor = 0; // a counter that iterates over each sensor
  bool sentSomething = false;

  float distance = 0;
  if (mqttIsIdle) {
    // do nothing if mqtt is busy
    if (ping_ping(pingSetting+currentSensor, maxDistance, &distance) ) {
      if (oldDistance[currentSensor] != distance) {
        os_sprintf(pingMessage,"Sensor %d: Distance ~ %d mm", currentSensor, (int)distance);
        mqttIsIdle=false;
        sentSomething=true;
        MQTT_Publish(&mqttClient, pingTopic, pingMessage, strlen(pingMessage), 0, 0);
        os_printf("%s:%s\n", pingTopic, pingMessage);
        oldDistance[currentSensor] = distance;
      }
    } else {
      if (oldDistance[currentSensor] != -1.0) {
        os_sprintf(pingMessage,"Sensor %d: Failed to get any response.", currentSensor);
        MQTT_Publish(&mqttClient, pingTopic, pingMessage, strlen(pingMessage), 0, 0);
        os_printf("%s:%s\n", pingTopic, pingMessage);
        oldDistance[currentSensor] = -1.0; //
      }
    }
    currentSensor = currentSensor>=NUMBER_OF_SENSORS-1?0:currentSensor+1;
  } else {
    os_printf("mqttIsIdle\n");
  }

  // we re-arm timer each iteration, be really quick if nothing was sent
  os_timer_arm(&loop_timer, sentSomething?PING_SAMPLE_PERIOD:1, false);
}

static void ICACHE_FLASH_ATTR
setup(void) {
  CFG_Load();

  MQTT_InitConnection(&mqttClient, sysCfg.mqtt_host, sysCfg.mqtt_port, sysCfg.security);
  MQTT_InitClient(&mqttClient, sysCfg.device_id, sysCfg.mqtt_user, sysCfg.mqtt_pass, sysCfg.mqtt_keepalive, 1);
  MQTT_InitLWT(&mqttClient, "/lwt", "offline", 0, 0);
  MQTT_OnPublished(&mqttClient, mqttPublishedCb);
  WIFI_Connect(sysCfg.sta_ssid, sysCfg.sta_pwd, wifiConnectCb);

  ping_init(pingSetting+0,  2,  0, PING_MM);   // trigger= GPIO2, echo= GPIO0, set the pins to the same value for one-pin-mode
  ping_init(pingSetting+1,  5,  4, PING_MM);   // trigger= GPIO5, echo= GPIO4, set the pins to the same value for one-pin-mode
  ping_init(pingSetting+2, 13, 12, PING_MM);   // trigger=GPIO13, echo=GPIO12, set the pins to the same value for one-pin-mode

  os_printf("System initiated\n");

  // Start loop timer
  os_timer_disarm(&loop_timer);
  os_timer_setfn(&loop_timer, (os_timer_func_t *) loop, NULL);
  os_timer_arm(&loop_timer, PING_SAMPLE_PERIOD, false);
}

void user_init(void) {
  // Initialize the GPIO subsystem.
  gpio_init();

  // Make uart0 work with just the TX pin. Baud:115200,n,8,1
  // The RX pin is now free for GPIO use (GPIO3).
  stdout_init();

  //Set the setup timer
  os_timer_disarm(&loop_timer);
  os_timer_setfn(&loop_timer, (os_timer_func_t *) setup, NULL);
  os_timer_arm(&loop_timer, 1000, false);

  INFO("\nSystem started ...\n");
}
