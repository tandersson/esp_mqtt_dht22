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
#include "driver/uart.h"
#include "driver/dht22.h"
#include "osapi.h"
#include "mqtt.h"
#include "wifi.h"
#include "config.h"
#include "debug.h"
#include "gpio.h"
#include "user_interface.h"
#include "mem.h"

#define SAMPLE_PERIOD 10000 /* milliseconds */
#define LED_GPIO 0
#define LED_GPIO_MUX PERIPHS_IO_MUX_GPIO0_U
#define LED_GPIO_FUNC FUNC_GPIO0

#define MQTT_TOPIC_MAX_LEN 80
#define MQTT_DATA_MAX_LEN 20

LOCAL os_timer_t dht22_timer;

MQTT_Client mqttClient;
uint8_t mac_addr[6];
char root_topic[20]; /* globally unique root topic */

LOCAL void ICACHE_FLASH_ATTR dht22_cb(void *arg)
{
	struct dht_sensor_data* r = DHTRead();
	float lastTemp = r->temperature;
	float lastHum = r->humidity;
	char *topic_buf = (char*)os_zalloc(MQTT_TOPIC_MAX_LEN);
	char *data_buf = (char*)os_zalloc(MQTT_DATA_MAX_LEN);

	if(r->success)
	{
		INFO("Temperature: %d.%d *C, Humidity: %d.%d %%\r\n", (int)(lastTemp),(int)((lastTemp - (int)lastTemp)*100), (int)(lastHum),(int)((lastHum - (int)lastHum)*100));
		os_sprintf(topic_buf,"%s/temperature", root_topic);
		INFO("Topic: %s\r\n", topic_buf);
		os_sprintf(data_buf,"%d.%d", (int)(lastTemp),(int)((lastTemp - (int)lastTemp)*100));
		MQTT_Publish(&mqttClient, topic_buf, data_buf, os_strlen(data_buf), 0, 0);
		os_sprintf(topic_buf,"%s/humidity", root_topic);
		INFO("Topic: %s\r\n", topic_buf);
		os_sprintf(data_buf,"%d.%d", (int)(lastHum),(int)((lastHum - (int)lastHum)*100));
		MQTT_Publish(&mqttClient, topic_buf, data_buf, os_strlen(data_buf), 0, 0);
	}
	else
	{
		INFO("Error reading temperature and humidity\r\n");
	}
	os_free(topic_buf);
	os_free(data_buf);
}

void wifiConnectCb(uint8_t status)
{
	if(status == STATION_GOT_IP){
		MQTT_Connect(&mqttClient);
	} else {
		MQTT_Disconnect(&mqttClient);
	}
}
void mqttConnectedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	char *topic_match_buf = (char*)os_zalloc(MQTT_TOPIC_MAX_LEN);
	os_sprintf(topic_match_buf, "%s/led", root_topic);
	INFO("MQTT: Connected\r\n");
	MQTT_Subscribe(&mqttClient, topic_match_buf, 0);
	os_free(topic_match_buf);
}

void mqttDisconnectedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Disconnected\r\n");
}

void mqttPublishedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Published\r\n");
}

void mqttDataCb(uint32_t *args, const char* topic, uint32_t topic_len, const char *data, uint32_t data_len)
{
	char *topic_buf = (char*)os_zalloc(topic_len+1);
	char *data_buf = (char*)os_zalloc(data_len+1);
	char *topic_match_buf = (char*)os_zalloc(MQTT_TOPIC_MAX_LEN);

	MQTT_Client* client = (MQTT_Client*)args;

	os_memcpy(topic_buf, topic, topic_len);
	topic_buf[topic_len] = 0;

	os_memcpy(data_buf, data, data_len);
	data_buf[data_len] = 0;

	os_sprintf(topic_match_buf, "%s/led", root_topic);

	INFO("MQTT topic: %s, data: %s \r\n", topic_buf, data_buf);

	if (os_strcmp(topic_buf,topic_match_buf) == 0)
	{
		if((os_strcmp(data_buf,"on") == 0) || (os_strcmp(data_buf,"ON") == 0))
		{
			GPIO_OUTPUT_SET(LED_GPIO, 1);
		}
		else if((os_strcmp(data_buf,"off") == 0) || (os_strcmp(data_buf,"OFF") == 0))
		{
			GPIO_OUTPUT_SET(LED_GPIO, 0);
		}
	}
	os_free(topic_buf);
	os_free(data_buf);
	os_free(topic_match_buf);
}


void user_init(void)
{
	PIN_FUNC_SELECT(LED_GPIO_MUX, LED_GPIO_FUNC);
	GPIO_OUTPUT_SET(LED_GPIO, 0);

	uart_init(BIT_RATE_115200, BIT_RATE_115200);
	DHTInit(DHT22, 2000);

	os_delay_us(1000000);

	CFG_Load();

    wifi_get_macaddr(0x00, mac_addr);
    os_sprintf(root_topic, "/esp8266_%02x%02x%02x%02x%02x%02x",
        mac_addr[0], mac_addr[1], mac_addr[2], 
        mac_addr[3], mac_addr[4], mac_addr[5]);

	MQTT_InitConnection(&mqttClient, sysCfg.mqtt_host, sysCfg.mqtt_port, SEC_NONSSL);
	MQTT_InitClient(&mqttClient, sysCfg.device_id, sysCfg.mqtt_user,
	    sysCfg.mqtt_pass, sysCfg.mqtt_keepalive, 1);
	MQTT_OnConnected(&mqttClient, mqttConnectedCb);
	MQTT_OnDisconnected(&mqttClient, mqttDisconnectedCb);
	MQTT_OnPublished(&mqttClient, mqttPublishedCb);
	MQTT_OnData(&mqttClient, mqttDataCb);

	WIFI_Connect(sysCfg.sta_ssid, sysCfg.sta_pwd, wifiConnectCb);

	INFO("\r\nSystem started ...\r\n");
    INFO("SSID: %s\r\n",sysCfg.sta_ssid);
    INFO("MQTT Server: %s\r\n",sysCfg.mqtt_host);
    INFO("MQTT Port: %d\r\n",sysCfg.mqtt_port);
    INFO("MQTT User: %s\r\n",sysCfg.mqtt_user);
    INFO("MQTT Root Topic: %s\r\n", root_topic);

	os_timer_disarm(&dht22_timer);
	os_timer_setfn(&dht22_timer, (os_timer_func_t *)dht22_cb, (void *)0);
	os_timer_arm(&dht22_timer, SAMPLE_PERIOD, 1);
}
