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
#include <ets_sys.h>
#include <osapi.h>
#include <wifi.h>
#include <config.h>
#include <debug.h>
#include <gpio.h>
#include <mem.h>
#include <stddef.h>
#include "driver/uart.h"
#include "driver/dht22.h"
#include "driver/ds18b20.h"
#include "mqtt.h"
#include "user_interface.h"

#define SAMPLE_PERIOD 60000 /* milliseconds */
#define LED_GPIO 0
#define LED_GPIO_MUX PERIPHS_IO_MUX_GPIO0_U
#define LED_GPIO_FUNC FUNC_GPIO0

#define MQTT_TOPIC_MAX_LEN 80
#define MQTT_DATA_MAX_LEN 80

/* Select the wanted sensor type */
//#define DHT22_SENSOR
#define DS18B20_SENSOR

#ifdef DHT22_SENSOR
LOCAL os_timer_t dht22_timer;
#endif

#ifdef DS18B20_SENSOR
LOCAL os_timer_t ds18b20_timer;
#endif

MQTT_Client mqttClient;
uint8_t mac_addr[6];

int dmqtz_in_index = DOMOTICZ_IN_INDEX; /* domoticz index (sensor), adjust for your system, should be configurable */
int dmqtz_out_index = DOMOTICZ_OUT_INDEX; /* domoticz index, adjust for your system, should be configurable */

#ifdef DHT22_SENSOR
LOCAL void ICACHE_FLASH_ATTR dht22_cb(void *arg)
{
	struct dht_sensor_data* r = DHTRead();
	float lastTemp = r->temperature;
	float lastHum = r->humidity;
	char *topic_buf = (char*)os_zalloc(MQTT_TOPIC_MAX_LEN);
	char *data_buf = (char*)os_zalloc(MQTT_DATA_MAX_LEN);
	char *hum_str = (char*)os_zalloc(10);
	char *temp_str = (char*)os_zalloc(10);	

	if(r->success)
	{
		os_sprintf(temp_str, "%d.%d", (int)(lastTemp),(int)((lastTemp - (int)lastTemp)*100));
		os_sprintf(hum_str, "%d.%d", (int)(lastHum),(int)((lastHum - (int)lastHum)*100));		
		os_sprintf(topic_buf,"domoticz/in");
		os_sprintf(data_buf,"{\"idx\":%d,\"nvalue\":0,\"svalue\":\"%s;%s;%d\"}", dmqtz_in_index, temp_str, hum_str, 0);
		INFO("Topic: %s, Data: %s\r\n", topic_buf, data_buf);
		MQTT_Publish(&mqttClient, topic_buf, data_buf, os_strlen(data_buf), 0, 0);
	}
	else
	{
		INFO("Error reading temperature and humidity\r\n");
	}
	os_free(topic_buf);
	os_free(data_buf);
	os_free(hum_str);
	os_free(temp_str);
}
#endif

#ifdef DS18B20_SENSOR
LOCAL void ICACHE_FLASH_ATTR ds18b20_cb(void *arg)
{
    char *temp_str = (char*)os_zalloc(10);
	char *topic_buf = (char*)os_zalloc(MQTT_TOPIC_MAX_LEN);
	char *data_buf = (char*)os_zalloc(MQTT_DATA_MAX_LEN);

	if(ds18b20_read_temp_str(temp_str))
	{
		//INFO("Temperature: %s C \r\n", temp_str);
		os_sprintf(topic_buf,"domoticz/in");
		os_sprintf(data_buf,"{\"idx\":%d,\"nvalue\":0,\"svalue\":\"%s\"}", dmqtz_in_index, temp_str);
		INFO("Topic: %s, Data: %s\r\n", topic_buf, data_buf);
		MQTT_Publish(&mqttClient, topic_buf, data_buf, os_strlen(data_buf), 0, 0);
	}
	else
	{
		INFO("Could not read the DS18B20 temperature sensor\r\n");
	}	
	os_free(temp_str);	
	os_free(topic_buf);
	os_free(data_buf);
}
#endif



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
	os_sprintf(topic_match_buf, "%s", "domoticz/out");
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
	char *data_match_str = (char*)os_zalloc(MQTT_TOPIC_MAX_LEN);

	MQTT_Client* client = (MQTT_Client*)args;

	os_memcpy(topic_buf, topic, topic_len);
	topic_buf[topic_len] = 0;

	os_memcpy(data_buf, data, data_len);
	data_buf[data_len] = 0;

	os_sprintf(data_match_str, "\"idx\" : %d,", dmqtz_out_index);

	if (os_strcmp(topic_buf,"domoticz/out") == 0)
	{
		if (os_strstr(data_buf,data_match_str))
		{
			if(os_strstr(data_buf,"\"nvalue\" : 0,"))
			{
				INFO("MQTT topic: %s, data: %s - LED OFF \r\n", topic_buf, data_buf);
				GPIO_OUTPUT_SET(LED_GPIO, 0);
			}
			else if(os_strstr(data_buf,"\"nvalue\" : 1,"))
			{
				INFO("MQTT topic: %s, data: %s - LED ON \r\n", topic_buf, data_buf);
				GPIO_OUTPUT_SET(LED_GPIO, 1);
			}
		}

	}
	os_free(topic_buf);
	os_free(data_buf);
	os_free(data_match_str);
}


void user_init(void)
{
	PIN_FUNC_SELECT(LED_GPIO_MUX, LED_GPIO_FUNC);
	GPIO_OUTPUT_SET(LED_GPIO, 0);

	uart_init(BIT_RATE_115200, BIT_RATE_115200);

#ifdef DHT22_SENSOR	
	DHTInit(DHT22, 2000);
#endif

	os_delay_us(1000000);

	CFG_Load();

    wifi_get_macaddr(0x00, mac_addr);

	MQTT_InitConnection(&mqttClient, sysCfg.mqtt_host, sysCfg.mqtt_port, SEC_NONSSL);
	MQTT_InitClient(&mqttClient, sysCfg.device_id, sysCfg.mqtt_user,
	    sysCfg.mqtt_pass, sysCfg.mqtt_keepalive, 1);
	MQTT_OnConnected(&mqttClient, mqttConnectedCb);
	MQTT_OnDisconnected(&mqttClient, mqttDisconnectedCb);
	MQTT_OnPublished(&mqttClient, mqttPublishedCb);
	MQTT_OnData(&mqttClient, mqttDataCb);

	WIFI_Connect(sysCfg.sta_ssid, sysCfg.sta_pwd, wifiConnectCb);

	INFO("\r\nSystem started ...\r\n");
    INFO("MAC Addr: %02x%02x%02x%02x%02x%02x\r\n",
        mac_addr[0], mac_addr[1], mac_addr[2], 
        mac_addr[3], mac_addr[4], mac_addr[5]);
    INFO("SSID: %s\r\n",sysCfg.sta_ssid);
    INFO("MQTT Server: %s\r\n",sysCfg.mqtt_host);
    INFO("MQTT Port: %d\r\n",sysCfg.mqtt_port);
    INFO("MQTT User: %s\r\n",sysCfg.mqtt_user);
    //INFO("MQTT Root Topic: %s\r\n", root_topic);

#ifdef DHT22_SENSOR
	os_timer_disarm(&dht22_timer);
	os_timer_setfn(&dht22_timer, (os_timer_func_t *)dht22_cb, (void *)0);
	os_timer_arm(&dht22_timer, SAMPLE_PERIOD, 1);
#endif

#ifdef DS18B20_SENSOR
	os_timer_disarm(&ds18b20_timer);
	os_timer_setfn(&ds18b20_timer, (os_timer_func_t *)ds18b20_cb, (void *)0);
	os_timer_arm(&ds18b20_timer, SAMPLE_PERIOD, 1);
#endif
}
