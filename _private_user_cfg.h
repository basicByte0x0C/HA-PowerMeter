#ifndef PRIVATE_USER_CFG_H
#define PRIVATE_USER_CFG_H

/* Define here all private data needed for the project */
/* This file shall not be uploaded anywhere, only the template */

#define PRIVATE_CFG_WIFI_SSID               "WifiSSID"
#define PRIVATE_CFG_WIFI_PASSWORD           "WiFi+Password1"
#define PRIVATE_CFG_WIFI_STATIC_IP          IPAddress(192, 168, 0, 3)
#define PRIVATE_CFG_WIFI_GATEWAY            IPAddress(192, 168, 0, 1)
#define PRIVATE_CFG_WIFI_SUBMASK            IPAddress(255, 255, 255, 0)
#define PRIVATE_CFG_WIFI_DNS                IPAddress(9, 9, 9, 9)
#define PRIVATE_CFG_MQTT_USER               "mqtt_username"
#define PRIVATE_CFG_MQTT_PASSWORD           "mqtt_Password1"
#define PRIVATE_CFG_MQTT_BROKER_ADDRESS     IPAddress(192, 168, 0, 2)
#define PRIVATE_CFG_MQTT_MAC_ADDRESS        0x00, 0x01, 0x02, 0x03, 0x04, 0x05
#define PRIVATE_CFG_HA_DEVICE_NAME          "MyDevince"
#define PRIVATE_CFG_MQTT_BASE_TOPIC         "mydevice/sensors/"        

#endif /* PRIVATE_USER_CFG_H */

