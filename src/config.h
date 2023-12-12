#ifndef _CONFIG_H
#define _CONFIG_H

#define TINY_GSM_MODEM_SIM7080

/* Define the serial console for debug prints, if needed */
#define TINY_GSM_DEBUG SerialMon
/* uncomment to dump all AT commands */
// #define DEBUG_DUMP_AT_COMMAND

#define UPLOAD_INTERVAL     10000

#define SerialMon           Serial
#define MONITOR_BAUDRATE    115200

#define SerialAT            Serial1
#define SIM7020_BAUDRATE    115200
#define SIM7020_RESET       -1
#define SIM7020_EN          -1
#define SIM7020_TX          13
#define SIM7020_RX          5

// MQTT Setting
#define MQTT_BROKER             "mqtt.m5stack.com"
#define MQTT_PORT               1883
#define MQTT_USERNAME           "IoTBase-Thermal"
#define MQTT_PASSWORD           "IoTBase-Thermal"

#define MQTT_SYNC_TIME_D_TOPIC  "IoTBase/thermal/timesync/down"
#define MQTT_SYNC_TIME_U_TOPIC  "IoTBase/thermal/timesync/up"

#define MQTT_TOPIC              "temp/humidity"                      //  上传数据主题

#endif /* _CONFIG_H */
