#ifndef WIFI_H
#define WIFI_H

#define WIFI_SSID "HoangViVina"
#define WIFI_PASS "244466666"
#define PORT 3333

void init_task_tcp();
void tcp_client(void *pvParameters);
void tcp_wifi_connection();

#endif // define WIFI_H