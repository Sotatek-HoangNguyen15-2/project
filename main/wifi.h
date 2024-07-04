#ifndef WIFI_H
#define WIFI_H

#define WIFI_SSID "P14"
#define WIFI_PASS "23456789"
#define PORT 3333

void init_task_tcp();
void tcp_client(void *pvParameters);
void tcp_wifi_connection();

#endif // define WIFI_H