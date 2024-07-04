#include "esp_log.h"
#include "esp_wifi.h"

#include "esp_mac.h"
#include "esp_eth.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_http_client.h"
#include "esp_event.h"
#include "esp_system.h"

#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/ip_addr.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "nvs_flash.h"
#include "ping/ping_sock.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "motor.h"
#include "wifi.h"

static const char *TAG = "TCP SOCKET Client";

void init_task_tcp()
{
  tcp_wifi_connection();
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  xTaskCreate(tcp_client, "TCP SOCKET Client", 8092, NULL, 1, NULL);
}

void tcp_client(void *pvParameters)
{
  char rx_buffer[5];
  char host_ip[] = "192.168.35.11"; // Server IP0
  int addr_family = 0;
  int ip_protocol = 0;

  while (1)
  {
    struct sockaddr_in dest_addr;
    inet_pton(AF_INET, host_ip, &dest_addr.sin_addr);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;

    int sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (sock < 0)
    {
      ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
      break;
    }
    ESP_LOGI(TAG, "Socket created ");
    ESP_LOGI(TAG, "Connecting to %s:%d", host_ip, PORT);

    int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0)
    {
      ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
      continue;
      ;
    }
    ESP_LOGI(TAG, "Successfully connected");
    err = send(sock, "2", strlen("2"), 0);
    if (err < 0)
    {
      ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
    }
    else
    {
      ESP_LOGI(TAG, "Request sent successfully");
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS); // Chờ 5 giây
    err = send(sock, "stop", strlen("stop"), 0);
    if (err < 0)
    {
      ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
    }
    else
    {
      ESP_LOGI(TAG, "Request sent successfully");
    }
    while (1)
    {
      int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
      if (len < 0)
      {
        ESP_LOGE(TAG, "recv failed: errno %d", errno);
        break;
      }
      else if (len == 0)
      {
        ESP_LOGI(TAG, "Connection closed");
        break;
      }
      else
      {
        rx_buffer[len] = 0; // Null-terminate received data
        ESP_LOGI(TAG, "Received %d bytes: '%s'", len, rx_buffer);

        // Chuyển đổi dữ liệu nhận được thành giá trị duty cycle
        int duty = atoi(rx_buffer);
        if (duty >= 0 && duty <= 8191)
        {
          motor_vibrate(8191 - duty, LEDC_CHANNEL_0);
        }
      }
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS); // Chờ 5 giây

    if (sock != -1)
    {
      shutdown(sock, 0);
      close(sock);
    }
  }
}

static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
  switch (event_id)
  {
  case WIFI_EVENT_STA_START:
    printf("WiFi connecting WIFI_EVENT_STA_START ... \n");
    break;
  case WIFI_EVENT_STA_CONNECTED:
    printf("WiFi connected WIFI_EVENT_STA_CONNECTED ... \n");
    break;
  case WIFI_EVENT_STA_DISCONNECTED:
    printf("WiFi lost connection WIFI_EVENT_STA_DISCONNECTED ... \n");
    break;
  case IP_EVENT_STA_GOT_IP:
    printf("WiFi got IP ... \n\n");
    break;
  default:
    break;
  }
}

void tcp_wifi_connection()
{
  nvs_flash_init();
  esp_netif_init();
  esp_event_loop_create_default();
  esp_netif_create_default_wifi_sta();
  wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&wifi_initiation);
  esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
  esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
  wifi_config_t wifi_configuration = {
      .sta = {
          .ssid = WIFI_SSID,
          .password = WIFI_PASS}};
  esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_start();
  esp_wifi_connect();
}
