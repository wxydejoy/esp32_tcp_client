/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "addr_from_stdin.h"
#include "lwip/err.h"
#include "lwip/sockets.h"

#include "driver/uart.h"
#include "driver/gpio.h"

// TCP/WIFI
#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#else
#define HOST_IP_ADDR ""
#endif

#define PORT CONFIG_EXAMPLE_PORT

// UART
#define ECHO_TEST_TXD 4
#define ECHO_TEST_RXD 5
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM 2
#define ECHO_UART_BAUD_RATE 115200
#define ECHO_TASK_STACK_SIZE 2048

#define BUF_SIZE (1024)

static const char *TAG = "wxy";

int sock = 0;
// static const char *payload = "Message from ESP32 ";

void init()
{
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;

#if defined(CONFIG_EXAMPLE_IPV4)
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(host_ip);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
#elif defined(CONFIG_EXAMPLE_IPV6)
    struct sockaddr_in6 dest_addr = {0};
    inet6_aton(host_ip, &dest_addr.sin6_addr);
    dest_addr.sin6_family = AF_INET6;
    dest_addr.sin6_port = htons(PORT);
    dest_addr.sin6_scope_id = esp_netif_get_netif_impl_index(EXAMPLE_INTERFACE);
    addr_family = AF_INET6;
    ip_protocol = IPPROTO_IPV6;
#elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
    struct sockaddr_storage dest_addr = {0};
    ESP_ERROR_CHECK(get_addr_from_stdin(PORT, SOCK_STREAM, &ip_protocol, &addr_family, &dest_addr));
#endif
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 1;
    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    ESP_LOGI(TAG, "Successfully created uart");

    sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    }
    else
    {
        ESP_LOGI(TAG, "Socket created, connecting to %s:%d,sock:%d", host_ip, PORT, sock);
    }

    while (1)
    {
        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(struct sockaddr_in6));
        if (err != 0)
        {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
        }
        else
        {
            break;
        }
    }
}

static void tcp_client_task(void *pvParameters)
{
    char rx_buffer[128];

    while (1)
    {

        
        // tcp -> uart
        int socket_len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0); //-1是因为要把'\0'加进去
        // Error occurred during receiving
        if (socket_len < 0)
        {
            ESP_LOGE(TAG, "recv failed: errno %d", errno);
            break;
        }
        // Data received
        else
        {
            rx_buffer[socket_len] = 0; // Null-terminate whatever we received and treat like a string
            ESP_LOGI(TAG, "Received %d bytes ", socket_len);
            ESP_LOGI(TAG, "%s", rx_buffer);

            uart_write_bytes(ECHO_UART_PORT_NUM, rx_buffer, strlen(rx_buffer));
        }

        // vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    

    

   
}

static void uart_client_task(void *pvParameters)
{

    uint8_t *uart_data = (uint8_t *)malloc(BUF_SIZE);
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);

    while (1)
    {
        // uart -> tcp
        int uart_len = uart_read_bytes(ECHO_UART_PORT_NUM, uart_data, BUF_SIZE, 20 / portTICK_RATE_MS);

        // Data received
        if (uart_len > 0)
        {
            ESP_LOGI(TAG, "Received %d bytes from uart:", uart_len);
            ESP_LOGI(TAG, "%s", uart_data);
            int err = send(sock, uart_data, uart_len, 0);
            // int err = send(sock, payload, strlen(payload), 0);

            if (err < 0)
            {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
        }
    }
    free(uart_data);
}

void app_main(void)
{

    // wifi
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    init();
    // tcp -> uart
    xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 6, NULL);
    // uart -> tcp
    xTaskCreate(uart_client_task, "uart_client", 4096, NULL, 5, NULL);
}
