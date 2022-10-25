/**
 * @file tcp_server.c
 * @author Eduardo Eller Behr (eellerbehr@gmail.com)
 * @brief 
 * @date 2022-09-19
 * 
 * @copyright (c) 2022
 * 
 */

#include <pwm_task.h>
#include <interpreter.h>

#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>


#define PORT                        CONFIG_EXAMPLE_PORT
#define KEEPALIVE_IDLE              CONFIG_EXAMPLE_KEEPALIVE_IDLE
#define KEEPALIVE_INTERVAL          CONFIG_EXAMPLE_KEEPALIVE_INTERVAL
#define KEEPALIVE_COUNT             CONFIG_EXAMPLE_KEEPALIVE_COUNT

static const char *TAG = "TCP Server";

static inline void send_prompt(const int socket){
    static const char* prompt = "\n>>> ";
    send(socket, prompt, strlen(prompt), 0);
}

static inline void send_welcome(const int socket){
    const char* welcome_string = "Copyright (c) Eduardo Eller Behr 2022\n\
Bem vindo ao TCP Shell no ESP32!\n\
Digite 'help' para saber mais.\n";
    send(socket, welcome_string, strlen(welcome_string), 0);
}

static void do_retransmit(const int sock)
{
    int len;
    char rx_buffer[128];
    char tx_buffer[cResponseLineSize]; 
    
    send_welcome(sock);
    
    do {
        char* pResponse = &tx_buffer[0];

        send_prompt(sock);
        len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len < 0) {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        } else if (len == 0) {
            ESP_LOGW(TAG, "Connection closed");
        } else {
            rx_buffer[len] = 0; // Null-terminate whatever is received and treat it like a string
            ESP_LOGI(TAG, "Received %d bytes: %s", len, rx_buffer);

            CommandLine_t cmdline = {};
            
            cmdline = parse_command_line(rx_buffer);
            printf("Parsed command name: %s\n", cmdline.cmd->command_name);

            int status = execute_command(&cmdline, &pResponse);

            printf("   ← %s\n", pResponse);

            int to_write = strlen(pResponse);
            while (to_write > 0) {
                
                int written = send(sock, pResponse, to_write, 0);
                if (written < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                }
                to_write -= written;
            }
            if(status == eNeedsDeallocation){
                free(pResponse);
                ESP_LOGI(TAG, "Cleaned pResponse");
            }

            // add_to_history(&cmdline); // TODO:

            free_command_line_arguments(&cmdline);
        }
    } while (len > 0);
}

// /**
//  * @brief Memoriza qual IP possui cada socket
//  * TODO:
//  */
// typedef struct IpSocketPair {
//     int socket;
//     char ipv4; [16]/// e.g. xxx.xxx.xxx.xxx
// } IpSocketPair;

static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    struct sockaddr_storage dest_addr;

    if (addr_family == AF_INET) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);
        ip_protocol = IPPROTO_IP;
    }

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1) {

        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Set tcp keepalive option
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        // Convert ip address to string
        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }

        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

        // IpSocketPair* ip_addr_pair = malloc(sizeof(IpSocketPair)); // TODO: gravar associação entre IP e socket
        
        do_retransmit(sock);

        shutdown(sock, 0);
        close(sock);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}

void app_main(void)
{
    xTaskCreate(
        pwm_task,   // função
        "pwm",      // nome
        4096,       // memória
        NULL,       // ponteiro para parametros
        5,          // prioridade
        NULL        // handle
    );


    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();


    wifi_init_config_t icfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_init(&icfg));
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_mode(WIFI_MODE_AP));


    const char* cSSID = "PWM via WiFi";
    const char* cPassword = "ineppwmwifi";
    wifi_config_t cfg = {};
    memcpy(&cfg.ap.ssid[0], cSSID, strlen(cSSID));
    memcpy(&cfg.ap.password[0], cPassword, strlen(cPassword));
    cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    cfg.ap.max_connection = 5;

    ESP_LOGI(TAG, "Servindo ponto de acesso com SSID '%s' e senha '%s'", cfg.ap.ssid, cfg.ap.password);

    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_config(ESP_IF_WIFI_AP, &cfg));

    
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_start());

    const size_t cTcpTaskStackSize = 4096 + 2*cResponseLineSize;
#ifdef CONFIG_EXAMPLE_IPV4
    xTaskCreate(tcp_server_task, "tcp_server", cTcpTaskStackSize, (void*)AF_INET, 5, NULL);
#endif
}
