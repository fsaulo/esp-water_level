
#include <string.h>
#include <stdarg.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define UDP_LOG_QUEUE_LENGTH 20

static QueueHandle_t log_queue;
#include "udp_log.h"


static int udp_sock = -1;
static struct sockaddr_in dest_addr;
static vprintf_like_t original_vprintf = NULL;

static int udp_log_vprintf(const char *fmt, va_list args)
{
    char buffer[UDP_LOG_BUFFER_SIZE];

    va_list args_copy1;
    va_list args_copy2;

    va_copy(args_copy1, args);
    va_copy(args_copy2, args);

    int len = vsnprintf(buffer, sizeof(buffer), fmt, args_copy1);
    va_end(args_copy1);

    if (log_queue && len > 0) {
        char *msg = malloc(len + 1);
        if (msg) {
            memcpy(msg, buffer, len + 1);
            xQueueSend(log_queue, &msg, 0);
        }
    }

    int ret = original_vprintf(fmt, args_copy2);
    va_end(args_copy2);

    return ret;
}

static void udp_log_task(void *arg)
{
    char *msg;

    while (1) {
        if (xQueueReceive(log_queue, &msg, portMAX_DELAY)) {
            if (udp_sock >= 0) {
                sendto(udp_sock, msg, strlen(msg), 0,
                       (struct sockaddr *)&dest_addr,
                       sizeof(dest_addr));
            }
            free(msg);
        }
    }
}

void udp_log_init(void)
{
    udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_sock < 0) {
        return;
    }

    int broadcast = 1;
    setsockopt(udp_sock, SOL_SOCKET, SO_BROADCAST,
               &broadcast, sizeof(broadcast));

    fcntl(udp_sock, F_SETFL, O_NONBLOCK);

    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_LOG_PORT);
    dest_addr.sin_addr.s_addr = inet_addr("192.168.1.3");

    log_queue = xQueueCreate(UDP_LOG_QUEUE_LENGTH, sizeof(char*));

    xTaskCreate(udp_log_task,
                "udp_log_task",
                4096,
                NULL,
                3,
                NULL);

    original_vprintf = esp_log_set_vprintf(udp_log_vprintf);
}
