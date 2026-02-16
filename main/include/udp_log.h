#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define UDP_LOG_PORT 3333
#define UDP_LOG_BUFFER_SIZE 512

static const char *TAG_UDP = "UDP_LOG";


/**
 * @brief Initialize UDP broadcast logging.
 *
 * Must be called after Wi-Fi connection (IP_EVENT_STA_GOT_IP).
 *
 * Mirrors ESP_LOG output to:
 *   - UART (default)
 *   - UDP broadcast (255.255.255.255:333)
 */
void udp_log_init(void);

#ifdef __cplusplus
}
#endif
