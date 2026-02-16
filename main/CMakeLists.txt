idf_component_register(SRCS ./app_main.c ./jsnsr04t.c ./udp_log.c
                       REQUIRES esp_netif lwip driver nvs_flash spi_flash esp_http_client
                       PRIV_REQUIRES
                       INCLUDE_DIRS "include")
