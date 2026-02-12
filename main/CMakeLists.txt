idf_component_register(SRCS ./app_main.c ./jsnsr04t.c
                       REQUIRES driver nvs_flash spi_flash esp_http_client
                       PRIV_REQUIRES
                       INCLUDE_DIRS "include")
