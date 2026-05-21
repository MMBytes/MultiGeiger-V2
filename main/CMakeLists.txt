idf_component_register(
    SRCS
        "main.c"
        "config.c"
        "http_server.c"
        "tube.c"
        "ntp.c"
        "transmission.c"
        "bme280.c"
        "bme688.c"
        "sht45.c"
        "bmp390.c"
        "bmp581.c"
        "env_sensor.c"
        "i2c_bus.c"
        "als.c"
        "veml7700.c"
        "pm_sensor.c"
        "sps30.c"
        "noise_sensor.c"
        "dnms.c"
        "applog.c"
        "syslog.c"
        "coredump.c"
        "log_ftp.c"
        "net_arp.c"
        "periodic.c"
        "mqtt.c"
        "mqtt_discovery.c"
        "speaker.c"
        "display.c"
        "display_serlcd.c"
        "neopixel.c"
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash
        esp_wifi
        esp_netif
        esp_event
        esp_timer
        esp_driver_gpio
        esp_driver_gptimer
        esp_driver_i2c
        esp_adc
        esp_driver_ledc
        esp_driver_rmt
        esp_http_client
        esp_http_server
        esp-tls
        mbedtls
        lwip
        app_update
        esp_partition
        spi_flash
        espcoredump            # V2.4.18: panic dump to flash, read via /coredump.elf
        # NOTE: `mqtt` is NOT listed here — V2.4.2's MQTT client
        # (`mqtt_client.h` in main/mqtt.c) ships via the IDF Component
        # Manager as `espressif/mqtt`, declared in main/idf_component.yml.
        # The Component Manager auto-injects the resolved component into
        # the build graph; adding it here would either no-op or break
        # depending on resolver state. See IDF v6.0 migration guide
        # `docs/en/migration-guides/release-6.x/6.0/protocols.rst:151`.
)
