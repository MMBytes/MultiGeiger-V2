idf_component_register(
    SRCS
        "main.c"
        "config.c"
        "http_server.c"
        "tube.c"
        "tube_pcnt.c"
        "tube_types.c"
        "history.c"
        "ntp.c"
        "gnss.c"
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
        "fuel_gauge.c"
        "pm_sensor.c"
        "sps30.c"
        "noise_sensor.c"
        "dnms.c"
        "sgp41.c"
        "telemetry.c"
        "sd_card.c"
        "sd_logger.c"
        "sensirion_gas_index_algorithm.c"
        "applog.c"
        "diag.c"
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
        "display_tft.c"
        "neopixel.c"
        "led.c"
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash
        esp_wifi
        esp_netif
        esp_event
        esp_timer
        esp_driver_gpio
        esp_driver_gptimer
        esp_driver_pcnt        # V2.5.16: parallel PCNT width-comb diag (tube_pcnt.c)
        esp_driver_i2c
        esp_adc
        esp_driver_ledc
        esp_driver_rmt
        esp_driver_spi         # V2.6.11: onboard ST7789 TFT SPI bus (display_tft.c, adafruit_esp32s3_tft_feather only)
        esp_lcd                # V2.6.11: built-in ST7789 panel driver (display_tft.c) — no idf_component.yml entry, ships in-tree
        esp_http_client
        esp_http_server
        esp-tls
        mbedtls
        lwip
        app_update
        esp_partition
        spi_flash
        espcoredump            # V2.4.18: panic dump to flash, read via /coredump.elf
        esp_psram              # V2.6.10: applog.c boot-time PSRAM mode/speed log line
        fatfs                  # V2.6.19: standalone SD logging (sd_card.c/sd_logger.c)
        sdmmc
        esp_driver_sdmmc
        esp_driver_sdspi
        # NOTE: `mqtt` is NOT listed here — V2.4.2's MQTT client
        # (`mqtt_client.h` in main/mqtt.c) ships via the IDF Component
        # Manager as `espressif/mqtt`, declared in main/idf_component.yml.
        # The Component Manager auto-injects the resolved component into
        # the build graph; adding it here would either no-op or break
        # depending on resolver state. See IDF v6.0 migration guide
        # `docs/en/migration-guides/release-6.x/6.0/protocols.rst:151`.
)
