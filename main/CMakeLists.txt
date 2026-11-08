# V2.6.23: LoRaWAN (heltec_wifi_lora32_v4_r2; V2.8.11: + seeed_xiao_esp32s3_wio). Deliberately NOT the
# speaker.c always-compile-stub pattern: compiling lorawan.cpp requires the
# RadioLib managed component, which only exists when CONFIG_GEIGER_LORAWAN
# pulled it (see idf_component.yml rules). Keyed off the same single symbol.
set(GEIGER_LORAWAN_SRCS "")
if(CONFIG_GEIGER_LORAWAN)
    list(APPEND GEIGER_LORAWAN_SRCS "lorawan.cpp")
endif()

idf_component_register(
    SRCS
        "main.c"
        "config.c"
        "config_form.c"
        "http_server.c"
        "tls_cert.c"
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
        "ota_guard.c"          # V2.8.20: OTA rollback, app side (confirm after probation)
        "ota_guard_logic.c"    # V2.8.20: pure probation rule (host-tested)
        ${GEIGER_LORAWAN_SRCS}
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
        esp_https_server       # V2.8.0: TLS wrapper around esp_http_server (PSRAM boards; garbage-collected elsewhere)
        esp-tls
        mbedtls
        lwip
        app_update
        esp_partition
        spi_flash
        espcoredump            # V2.4.18: panic dump to flash, read via /coredump.elf
        esp_psram              # V2.6.10: applog.c boot-time PSRAM mode/speed log line
        efuse                  # V2.8.6: syslog.c banner — package version, flash encryption state
        bootloader_support     # V2.8.6: syslog.c banner — secure boot state (esp_secure_boot_enabled)
        esp_app_format         # V2.8.6: syslog.c banner — ELF SHA-256 (esp_app_get_elf_sha256)
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

# V2.8.21: the -Wno-error=overloaded-virtual demotion that V2.6.23 added for
# RadioLib's translation units and lorawan.cpp is gone. RadioLib 8.0.0 no
# longer trips -Woverloaded-virtual: its first build here printed no warning
# with the demotion still in place, so RadioLib now compiles under the full
# -Werror like every other component.
