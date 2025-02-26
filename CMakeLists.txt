idf_component_register(SRCS "sths34pf80.c" "sths34pf80-pid/sths34pf80_reg.c"
                    INCLUDE_DIRS "include" "sths34pf80-pid"
                    REQUIRES esp_timer esp_driver_i2c driver)
