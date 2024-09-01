idf_component_register(SRCS "imu.cpp"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES "esp_timer" "i2cdev" "mpu6050")
