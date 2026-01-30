cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(GET_SAMPLE)

set(app_sources
  src/main.c
  src/device_config/device_config.c
  src/command_handler/command_handler.c
  src/parsing_json/json_handler.c
  src/sensor/bmi270.c
  src/thread_handler/bluetooth_handler/bluetooth_handler.c
  src/thread_handler/sensor_handler/sensor_handler.c
  src/thread_handler/sensor_handler/sensor_init.c
  src/thread_handler/sensor_handler/sensor_threads.c
  src/thread_handler/sensor_handler/sensor_callback.c
  src/thread_handler/uart_handler/uart_handler.c
  src/thread_handler/buzzer_handler/buzzer_handler.c
)

target_sources(app PRIVATE ${app_sources})


