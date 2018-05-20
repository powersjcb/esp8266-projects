cmake_minimum_required(VERSION 2.8.4)
SET(BOARD_HARDWARE_DIR /Users/powersjcb/Library/Arduino15/packages/esp8266)
SET(CMAKE_C_COMPILER "${ARDUINO_SDK_PATH}/hardware/espressif/esp32/tools/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc")
SET(CMAKE_C_COMPILER_WORKS 1)
SET(CMAKE_CXX_COMPILER "${ARDUINO_SDK_PATH}/hardware/espressif/esp32/tools/xtensa-esp32-elf/bin/xtensa-esp32-elf-g++")
SET(CMAKE_CXX_COMPILER_WORKS 1)
SET(CMAKE_AR "${ARDUINO_SDK_PATH}/hardware/espressif/esp32/tools/xtensa-esp32-elf/bin/xtensa-esp32-elf-ar")
SET(CMAKE_RANLIB "${ARDUINO_SDK_PATH}/hardware/espressif/esp32/tools/xtensa-esp32-elf/bin/xtensa-esp32-elf-ranlib")


set(ARDUINO_SDK_PATH /Applications/Arduino.app/Contents/Java)


set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/esp8266-cmake/cmake/ArduinoToolchain.cmake)
set(PROJECT_NAME esp8266_6050)
set(TEST rebuild)
project(${PROJECT_NAME})

set(${CMAKE_PROJECT_NAME}_SKETCH esp8266_6050.ino)

#### Uncomment below additional settings as needed.
set(${CMAKE_PROJECT_NAME}_BOARD wemosd1miniesp8266)
# set(${CMAKE_PROJECT_NAME}_PORT /dev/ttyACM0)
# set(mega.build.mcu atmega2560)
# set(mega.upload.protocol wiring)
# set(mega.upload.speed 115200)

generate_arduino_firmware(${CMAKE_PROJECT_NAME})
