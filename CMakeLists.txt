cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(trik-nrf52833)

target_include_directories(app PRIVATE 
    src/usb
    src/periphery
    src/i2c
)

target_sources(app PRIVATE 
    src/main.c
    src/periphery/encoder.c
    src/periphery/saadc.c
    src/periphery/pwm.c
    src/periphery/port_table.c
    src/periphery/analog_sensor.c
    src/periphery/periphery_handler.c
    src/usb/internal/usb_device_next.c
    src/usb/internal/raw_usbd_init.c
    src/usb/internal/bt_hci_usb.c
    src/usb/protocol_schema.c
    src/i2c/i2c_handler.c
)

target_include_directories(app PRIVATE src)
