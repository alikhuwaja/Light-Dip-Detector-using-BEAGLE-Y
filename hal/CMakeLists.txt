add_library(hal STATIC
  src/encoder.c
  src/light_sensor.c
  src/pwm_led.c
)

target_include_directories(hal
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(hal PUBLIC c_std_11)
# If HAL directly needs libs, you can expose them here:
# target_link_libraries(hal PUBLIC gpiod m)

