# app/CMakeLists.txt — build the app using the HAL library

add_executable(test_sampler_with_dips
  src/main.c
  src/udp.c
  src/sampler.c
  src/dip_detector.c
  src/periodTimer.c
)

# Headers
target_include_directories(test_sampler_with_dips PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include        # app/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../hal/include # hal/include
)

# POSIX APIs
target_compile_definitions(test_sampler_with_dips PRIVATE _POSIX_C_SOURCE=200809L)

# Link libs
target_link_libraries(test_sampler_with_dips
  PRIVATE
    hal
    pthread
    gpiod
    m
)

# (Optional) put the executable at build/test_sampler_with_dips
set_target_properties(test_sampler_with_dips PROPERTIES
  OUTPUT_NAME light_sampler
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

