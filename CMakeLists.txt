# Root CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

project(as2_light_dip_detector
  VERSION 1.0
  DESCRIPTION "Light dip detector (LED + encoder + sampler + UDP)"
  LANGUAGES C)

# --- Build defaults ---
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
# Default to Release unless caller sets -DCMAKE_BUILD_TYPE=Debug
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# POSIX features used by the app
add_compile_definitions(_POSIX_C_SOURCE=200809L)

# --- Options ---
option(ENABLE_PEDANTIC "Enable extra warnings" ON)
option(ENABLE_ASAN     "Enable AddressSanitizer" OFF)
option(ENABLE_PTHREAD  "Link pthread globally (also linked per-target)" ON)

# --- Warnings / color ---
if(ENABLE_PEDANTIC)
  add_compile_options(-Wall -Wextra -Wpedantic)
  # Avoid -Werror on the target unless you really want hard failures:
  # add_compile_options(-Werror)
endif()
# Pretty diagnostics (supported by GCC/Clang)
add_compile_options(-fdiagnostics-color=always)

# --- Sanitizers (install libasan6 on target if enabled) ---
if(ENABLE_ASAN)
  add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address)
endif()

# --- pthread (safe to also link per-target) ---
if(ENABLE_PTHREAD)
  add_link_options(-pthread)
endif()

# --- Subdirs ---
add_subdirectory(hal)
add_subdirectory(app)

