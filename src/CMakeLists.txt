# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(SRCS ${app_sources}
                       INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/include
                       REQUIRES esp_wifi esp_event nvs_flash esp_now freertos driver)
