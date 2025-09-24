# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources 
    "main.c"
    "bsw/*.c"
    "input/*.c" 
    "logic/*.c"
    "output/*.c"
    "system/*.c")

idf_component_register(SRCS ${app_sources}
                       INCLUDE_DIRS "."
                       REQUIRES bt nvs_flash driver esp_adc esp_wifi esp_event esp_timer esp_system esp_common)
