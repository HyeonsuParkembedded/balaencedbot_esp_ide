# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources 
    "main.c"
    "bsw/*.c"
    "input/*.c" 
    "logic/*.c"
    "output/*.c"
    "system/*.c")

# 사용하지 않는 old 파일과 example 파일 제외
list(FILTER app_sources EXCLUDE REGEX ".*_old\\.c$")
list(FILTER app_sources EXCLUDE REGEX ".*_example\\.c$")

idf_component_register(SRCS ${app_sources}
                       INCLUDE_DIRS "." 
                       REQUIRES bt nvs_flash driver esp_adc esp_wifi esp_event esp_timer esp_system esp_common
                       PRIV_REQUIRES bt)

# NimBLE esp-hci header directory 추가
target_include_directories(${COMPONENT_LIB} PRIVATE 
    "${IDF_PATH}/components/bt/host/nimble/esp-hci/include")
