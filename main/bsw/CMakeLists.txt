# BSW (Basic Software) 계층 CMake 설정
# 하드웨어 추상화 계층을 위한 드라이버들

# BSW 소스 파일들
set(BSW_SOURCES
    "pwm_driver.c"
    "i2c_driver.c"
    "ble_driver.c"
)

# 조건부 컴파일을 위한 소스 파일 추가
if(NOT CONFIG_APP_BUILD_TYPE_PURE)
    list(APPEND BSW_SOURCES
        ${BSW_SOURCES}
    )
endif()