set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(ARM_TOOLCHAIN_DIR
    "D:/ST/STM32CubeCLT_1.22.0/GNU-tools-for-STM32/bin"
    CACHE PATH "GNU Arm Embedded toolchain bin directory")

find_program(CMAKE_C_COMPILER arm-none-eabi-gcc
    HINTS "${ARM_TOOLCHAIN_DIR}" REQUIRED)
find_program(CMAKE_CXX_COMPILER arm-none-eabi-g++
    HINTS "${ARM_TOOLCHAIN_DIR}" REQUIRED)
find_program(CMAKE_ASM_COMPILER arm-none-eabi-gcc
    HINTS "${ARM_TOOLCHAIN_DIR}" REQUIRED)
find_program(CMAKE_OBJCOPY arm-none-eabi-objcopy
    HINTS "${ARM_TOOLCHAIN_DIR}" REQUIRED)
find_program(CMAKE_SIZE arm-none-eabi-size
    HINTS "${ARM_TOOLCHAIN_DIR}" REQUIRED)

set(CMAKE_EXECUTABLE_SUFFIX ".elf")

set(TARGET_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(COMMON_FLAGS "${TARGET_FLAGS} -ffunction-sections -fdata-sections")

set(CMAKE_C_FLAGS_INIT "${COMMON_FLAGS}")
set(CMAKE_CXX_FLAGS_INIT
    "${COMMON_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")
set(CMAKE_ASM_FLAGS_INIT "${TARGET_FLAGS} -x assembler-with-cpp")

set(CMAKE_EXE_LINKER_FLAGS_INIT
    "${TARGET_FLAGS} -T${CMAKE_SOURCE_DIR}/STM32F407xx_FLASH.ld --specs=nano.specs -Wl,--gc-sections -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--print-memory-usage")

set(TOOLCHAIN_LINK_LIBRARIES m)
