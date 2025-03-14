#ifndef SDK_CONFIG_H__
#define SDK_CONFIG_H__

/* Project Configuration */

/*  Freertos Configuration */

#define CONFIG_TARGET_NAME "openamp_for_linux"
/* end of  Freertos Configuration */

/* AMP Config */

#define CONFIG_IPI_IRQ_NUM 9
#define CONFIG_IPI_IRQ_NUM_PRIORITY 1
#define CONFIG_IPI_CHN_BITMASK 255
/* end of AMP Config */
/* end of Project Configuration */
#define CONFIG_USE_FREERTOS

/* Arch configuration */

#define CONFIG_TARGET_ARMv8
#define CONFIG_ARCH_NAME "armv8"

/* Arm architecture configuration */

#define CONFIG_ARCH_ARMV8_AARCH64
/* CONFIG_ARCH_ARMV8_AARCH32 is not set */

/* Compiler configuration */

#define CONFIG_ARM_GCC_SELECT
/* CONFIG_ARM_CLANG_SELECT is not set */
#define CONFIG_TOOLCHAIN_NAME "gcc"
#define CONFIG_TARGET_ARMV8_AARCH64
#define CONFIG_ARCH_EXECUTION_STATE "aarch64"
#define CONFIG_ARM_NEON
#define CONFIG_ARM_CRC
#define CONFIG_ARM_CRYPTO
#define CONFIG_ARM_FLOAT_POINT
/* CONFIG_GCC_CODE_MODEL_TINY is not set */
#define CONFIG_GCC_CODE_MODEL_SMALL
/* CONFIG_GCC_CODE_MODEL_LARGE is not set */
/* end of Compiler configuration */
#define CONFIG_USE_CACHE
#define CONFIG_USE_MMU
/* CONFIG_BOOT_WITH_FLUSH_CACHE is not set */
/* CONFIG_MMU_DEBUG_PRINTS is not set */
/* end of Arm architecture configuration */
/* end of Arch configuration */

/* Soc configuration */

#define CONFIG_TARGET_PHYTIUMPI
/* CONFIG_TARGET_E2000Q is not set */
/* CONFIG_TARGET_E2000D is not set */
/* CONFIG_TARGET_E2000S is not set */
/* CONFIG_TARGET_FT2004 is not set */
/* CONFIG_TARGET_D2000 is not set */
/* CONFIG_TARGET_PD2308 is not set */
#define CONFIG_SOC_NAME "phytiumpi"
#define CONFIG_SOC_CORE_NUM 4
#define CONFIG_F32BIT_MEMORY_ADDRESS 0x80000000
#define CONFIG_F32BIT_MEMORY_LENGTH 0x80000000
#define CONFIG_F64BIT_MEMORY_ADDRESS 0x2000000000
#define CONFIG_F64BIT_MEMORY_LENGTH 0x800000000
#define CONFIG_TARGET_E2000
#define CONFIG_DEFAULT_DEBUG_PRINT_UART1
/* CONFIG_DEFAULT_DEBUG_PRINT_UART0 is not set */
/* CONFIG_DEFAULT_DEBUG_PRINT_UART2 is not set */
#define CONFIG_SPIN_MEM 0x80000000
/* end of Soc configuration */

/* Board Configuration */

#define CONFIG_BOARD_NAME "firefly"
/* CONFIG_USE_SPI_IOPAD is not set */
/* CONFIG_USE_GPIO_IOPAD is not set */
/* CONFIG_USE_CAN_IOPAD is not set */
/* CONFIG_USE_QSPI_IOPAD is not set */
/* CONFIG_USE_PWM_IOPAD is not set */
/* CONFIG_USE_MIO_IOPAD is not set */
/* CONFIG_USE_TACHO_IOPAD is not set */
/* CONFIG_USE_UART_IOPAD is not set */
/* CONFIG_USE_THIRD_PARTY_IOPAD is not set */
#define CONFIG_FIREFLY_DEMO_BOARD

/* IO mux configuration when board start up */

/* end of IO mux configuration when board start up */
/* CONFIG_CUS_DEMO_BOARD is not set */

/* Build project name */

/* end of Build project name */
/* end of Board Configuration */

/* Sdk common configuration */

#define CONFIG_LOG_VERBOS
/* CONFIG_LOG_DEBUG is not set */
/* CONFIG_LOG_INFO is not set */
/* CONFIG_LOG_WARN is not set */
/* CONFIG_LOG_ERROR is not set */
/* CONFIG_LOG_NONE is not set */
/* CONFIG_LOG_EXTRA_INFO is not set */
#define CONFIG_LOG_DISPALY_CORE_NUM
/* CONFIG_BOOTUP_DEBUG_PRINTS is not set */
#define CONFIG_USE_DEFAULT_INTERRUPT_CONFIG
/* CONFIG_INTERRUPT_ROLE_MASTER is not set */
#define CONFIG_INTERRUPT_ROLE_SLAVE
/* end of Sdk common configuration */

/* Image information configuration */

/* CONFIG_IMAGE_INFO is not set */
/* end of Image information configuration */

/* Drivers configuration */

#define CONFIG_USE_IOMUX
/* CONFIG_ENABLE_IOCTRL is not set */
#define CONFIG_ENABLE_IOPAD
/* CONFIG_USE_SPI is not set */
/* CONFIG_USE_QSPI is not set */
#define CONFIG_USE_SERIAL

/* Usart Configuration */

#define CONFIG_ENABLE_Pl011_UART
/* end of Usart Configuration */
#define CONFIG_USE_GPIO
#define CONFIG_ENABLE_FGPIO
/* CONFIG_USE_ETH is not set */
/* CONFIG_USE_CAN is not set */
/* CONFIG_USE_I2C is not set */
/* CONFIG_USE_TIMER is not set */
/* CONFIG_USE_MIO is not set */
/* CONFIG_USE_SDMMC is not set */
/* CONFIG_USE_PCIE is not set */
/* CONFIG_USE_WDT is not set */
/* CONFIG_USE_DMA is not set */
/* CONFIG_USE_NAND is not set */
/* CONFIG_USE_RTC is not set */
/* CONFIG_USE_SATA is not set */
/* CONFIG_USE_USB is not set */
/* CONFIG_USE_ADC is not set */
#define CONFIG_USE_PWM

/* FPWM Configuration */

#define CONFIG_USE_FPWM
/* end of FPWM Configuration */
/* CONFIG_USE_IPC is not set */
/* CONFIG_USE_MEDIA is not set */
/* CONFIG_USE_SCMI_MHU is not set */
/* CONFIG_USE_I2S is not set */
/* CONFIG_USE_I3C is not set */
/* end of Drivers configuration */

/* Build setup */

#define CONFIG_CHECK_DEPS
/* CONFIG_OUTPUT_BINARY is not set */

/* Optimization options */

/* CONFIG_DEBUG_NOOPT is not set */
/* CONFIG_DEBUG_CUSTOMOPT is not set */
#define CONFIG_DEBUG_FULLOPT
#define CONFIG_DEBUG_OPT_UNUSED_SECTIONS
#define CONFIG_DEBUG_LINK_MAP
/* CONFIG_CCACHE is not set */
/* CONFIG_ARCH_COVERAGE is not set */
/* CONFIG_LTO_FULL is not set */
/* end of Optimization options */

/* Debug options */

/* CONFIG_DEBUG_ENABLE_ALL_WARNING is not set */
/* CONFIG_WALL_WARNING_ERROR is not set */
/* CONFIG_STRICT_PROTOTYPES is not set */
/* CONFIG_DEBUG_SYMBOLS is not set */
/* CONFIG_FRAME_POINTER is not set */
/* CONFIG_OUTPUT_ASM_DIS is not set */
/* CONFIG_ENABLE_WSHADOW is not set */
/* CONFIG_ENABLE_WUNDEF is not set */
#define CONFIG_DOWNGRADE_DIAG_WARNING
/* end of Debug options */

/* Lib */

#define CONFIG_USE_COMPILE_CHAIN
/* CONFIG_USE_NEWLIB is not set */
/* CONFIG_USE_USER_DEFINED is not set */
/* end of Lib */
/* CONFIG_ENABLE_CXX is not set */

/* Linker Options */

#define CONFIG_DEFAULT_LINKER_SCRIPT
/* CONFIG_USER_DEFINED_LD is not set */
#define CONFIG_IMAGE_LOAD_ADDRESS 0xb0100000
#define CONFIG_IMAGE_MAX_LENGTH 0x1000000
#define CONFIG_HEAP_SIZE 1
#define CONFIG_STACK_SIZE 0x400
/* end of Linker Options */
/* end of Build setup */

/* Component Configuration */

/* Freertos Uart Drivers */

#define CONFIG_FREERTOS_USE_UART
/* end of Freertos Uart Drivers */

/* Freertos Pwm Drivers */

#define CONFIG_FREERTOS_USE_PWM
/* end of Freertos Pwm Drivers */

/* Freertos Qspi Drivers */

/* CONFIG_FREERTOS_USE_QSPI is not set */
/* end of Freertos Qspi Drivers */

/* Freertos Wdt Drivers */

/* CONFIG_FREERTOS_USE_WDT is not set */
/* end of Freertos Wdt Drivers */

/* Freertos Eth Drivers */

/* CONFIG_FREERTOS_USE_XMAC is not set */
/* CONFIG_FREERTOS_USE_GMAC is not set */
/* end of Freertos Eth Drivers */

/* Freertos Gpio Drivers */

#define CONFIG_FREERTOS_USE_GPIO
/* end of Freertos Gpio Drivers */

/* Freertos Spim Drivers */

/* CONFIG_FREERTOS_USE_FSPIM is not set */
/* end of Freertos Spim Drivers */

/* Freertos DMA Drivers */

/* CONFIG_FREERTOS_USE_FDDMA is not set */
/* CONFIG_FREERTOS_USE_FGDMA is not set */
/* end of Freertos DMA Drivers */

/* Freertos Adc Drivers */

/* CONFIG_FREERTOS_USE_ADC is not set */
/* end of Freertos Adc Drivers */

/* Freertos Can Drivers */

/* CONFIG_FREERTOS_USE_CAN is not set */
/* end of Freertos Can Drivers */

/* Freertos I2c Drivers */

/* CONFIG_FREERTOS_USE_I2C is not set */
/* end of Freertos I2c Drivers */

/* Freertos Mio Drivers */

/* CONFIG_FREERTOS_USE_MIO is not set */
/* end of Freertos Mio Drivers */

/* Freertos Timer Drivers */

/* CONFIG_FREERTOS_USE_TIMER is not set */
/* end of Freertos Timer Drivers */

/* Freertos Media Drivers */

/* CONFIG_FREERTOS_USE_MEDIA is not set */
/* end of Freertos Media Drivers */

/* Freertos I2s Drivers */

/* CONFIG_FREERTOS_USE_I2S is not set */
/* end of Freertos I2s Drivers */
/* end of Component Configuration */

/* Third-party configuration */

/* CONFIG_USE_LWIP is not set */
#define CONFIG_USE_LETTER_SHELL

/* Letter Shell Configuration */

#define CONFIG_LS_PL011_UART
#define CONFIG_DEFAULT_LETTER_SHELL_USE_UART1
/* CONFIG_DEFAULT_LETTER_SHELL_USE_UART0 is not set */
/* CONFIG_DEFAULT_LETTER_SHELL_USE_UART2 is not set */
/* end of Letter Shell Configuration */
#define CONFIG_USE_AMP
#define CONFIG_USE_LIBMETAL

/* OpenAmp */

#define CONFIG_USE_OPENAMP
#define CONFIG_USE_OPENAMP_IPI
#define CONFIG_OPENAMP_RESOURCES_ADDR 0xc0000000
#define CONFIG_VRING_TX_ADDR 0xffffffff
#define CONFIG_VRING_RX_ADDR 0xffffffff
#define CONFIG_VRING_SIZE 0x100
#define CONFIG_POLL_BASE_ADDR 0xc0224000
#define CONFIG_SKIP_SHBUF_IO_WRITE
#define CONFIG_USE_MASTER_VRING_DEFINE

/* Baremetal config */

/* CONFIG_MEM_NO_CACHE is not set */
/* CONFIG_MEM_WRITE_THROUGH is not set */
#define CONFIG_MEM_NORMAL
/* end of Baremetal config */
/* end of OpenAmp */
/* CONFIG_USE_YMODEM is not set */
/* CONFIG_USE_SFUD is not set */
#define CONFIG_USE_BACKTRACE
/* CONFIG_USE_FATFS_0_1_4 is not set */
#define CONFIG_USE_TLSF
/* CONFIG_USE_SPIFFS is not set */
/* CONFIG_USE_LITTLE_FS is not set */
/* CONFIG_USE_LVGL is not set */
/* CONFIG_USE_FREEMODBUS is not set */
/* CONFIG_USE_CHERRY_USB is not set */
/* CONFIG_USE_FSL_SDMMC is not set */
/* CONFIG_USE_FSL_WIFI is not set */
/* end of Third-party configuration */

/* FreeRTOS Kernel Configuration */

#define CONFIG_FREERTOS_OPTIMIZED_SCHEDULER
#define CONFIG_FREERTOS_HZ 1000
#define CONFIG_FREERTOS_MAX_PRIORITIES 32
#define CONFIG_FREERTOS_KERNEL_INTERRUPT_PRIORITIES 13
#define CONFIG_FREERTOS_MAX_API_CALL_INTERRUPT_PRIORITIES 11
#define CONFIG_FREERTOS_THREAD_LOCAL_STORAGE_POINTERS 1
#define CONFIG_FREERTOS_MINIMAL_TASK_STACKSIZE 1024
#define CONFIG_FREERTOS_MAX_TASK_NAME_LEN 32
#define CONFIG_FREERTOS_TIMER_TASK_PRIORITY 1
#define CONFIG_FREERTOS_TIMER_TASK_STACK_DEPTH 2048
#define CONFIG_FREERTOS_TIMER_QUEUE_LENGTH 10
#define CONFIG_FREERTOS_QUEUE_REGISTRY_SIZE 0
#define CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
#define CONFIG_FREERTOS_USE_TRACE_FACILITY
#define CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS
/* CONFIG_FREERTOS_USE_TICKLESS_IDLE is not set */
#define CONFIG_FREERTOS_TOTAL_HEAP_SIZE 10240
#define CONFIG_FREERTOS_TASK_FPU_SUPPORT 1
/* CONFIG_FREERTOS_USE_POSIX is not set */
/* end of FreeRTOS Kernel Configuration */

#endif
