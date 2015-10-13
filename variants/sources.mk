#|---------------------------------------------------------------------------------------|
#| Source files                                                                          |
#|---------------------------------------------------------------------------------------|
SRC_DISCARDED=

SRC_ARCH_ARM=\
$(CORE_ARM_PATH)/core_init.h            \
$(CORE_ARM_PATH)/core_math.cpp          \
$(CORE_ARM_PATH)/core_math.hpp          \
$(CORE_ARM_PATH)/core_abi.cpp           \
$(CORE_ARM_PATH)/core_character.h       \
$(CORE_ARM_PATH)/core_hooks.c           \
$(CORE_ARM_PATH)/core_hooks.h           \
$(CORE_ARM_PATH)/core_itoa.c            \
$(CORE_ARM_PATH)/core_itoa.h            \
$(CORE_ARM_PATH)/core_main.cpp          \
$(CORE_ARM_PATH)/core_shift.c           \
$(CORE_ARM_PATH)/core_shift.h           \
$(CORE_ARM_PATH)/core_cortex_vectors.c  \
$(CORE_ARM_PATH)/core_cortex_vectors.h  \
$(CORE_ARM_PATH)/CorePrint.cpp          \
$(CORE_ARM_PATH)/CorePrint.hpp          \
$(CORE_ARM_PATH)/CorePrintable.hpp      \
$(CORE_ARM_PATH)/CoreHardwareSerial.hpp \
$(CORE_ARM_PATH)/CoreRingBuffer.cpp     \
$(CORE_ARM_PATH)/CoreRingBuffer.hpp     \
$(CORE_ARM_PATH)/CoreStream.cpp         \
$(CORE_ARM_PATH)/CoreStream.hpp         \
$(CORE_ARM_PATH)/CoreString.cpp         \
$(CORE_ARM_PATH)/CoreString.hpp         \
$(CORE_ARM_PATH)/IPAddress.cpp          \
$(CORE_ARM_PATH)/IPAddress.hpp

SRC_ARCH_COMMON=\
$(CORE_COMMON_PATH)/core_watchdog.h     \
$(CORE_COMMON_PATH)/core_reset.h        \
$(CORE_COMMON_PATH)/core_binary.h       \
$(CORE_COMMON_PATH)/core_constants.h    \
$(CORE_COMMON_PATH)/core_new.cpp        \
$(CORE_COMMON_PATH)/core_new.hpp        \
$(CORE_COMMON_PATH)/Server.hpp          \
$(CORE_COMMON_PATH)/Udp.hpp             \
$(CORE_COMMON_PATH)/Client.hpp

SRC_ARCH_AVR=\
$(CORE_AVR_PATH)/core_dtostrf.c         \
$(CORE_AVR_PATH)/core_dtostrf.h         \
$(CORE_AVR_PATH)/core_interrupt.h       \
$(CORE_AVR_PATH)/core_pgmspace.h

SRC_PORT_SAM=\
$(CORE_SAM_PATH)/Arduino.h              \
$(CORE_SAM_PATH)/core_init.c            \
$(CORE_SAM_PATH)/core_analog.h          \
$(CORE_SAM_PATH)/core_delay.c           \
$(CORE_SAM_PATH)/core_delay.h           \
$(CORE_SAM_PATH)/core_digital.c         \
$(CORE_SAM_PATH)/core_digital.h         \
$(CORE_SAM_PATH)/core_interrupts.c      \
$(CORE_SAM_PATH)/core_interrupts.h      \
$(CORE_SAM_PATH)/core_private.c         \
$(CORE_SAM_PATH)/core_private.h         \
$(CORE_SAM_PATH)/core_reset.c           \
$(CORE_SAM_PATH)/core_watchdog.c        \
$(CORE_SAM_PATH)/core_variant.h         \
$(CORE_SAM_PATH)/core_pulse.cpp         \
$(CORE_SAM_PATH)/core_pulse.hpp         \
$(CORE_SAM_PATH)/core_tone.cpp          \
$(CORE_SAM_PATH)/core_tone.hpp          \
$(CORE_SAM_PATH)/core_analog.c          \
$(CORE_SAM_PATH)/CoreSerial.hpp         \
$(CORE_SAM_PATH)/CoreSPI.cpp            \
$(CORE_SAM_PATH)/CoreSPI.hpp            \
$(CORE_SAM_PATH)/CoreWire.cpp           \
$(CORE_SAM_PATH)/CoreWire.hpp           \
$(CORE_SAM_PATH)/CoreSerial.cpp

SOURCES=$(SRC_ARCH_ARM) $(SRC_ARCH_COMMON) $(SRC_ARCH_AVR) $(SRC_PORT_SAM)