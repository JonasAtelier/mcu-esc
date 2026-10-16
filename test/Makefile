CC ?= cc
# gnu11: the esc_* log macros use `, ##__VA_ARGS__`.
CFLAGS += -std=gnu11 -Wall -Wextra -Werror -pedantic
# Signed overflow usually wraps anyway, so a test of wrapping arithmetic
# passes on the broken code too. UBSan makes it abort instead.
CFLAGS += -fsanitize=undefined -fno-sanitize-recover=all

# can.h is mcu-can's, from the checkout next to this one: the CAN models'
# headers include it. Point MCU_CAN elsewhere if yours is not there.
MCU_CAN ?= ../../mcu-can
ifeq ($(wildcard $(MCU_CAN)/include/can.h),)
$(error can.h not found under $(MCU_CAN)/include: clone mcu-can next to this repo or pass MCU_CAN=/path/to/mcu-can)
endif

# src/ for the common private header, and every model folder, because the
# tests include a model's private headers by plain name.
CPPFLAGS += -I../include -I../src $(addprefix -I,$(wildcard ../src/*/)) \
	    -I$(MCU_CAN)/include

# The models' drivers touch the platform only through esc_priv.h: the DJI
# and HT-03 drivers through nothing at all but can_send(), which their tests
# fake, so they run once, on the ESP-IDF stubs. The core, the C615 and the
# bus layer run twice: through src/esc_port_esp.c against idf_stubs/ (esp/)
# and through src/esc_port_stm.c against hal_stubs/ (stm/). The cases live
# once, in *_cases.h. test_dji_math.c and test_ht03_math.c touch nothing but
# the pure headers; test_names.c is the coexistence check against copies of
# esp-c620-control's rm.h, upid's upid.h and esp-sn65hvd230-can's hvd230can.h
# in fixtures/.

MODELS := -DCONFIG_ESC_C610=1 -DCONFIG_ESC_C620=1 -DCONFIG_ESC_GM6020=1 \
	  -DCONFIG_ESC_C615=1 -DCONFIG_ESC_HT03=1 -DCONFIG_ESC_DSHOT=1
# What ESP-IDF passes: ESP_PLATFORM, and every model on.
ESP_CPPFLAGS := -Iidf_stubs -DESP_PLATFORM $(MODELS)
# What stm-forge passes: the family header the code includes, the HAL modules
# an app's hal_conf would enable, and every model option on.
STM_CPPFLAGS := -Ihal_stubs \
		-DCMSIS_device_header='"stm32_hal_stub.h"' \
		-DHAL_I2C_MODULE_ENABLED -DHAL_SPI_MODULE_ENABLED \
		-DHAL_UART_MODULE_ENABLED -DHAL_TIM_MODULE_ENABLED \
		-DHAL_DMA_MODULE_ENABLED \
		$(MODELS) -DCONFIG_ESC_DEBUG=1

# The core and the stubs every binary of a suite links.
CORE     := ../src/esc.c ../src/esc_log.c
ESP_CORE := $(CORE) ../src/esc_port_esp.c idf_stubs/idf_stubs.c
STM_CORE := $(CORE) ../src/esc_port_stm.c hal_stubs/hal_stubs.c

DJI  := ../src/dji/dji.c
HT03 := ../src/ht03/ht03.c
C615 := ../src/c615/c615.c
DSHOT := ../src/dshot/dshot.c

MATH_TESTS  := test_dji_math test_ht03_math test_dshot_math test_names
MODEL_TESTS := test_dji test_ht03
ESP_TESTS   := $(addprefix esp/,test_core test_c615 test_dshot \
		test_dshot_nosync test_dshot_c3 test_port_esp)
STM_TESTS   := $(addprefix stm/,test_core test_c615 test_dshot \
		test_dshot_nof4 test_port_stm)
TESTS       := $(MATH_TESTS) $(MODEL_TESTS) $(ESP_TESTS) $(STM_TESTS)

.PHONY: all clean test

all: $(TESTS)

test_dji_math: test_dji_math.c
test_ht03_math: test_ht03_math.c
test_dshot_math: test_dshot_math.c
test_names: test_names.c
test_names: CPPFLAGS += -Ifixtures

test_dji: test_dji.c $(DJI) $(ESP_CORE)
test_ht03: test_ht03.c $(HT03) $(ESP_CORE)

esp/test_core: esp/test_core.c $(ESP_CORE)
esp/test_c615: esp/test_c615.c $(C615) $(ESP_CORE)
esp/test_dshot: esp/test_dshot.c $(DSHOT) $(ESP_CORE)
esp/test_dshot_nosync: esp/test_dshot_nosync.c $(DSHOT) $(ESP_CORE)
esp/test_dshot_nosync: CPPFLAGS += -DSTUB_RMT_NO_TX_SYNCHRO
esp/test_dshot_c3: esp/test_dshot_c3.c $(DSHOT) $(ESP_CORE)
esp/test_dshot_c3: CPPFLAGS += -DSTUB_RMT_C3
esp/test_port_esp: esp/test_port_esp.c $(ESP_CORE)

stm/test_core: stm/test_core.c $(STM_CORE)
stm/test_c615: stm/test_c615.c $(C615) $(STM_CORE)
stm/test_dshot: stm/test_dshot.c $(DSHOT) $(STM_CORE)
stm/test_dshot_nof4: stm/test_dshot_nof4.c $(DSHOT) $(STM_CORE)
stm/test_dshot_nof4: CPPFLAGS += -DSTUB_NOT_F4
stm/test_port_stm: stm/test_port_stm.c $(STM_CORE)

$(MATH_TESTS) $(MODEL_TESTS) $(ESP_TESTS): CPPFLAGS += $(ESP_CPPFLAGS)
$(STM_TESTS): CPPFLAGS += $(STM_CPPFLAGS)

# Every printf() in a binary that links the HAL stubs also lands in
# stub_log_last, so a test can check what was logged.
$(STM_TESTS): LDFLAGS += -Wl,--wrap=printf

$(TESTS):
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) $(filter %.c,$^) -o $@ -lm

test: $(TESTS)
	@set -e; for t in $(TESTS); do echo "./$$t"; ./$$t; done

clean:
	rm -f $(TESTS)

# Headers are prerequisites so that editing one rebuilds. Without this, `make
# test` reruns a stale binary and reports green for code it never compiled.
# Recipes use $(filter %.c,$^) so the headers never reach the compiler.
HDRS  := $(wildcard ../include/*.h ../src/*.h ../src/*/*.h) \
	 $(MCU_CAN)/include/can.h
STUBS := $(wildcard idf_stubs/*.h idf_stubs/*/*.h hal_stubs/*.h fixtures/*.h)
$(TESTS): $(HDRS) $(STUBS) $(wildcard *_cases.h)
