# ESP-IDF only. stm-forge never reads this file: it compiles every .c under
# src/, model folders included, puts src/ on the include path of this
# library's sources and adds include/ -- which is why each port file, and
# every model driver, is empty when its platform or option is off. Here the
# model files are listed one by one, and PRIV_INCLUDE_DIRS "src" lets a
# model folder reach esc_priv.h by plain name.
#
# The core is always built, even with CONFIG_ESC off; each model driver only
# when its Kconfig option is on. Kconfig values are available here during the
# build phase, and only SRCS depends on them -- never REQUIRES, which IDF
# resolves before sdkconfig. That is also why ESC off still builds the core:
# an empty SRCS would need a second, differently-shaped register call, and an
# unreferenced core costs nothing once --gc-sections has run.
#
# REQUIRES can is mcu-can, under its esp-forge component name: dji.h and
# ht03.h include can.h. For the same reason as above it is required even
# when only the C615 is picked; mcu-can's core is then linked out.
#
# No file name here is one of esp-c620-control's (core.c, loop.c, status.c,
# internal.h), and no global symbol either: an app links both.
#
# REQUIRES driver, not esp_driver_*: the split components do not exist on
# IDF 5.2, and driver still pulls them in on later versions; LEDC and RMT
# are there.
set(srcs "src/esc.c" "src/esc_log.c" "src/esc_port_esp.c")

if(CONFIG_ESC AND NOT (CONFIG_ESC_C610 OR CONFIG_ESC_C620 OR
                       CONFIG_ESC_GM6020 OR CONFIG_ESC_C615 OR CONFIG_ESC_HT03 OR
                       CONFIG_ESC_DSHOT))
    message(WARNING "esc: CONFIG_ESC is on but no model is selected; "
                    "set CONFIG_ESC_<MODEL>=y (esp-forge: ESC_<MODEL> = y)")
endif()

if(CONFIG_ESC_C610 OR CONFIG_ESC_C620 OR CONFIG_ESC_GM6020)
    list(APPEND srcs "src/dji/dji.c")
endif()
if(CONFIG_ESC_C615)
    list(APPEND srcs "src/c615/c615.c")
endif()
if(CONFIG_ESC_DSHOT)
    list(APPEND srcs "src/dshot/dshot.c")
endif()
if(CONFIG_ESC_HT03)
    list(APPEND srcs "src/ht03/ht03.c")
endif()

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "src"
                       REQUIRES driver can)
