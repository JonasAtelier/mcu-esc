# Read by ESP-IDF (and so esp-forge) as a component, and by any other CMake
# project that add_subdirectory()s this repo, such as a CubeMX-generated STM32
# one. stm-forge never reads it: it compiles every .c under src/, model
# folders included, puts src/ on the include path of this library's sources
# and adds include/ -- which is why each port file, and every model driver,
# is empty when its platform or option is off. ESP_PLATFORM tells the two
# apart; IDF sets it in its early requirements pass as well as in the build.

if(NOT ESP_PLATFORM)
    # A static library `esc`. Every .c under src/, as stm-forge does: a model
    # file compiles to nothing unless its CONFIG_ESC_<MODEL> is 1, so there is
    # no source list here to keep in step with the IDF one below.
    file(GLOB_RECURSE esc_sources CONFIGURE_DEPENDS src/*.c)
    add_library(esc STATIC ${esc_sources})
    # include/ for the app; src/ only for this library's own sources, so a
    # model folder reaches esc_priv.h by plain name and no private header is
    # ever on the app's include path.
    target_include_directories(esc PUBLIC include PRIVATE src)
    # lroundf() in the C615, DJI and DShot drivers.
    target_link_libraries(esc PUBLIC m)

    # The options are the Kconfig bools, read from Kconfig so a new model
    # needs no line here: a true CMake variable CONFIG_ESC_<MODEL> (or
    # CONFIG_ESC_DEBUG) becomes CONFIG_ESC_<MODEL>=1, what stm-forge passes.
    # Only bools: an int option cannot be 1-or-absent. Each `config` line in
    # Kconfig is followed directly by its type line, which is what the match
    # relies on. PUBLIC, because esc_log.h reads CONFIG_ESC_DEBUG in the
    # app's sources.
    file(READ Kconfig esc_kconfig)
    string(REGEX MATCHALL "config ESC_[A-Z0-9_]+[ \t\r\n]+bool" esc_options
           "${esc_kconfig}")
    list(TRANSFORM esc_options REPLACE "config (ESC_[A-Z0-9_]+)[ \t\r\n]+bool"
         "CONFIG_\\1")
    set(esc_models_on "")
    foreach(option IN LISTS esc_options)
        if(${option})
            target_compile_definitions(esc PUBLIC ${option}=1)
            if(NOT option STREQUAL "CONFIG_ESC_DEBUG")
                list(APPEND esc_models_on ${option})
            endif()
        endif()
    endforeach()
    if(NOT esc_models_on)
        message(WARNING "esc: no model is selected; "
                        "set(CONFIG_ESC_<MODEL> ON) before add_subdirectory()")
    endif()

    # The CAN models include mcu-can's can.h and send through can_send(), so
    # they need mcu-can's target `can`: its include dir and options reach
    # this library through the link. Only they need it, as on stm-forge; a
    # C615- or DShot-only build does without mcu-can. The target must exist
    # already, so mcu-can is add_subdirectory()d first.
    if(CONFIG_ESC_C610 OR CONFIG_ESC_C620 OR CONFIG_ESC_GM6020 OR
       CONFIG_ESC_HT03)
        if(NOT TARGET can)
            message(FATAL_ERROR "esc: the CAN models (C610, C620, GM6020, "
                    "HT-03) need mcu-can; add_subdirectory(mcu-can) before "
                    "add_subdirectory(mcu-esc)")
        endif()
        target_link_libraries(esc PUBLIC can)
    endif()

    # The HAL is the app's. MCU_HAL_TARGET names a target carrying its include
    # dirs and defines (USE_HAL_DRIVER, the device, the hal_conf.h directory)
    # -- stm32cubemx in a CubeMX-generated project; MCU_DEVICE_HEADER is the
    # family header the public headers include as CMSIS_device_header, which
    # CubeMX does not define. Both are shared by every mcu-* library, so an
    # app sets them once. Left unset, the app provides them its own way.
    #
    # Only the target's include dirs and defines are copied, the target is
    # never linked: CubeMX up to 6.13 also hangs main.c, the HAL and the
    # startup file on stm32cubemx as INTERFACE sources, which linking would
    # compile into libesc.a. A misspelt name fails at generate time.
    if(MCU_HAL_TARGET)
        target_include_directories(esc PRIVATE
            $<TARGET_PROPERTY:${MCU_HAL_TARGET},INTERFACE_INCLUDE_DIRECTORIES>)
        target_compile_definitions(esc PRIVATE
            $<TARGET_PROPERTY:${MCU_HAL_TARGET},INTERFACE_COMPILE_DEFINITIONS>)
    endif()
    if(MCU_DEVICE_HEADER)
        target_compile_definitions(esc PUBLIC
            "CMSIS_device_header=\"${MCU_DEVICE_HEADER}\"")
    endif()
    return()
endif()

# ESP-IDF: the model files are listed one by one, and PRIV_INCLUDE_DIRS "src"
# lets a model folder reach esc_priv.h by plain name.
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
# when only the C615 is picked; mcu-can's core is then linked out. IDF names
# a local component after its folder, so outside esp-forge mcu-can sits in a
# folder named can, or comes from the component manager under the key can.
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
