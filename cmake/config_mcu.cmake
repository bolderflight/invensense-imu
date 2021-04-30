# macro that sets up definitions, compile options, and link options depending on MCU
macro (configMcu MCU)
  # Add math libraries
  link_directories(${CMAKE_SOURCE_DIR}/lib)
  # MCU specific config
  if (MCU STREQUAL "MK20DX128")
    message("Configuring MK20DX128 build.")
    # Setup def for the loader
    set(MCU_LOAD mk20dx128)
    # Definitions
    add_definitions(
      -D__MCU__
      -D__MK20DX128__
      -DF_CPU=48000000 
      -DUSB_SERIAL
      -DTEENSYDUINO=141  
      -DARDUINO=10805
    )
    # Compile options
    add_compile_options(
      $<$<COMPILE_LANGUAGE:C>:-std=c11>
      $<$<COMPILE_LANGUAGE:CXX>:-std=c++20>
      $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
      $<$<COMPILE_LANGUAGE:CXX>:-felide-constructors>
      $<$<COMPILE_LANGUAGE:CXX>:-Wno-error=narrowing>
      $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
      $<$<COMPILE_LANGUAGE:CXX>:-Wno-volatile>
      -g 
      -Os 
      -Wno-psabi 
      -mthumb 
      -ffunction-sections 
      -fdata-sections 
      -nostdlib 
      -MMD
      -mcpu=cortex-m4 
    )
    # Linker script
    set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/ld/mk20dx128.ld")
    # Link options
    add_link_options(
      -Os 
      --specs=nano.specs
      LINKER:--defsym=__rtc_localtime=0
      -Wl,--gc-sections,--relax
      -mcpu=cortex-m4 
      -T${LINKER_SCRIPT}
    )
    # Link libraries
    link_libraries(
      -larm_cortexM4l_math
      -lm 
      -lc
      -lstdc++
    )
  elseif (MCU STREQUAL "MK20DX256")
    message("Configuring MK20DX256 build.")
    # Setup def for the loader
    set(MCU_LOAD mk20dx256)
    # Definitions
    add_definitions(
      -D__MCU__
      -D__MK20DX256__
      -DF_CPU=72000000 
      -DUSB_SERIAL
      -DTEENSYDUINO=141  
      -DARDUINO=10805
    )
    # Compile options
    add_compile_options(
      $<$<COMPILE_LANGUAGE:C>:-std=c11>
      $<$<COMPILE_LANGUAGE:CXX>:-std=c++20>
      $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
      $<$<COMPILE_LANGUAGE:CXX>:-felide-constructors>
      $<$<COMPILE_LANGUAGE:CXX>:-Wno-error=narrowing>
      $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
      $<$<COMPILE_LANGUAGE:CXX>:-Wno-volatile>
      -g 
      -Os 
      -Wno-psabi 
      -mthumb 
      -ffunction-sections 
      -fdata-sections 
      -nostdlib 
      -MMD
      -mcpu=cortex-m4 
    )
    # Linker script
    set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/ld/mk20dx256.ld")
    # Link options
    add_link_options(
      -Os 
      --specs=nano.specs
      LINKER:--defsym=__rtc_localtime=0
      -Wl,--gc-sections,--relax
      -mcpu=cortex-m4 
      -T${LINKER_SCRIPT}
    )
    # Link libraries
    link_libraries(
      -larm_cortexM4l_math
      -lm 
      -lc
      -lstdc++
    )
  elseif (MCU STREQUAL "MK64FX512")
    message("Configuring MK64FX512 build.")
    # Setup def for the loader
    set(MCU_LOAD mk64fx512)
    # Definitions
    add_definitions(
      -D__MCU__
      -D__MK64FX512__
      -DF_CPU=120000000 
      -DUSB_SERIAL
      -DTEENSYDUINO=141  
      -DARDUINO=10805
    )
    # Compile options
    add_compile_options(
      $<$<COMPILE_LANGUAGE:C>:-std=c11>
      $<$<COMPILE_LANGUAGE:CXX>:-std=c++20>
      $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
      $<$<COMPILE_LANGUAGE:CXX>:-felide-constructors>
      $<$<COMPILE_LANGUAGE:CXX>:-Wno-error=narrowing>
      $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
      $<$<COMPILE_LANGUAGE:CXX>:-Wno-volatile>
      -g 
      -Os 
      -Wno-psabi 
      -mthumb 
      -ffunction-sections 
      -fdata-sections 
      -nostdlib 
      -MMD
      -mcpu=cortex-m4 
      -mfloat-abi=hard 
      -mfpu=fpv4-sp-d16
    )
    # Linker script
    set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/ld/mk64fx512.ld")
    # Link options
    add_link_options(
      -Os 
      --specs=nano.specs
      LINKER:--defsym=__rtc_localtime=0
      -Wl,--gc-sections,--relax
      -mcpu=cortex-m4 
      -mfloat-abi=hard 
      -mfpu=fpv4-sp-d16
      -T${LINKER_SCRIPT}
    )
    # Link libraries
    link_libraries(
      -larm_cortexM4lf_math
      -lm 
      -lc
      -lstdc++
    )
  elseif (MCU STREQUAL "MK66FX1M0")
    message("Configuring MK66FX1M0 build.")
    # Setup def for the loader
    set(MCU_LOAD mk66fx1m0)
    # Definitions
    add_definitions(
      -D__MCU__
      -D__MK66FX1M0__
      -DF_CPU=180000000 
      -DUSB_SERIAL
      -DTEENSYDUINO=141  
      -DARDUINO=10805
    )
    # Compile options
    add_compile_options(
      $<$<COMPILE_LANGUAGE:C>:-std=c11>
      $<$<COMPILE_LANGUAGE:CXX>:-std=c++20>
      $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
      $<$<COMPILE_LANGUAGE:CXX>:-felide-constructors>
      $<$<COMPILE_LANGUAGE:CXX>:-Wno-error=narrowing>
      $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
      $<$<COMPILE_LANGUAGE:CXX>:-Wno-volatile>
      -g 
      -Os
      -Wno-psabi 
      -mthumb 
      -ffunction-sections 
      -fdata-sections 
      -nostdlib 
      -MMD
      -mcpu=cortex-m4 
      -mfloat-abi=hard 
      -mfpu=fpv4-sp-d16
    )
    # Linker script
    set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/ld/mk66fx1m0.ld")
    # Link options
    add_link_options(
      -Os 
      --specs=nano.specs
      LINKER:--defsym=__rtc_localtime=0
      -Wl,--gc-sections,--relax
      -mcpu=cortex-m4 
      -mfloat-abi=hard 
      -mfpu=fpv4-sp-d16
      -T${LINKER_SCRIPT}
    )
    # Link libraries
    link_libraries(
      -larm_cortexM4lf_math
      -lm 
      -lc
      -lstdc++
    )
  elseif (MCU STREQUAL "MKL26Z64")
    message("Configuring MKL26Z64 build.")
    # Setup def for the loader
    set(MCU_LOAD mkl26z64)
    # Definitions
    add_definitions(
      -D__MCU__
      -D__MKL26Z64__
      -DF_CPU=48000000 
      -DUSB_SERIAL
      -DTEENSYDUINO=141  
      -DARDUINO=10805
    )
    # Compile options
    add_compile_options(
      $<$<COMPILE_LANGUAGE:C>:-std=c11>
      $<$<COMPILE_LANGUAGE:CXX>:-std=c++20>
      $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
      $<$<COMPILE_LANGUAGE:CXX>:-felide-constructors>
      $<$<COMPILE_LANGUAGE:CXX>:-Wno-error=narrowing>
      $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
      $<$<COMPILE_LANGUAGE:CXX>:-Wno-volatile>
      -g 
      -Os 
      -Wno-psabi 
      -mthumb 
      -ffunction-sections 
      -fdata-sections 
      -nostdlib 
      -MMD
      -mcpu=cortex-m0plus
    )
    # Linker script
    set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/ld/mkl26z64.ld")
    # Link options
    add_link_options(
      -Os 
      --specs=nano.specs
      LINKER:--defsym=__rtc_localtime=0
      -Wl,--gc-sections,--relax
      -mcpu=cortex-m0plus 
      -T${LINKER_SCRIPT}
    )
    # Link libraries
    link_libraries(
      -larm_cortexM0l_math
      -lm 
      -lc
      -lstdc++
    )
  elseif (MCU STREQUAL "IMXRT1062_T40")
    message("Configuring IMXRT1062 T4.0 build.")
    # Setup def for the loader
    set(MCU_LOAD imxrt1062)
    # Definitions
    add_definitions(
      -D__MCU__
      -D__IMXRT1062__
      -DF_CPU=528000000 
      -DUSB_SERIAL
      -DTEENSYDUINO=153
      -DARDUINO=10810
      -DARDUINO_TEENSY40
    )
    # Compile options
    add_compile_options(
      $<$<COMPILE_LANGUAGE:C>:-std=gnu11>
      $<$<COMPILE_LANGUAGE:CXX>:-std=c++20>
      $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
      $<$<COMPILE_LANGUAGE:CXX>:-felide-constructors>
      $<$<COMPILE_LANGUAGE:CXX>:-Wno-error=narrowing>
      $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
      $<$<COMPILE_LANGUAGE:CXX>:-Wno-volatile>
      -g 
      -Os 
      -Wno-psabi 
      -mthumb 
      -ffunction-sections 
      -fdata-sections 
      -nostdlib 
      -MMD
      -mcpu=cortex-m7
      -mfloat-abi=hard 
      -mfpu=fpv5-d16
    )
    # Linker script
    set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/ld/imxrt1062.ld")
    # Link options
    add_link_options(
      -Os 
      --specs=nano.specs
      LINKER:--defsym=__rtc_localtime=0
      -Wl,--gc-sections,--relax
      -mcpu=cortex-m7 
      -mfloat-abi=hard 
      -mfpu=fpv5-d16
      -T${LINKER_SCRIPT}
    )
    # Link libraries
    link_libraries(
      -larm_cortexM7lfsp_math
      -lm 
      -lc
      -lstdc++
    )
  elseif (MCU STREQUAL "IMXRT1062_T41")
    message("Configuring IMXRT1062 T4.1 build.")
    # Setup def for the loader
    set(MCU_LOAD imxrt1062)
    # Definitions
    add_definitions(
      -D__MCU__
      -D__IMXRT1062__
      -DF_CPU=528000000 
      -DUSB_SERIAL
      -DTEENSYDUINO=153
      -DARDUINO=10810
      -DARDUINO_TEENSY40
    )
    # Compile options
    add_compile_options(
      $<$<COMPILE_LANGUAGE:C>:-std=gnu11>
      $<$<COMPILE_LANGUAGE:CXX>:-std=c++20>
      $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
      $<$<COMPILE_LANGUAGE:CXX>:-felide-constructors>
      $<$<COMPILE_LANGUAGE:CXX>:-Wno-error=narrowing>
      $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
      $<$<COMPILE_LANGUAGE:CXX>:-Wno-volatile>
      -g 
      -Os 
      -Wno-psabi 
      -mthumb 
      -ffunction-sections 
      -fdata-sections 
      -nostdlib 
      -MMD
      -mcpu=cortex-m7
      -mfloat-abi=hard 
      -mfpu=fpv5-d16
    )
    # Linker script
    set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/ld/imxrt1062_t41.ld")
    # Link options
    add_link_options(
      -Os 
      --specs=nano.specs
      LINKER:--defsym=__rtc_localtime=0
      -Wl,--gc-sections,--relax
      -mcpu=cortex-m7 
      -mfloat-abi=hard 
      -mfpu=fpv5-d16
      -T${LINKER_SCRIPT}
    )
    # Link libraries
    link_libraries(
      -larm_cortexM7lfsp_math
      -lm 
      -lc
      -lstdc++
    )
  else ()
    message(FATAL_ERROR "ERROR: Unknown MCU selected. Available MCUs are: MK20DX128, MK20DX256, MK64FX512, MK66FX1M0, and MKL26Z64. Use -DMCU to specify the target.")
  endif ()
endmacro ()
