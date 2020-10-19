# macro that flashes a target MCU
function (flashMcu TARGET MCU)
  string(APPEND HEX_TARGET ${TARGET} _hex)
  string(APPEND HEX_CMD ${TARGET} .hex)
  string(APPEND UPLOAD_TARGET ${TARGET} _upload)
  string(TOLOWER ${MCU} MCU_LOWER)
  string(APPEND MCU_CMD -mmcu= ${MCU_LOWER})
  # Add 'make hex' target
  add_custom_command(OUTPUT ${HEX_CMD}
    DEPENDS ${TARGET}
    COMMAND ${CMAKE_SIZE} ${TARGET}
    COMMAND ${CMAKE_OBJCOPY} -O ihex -R .eeprom ${TARGET} ${HEX_CMD}
  )
  add_custom_target(${HEX_TARGET}
    DEPENDS ${HEX_CMD}
  )
  # Add 'make upload' target to upload binary
  # Check whether we are using WSL or linux to determine loader to use
  file(READ "/proc/version" linux_version)
  string(TOLOWER ${linux_version} linux_version_lower)
  string(FIND ${linux_version_lower} "microsoft" wsl)
  if (wsl GREATER -1)
    add_custom_target(${UPLOAD_TARGET}
      COMMAND ${CMAKE_SOURCE_DIR}/tools/teensy_loader_cli.exe ${MCU_CMD} -w ${HEX_CMD} -v
      DEPENDS ${HEX_CMD}
    )
  else (wsl GREATER -1)
    add_custom_target(${UPLOAD_TARGET}
      COMMAND ${CMAKE_SOURCE_DIR}/tools/teensy_loader_cli ${MCU_CMD} -s ${HEX_CMD} -v
      DEPENDS ${HEX_CMD}
    )
  endif (wsl GREATER -1)
  # Linker
  set_property(TARGET ${TARGET} PROPERTY LINK_DEPENDS ${LINKER_SCRIPT})
endfunction ()
