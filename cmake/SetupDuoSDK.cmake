function(setup_duo)
  set(SETUP_SCRIPT "${PROJECT_SOURCE_DIR}/cmake/setup_duo")
  execute_process(
    COMMAND ${SETUP_SCRIPT}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
endfunction()
