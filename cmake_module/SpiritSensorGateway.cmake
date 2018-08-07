
macro(fetchSpiritSensorGateway _download_module_path _download_root)
    message(STATUS "Building SpiritSensorGateway")
    message(STATUS "Gateway download directory: ${_download_root}")

    set(SPIRIT_SENSOR_GATEWAY_DOWNLOAD_ROOT ${_download_root})
    configure_file(
            ${_download_module_path}/SpiritSensorGateway_download.cmake.in
            ${_download_root}/CMakeLists.txt
            @ONLY
    )
    unset(SPIRIT_SENSOR_GATEWAY_DOWNLOAD_ROOT)

    execute_process(
            COMMAND
            "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
            WORKING_DIRECTORY ${_download_root}
    )
    execute_process(
            COMMAND
            "${CMAKE_COMMAND}" --build .
            WORKING_DIRECTORY ${_download_root}
    )

    # Adds the targets: SpiritSensorGateway
    add_subdirectory(
            ${_download_root}/SpiritSensorGateway-src
            ${_download_root}/SpiritSensorGateway-build
    )

    find_package(SpiritSensorGateway)
    set(EXTERNAL_GATEWAY_INCLUDE_DIR "${_download_root}/SpiritSensorGateway-src")
    message(STATUS "Done with SpiritSensorGateway")
    message(STATUS)
endmacro()
