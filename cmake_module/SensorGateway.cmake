
macro(fetchSensorGateway _download_module_path _download_root)
    message(STATUS "Building SensorGateway")
    message(STATUS "Gateway download directory: ${_download_root}")

    set(SENSOR_GATEWAY_DOWNLOAD_ROOT ${_download_root})
    configure_file(
            ${_download_module_path}/SensorGateway_download.cmake.in
            ${_download_root}/CMakeLists.txt
            @ONLY
    )
    unset(SENSOR_GATEWAY_DOWNLOAD_ROOT)

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

    # Adds the targets: SensorGateway
    add_subdirectory(
            ${_download_root}/SensorGateway-src
            ${_download_root}/SensorGateway-build
    )

    find_package(SensorGateway)
    set(EXTERNAL_GATEWAY_INCLUDE_DIR "${_download_root}/SensorGateway-src")
    message(STATUS "Done with SensorGateway")
    message(STATUS)
endmacro()
