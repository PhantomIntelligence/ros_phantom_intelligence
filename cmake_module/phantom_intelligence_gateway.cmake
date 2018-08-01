
macro(fetch_phantom_intelligence_gateway _download_module_path _download_root)
    message(STATUS "Building Phantom Intelligence Gateway")
    message(STATUS "Gateway download directory: ${_download_root}")

    set(PHANTOM_INTELLIGENCE_GATEWAY_DOWNLOAD_ROOT ${_download_root})
    configure_file(
            ${_download_module_path}/phantom_intelligence_gateway_download.cmake.in
            ${_download_root}/CMakeLists.txt
            @ONLY
    )
    unset(PHANTOM_INTELLIGENCE_GATEWAY_DOWNLOAD_ROOT)

    execute_process(
            COMMAND
            "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
            WORKING_DIRECTORY
            ${_download_root}
    )
    execute_process(
            COMMAND
            "${CMAKE_COMMAND}" --build .
            WORKING_DIRECTORY
            ${_download_root}
    )

    # Adds the targets: SpiritSensorGateway
    add_subdirectory(
            ${_download_root}/phantom_intelligence_gateway_src
            ${_download_root}/phantom_intelligence_gateway_build
    )

    message(STATUS "Done with Gateway")
endmacro()
