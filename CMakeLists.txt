option(PLUGIN_LAS_IO "LAS IO plugin using laszip" OFF)

if (PLUGIN_LAS_IO)
    project(LASIO)

    AddPlugin(NAME ${PROJECT_NAME} TYPE io)

    list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
    find_package(LASzip MODULE REQUIRED)

    target_compile_definitions(${PROJECT_NAME} PRIVATE -DQT_FORCE_ASSERTS)

    target_link_libraries(LASIO LASzip::LASzip)

    add_subdirectory(include)
    add_subdirectory(src)
    add_subdirectory(ui)

    if (WIN32)
        install(FILES ${LASZIP_DLL} DESTINATION ${CLOUDCOMPARE_DEST_FOLDER})
    endif()
endif ()
