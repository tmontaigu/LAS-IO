option( PLUGIN_LAS_IO "LAS IO plugin using laszip" OFF )

if( PLUGIN_LAS_IO )
	project( LASIO )

    AddPlugin( NAME ${PROJECT_NAME} TYPE io )

    list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

    target_compile_definitions(${PROJECT_NAME} PRIVATE -DQT_FORCE_ASSERTS)

    find_package(LASzip REQUIRED)

    message("LOL ${LASZIP_INCLUDE_DIR}")
    message("LOL ${LASZIP_LIBRARIES}")

    target_link_libraries(LASIO ${LASZIP_LIBRARIES})
    target_include_directories(LASIO PRIVATE ${LASZIP_INCLUDE_DIR})
    
    add_subdirectory( include )
    add_subdirectory( src )
    add_subdirectory( ui )

    # TODO install LASZIPLIBRARIES


endif()
