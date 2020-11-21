option( PLUGIN_EXAMPLE_IO "LAS IO plugin using laszip" OFF )

if( PLUGIN_LAS_IO )
	project( LASIO )

    AddPlugin( NAME ${PROJECT_NAME} TYPE io )

    list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

    find_package(LASzip REQUIRED)

    message("LOL ${LASZIP_INCLUDE_DIR}")
    message("LOL ${LASZIP_LIBRARIES}")

    target_link_libraries(LASIO ${LASZIP_LIBRARIES})
    target_include_directories(LASIO PRIVATE ${LASZIP_INCLUDE_DIR})
    
    add_subdirectory( include )
    add_subdirectory( src )

    # TODO install LASZIPLIBRARIES


endif()
