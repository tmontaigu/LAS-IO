
target_sources(${PROJECT_NAME}
        PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/LasPlugin.h
        ${CMAKE_CURRENT_LIST_DIR}/LasIOFilter.h
        ${CMAKE_CURRENT_LIST_DIR}/LasDetails.h
        ${CMAKE_CURRENT_LIST_DIR}/LasOpenDialog.h
        ${CMAKE_CURRENT_LIST_DIR}/LasSaveDialog.h
        ${CMAKE_CURRENT_LIST_DIR}/LasScalarFieldLoader.h
        ${CMAKE_CURRENT_LIST_DIR}/LasScalarFieldSaver.h
        ${CMAKE_CURRENT_LIST_DIR}/LasWaveformLoader.h
        ${CMAKE_CURRENT_LIST_DIR}/LasSavedInfo.h
        ${CMAKE_CURRENT_LIST_DIR}/LasWaveformSaver.h

        )

target_include_directories(${PROJECT_NAME}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        )
