
target_sources( ${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/LASPlugin.cpp
        ${CMAKE_CURRENT_LIST_DIR}/LASIOFilter.cpp
        ${CMAKE_CURRENT_LIST_DIR}/LASOpenDialog.cpp
        ${CMAKE_CURRENT_LIST_DIR}/LASSaveDialog.cpp
        ${CMAKE_CURRENT_LIST_DIR}/LASDetails.cpp
        ${CMAKE_CURRENT_LIST_DIR}/LasScalarFieldLoader.cpp
        ${CMAKE_CURRENT_LIST_DIR}/LasScalarFieldSaver.cpp
        ${CMAKE_CURRENT_LIST_DIR}/LasWaveformLoader.cpp
        )
