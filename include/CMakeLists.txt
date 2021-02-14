
target_sources( ${PROJECT_NAME}
    PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/LASPlugin.h
		${CMAKE_CURRENT_LIST_DIR}/LASIOFilter.h
		${CMAKE_CURRENT_LIST_DIR}/LASDetails.h
		${CMAKE_CURRENT_LIST_DIR}/LASOpenDialog.h
		${CMAKE_CURRENT_LIST_DIR}/LASSaveDialog.h
		${CMAKE_CURRENT_LIST_DIR}/LasScalarFieldLoader.h
		${CMAKE_CURRENT_LIST_DIR}/LasScalarFieldSaver.h
		${CMAKE_CURRENT_LIST_DIR}/LasWaveformLoader.h
		${CMAKE_CURRENT_LIST_DIR}/LasSavedInfo.h
		)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
	    ${CMAKE_CURRENT_SOURCE_DIR}
)
