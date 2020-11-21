
target_sources( ${PROJECT_NAME}
    PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/LASPlugin.h
		${CMAKE_CURRENT_LIST_DIR}/LASIOFilter.h
		${CMAKE_CURRENT_LIST_DIR}/LASDetails.h
		)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
	    ${CMAKE_CURRENT_SOURCE_DIR}
)
