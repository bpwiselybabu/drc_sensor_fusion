set(CMAKE_CXX_FLAGS "-std=c++0x -O2 -march=native -mno-avx  -msse4.2 ${CMAKE_CXX_FLAGS}")
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

include_directories(DMR)

add_library(${PROJECT_NAME}_DMR 	alphaBlend.cpp
			        	bilateralFilter.cpp
			    		guidedFilter.cpp
			    		depthEval.cpp
			    		depthFilter.cpp
			   		jointBilateralFilter.cpp
			    		jointNearest.cpp
			    		minmaxFilter.cpp
			    		split.cpp
			    		trilateralFilter.cpp
			    		util.cpp
			    		viewSynthesis.cpp
	  )

target_link_libraries(${PROJECT_NAME}_DMR ${OpenCV_LIBS})

install(TARGETS ${PROJECT_NAME}_DMR
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
