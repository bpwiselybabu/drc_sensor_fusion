SET(PROJECT_NAME visiontools)

PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/CMakeModules/")

SET (CMAKE_VERBOSE_MAKEFILE OFF)

IF( NOT CMAKE_BUILD_TYPE )
   SET( CMAKE_BUILD_TYPE Release )
ENDIF()

IF (CMAKE_COMPILER_IS_GNUCXX )
   SET(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g")
   SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -g ")

  ADD_DEFINITIONS("-Wall -Werror -Wno-unused-variable
                   -Wno-unused-but-set-variable -Wno-unknown-pragmas -Wno-dev")
ENDIF()


SET (INCLUDE_DIRS )

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Boost REQUIRED iostreams system filesystem)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(GLUT REQUIRED)
find_package(catkin REQUIRED COMPONENTS sophus)
find_package(pangolin QUIET)

IF(pangolin_FOUND)
    message("Pangolin found")
    ADD_DEFINITIONS("-DWITH_PANGOLIN")
    LIST(APPEND INCLUDE_DIRS ${pangolin_INCLUDE_DIRS})
ENDIF()
LIST(APPEND INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR})


SET (LIB_NAMES GL glut Sophus ${OPENGL_LIBRARIES} )

#FOREACH(lib ${LIB_NAMES})
#  FIND_LIBRARY(LIB_${lib} ${lib})
#  IF (LIB_${lib})
#    MESSAGE(STATUS "found library '${lib}': ${LIB_${lib}}")
#  ELSE (LIB_${lib})
#    MESSAGE(FATAL_ERROR "cannot find library '${lib}' - this will not work ...")
#  ENDIF (LIB_${lib})
#  LIST(APPEND LIBS ${LIB_${lib}})
#ENDFOREACH(lib)

SET (CLASSES  draw2d
              draw3d
              sample
              performance_monitor
              linear_camera)

SET (SOURCE_DIR "visiontools")

SET (SOURCES ${SOURCE_DIR}/${SOURCE_DIR}/gl_data.h
             ${SOURCE_DIR}/${SOURCE_DIR}/abstract_camera.h
             ${SOURCE_DIR}/${SOURCE_DIR}/accessor_macros.h
             ${SOURCE_DIR}/${SOURCE_DIR}/ringbuffer.h
             ${SOURCE_DIR}/${SOURCE_DIR}/stopwatch.h
             ${SOURCE_DIR}/${SOURCE_DIR}/gnuplot-iostream.h)

FOREACH(class ${CLASSES})
  LIST(APPEND SOURCES ${SOURCE_DIR}/${class}.cpp ${SOURCE_DIR}/${SOURCE_DIR}/${class}.h)
ENDFOREACH(class)

LINK_LIBRARIES (${PROJECT_NAME} ${LIBS} ${catkin_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} util ${Boost_LIBRARIES})
IF(pangolin_FOUND)
LINK_LIBRARIES (${PROJECT_NAME} ${pangolin_LIBRARIES})
ENDIF()
INCLUDE_DIRECTORIES(${INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

catkin_package(
	INCLUDE_DIRS visiontools
  	LIBRARIES ${PROJECT_NAME}
  	CATKIN_DEPENDS sophus
  	DEPENDS opencv)

INSTALL(DIRECTORY visiontools DESTINATION ${CMAKE_INSTALL_PREFIX}/include FILES_MATCHING PATTERN "*.h" )
INSTALL(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
