
cmake_minimum_required(VERSION 2.8.3)
project(cav_simulation)

add_compile_options(-std=c++11)

find_package( catkin REQUIRED COMPONENTS 
	cav_msgs	
	roscpp 
	rospy 
	std_msgs)

catkin_package(CATKIN_DEPENDS cav_msgs)

 
## Build
include_directories(
	${catkin_INCLUDE_DIRS} 
	${CMAKE_CURRENT_SOURCE_DIR} 
	${CMAKE_CURRENT_BINARY_DIR}
	src )


## src
	file(GLOB proj_SRC "src/*.cpp")
	file(GLOB GUI_SRC "src/*/*.cpp")

 

add_executable(${PROJECT_NAME} ${proj_SRC} ${GUI_SRC})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}) 



## Install 
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
