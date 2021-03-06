##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(merbots_gui)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(
	catkin
	REQUIRED COMPONENTS
		qt_build
		roscpp
		std_msgs
		geometry_msgs
		nav_msgs
		sensor_msgs
		cola2_msgs
		auv_msgs
		cv_bridge
		image_transport
		compressed_image_transport
		merbots_ibvs
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
	roscpp
	std_msgs
	geometry_msgs
	nav_msgs
	sensor_msgs
	cola2_msgs
	auv_msgs
	cv_bridge
	image_transport
	compressed_image_transport
	merbots_ibvs
)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}) 

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically
# included via the dependency call in package.xml
# Add the appropriate components to the component list here
rosbuild_prepare_qt4(QtCore QtGui QtNetwork QtOpengl)

##############################################################################
# Sections
##############################################################################
file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/merbots_gui/*.h*)


QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*)

##############################################################################
# Binaries
##############################################################################

add_executable(merbots_gui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(merbots_gui ${QT_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS merbots_gui RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
