cmake_minimum_required(VERSION 3.0.2)
project(bw_ros_serial)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
)

find_package(OpenCV)


catkin_package(

)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV}
)

set(SOURCES
  ./ads_include/AdsLib/AdsDef.cpp
  ./ads_include/AdsLib/AdsDevice.cpp
  ./ads_include/AdsLib/AdsFile.cpp
  ./ads_include/AdsLib/AdsLib.cpp
  ./ads_include/AdsLib/Frame.cpp
  ./ads_include/AdsLib/LicenseAccess.cpp
  ./ads_include/AdsLib/Log.cpp
  ./ads_include/AdsLib/RouterAccess.cpp
  ./ads_include/AdsLib/RTimeAccess.cpp
  ./ads_include/AdsLib/Sockets.cpp

  ./ads_include/AdsLib/standalone/AdsLib.cpp
  ./ads_include/AdsLib/standalone/AmsConnection.cpp
  ./ads_include/AdsLib/standalone/AmsNetId.cpp
  ./ads_include/AdsLib/standalone/AmsPort.cpp
  ./ads_include/AdsLib/standalone/AmsRouter.cpp
  ./ads_include/AdsLib/standalone/NotificationDispatcher.cpp
)
add_library(public_lib1 ${SOURCES})
target_include_directories(public_lib1 PUBLIC ./ads_include/AdsLib/.) 


add_executable(fusion_ins src/fusion.cpp)
target_link_libraries(fusion_ins ${catkin_LIBRARIES} ${OpenCV_LIBS} public_lib1)

add_executable(comparsion src/comparsion.cpp)
target_link_libraries(comparsion ${catkin_LIBRARIES})