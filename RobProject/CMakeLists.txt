CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)
# The name of the project. (EDIT THIS)
PROJECT(Avasa-PP)
 
# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})
 
##### IMPORTANT EDIT THESE PATHS TO REFLECT YOUR ROBWORK INSTALLATION #####
# Now set the RW/RWS root (edit this if necessary)
SET(RW_ROOT "/home/ali/WorkSpace/RobWork/trunk/RobWork/")

# set some robwork settings (EDIT THESE TO FIT YOUR ENVIRONMENT)
SET(CMAKE_BUILD_TYPE Release)
 
#Include default settings for constructing a robwork dependent project
SET(CMAKE_MODULE_PATH ${RW_ROOT}/build ${CMAKE_MODULE_PATH})
FIND_PACKAGE(RobWork)
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )
 
# if you have additional libraries or include dirs then add them here
INCLUDE_DIRECTORIES(${ROOT} ${ROBWORK_INCLUDE_DIR})
 
# And now we add any targets that we want
add_executable(pathPlanning pathPlanning.cpp)

target_link_libraries(pathPlanning ${ROBWORK_LIBRARIES})

LINK_DIRECTORIES("/usr/local/lib")
        SET(OpenCV_LIBS opencv_core opencv_imgproc opencv_imgcodecs opencv_calib3d opencv_video opencv_videoio opencv_features2d opencv_ml opencv_highgui opencv_objdetect opencv_contrib opencv_legacy opencv_gpu )
