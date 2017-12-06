FIND_PACKAGE( OpenCV REQUIRED )

FIND_PACKAGE( PCL REQUIRED COMPONENTS common visualization io filters )
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") # use this in Ubuntu 16.04

ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

MESSAGE(STATUS "PCL include is" ${PCL_INCLUDE_DIRS} )
MESSAGE(STATUS "PCL lib is" ${PCL_LIBRARIES} )

ADD_LIBRARY( tools STATIC Tools.cpp )
TARGET_LINK_LIBRARIES( tools 
    ${OpenCV_LIBS} )

ADD_LIBRARY( disparity STATIC Disparity.cpp )
TARGET_LINK_LIBRARIES( disparity 
    ${OpenCV_LIBS} )

ADD_LIBRARY( mypointcloud STATIC MyPointCloud.cpp )
TARGET_LINK_LIBRARIES( mypointcloud 
    tools 
    ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} )

ADD_LIBRARY( features STATIC Features.cpp)
TARGET_LINK_LIBRARIES( features 
    tools
    ${OpenCV_LIBS} )

ADD_EXECUTABLE( main main.cpp )
TARGET_LINK_LIBRARIES( main 
    mypointcloud 
    disparity
    features )

