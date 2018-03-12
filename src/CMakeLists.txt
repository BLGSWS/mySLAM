# OpenCV
FIND_PACKAGE( OpenCV REQUIRED )

# PCL
FIND_PACKAGE( PCL REQUIRED COMPONENTS common visualization io filters )
LIST(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") # use this in Ubuntu 16.04

ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

# G2O
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
SET( G2O_ROOT /usr/local/include/g2o )
FIND_PACKAGE( G2O )
# CSparse
FIND_PACKAGE( CSparse )

INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )

set(CMAKE_CXX_FLAGS "-std=c++11")

ADD_LIBRARY( tools STATIC Tools.cpp )
TARGET_LINK_LIBRARIES( tools 
    ${OpenCV_LIBS} )

ADD_LIBRARY( pretreat STATIC Pretreat.cpp )
TARGET_LINK_LIBRARIES( pretreat 
    tools 
    ${OpenCV_LIBS} )

ADD_LIBRARY( map STATIC Map.cpp )
TARGET_LINK_LIBRARIES( map 
    tools 
    ${PCL_LIBRARIES} )

ADD_LIBRARY( motion STATIC Motion.cpp)
TARGET_LINK_LIBRARIES( motion 
    tools
    ${OpenCV_LIBS} )

ADD_LIBRARY( optimizer STATIC Optimizer.cpp)
TARGET_LINK_LIBRARIES( optimizer
    motion
    tools
    g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY} )

ADD_LIBRARY( vo STATIC VisualOdometry.cpp )
TARGET_LINK_LIBRARIES( vo
    motion
    optimizer )

ADD_EXECUTABLE( main main.cpp )
TARGET_LINK_LIBRARIES( main 
    vo 
    pretreat
    map )

