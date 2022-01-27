set(DEPENDENCY_LIBRARIES ${OpenSim_LIBRARIES} RealTime Common)

# global files
file(GLOB includes include/*.h)
file(GLOB sources src/*.cpp)
# file(GLOB applications applications/*.cpp)
file(
  GLOB applications
  applications/OnlineLowerLimbUIMUIK.cpp
  applications/OnlineUpperLimbUIMUIK.cpp
)
# file(GLOB tests tests/*.cpp)

# dependencies
include_directories(include/)
include_directories(../RealTime/include/)
include_directories(../Common/include/)

# dynamic library
set(target UIMU)

addlibrary(
  KIT ${target}
  LINKLIBS ${DEPENDENCY_LIBRARIES}
  INCLUDES ${includes}
  SOURCES ${sources})

# applications
addapplications(
  SOURCES ${applications}
  LINKLIBS ${target} ${DEPENDENCY_LIBRARIES}
)


