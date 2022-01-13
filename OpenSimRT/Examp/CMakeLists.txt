# global files
#file(GLOB includes include/*.h)
#file(GLOB sources src/*.cpp)
## file(GLOB tests tests/*.cpp)
#file(GLOB tests
#	#  tests/TestCircularBuffer.cpp
#	#  tests/TestLowPassSmoothFilter.cpp
#	#  tests/TestButterWorthFilter.cpp
#	#  tests/TestSyncManager.cpp
#  )
find_package(OpenSim 4.0 REQUIRED PATHS "${OPENSIM_INSTALL_DIR}")

# dependencies
include_directories(include/)
set(DEPENDENCY_LIBRARIES ${OpenSim_LIBRARIES})

# dynamic library
set(target Examp)

add_executable(example src/example.cpp )

#addLibrary(
#  KIT ${target}
#  SOURCES ${sources}
#  INCLUDES ${includes}
#  LINKLIBS ${DEPENDENCY_LIBRARIES}
#  )
#
## tests
#addTests(
#  TESTPROGRAMS ${tests}
#  LINKLIBS ${target} ${DEPENDENCY_LIBRARIES}
#  )
