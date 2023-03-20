cmake_minimum_required(VERSION 2.8)
set(DBOW2_TARGET "DBoW2")
project(${DBOW2_TARGET})

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Target definition
set(HDRS_DBOW2
  DBoW2/BowVector.h
  DBoW2/FORB.h
  DBoW2/FClass.h
  DBoW2/FeatureVector.h
  DBoW2/ScoringObject.h
  DBoW2/TemplatedVocabulary.h)
set(SRCS_DBOW2
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
  DUtils/Random.h
  DUtils/Timestamp.h)
set(SRCS_DUTILS
  DUtils/Random.cpp
  DUtils/Timestamp.cpp)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
add_library(DBoW2 SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})

find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 3.0 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 3.0 not found.")
   endif()
endif()
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(${DBOW2_TARGET} PUBLIC ${OpenCV_LIBS})

find_package(Boost REQUIRED COMPONENTS serialization)
target_link_libraries(${DBOW2_TARGET} PRIVATE Boost::serialization)
