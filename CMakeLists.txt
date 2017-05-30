
cmake_minimum_required(VERSION 2.8.11)
project(VDDR)

add_definitions(-std=c++11)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

# Find the QtWidgets library
find_package(Qt5 COMPONENTS Core Widgets gui  PrintSupport REQUIRED)
find_package(Qt5PrintSupport REQUIRED)
qt5_add_resources (RESOURCE_FILES vddr.qrc )


# Find CGAL and CGAL Qt5
find_package(CGAL COMPONENTS Core Qt5 OpenGL)
include( ${CGAL_USE_FILE} )


# include helper file
include( ${CGAL_USE_FILE} )


# Include directories
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/CGAL ${PROJECT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${Qt5Widgets_INCLUDE_DIRS} ${Qt5PrintSupport_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

file(GLOB INCLUDE_LIST ${PROJECT_SOURCE_DIR}/include/*.h ${PROJECT_SOURCE_DIR}/CGAL/*.h)
aux_source_directory(${PROJECT_SOURCE_DIR}/src              SRC_LIST)


if(CGAL_FOUND AND CGAL_Qt5_FOUND AND Qt5_FOUND AND OPENGL_FOUND)

    set(CMAKE_AUTOUIC ON)

    SET(vddr_SOURCES vddr.cpp vddrApplicationWindow.cpp)
    SET(vddr_HEADERS vddrApplicationWindow.h)


    add_executable  (VDDR vddr.cpp ${UI_FILES} ${RESOURCE_FILES} ${INCLUDE_LIST} ${SRC_LIST} ${vddr_HEADERS_MOC} ${vddr_SOURCES})
    add_executable  (VDDRCommand vddr_cmdline.cpp   ${INCLUDE_LIST} ${SRC_LIST})
    add_executable  (VDDRCommand2 vddr_cmdline2.cpp   ${INCLUDE_LIST} ${SRC_LIST})


    # Link with Qt libraries
    #target_link_libraries( VDDR Qt5::PrintSupport Qt5::Widgets)

    # Link with CGAL
    target_link_libraries( VDDR ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} Qt5::PrintSupport Qt5::Widgets )
    target_link_libraries( VDDRCommand ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} Qt5::PrintSupport Qt5::Widgets )
    target_link_libraries( VDDRCommand2 ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} Qt5::PrintSupport Qt5::Widgets )

    add_to_cached_list( CGAL_EXECUTABLE_TARGETS VDDR )

else (CGAL_FOUND AND CGAL_CGAL_Qt5_FOUND AND QT5_FOUND AND OPENGL_FOUND)

  set(AABB_MISSING_DEPS "")

  if(NOT CGAL_FOUND)
    set(AABB_MISSING_DEPS "the CGAL library, ${AABB_MISSING_DEPS}")
  endif()

  if(NOT CGAL_Qt5_FOUND)
    set(AABB_MISSING_DEPS "the CGAL Qt5 library, ${AABB_MISSING_DEPS}")
  endif()

  if(NOT Qt5_FOUND)
    set(AABB_MISSING_DEPS "Qt5, ${AABB_MISSING_DEPS}")
  endif()

  if(NOT OPENGL_FOUND)
    set(AABB_MISSING_DEPS "OpenGL, ${AABB_MISSING_DEPS}")
  endif()


  message(STATUS "NOTICE: This demo requires ${AABB_MISSING_DEPS} and will not be compiled.")

endif (CGAL_FOUND AND CGAL_Qt5_FOUND AND Qt5_FOUND AND OPENGL_FOUND)
