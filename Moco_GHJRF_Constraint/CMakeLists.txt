cmake_minimum_required(VERSION 3.2)
project(OpenSimMoco_MocoGHJRFConstraint)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenSimMoco REQUIRED HINTS
    "${CMAKE_SOURCE_DIR}/../../../../../")
include("${OpenSimMoco_USE_FILE}")

add_library(osimMocoGHJRFConstraint SHARED
        MocoGHJRFConstraint.h
        MocoGHJRFConstraint.cpp
        osimMocoGHJRFConstraintDLL.h
        RegisterTypes_osimMocoGHJRFConstraint.h
        RegisterTypes_osimMocoGHJRFConstraint.cpp
        )
target_link_libraries(osimMocoGHJRFConstraint osimMoco)

string(TOUPPER MocoGHJRFConstraint example_name_upper)
set_target_properties(osimMocoGHJRFConstraint PROPERTIES
        DEFINE_SYMBOL OSIM${example_name_upper}_EXPORTS
        )

# For Windows: make sure DLLs for dependencies are available.
MocoCopyDLLs(DEP_NAME OpenSimMoco DEP_BIN_DIR "${OpenSimMoco_BIN_DIR}")
if(WIN32)
    add_dependencies(osimMocoGHJRFConstraint Copy_OpenSimMoco_DLLs)
endif()

