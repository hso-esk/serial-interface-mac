##
## @code
##  ___ _____ _   ___ _  _____ ___  ___  ___ ___
## / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
## \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
## |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
## embedded.connectivity.solutions.==============
## @endcode
##
## @file
## @copyright  STACKFORCE GmbH, Heitersheim, Germany, http://www.stackforce.de
## @author     STACKFORCE
## @brief      STACKFORCE Serial MAC Module
##
## This file is part of the STACKFORCE Serial MAC Library
## (below "libserialmac").
##
## libserialmac is free software: you can redistribute it and/or modify
## it under the terms of the GNU Affero General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## libserialmac is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU Affero General Public License for more details.
##
## You should have received a copy of the GNU Affero General Public License
## along with libserialmac.  If not, see <http://www.gnu.org/licenses/>.
##
cmake_minimum_required (VERSION 2.8)
project (serialmac)


#---------------------------------------------------------------------------------------
# PRODUCT AND VERSION
#---------------------------------------------------------------------------------------
set (SERIALMAC_VERSION_MAJOR 0 CACHE STRING "major version")
set (SERIALMAC_VERSION_MINOR 1 CACHE STRING "minor version")
set (SERIALMAC_VERSION_PATCH 2 CACHE STRING "patch version")
set (SERIALMAC_PRODUCT_COMPANY "STACKFORCE" CACHE STRING "company")
set (SERIALMAC_PRODUCT_NAME    "Serial MAC" CACHE STRING "product")
set (SERIALMAC_VERSION ${SERIALMAC_VERSION_MAJOR}.${SERIALMAC_VERSION_MINOR}.${SERIALMAC_VERSION_PATCH} )

# configure a header file to pass some of the CMake settings
# to the source code
configure_file(
    ${PROJECT_SOURCE_DIR}/version.h.in
    ${PROJECT_BINARY_DIR}/version.h
    )

#---------------------------------------------------------------------------------------
# TARGET
#---------------------------------------------------------------------------------------
set (SERIALMAC_SOURCES
     ${PROJECT_SOURCE_DIR}/sf_serialmac.c
     )

# Build static libraries only
option(BUILD_SHARED_LIBS "Set to ON to build shared libraries" OFF)

add_library(serialmac ${SERIALMAC_SOURCES})

target_include_directories(serialmac PUBLIC ${PROJECT_SOURCE_DIR}
                                            ${PROJECT_BINARY_DIR})

# NOTE target_include_directories(PUBLIC) is not enough to get Kdevelop
# to find the includes, hence the are defined again with INTERFACE_INCLUDE_DIRECTORIES
set_target_properties(serialmac PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES ${PROJECT_SOURCE_DIR}
    VERSION  ${SERIALMAC_VERSION}
)

#---------------------------------------------------------------------------------------
# CRC
#---------------------------------------------------------------------------------------
add_subdirectory(${PROJECT_SOURCE_DIR}/dep/crc crc)

#---------------------------------------------------------------------------------------
# DOXYGEN
#---------------------------------------------------------------------------------------
#add a target to generate API documentation with Doxygen
find_package(Doxygen)
option(BUILD_DOCUMENTATION "Create and install the HTML based API documentation (requires Doxygen)" ${DOXYGEN_FOUND})

if(BUILD_DOCUMENTATION)
    if(NOT DOXYGEN_FOUND)
        message(FATAL_ERROR "Doxygen is needed to build the documentation.")
    endif()

    get_target_property( DOXYGEN_INPUT_SERIALMAC serialmac INCLUDE_DIRECTORIES)
    get_target_property( DOXYGEN_INPUT_CRC crc INCLUDE_DIRECTORIES)
    set( DOXYGEN_INPUT ${DOXYGEN_INPUT_SERIALMAC}
                       ${DOXYGEN_INPUT_CRC})

    string(CONCAT DOXYFILE_INPUT ${DOXYFILE_INPUT} " "
        ${PROJECT_SOURCE_DIR}/README.md)
    foreach(loop_var IN LISTS DOXYGEN_INPUT)
        string(CONCAT DOXYFILE_INPUT ${DOXYFILE_INPUT} " " ${loop_var})
    endforeach(loop_var)

    message(STATUS "doxyfile input: ${DOXYFILE_INPUT}")

    configure_file(${PROJECT_SOURCE_DIR}/doc/Doxyfile.in
        ${PROJECT_BINARY_DIR}/Doxyfile @ONLY)

    add_custom_target(doc
        ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen" VERBATIM
        SOURCES ${PROJECT_BINARY_DIR}/Doxyfile
        )
    install(DIRECTORY ${PROJECT_BINARY_DIR}/doc/html DESTINATION share/doc)
endif()

#---------------------------------------------------------------------------------------
# LINKING
#---------------------------------------------------------------------------------------
target_link_libraries(serialmac crc)

#---------------------------------------------------------------------------------------
# INSTALLATION
#---------------------------------------------------------------------------------------
install(TARGETS serialmac DESTINATION lib)
install(FILES ${PROJECT_SOURCE_DIR}/sf_serialmac.h DESTINATION include)

