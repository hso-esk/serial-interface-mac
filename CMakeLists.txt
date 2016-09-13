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
# TARGET
#---------------------------------------------------------------------------------------
set (SERIALMAC_SOURCES
     ${PROJECT_SOURCE_DIR}/sf_serialmac.c
     )

if(STATIC_BUILD)
	add_library(serialmac STATIC ${SERIALMAC_SOURCES})
else()
	add_library(serialmac SHARED ${SERIALMAC_SOURCES})
endif()

target_include_directories(serialmac PUBLIC ${PROJECT_SOURCE_DIR})
# NOTE target_include_directories(PUBLIC) is not enough to get Kdevelop
# to find the includes, hence the are defined again with INTERFACE_INCLUDE_DIRECTORIES
set_target_properties(serialmac PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES ${PROJECT_SOURCE_DIR}
)


#---------------------------------------------------------------------------------------
# PRODUCT AND VERSION
#---------------------------------------------------------------------------------------
set (SERIALMAC_VERSION_MAJOR 0 CACHE STRING "major version")
set (SERIALMAC_VERSION_MINOR 1 CACHE STRING "minor version")
set (SERIALMAC_VERSION_PATCH 1 CACHE STRING "patch version")
set (SERIALMAC_PRODUCT_COMPANY "STACKFORCE" CACHE STRING "company")
set (SERIALMAC_PRODUCT_NAME    "Serial MAC" CACHE STRING "product")
# configure a header file to pass some of the CMake settings
# to the source code
configure_file(
    ${PROJECT_SOURCE_DIR}/version.h.in
    ${PROJECT_BINARY_DIR}/version.h
    )
target_include_directories(serialmac PUBLIC ${PROJECT_BINARY_DIR})

#---------------------------------------------------------------------------------------
# DOXYGEN
#---------------------------------------------------------------------------------------
# add a target to generate API documentation with Doxygen
#find_package(Doxygen)
#
#if(DOXYGEN_FOUND)
#    set (SERIALMAC_HEADERS
#         ${PROJECT_SOURCE_DIR}/sf_serialmac.h
#         )
#    string(CONCAT DOXYFILE_INPUT ${DOXYFILE_INPUT} " "
#        ${PROJECT_SOURCE_DIR}/../README.md)
#    foreach(loop_var IN LISTS SERIALMAC_HEADERS)
#        string(CONCAT DOXYFILE_INPUT ${DOXYFILE_INPUT} " " ${loop_var})
#    endforeach(loop_var)
#
#    MESSAGE(STATUS "doxyfile input: ${DOXYFILE_INPUT}")
#
#    # TODO: move doxyfiles below lib or this config to root dir
#    configure_file(${PROJECT_SOURCE_DIR}/../doc/Doxyfile.in
#        ${PROJECT_BINARY_DIR}/Doxyfile @ONLY)
#
#    add_custom_target(doc
#        ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/Doxyfile
#        WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
#        COMMENT "Generating API documentation with Doxygen" VERBATIM
#        SOURCES ${PROJECT_BINARY_DIR}/Doxyfile
#        )
#else()
#    add_custom_target(doc SOURCES)
#endif(DOXYGEN_FOUND)


#---------------------------------------------------------------------------------------
# CRC
#---------------------------------------------------------------------------------------
add_subdirectory(${PROJECT_SOURCE_DIR}/dep/crc crc)

#---------------------------------------------------------------------------------------
# LINKING
#---------------------------------------------------------------------------------------
target_link_libraries(serialmac crc)

#---------------------------------------------------------------------------------------
# INSTALLATION
#---------------------------------------------------------------------------------------
install (TARGETS serialmac DESTINATION lib)
install (FILES ${PROJECT_SOURCE_DIR}/sf_serialmac.h DESTINATION include)

