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
## @author     Lars Moellendorf
## @author     Adrian Antonana
## @author     Johannes Bruder
## @brief      STACKFORCE Serial MAC Module
##
## This file is part of the STACKFORCE Serial MAC Library
## (below "libsfserialmac").
##
## libsfserialmac is free software: you can redistribute it and/or modify
## it under the terms of the GNU Affero General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## libsfserialmac is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU Affero General Public License for more details.
##
## You should have received a copy of the GNU Affero General Public License
## along with libsfserialmac.  If not, see <http://www.gnu.org/licenses/>.
##

cmake_minimum_required (VERSION 3.6)
project(sfserialmac)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

#---------------------------------------------------------------------------------------
# Find required packages
#---------------------------------------------------------------------------------------
find_package(sfcrc 0.1.2 REQUIRED)
get_target_property(CRC_INCLUDE_DIRS sfcrc INTERFACE_INCLUDE_DIRECTORIES)

#---------------------------------------------------------------------------------------
# Options
#---------------------------------------------------------------------------------------
option(BUILD_DOCUMENTATION "Create and install the HTML based API documentation" ${DOXYGEN_FOUND})

#---------------------------------------------------------------------------------------
# Product and version
#---------------------------------------------------------------------------------------
set (SERIALMAC_VERSION_MAJOR 0 CACHE STRING "major version")
set (SERIALMAC_VERSION_MINOR 1 CACHE STRING "minor version")
set (SERIALMAC_VERSION_PATCH 2 CACHE STRING "patch version")
set (SERIALMAC_PRODUCT_COMPANY "STACKFORCE" CACHE STRING "company")
set (SERIALMAC_PRODUCT_NAME    "Serial MAC" CACHE STRING "product")
set (SERIALMAC_VERSION ${SERIALMAC_VERSION_MAJOR}.${SERIALMAC_VERSION_MINOR}.${SERIALMAC_VERSION_PATCH})

#---------------------------------------------------------------------------------------
# Serialmac headers and sources
#---------------------------------------------------------------------------------------
set(SERIALMAC_SOURCES
    ${PROJECT_SOURCE_DIR}/sf_serialmac.c
)

# configure version header
configure_file(
    ${PROJECT_SOURCE_DIR}/version.h.in
    ${PROJECT_BINARY_DIR}/version.h
)

#---------------------------------------------------------------------------------------
# Serialmac targets
#---------------------------------------------------------------------------------------
add_library(sfserialmac_o OBJECT ${SERIALMAC_SOURCES})
add_library(sfserialmac SHARED $<TARGET_OBJECTS:sfserialmac_o>)
add_library(sfserialmac-static STATIC $<TARGET_OBJECTS:sfserialmac_o>)

target_include_directories(sfserialmac_o PRIVATE ${CRC_INCLUDE_DIRS})

set_target_properties(sfserialmac_o PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
set_target_properties(sfserialmac PROPERTIES VERSION ${SERIALMAC_VERSION})
set_target_properties(sfserialmac-static PROPERTIES VERSION ${SERIALMAC_VERSION})
set_target_properties(sfserialmac PROPERTIES OUTPUT_NAME sfserialmac)
set_target_properties(sfserialmac-static PROPERTIES OUTPUT_NAME sfserialmac)

target_include_directories(sfserialmac INTERFACE
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/sfserialmac>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
)

target_include_directories(sfserialmac-static INTERFACE
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/sfserialmac>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
)

target_link_libraries(sfserialmac-static PUBLIC sfcrc-static)
target_link_libraries(sfserialmac PUBLIC sfcrc)

#---------------------------------------------------------------------------------------
# DOXYGEN
#---------------------------------------------------------------------------------------
#add a target to generate API documentation with Doxygen

if(BUILD_DOCUMENTATION)
    find_package(Doxygen 1.8.13 REQUIRED)

    # variables for use by generic doxygen configuration file
    set(DOXYGEN_PRODUCT_NAME "${SERIALMAC_PRODUCT_NAME}")
    set(DOXYGEN_PROJECT_NUMBER "${SERIALMAC_VERSION}")
    set(DOXYGEN_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/doc)
    set(DOXYGEN_CUSTOM_RESOURCES_DIR ${PROJECT_BINARY_DIR}/doxygen)

    string(APPEND DOXYGEN_INPUT ${PROJECT_SOURCE_DIR}/README.md " ")
    string(APPEND DOXYGEN_INPUT ${PROJECT_SOURCE_DIR} " ")
    string(APPEND DOXYGEN_INPUT ${PROJECT_BINARY_DIR})

    message(STATUS "doxyfile input: ${DOXYFILE_INPUT}")

    set(SF_CORPORATE_DESIGN_URL "INSERT_HERE_THE_URL")
    message(STATUS "Download files for STACKFORCE corporate design:")
    file(DOWNLOAD
        "${SF_CORPORATE_DESIGN_URL}/bootstrap.min.css"
        ${PROJECT_BINARY_DIR}/bootstrap.min.css
        SHOW_PROGRESS
        )
    file(DOWNLOAD
        "${SF_CORPORATE_DESIGN_URL}/sf_stylesheet.css"
        ${PROJECT_BINARY_DIR}/sf_stylesheet.css
        SHOW_PROGRESS
        )
    file(DOWNLOAD
        "${SF_CORPORATE_DESIGN_URL}/sf_footer.html"
        ${PROJECT_BINARY_DIR}/sf_footer.html
        SHOW_PROGRESS
        )
    file(DOWNLOAD
        "${SF_CORPORATE_DESIGN_URL}/sf_header.html"
        ${PROJECT_BINARY_DIR}/sf_header.html
        SHOW_PROGRESS
        )
    file(DOWNLOAD
        "${SF_CORPORATE_DESIGN_URL}/stackforce_logo.svg"
        ${PROJECT_BINARY_DIR}/stackforce_logo.svg
        SHOW_PROGRESS
        )
    configure_file(${PROJECT_SOURCE_DIR}/doc/Doxyfile.in
        ${PROJECT_BINARY_DIR}/Doxyfile @ONLY)

    add_custom_target(doc ALL
        ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen" VERBATIM
        SOURCES ${PROJECT_BINARY_DIR}/Doxyfile
    )

    install(DIRECTORY ${PROJECT_BINARY_DIR}/doc/html DESTINATION ${CMAKE_INSTALL_DATADIR}/doc/${PROJECT_NAME})
endif()

#---------------------------------------------------------------------------------------
# Install/export targets and files
#---------------------------------------------------------------------------------------
set(CONFIG_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")
set(PROJECT_VERSION_FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake")
set(PROJECT_CONFIG_FILE "${PROJECT_NAME}-config.cmake")
set(TARGETS_EXPORT_NAME "${PROJECT_NAME}Targets")

# generate package version file
write_basic_package_version_file(
    ${PROJECT_VERSION_FILE}
    VERSION ${SERIALMAC_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# install targets
install(TARGETS sfserialmac sfserialmac-static
        EXPORT ${TARGETS_EXPORT_NAME}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# install headers
install(FILES ${PROJECT_SOURCE_DIR}/sf_serialmac.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

# install version file
install(FILES ${PROJECT_VERSION_FILE}
        DESTINATION ${CONFIG_INSTALL_DIR}
)

# build directory package config
export(EXPORT ${TARGETS_EXPORT_NAME}
       FILE ${PROJECT_CONFIG_FILE}
)

# installed package config
install(EXPORT ${TARGETS_EXPORT_NAME}
        DESTINATION ${CONFIG_INSTALL_DIR}
        FILE ${PROJECT_CONFIG_FILE}
)

# add to CMake registry
export(PACKAGE ${PROJECT_NAME})

#---------------------------------------------------------------------------------------
# Create debian package
#---------------------------------------------------------------------------------------
set(CPACK_GENERATOR "DEB")

# check if command dpkg is available
find_program(DPKG_CMD dpkg)
if(DPKG_CMD)
    message(STATUS "DEB Generator: Found dpkg: ${DPKG_CMD}")
else()
    message(FATAL_ERROR "DEB Generator: dpkg not found")
endif()

set(CPACK_PACKAGE_DIRECTORY ${CMAKE_BINARY_DIR}/packages)
message(STATUS "DEB Generator: debian packages will be generated under ${CPACK_PACKAGE_DIRECTORY}")
set(CPACK_DEBIAN_PACKAGE_NAME "lib${PROJECT_NAME}")
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "STACKFORCE serialmac")
set(CPACK_PACKAGE_VENDOR "STACKFORCE GmbH")
set(CPACK_PACKAGE_CONTACT "info@stackforce.de")
set(CPACK_DEBIAN_PACKAGE_VERSION ${SERIALMAC_VERSION})
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_STRIP_FILES TRUE)
set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/LICENSE")

include(CPack)