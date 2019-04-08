# Copyright 2019 Collabora, Ltd.
# Distributed under the Boost Software License, Version 1.0.
# (See accompanying file LICENSE_1_0.txt or copy at
# http://www.boost.org/LICENSE_1_0.txt)
#
# Original Author:
# 2019 Ryan Pavlik <ryan.pavlik@collabora.com>

#.rst:
# FindCheck
# ---------------
#
# Find the "Check" C unit testing framework.
#
# See https://libcheck.github.io
#
# The Debian package for this is called ``check``
#
# Targets
# ^^^^^^^
#
# If successful, the following imported targets are created.
#
# ``libcheck::check``
#
# ``libcheck::subunit``
#
# Cache variables
# ^^^^^^^^^^^^^^^
#
# The following cache variable may also be set to assist/control the operation of this module:
#
# ``LIBCHECK_ROOT_DIR``
#  The root to search for libcheck.

set(LIBCHECK_ROOT_DIR "${LIBCHECK_ROOT_DIR}" CACHE PATH "Root to search for libcheck")

find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(PC_LIBCHECK QUIET check)
endif()
find_path(LIBCHECK_INCLUDE_DIR
    NAMES
    check.h
    PATHS
    ${LIBCHECK_ROOT_DIR}
    HINTS
    ${PC_LIBCHECK_INCLUDE_DIRS}
    PATH_SUFFIXES
    include
)
find_library(LIBCHECK_LIBRARY
    NAMES
    check_pic
    check
    PATHS
    ${LIBCHECK_ROOT_DIR}
    HINTS
    ${PC_LIBCHECK_LIBRARY_DIRS}
    PATH_SUFFIXES
    lib
)
find_library(LIBCHECK_SUBUNIT_LIBRARY
    NAMES
    subunit
    PATHS
    ${LIBCHECK_ROOT_DIR}
    HINTS
    ${PC_LIBCHECK_LIBRARY_DIRS}
    PATH_SUFFIXES
    lib
)
find_library(LIBCHECK_LIBRT rt)
find_library(LIBCHECK_LIBM m)

# include(CMakeFindDependencyMacro)
# find_dependency(Threads)
find_package(Threads QUIET)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Libcheck
    REQUIRED_VARS
    LIBCHECK_INCLUDE_DIR
    LIBCHECK_LIBRARY
    LIBCHECK_SUBUNIT_LIBRARY
    THREADS_FOUND
)
if(LIBCHECK_FOUND)
    if(NOT TARGET libcheck::subunit)
        add_library(libcheck::subunit UNKNOWN IMPORTED)
        set_target_properties(libcheck::subunit PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${LIBCHECK_INCLUDE_DIR}")
        set_target_properties(libcheck::subunit PROPERTIES
            IMPORTED_LINK_INTERFACE_LANGUAGES "C"
            IMPORTED_LOCATION ${LIBCHECK_SUBUNIT_LIBRARY})
    endif()
    if(NOT TARGET libcheck::check)
        add_library(libcheck::check UNKNOWN IMPORTED)

        set_target_properties(libcheck::check PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${LIBCHECK_INCLUDE_DIR}")
        set_target_properties(libcheck::check PROPERTIES
            IMPORTED_LINK_INTERFACE_LANGUAGES "C"
            IMPORTED_LOCATION ${LIBCHECK_LIBRARY})
        set_property(TARGET libcheck::check PROPERTY
                IMPORTED_LINK_INTERFACE_LIBRARIES libcheck::subunit Threads::Threads)

        # if we found librt or libm, link them.
        if(LIBCHECK_LIBRT)
            set_property(TARGET libcheck::check APPEND PROPERTY
                IMPORTED_LINK_INTERFACE_LIBRARIES ${LIBCHECK_LIBRT})
        endif()
        if(LIBCHECK_LIBM)
            set_property(TARGET libcheck::check APPEND PROPERTY
                IMPORTED_LINK_INTERFACE_LIBRARIES ${LIBCHECK_LIBM})
        endif()

    endif()
    mark_as_advanced(LIBCHECK_INCLUDE_DIR LIBCHECK_LIBRARY LIBCHECK_SUBUNIT_LIBRARY)
endif()
mark_as_advanced(LIBCHECK_ROOT_DIR LIBCHECK_LIBRT LIBCHECK_LIBM)