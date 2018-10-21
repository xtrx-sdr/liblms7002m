project(liblms7002m C)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
cmake_minimum_required(VERSION 2.8)

# Set the version information here
set(MAJOR_VERSION 0)
set(API_COMPAT    0)
set(MINOR_VERSION 1)
set(MAINT_VERSION git)

set(LIBVER  "${MAJOR_VERSION}.${API_COMPAT}.${MINOR_VERSION}")

# determine target architecture
execute_process(COMMAND ${CMAKE_C_COMPILER} -print-multiarch OUTPUT_VARIABLE CC_ARCH OUTPUT_STRIP_TRAILING_WHITESPACE)
if(CC_ARCH MATCHES "i386")
        message(STATUS "Building for i386")
        set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "i386")
        set(ARCH "x86")
elseif(CC_ARCH MATCHES "x86_64")
        message(STATUS "Building for x86_64")
        set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
        set(ARCH "x86")
elseif(CC_ARCH MATCHES "aarch64")
        message(STATUS "Building for aarch64")
        set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "arm64")
        set(ARCH "arm")
elseif(CC_ARCH MATCHES "arm.*eabihf")
        message(STATUS "Building for armhf")
        set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "armhf")
        set(ARCH "arm")
else()
        message(WARNING "Unknown target architecture ${CC_ARCH}, forcing to FORCE_ARCH=${FORCE_ARCH} variable if it was provided")
        set(ARCH ${FORCE_ARCH})
endif()

set(CROSS_COMPILE_INCLUDE_PATH "/usr/${CC_ARCH}/include")
set(CROSS_COMPILE_LIB_PATH     "/usr/${CC_ARCH}/lib")


set(LIBLMS7002M_LIBRARY_DIR      lib${LIB_SUFFIX})
set(LIBLMS7002M_INCLUDE_DIR      include)

find_package(PythonInterp REQUIRED)
execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "from Cheetah.Template import Template" RESULT_VARIABLE retcode ERROR_QUIET OUTPUT_QUIET)
if(NOT "${retcode}" STREQUAL "0")
    message(FATAL_ERROR "Python module Cheetah is reuired! Install 'python-cheetah' package.")
endif()

file(GLOB LMS7002M_REG_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/regs/*.json")

add_custom_command(
    OUTPUT  ${CMAKE_CURRENT_BINARY_DIR}/lms7002m_defs.h
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/regs/enum_parser.py ${LMS7002M_REG_SOURCES} > ${CMAKE_CURRENT_BINARY_DIR}/lms7002m_defs.h
    DEPENDS ${LMS7002M_REG_SOURCES}
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(LIBLMS7_FILES liblms7002m.c ${CMAKE_CURRENT_BINARY_DIR}/lms7002m_defs.h)
if(CMAKE_SYSTEM_NAME MATCHES "Windows")
    set(LIBLMS7_TYPE STATIC)
else()
    set(LIBLMS7_TYPE SHARED)
endif()
add_library(lms7compact ${LIBLMS7_TYPE} ${LIBLMS7_FILES})

install(TARGETS lms7compact DESTINATION ${LIBLMS7002M_LIBRARY_DIR})
########################################################################
# install headers
########################################################################
install(FILES liblms7002m.h DESTINATION ${LIBLMS7002M_INCLUDE_DIR})

########################################################################
# package generator
########################################################################
if(NOT CPACK_GENERATOR)
    set(CPACK_GENERATOR DEB)
endif()
set(CPACK_PACKAGE_NAME                "liblms7002m")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "lms7002m compact library")
set(CPACK_PACKAGE_VENDOR              "Fairwaves, Inc.")
set(CPACK_PACKAGE_CONTACT             "http://fairwaves.co/wp/contact-us/")
set(CPACK_PACKAGE_VERSION             ${LIBVER}-1)
set(CPACK_DEBIAN_PACKAGE_DEPENDS      "libc6 (>= 2.17)")

set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
set(CPACK_SET_DESTDIR               "")
set(CPACK_PACKAGING_INSTALL_PREFIX  "/usr")
set(CPACK_PACKAGE_CONTACT           "Sergey Kostanbaev <sergey.kostanbaev@fairwaves.co>")

include(CPack) #include last
