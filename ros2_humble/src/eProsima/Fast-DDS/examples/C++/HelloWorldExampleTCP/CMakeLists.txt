# Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.16.3)

project("HelloWorldExampleTCP" VERSION 1)

# Find requirements
if(NOT fastcdr_FOUND)
    find_package(fastcdr REQUIRED)
endif()

if(NOT foonathan_memory_FOUND)
    find_package(foonathan_memory REQUIRED)
endif()

if(NOT fastrtps_FOUND)
    find_package(fastrtps REQUIRED)
endif()

#Check C++11
include(CheckCXXCompilerFlag)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR
        CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    check_cxx_compiler_flag(-std=c++11 SUPPORTS_CXX11)
    if(NOT SUPPORTS_CXX11)
        message(FATAL_ERROR "Compiler doesn't support C++11")
    endif()
endif()

message(STATUS "Configuring HelloWorldTCP example...")
file(GLOB HELLOWORLD_EXAMPLE_SOURCES_CXX "*.cxx")
file(GLOB HELLOWORLD_EXAMPLE_SOURCES_CPP "*.cpp")
#file(GLOB XML_CONFIG_FILES "*.xml")
#message(STATUS "XML Files: " ${XML_CONFIG_FILES})
configure_file("HelloWorldSubscriber.xml" "HelloWorldSubscriber.xml" COPYONLY)
configure_file("HelloWorldPublisher.xml" "HelloWorldPublisher.xml" COPYONLY)
configure_file("dh2048.pem" "dh2048.pem" COPYONLY)
configure_file("server.pem" "server.pem" COPYONLY)
configure_file("ca.pem" "ca.pem" COPYONLY)


add_executable(HelloWorldExampleTCP ${HELLOWORLD_EXAMPLE_SOURCES_CXX} ${HELLOWORLD_EXAMPLE_SOURCES_CPP})
target_compile_definitions(HelloWorldExampleTCP PRIVATE
    $<$<AND:$<NOT:$<BOOL:${WIN32}>>,$<STREQUAL:"${CMAKE_BUILD_TYPE}","Debug">>:__DEBUG>
    $<$<BOOL:${INTERNAL_DEBUG}>:__INTERNALDEBUG> # Internal debug activated.
)
target_link_libraries(HelloWorldExampleTCP fastrtps fastcdr foonathan_memory fastdds::optionparser)
install(TARGETS HelloWorldExampleTCP
    RUNTIME DESTINATION examples/C++/HelloWorldExampleTCP/${BIN_INSTALL_DIR})
