cmake_minimum_required(VERSION 3.16)
project(SCANNER C)

set(CMAKE_C_STANDARD 11)

MACRO(HEADER_DIRECTORIES return_list)
    FILE(GLOB_RECURSE new_list /Users/ivo/ncs/*.h)
    SET(dir_list "")
    FOREACH(file_path ${new_list})
        GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
        SET(dir_list ${dir_list} ${dir_path})
    ENDFOREACH()
    LIST(REMOVE_DUPLICATES dir_list)
    SET(${return_list} ${dir_list})
ENDMACRO()

HEADER_DIRECTORIES(header_dir_list)

list(LENGTH header_dir_list header_dir_list_count)
message(STATUS "[INFO] Found ${header_dir_list_count} header directories.")

include_directories(${header_dir_list} )

add_executable(SCANNER
        pca10056/s140/config/sdk_config.h
        buttons.c
        main.c)
