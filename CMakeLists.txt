cmake_minimum_required(VERSION 3.15)

include(proj.conf)

find_package(bouffalo_sdk REQUIRED HINTS $ENV{BL_SDK_BASE})


#递归目录所有头文件
macro(CUSTOM_RECURSION_DIR root_dir)
    #判断是否为目录，如果是目录设置包含目录 
    if (IS_DIRECTORY ${root_dir})
       #MESSAGE( STATUS "这是个目录 = ${root_dir}")
       sdk_add_include_directories(${root_dir})
    endif()    
    #枚举目录的下所有文件或目录
    file(GLOB SUBFILES ${root_dir}/*)
    #遄历目录和文件
    foreach(sub ${SUBFILES})
        #如果是目录递归读取目录文件 
        if (IS_DIRECTORY ${sub})
            CUSTOM_RECURSION_DIR(${sub})
        else()
            #判断文件是否为C文件
            string(FIND ${sub} ".c" res)
            if(${res} GREATER_EQUAL 1)
                #MESSAGE( STATUS "这是个C文件 = ${sub}")
                target_sources(app PRIVATE ${sub}) 
            endif()
            #判断文件是否为C++文件
            string(FIND ${sub} ".cpp" res)
            if(${res} GREATER_EQUAL 1)
                #MESSAGE( STATUS "这是个CPP文件 = ${sub}")
                target_sources(app PRIVATE ${sub}) 
            endif()
        endif()
    endforeach()
endmacro()

CUSTOM_RECURSION_DIR(config)
CUSTOM_RECURSION_DIR(src)

add_subdirectory(lib/MPU6050)











sdk_set_main_file(main.c)

project(MPU6050)
