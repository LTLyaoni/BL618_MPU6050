sdk_generate_library()

# sdk_library_add_sources(bflb_romfs.c)
# sdk_add_include_directories(.)

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
                sdk_library_add_sources(${sub}) 
            endif()
            #判断文件是否为C++文件
            string(FIND ${sub} ".cpp" res)
            if(${res} GREATER_EQUAL 1)
                #MESSAGE( STATUS "这是个CPP文件 = ${sub}")
                sdk_library_add_sources(${sub}) 
            endif()
        endif()
    endforeach()
endmacro()

CUSTOM_RECURSION_DIR(.)








sdk_add_compile_definitions(-DCONFIG_FREETYPE)