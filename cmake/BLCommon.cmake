


macro(FIND_SOURCES_DIR SOURCES_LIST curdir)									#定义函数,2个参数:存放结果SOURCES_LIST；指定路径curdir；

    # #判断是否为目录，如果是目录设置包含目录 
    # if (IS_DIRECTORY ${curdir})
    #     MESSAGE( STATUS "这是个目录 = ${curdir}")
    #  #    sdk_add_include_directories(${root_dir})
    # endif()   


	# file(GLOB_RECURSE children "${curdir}/*.hpp" "${curdir}/*.h" )		#遍历获取{curdir}中*.hpp和*.h文件列表
    file(GLOB_RECURSE children "${curdir}/*.cpp" "${curdir}/*.c" )		#遍历获取{curdir}中*.hpp和*.h文件列表
	set(dirlist "")														#定义dirlist中间变量，并初始化

    set(sources_list "")	

	foreach(child ${children})											#for循环
		# string(REGEX REPLACE "(.*)/.*" "\\1" LIB_NAME ${child})			#字符串替换,用/前的字符替换/*h
        string(REPLACE "${CMAKE_CURRENT_SOURCE_DIR}/" "" sources ${child})

        MESSAGE( STATUS "源文件  = ${sources}")
        # MESSAGE( STATUS "信息 = ${child}")

        LIST(APPEND sources_list ${sources})	    

		# if(IS_DIRECTORY ${LIB_NAME})									#判断是否为路径
		# 	list (FIND dirlist ${LIB_NAME} list_index)					#去重，查找dirlist中是否有${LIB_NAME}指定的值，可以区分字符串相同数子后缀不同的路径：例如/app/test_1和/app/test_2
		# 	if(${list_index} LESS 0)									#若没找到则代表列表中没有该路径
		# 		LIST(APPEND dirlist ${LIB_NAME})						#将合法的路径加入dirlist变量中  
		# 	endif()														#结束判断
		# endif()															
	endforeach()														#结束for循环
	set(${SOURCES_LIST} ${sources_list})											#dirlist结果放入result变量中
endmacro()

macro(FIND_INCLUDE_DIR result curdir)									#定义函数,2个参数:存放结果result；指定路径curdir；
	file(GLOB_RECURSE children "${curdir}/*.hpp" "${curdir}/*.h" )		#遍历获取{curdir}中*.hpp和*.h文件列表
	set(dirlist "")														#定义dirlist中间变量，并初始化
	foreach(child ${children})											#for循环
		string(REGEX REPLACE "(.*)/.*" "\\1" LIB_NAME ${child})			#字符串替换,用/前的字符替换/*h
		if(IS_DIRECTORY ${LIB_NAME})									#判断是否为路径
			list (FIND dirlist ${LIB_NAME} list_index)					#去重，查找dirlist中是否有${LIB_NAME}指定的值，可以区分字符串相同数子后缀不同的路径：例如/app/test_1和/app/test_2
			if(${list_index} LESS 0)									#若没找到则代表列表中没有该路径
				LIST(APPEND dirlist ${LIB_NAME})						#将合法的路径加入dirlist变量中  
			endif()														#结束判断
		endif()															
	endforeach()														#结束for循环
	set(${result} ${dirlist})											#dirlist结果放入result变量中
endmacro()
