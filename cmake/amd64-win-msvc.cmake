set(CMAKE_C_COMPILER cl)
set(CMAKE_CXX_COMPILER cl)

message(STATUS "MSVC TOOLCHAIN LOADED")

add_compile_options(
    /utf-8
    /permissive-
    /EHsc
    /Zc:__cplusplus
    /Zc:preprocessor
    /diagnostics:caret  # 诊断信息格式
	/sdl-
	/GS-
	/Gy- # 禁用 GOMDAT 
	# 处理一些警告
    /W4
	/wd5285
)

add_link_options(
    /INCREMENTAL:NO     # 关闭 ilk
	/DEBUG:NONE
)

add_compile_definitions(
    NOMINMAX
    WIN32_LEAN_AND_MEAN
    _CRT_SECURE_NO_WARNINGS
    _SCL_SECURE_NO_WARNINGS
    UNICODE
    _UNICODE
)

add_compile_definitions(_WIN32_WINNT=0x0A01) #win11

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON) # dll export all symbols
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL") # /MD
