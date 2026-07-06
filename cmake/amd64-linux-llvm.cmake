# llvm-toolchain.cmake

set(CMAKE_C_COMPILER   clang-20)
set(CMAKE_CXX_COMPILER clang++-20)

add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-Wno-#warnings>")
set(CMAKE_POSITION_INDEPENDENT_CODE ON) # -fPIC
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON) #-fLTO

set(CMAKE_EXE_LINKER_FLAGS_INIT    "-fuse-ld=lld")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "-fuse-ld=lld")
set(CMAKE_MODULE_LINKER_FLAGS_INIT "-fuse-ld=lld")

message(STATUS "LLVM Toolchain Loaded")
