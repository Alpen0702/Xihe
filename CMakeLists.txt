# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.
#
cmake_minimum_required (VERSION 3.16)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

include_directories("F:/MyColds/METIS/include")
# link_directories("F:/UE_5.3/Engine/Source/ThirdParty/metis\5.1.0\libmetis\Release\metis.lib")
link_directories("F:/MyColds/METIS/build/windows/libmetis/Release")
# include_directories("F:/MyColds/METIS/GKlib")
# link_directories("F:/MyColds/METIS/build/windows/GKlib/Release")

add_compile_definitions($<$<CONFIG:DEBUG>:XH_DEBUG>)

# SET(CMAKE_TOOLCHAIN_FILE "E:/vcpkg/scripts/buildsystems/vcpkg.cmake")


project ("Xihe")

message(STATUS "CMAKE_TOOLCHAIN_FILE: ${CMAKE_TOOLCHAIN_FILE}")

# set(TBB_DIR "E:/vcpkg/packages/tbb_x64-windows/share/tbb")
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};E:/vcpkg/packages/tbb_x64-windows/share;E:/vcpkg/packages/tbb_x64-windows/debug/bin")


find_package(TBB CONFIG REQUIRED)


add_subdirectory("third_party")

# Include sub-projects.
add_subdirectory ("xihe")
