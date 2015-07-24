set(CXX_SOURCES)
auto_sources(files "*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}")
list(APPEND CXX_SOURCES ${files})

if(NOT IS_PPC64)
  add_definitions("-DUSE_SIMULATOR")
endif()

add_library(ppc64-asm STATIC ${CXX_SOURCES})

auto_sources(files "*.h" "${CMAKE_CURRENT_SOURCE_DIR}")
HHVM_PUBLIC_HEADERS(ppc64-asm ${files})