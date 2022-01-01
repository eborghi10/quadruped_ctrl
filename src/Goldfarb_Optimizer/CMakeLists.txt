FILE(GLOB_RECURSE headers *.hh)
FILE(GLOB_RECURSE sources *.cc)

set(CMAKE_CXX_FLAGS "-O3 -no-pie -ggdb ")  # no werror here!

add_library (Goldfarb_Optimizer SHARED ${headers} ${sources})
