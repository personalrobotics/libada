cmake_minimum_required(VERSION 2.8.12)

project(adaIk)

# If enable C++11
# Expecting ambiguous call to isnan()
# which can be resolved by explicitly calling std::isnan(double x) 
add_compile_options(-std=c++11 -fPIC)

#==================================================================
# Option 1: generate a .so library and put to devel/lib

add_library(adaIk SHARED ada_2018_05_07_ik.cpp)
target_compile_definitions(adaIk PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY)
target_compile_options(adaIk PRIVATE -w)

# Copy libadaIk.so to devel/lib
install(TARGETS adaIk
   LIBRARY DESTINATION lib
   ARCHIVE DESTINATION lib
   )

#==================================================================
# Option 2: Create a wrapper class, compile to libadaIkfast,
#			 link libada to this library

#project(libadaIkfast)

#set(sources
#	SharedLibraryAdaIkFast.hpp
#	SharedLibraryAdaIkFast.cpp
#)
#add_library(libadaIkfast SHARED ${sources})
#target_link_libraries(libadaIkfast PUBLIC dart)
#target_compile_definitions(libadaIkfast
#	PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY)
#target_compile_options(libadaIkfast PRIVATE -w)

# Export
#install(TARGETS libadaIkfast
#	EXPORT libadaIkfastTargets
#	LIBRARY DESTINATION lib
#	ARCHIVE DESTINATION lib
#	RUNTIME DESTINATION bin) # for windows

#install(EXPORT libadaIkfastTargets DESTINATION share/libadaIkfast/cmake)

#export(TARGETS libadaIkfast FILE libadaIkfast.cmake)