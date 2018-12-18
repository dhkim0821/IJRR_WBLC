add_subdirectory(Goldfarb)

if (CMAKE_SYSTEM_NAME MATCHES Linux)
   find_library(gfortran NAMES libgfortran.so PATHS /usr/lib/gcc/x86_64-linux-gnu/4.8/)
   if(gfortran)
	   message("**** SNOPT Note: Found gfortran compiler. SNOPT will be compiled ")
   	   add_subdirectory(snopt)
   else()
       message("**** SNOPT Note: Could not find gfortran compiler. SNOPT will not be compiled")
   endif()
endif (CMAKE_SYSTEM_NAME MATCHES Linux)

