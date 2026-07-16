# CMake 4.x can retain OpenMPI wrapper flags ("-I/path") as literal include
# directories when PCL pulls in VTK on Ubuntu 22.04.  Supplying the canonical
# OpenMPI include paths avoids generating invalid project-relative paths.
if(CMAKE_VERSION VERSION_GREATER_EQUAL "4.0" AND CMAKE_LIBRARY_ARCHITECTURE)
  set(_m20_openmpi_include "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/openmpi/include")
  if(EXISTS "${_m20_openmpi_include}")
    foreach(_m20_mpi_lang C CXX)
      if(NOT MPI_${_m20_mpi_lang}_COMPILER_INCLUDE_DIRS)
        set(
          MPI_${_m20_mpi_lang}_COMPILER_INCLUDE_DIRS
          "${_m20_openmpi_include};${_m20_openmpi_include}/openmpi"
          CACHE STRING "OpenMPI include directories for CMake 4 compatibility"
          FORCE
        )
      endif()
    endforeach()
  endif()
  unset(_m20_openmpi_include)
  unset(_m20_mpi_lang)
endif()
