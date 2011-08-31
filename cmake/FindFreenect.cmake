
find_path(Freenect_INCLUDE_DIR
  NAMES libfreenect.hpp
)

# Finally the library itself
find_library(Freenect_LIBRARY
  NAMES freenect libfreenect
  PATHS /usr/local/lib64
)

set(Freenect_INCLUDE_DIR Freenect_INCLUDE_DIR ${Freenect_INCLUDE_DIR}/libfreenect)
