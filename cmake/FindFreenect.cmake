
find_path(Freenect_INCLUDE_DIR
  NAMES libfreenect.hpp
)

# Finally the library itself
find_library(Freenect_LIBRARY
  NAMES freenect
)

