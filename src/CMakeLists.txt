
FILE(GLOB SRCS *.c)

ADD_EXECUTABLE(kinect_guard ${SRCS})

TARGET_LINK_LIBRARIES(kinect_guard
  ${ZeroMQ_LIBRARY}
  ${Freenect_LIBRARY}
)

