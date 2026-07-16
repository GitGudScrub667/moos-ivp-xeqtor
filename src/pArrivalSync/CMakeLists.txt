#--------------------------------------------------------
# The CMakeLists.txt for:                       pArrivalSync
#--------------------------------------------------------

SET(SRC
  ArrivalSync.cpp
  ArrivalSync_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pArrivalSync ${SRC})

TARGET_LINK_LIBRARIES(pArrivalSync
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   m
   pthread)
