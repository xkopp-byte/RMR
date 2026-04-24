set(SOURCES
    main.cpp
    mainwindow.cpp
    robot.cpp
    mapping.cpp
    mapvisualizer.cpp
    floodfill.cpp
)

set(HEADERS
    mainwindow.h
    robot.h
    mapping.h
    mapvisualizer.h
    floodfill.h
)

if(NOT DISABLE_JOYSTICK)
    add_compile_definitions(USING_QJOYSTICKS)
endif()

add_executable(${PROJECT_NAME} WIN32
    ${SOURCES}
    ${HEADERS}
    mainwindow.ui
)

target_compile_definitions(${PROJECT_NAME} PRIVATE ROBOT_LIBRARY)
if(DISABLE_OPENCV)
    target_compile_definitions(${PROJECT_NAME} PRIVATE DISABLE_OPENCV)
endif()
if(DISABLE_SKELETON)
    target_compile_definitions(${PROJECT_NAME} PRIVATE DISABLE_SKELETON)
endif()
if(DISABLE_JOYSTICK)
    target_compile_definitions(${PROJECT_NAME} PRIVATE DISABLE_JOYSTICK)
endif()
if(DISABLE_MAPPING)
    target_compile_definitions(${PROJECT_NAME} PRIVATE DISABLE_MAPPING)
endif()

target_link_libraries(${PROJECT_NAME} 
    PRIVATE librobot::librobot
    PRIVATE Qt6::Core Qt6::Widgets Qt6::Gui Qt6::Network
)

if(NOT DISABLE_OPENCV OR NOT DISABLE_MAPPING)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})
endif()

if(NOT DISABLE_JOYSTICK)
    target_link_libraries(${PROJECT_NAME} PRIVATE QJoysticks::QJoysticksLib)
endif()

if(WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE ws2_32 Winmm)
endif()

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/maps 
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})