cmake_minimum_required(VERSION 3.8)
project(px4sitl VERSION 0.1.0 LANGUAGES CXX)

find_package(Python COMPONENTS Interpreter REQUIRED)

execute_process(COMMAND ${Python_EXECUTABLE} -m pip install kconfiglib jinja2 pyros-genmsg jsonschema)

set(CONFIG px4_sitl_default CACHE STRING "" FORCE)
add_subdirectory(../PX4-Autopilot/PX4-Autopilot ${CMAKE_CURRENT_BINARY_DIR}/PX4-Autopilot)

install(PROGRAMS
    scripts/px4sitl.sh
    scripts/qgroundcontrol.sh
    DESTINATION bin
)

foreach(DIR cdinit_services)
    install(DIRECTORY ${DIR}
        DESTINATION share/${PROJECT_NAME}/
        USE_SOURCE_PERMISSIONS
    )
endforeach()
