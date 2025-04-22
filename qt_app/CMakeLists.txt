
cmake_minimum_required(VERSION 3.5)

project(Manipulator6DOF)

set(CMAKE_CXX_STANDARD 17)

# Qt setup
find_package(Qt5 COMPONENTS Widgets Charts REQUIRED)

# Dodanie zasobów qrc
qt5_add_resources(QRCS resources.qrc)

# Dodanie plików źródłowych
add_executable(Manipulator6DOF main.cpp ${QRCS})

# Podpięcie bibliotek Qt
target_link_libraries(Manipulator6DOF Qt5::Widgets Qt5::Charts Qt5::Gui)

