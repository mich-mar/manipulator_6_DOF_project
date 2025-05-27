cmake_minimum_required(VERSION 3.19)
project(manipulator_ui_v3 LANGUAGES CXX)

find_package(Qt6 6.5 REQUIRED COMPONENTS
    Core
    Widgets
    Charts
    LinguistTools
    SerialPort
)

qt_standard_project_setup()

# Zdefiniuj pliki tłumaczeń
set(TS_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/translations/manipulator_en.ts
    ${CMAKE_CURRENT_SOURCE_DIR}/translations/manipulator_pl.ts
)

# Zdefiniuj pliki źródłowe
set(PROJECT_SOURCES
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    serialReader.hpp
    serialReader.cpp
    crc.hpp
    crc.cpp
    sensorData.hpp
    dataQueue.hpp
    dataQueue.cpp
)

# Dodaj cel aktualizacji plików .ts
add_custom_target(update_translations
    COMMAND Qt6::lupdate ${CMAKE_CURRENT_SOURCE_DIR} -ts ${TS_FILES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Updating translations..."
)

# Generuj pliki .qm z .ts
qt_add_translations(${PROJECT_NAME}
    TS_FILES ${TS_FILES}
    RESOURCE_PREFIX "/translations"
)

# Dodaj zasoby do kompilacji
qt_add_resources(RESOURCES resources.qrc)

qt_add_executable(${PROJECT_NAME}
    WIN32 MACOSX_BUNDLE
    ${PROJECT_SOURCES}
    ${RESOURCES}
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Qt::Core
        Qt::Widgets
        Qt::Charts
        Qt::SerialPort
)

# Konfiguracja instalacji
include(GNUInstallDirs)

install(TARGETS ${PROJECT_NAME}
    BUNDLE  DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# Instalacja plików tłumaczeń
install(FILES ${QM_FILES}
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/translations
)

qt_generate_deploy_app_script(
    TARGET ${PROJECT_NAME}
    OUTPUT_SCRIPT deploy_script
    NO_UNSUPPORTED_PLATFORM_ERROR
)
install(SCRIPT ${deploy_script})
