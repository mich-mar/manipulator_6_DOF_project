cmake_minimum_required(VERSION 3.19)
project(manipulator_ui_v3 LANGUAGES CXX)

find_package(Qt6 6.5 REQUIRED COMPONENTS
    Core
    Widgets
    Charts
    LinguistTools
)

qt_standard_project_setup()

# Dodaj zasoby do kompilacji
qt_add_resources(RESOURCES resources.qrc)

qt_add_executable(manipulator_ui_v3
    WIN32 MACOSX_BUNDLE
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    ${RESOURCES}  # Dodaj skompilowane zasoby
    resources.qrc # Dodaj plik zasobów
)

target_link_libraries(manipulator_ui_v3
    PRIVATE
        Qt::Core
        Qt::Widgets
        Qt::Charts
)

# Zdefiniuj pliki tłumaczeń
set(TS_FILES
    translations/manipulator_en.ts
    translations/manipulator_pl.ts
)

# Generuj pliki .ts
qt_create_translation(QM_FILES ${PROJECT_SOURCES} ${TS_FILES})

include(GNUInstallDirs)

install(TARGETS manipulator_ui_v3
    BUNDLE  DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

qt_generate_deploy_app_script(
    TARGET manipulator_ui_v3
    OUTPUT_SCRIPT deploy_script
    NO_UNSUPPORTED_PLATFORM_ERROR
)
install(SCRIPT ${deploy_script})
