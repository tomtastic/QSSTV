cmake_minimum_required(VERSION 3.16)

project(qsstv VERSION 10.0.0 LANGUAGES C CXX OBJC)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Find Qt6 packages
find_package(Qt6 REQUIRED COMPONENTS
    Core
    Gui
    Widgets
    Network
    Xml
    SvgWidgets
)

# Find required libraries using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFTW3 REQUIRED fftw3 fftw3f)
pkg_check_modules(OPENJP2 REQUIRED libopenjp2)
pkg_check_modules(HAMLIB REQUIRED hamlib)

# Platform-specific libraries
if(APPLE)
    find_library(AUDIOTOOLBOX_FRAMEWORK AudioToolbox REQUIRED)
    find_library(COREAUDIO_FRAMEWORK CoreAudio REQUIRED)
    find_library(AVFOUNDATION_FRAMEWORK AVFoundation REQUIRED)
    pkg_check_modules(PULSEAUDIO REQUIRED libpulse libpulse-simple)
else()
    pkg_check_modules(PULSEAUDIO REQUIRED libpulse libpulse-simple)
    pkg_check_modules(ALSA REQUIRED alsa)
    pkg_check_modules(V4L2 REQUIRED libv4l2 libv4lconvert)
endif()

# Add subdirectories
add_subdirectory(src)
