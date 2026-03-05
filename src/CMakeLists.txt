# Source files
set(SOURCES
    main.cpp
    mainwindow.cpp
    appglobal.cpp
    config/baseconfig.cpp
    config/soundconfig.cpp
    config/directoriesconfig.cpp
    config/configdialog.cpp
    config/operatorconfig.cpp
    config/guiconfig.cpp
    config/rigconfig.cpp
    config/cwconfig.cpp
    config/drmprofileconfig.cpp
    config/ftpconfig.cpp
    config/hybridconfig.cpp
    config/repeaterconfig.cpp
    config/waterfallconfig.cpp
    config/frequencyselectwidget.cpp
    dispatch/dispatcher.cpp
    drmrx/bits2bytes.cpp
    drmrx/channeldecode.cpp
    drmrx/crc16_bytewise.cpp
    drmrx/crc8_c.cpp
    drmrx/deinterleaver.cpp
    drmrx/demodulator.cpp
    drmrx/drm.cpp
    drmrx/drmconstellationframe.cpp
    drmrx/drmpsdframe.cpp
    drmrx/drmrx.cpp
    drmrx/drmstatusframe.cpp
    drmrx/filter1.cpp
    drmrx/filter1c.cpp
    drmrx/fixform.cpp
    drmrx/getfoffsint.cpp
    drmrx/getmode.cpp
    drmrx/getofdm.cpp
    drmrx/getofdmsync.cpp
    drmrx/getsymbolidx.cpp
    drmrx/lubksb.cpp
    drmrx/ludcmp.cpp
    drmrx/mkfacmap.cpp
    drmrx/mkmscmap.cpp
    drmrx/msdhardfac.cpp
    drmrx/msdhardmsc.cpp
    drmrx/newfft.cpp
    drmrx/nrutil.cpp
    drmrx/psdcmean.cpp
    drmrx/psdmean.cpp
    drmrx/sourcedecoder.cpp
    drmrx/viterbi_decode.cpp
    drmtx/common/CDrmTransmitter.cpp
    drmtx/common/DRMSignalIO.cpp
    drmtx/common/DataIO.cpp
    drmtx/common/OFDM.cpp
    drmtx/common/Parameter.cpp
    drmtx/common/csoundout.cpp
    drmtx/common/datadecoding/DABMOT.cpp
    drmtx/common/datadecoding/DataDecoder.cpp
    drmtx/common/datadecoding/MOTSlideShow.cpp
    drmtx/common/FAC/FAC.cpp
    drmtx/common/interleaver/BlockInterleaver.cpp
    drmtx/common/interleaver/SymbolInterleaver.cpp
    drmtx/common/matlib/MatlibSigProToolbox.cpp
    drmtx/common/matlib/MatlibStdToolbox.cpp
    drmtx/common/mlc/BitInterleaver.cpp
    drmtx/common/mlc/ChannelCode.cpp
    drmtx/common/mlc/ConvEncoder.cpp
    drmtx/common/mlc/EnergyDispersal.cpp
    drmtx/common/mlc/MLC.cpp
    drmtx/common/mlc/QAMMapping.cpp
    drmtx/common/ofdmcellmapping/CellMappingTable.cpp
    drmtx/common/ofdmcellmapping/OFDMCellMapping.cpp
    drmtx/common/SDC/SDCTransmit.cpp
    drmtx/common/sourcedecoders/AudioSourceDecoder.cpp
    drmtx/common/tables/TableFAC.cpp
    drmtx/common/util/CRC.cpp
    drmtx/common/util/Utilities.cpp
    drmtx/bsrform.cpp
    drmtx/drmparams.cpp
    drmtx/drmtransmitter.cpp
    drmtx/drmtx.cpp
    dsp/downsamplefilter.cpp
    dsp/synthes.cpp
    dsp/filterparam.cpp
    dsp/filters.cpp
    dsp/filter.cpp
    editor/editor.cpp
    editor/editorscene.cpp
    editor/editorview.cpp
    editor/gradientdialog.cpp
    editor/graphicitems.cpp
    editor/canvassizeform.cpp
    editor/basegraphicitem.cpp
    editor/templateviewer.cpp
    logbook/logbook.cpp
    mainwidgets/gallerywidget.cpp
    mainwidgets/rxfunctions.cpp
    mainwidgets/rxwidget.cpp
    mainwidgets/txfunctions.cpp
    mainwidgets/txwidget.cpp
    rig/freqdisplay.cpp
    rig/rigcontrol.cpp
    sound/soundbase.cpp
    sound/wavio.cpp
    sound/calibration.cpp
    sound/waterfalltext.cpp
    sound/resamplefilter.cpp
    sstv/modes/modeavt.cpp
    sstv/modes/modebase.cpp
    sstv/modes/modebw.cpp
    sstv/modes/modegbr.cpp
    sstv/modes/modegbr2.cpp
    sstv/modes/modepd.cpp
    sstv/modes/modergb.cpp
    sstv/modes/moderobot1.cpp
    sstv/modes/moderobot2.cpp
    sstv/cw.cpp
    sstv/sstvparam.cpp
    sstv/sstvrx.cpp
    sstv/sstvtx.cpp
    sstv/syncprocessor.cpp
    sstv/visfskid.cpp
    utils/dirdialog.cpp
    utils/logging.cpp
    utils/loggingparams.cpp
    utils/supportfunctions.cpp
    utils/arraydumper.cpp
    utils/macroexpansion.cpp
    utils/qurlinfo.cpp
    utils/reedsolomoncoder.cpp
    utils/rs.cpp
    utils/qftp.cpp
    utils/hybridcrypt.cpp
    utils/hexconvertor.cpp
    utils/fftcalc.cpp
    utils/jp2io.cpp
    utils/color.cpp
    utils/filewatcher.cpp
    utils/ftpthread.cpp
    utils/ftpfunctions.cpp
    widgets/spectrumwidget.cpp
    widgets/vumeter.cpp
    widgets/fftdisplay.cpp
    widgets/markerwidget.cpp
    widgets/imageviewer.cpp
    widgets/textdisplay.cpp
    widgets/extviewer.cpp
    widgets/imagematrix.cpp
    widgets/waterfallform.cpp
    widgets/drmsegmentsview.cpp
    widgets/xyzplaintextedit.cpp
    widgets/testpatternselection.cpp
    xmlrpc/ipcmessage.cpp
    xmlrpc/maiaFault.cpp
    xmlrpc/maiaObject.cpp
    xmlrpc/maiaXmlRpcClient.cpp
    xmlrpc/maiaXmlRpcServer.cpp
    xmlrpc/maiaXmlRpcServerConnection.cpp
    xmlrpc/xmlinterface.cpp
)

# Platform-specific sources
if(NOT APPLE)
    list(APPEND SOURCES
        sound/soundalsa.cpp
        sound/soundpulse.cpp
        videocapt/cameradialog.cpp
        videocapt/imagesettings.cpp
        videocapt/v4l2control.cpp
        videocapt/videocapture.cpp
    )
else()
    list(APPEND SOURCES
        macos/soundmacos.cpp
        macos/CaptureDeviceAuthorization.m
    )
    # Compile Objective-C file as Objective-C++
    set_source_files_properties(macos/CaptureDeviceAuthorization.m PROPERTIES
        LANGUAGE CXX
        COMPILE_FLAGS "-x objective-c++"
    )
endif()

# Debug-only sources
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    list(APPEND SOURCES
        scope/scopeoffset.cpp
        scope/scopeview.cpp
        scope/scopeplot.cpp
    )
endif()

# UI files
set(UI_FILES
    mainwindow.ui
    config/guiconfig.ui
    config/soundconfig.ui
    config/directoriesconfig.ui
    config/configdialog.ui
    config/operatorconfig.ui
    config/rigconfig.ui
    config/cwconfig.ui
    config/drmprofileconfig.ui
    config/ftpconfig.ui
    config/hybridconfig.ui
    config/repeaterconfig.ui
    config/waterfallconfig.ui
    config/frequencyselectwidget.ui
    drmrx/drmconstellationframe.ui
    drmrx/drmpsdframe.ui
    drmrx/drmstatusframe.ui
    drmrx/fixform.ui
    drmtx/bsrform.ui
    editor/editorform.ui
    editor/gradientform.ui
    editor/textform.ui
    editor/canvassizeform.ui
    editor/templateviewer.ui
    mainwidgets/gallerywidget.ui
    mainwidgets/rxwidget.ui
    mainwidgets/txwidget.ui
    rig/freqdisplay.ui
    utils/loggingform.ui
    widgets/spectrumwidget.ui
    widgets/textdisplay.ui
    widgets/extviewer.ui
    widgets/sweepform.ui
    widgets/waterfallform.ui
    widgets/freqform.ui
    widgets/drmsegmentsview.ui
    widgets/testpatternselection.ui
    sound/calibration.ui
)

# Debug-only UI files
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    list(APPEND UI_FILES
        scope/scopeoffset.ui
        scope/plotform.ui
    )
endif()

# Resource files
set(RESOURCES
    qsstv.qrc
)

# Create executable
add_executable(qsstv
    ${SOURCES}
    ${UI_FILES}
    ${RESOURCES}
)

# Include directories
target_include_directories(qsstv PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    config
    dispatch
    drmrx
    drmtx
    dsp
    editor
    logbook
    mainwidgets
    rig
    scope
    sound
    sstv
    utils
    widgets
    xmlrpc
    videocapt
)

# Set AUTOUIC search paths
set_target_properties(qsstv PROPERTIES
    AUTOUIC_SEARCH_PATHS "${CMAKE_CURRENT_SOURCE_DIR};${CMAKE_CURRENT_SOURCE_DIR}/widgets;${CMAKE_CURRENT_SOURCE_DIR}/config;${CMAKE_CURRENT_SOURCE_DIR}/editor;${CMAKE_CURRENT_SOURCE_DIR}/mainwidgets;${CMAKE_CURRENT_SOURCE_DIR}/drmrx;${CMAKE_CURRENT_SOURCE_DIR}/drmtx;${CMAKE_CURRENT_SOURCE_DIR}/rig;${CMAKE_CURRENT_SOURCE_DIR}/sound;${CMAKE_CURRENT_SOURCE_DIR}/utils;${CMAKE_CURRENT_SOURCE_DIR}/scope"
)

# Link Qt libraries
target_link_libraries(qsstv PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::Network
    Qt6::Xml
    Qt6::SvgWidgets
)

# Link external libraries
target_link_libraries(qsstv PRIVATE
    ${FFTW3_LIBRARIES}
    ${OPENJP2_LIBRARIES}
    ${HAMLIB_LIBRARIES}
)

# Platform-specific linking
if(APPLE)
    target_link_libraries(qsstv PRIVATE
        ${AUDIOTOOLBOX_FRAMEWORK}
        ${COREAUDIO_FRAMEWORK}
        ${AVFOUNDATION_FRAMEWORK}
        ${PULSEAUDIO_LIBRARIES}
    )
    # Set macOS bundle properties
    set_target_properties(qsstv PROPERTIES
        MACOSX_BUNDLE TRUE
        MACOSX_BUNDLE_INFO_PLIST ${CMAKE_CURRENT_SOURCE_DIR}/Info.plist.in
        MACOSX_BUNDLE_ICON_FILE qsstv.icns
    )
    # Add icon to bundle resources
    set_source_files_properties(qsstv.icns PROPERTIES
        MACOSX_PACKAGE_LOCATION Resources
    )
    target_sources(qsstv PRIVATE qsstv.icns)
else()
    target_link_libraries(qsstv PRIVATE
        ${PULSEAUDIO_LIBRARIES}
        ${ALSA_LIBRARIES}
        ${V4L2_LIBRARIES}
        rt
    )
endif()

# Include directories for external libraries
target_include_directories(qsstv PRIVATE
    ${FFTW3_INCLUDE_DIRS}
    ${OPENJP2_INCLUDE_DIRS}
    ${HAMLIB_INCLUDE_DIRS}
    ${PULSEAUDIO_INCLUDE_DIRS}
)

if(NOT APPLE)
    target_include_directories(qsstv PRIVATE
        ${ALSA_INCLUDE_DIRS}
        ${V4L2_INCLUDE_DIRS}
    )
endif()

# Compiler flags
target_compile_options(qsstv PRIVATE
    -Wno-vla-cxx-extension
    -Wno-implicit-fallthrough
    -Wno-psabi
    $<$<CONFIG:Debug>:-O0>
    $<$<CONFIG:Release>:-O3>
)

# Definitions
target_compile_definitions(qsstv PRIVATE
    QT_DEPRECATED_WARNINGS
)

# Installation
include(GNUInstallDirs)

install(TARGETS qsstv
    BUNDLE DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(FILES qsstv.desktop
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/applications
)

install(FILES icons/qsstv.png
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/icons/hicolor/128x128/apps
)
