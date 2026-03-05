<div align="center">

# QSSTV
<img width="490" height="220" alt="image" src="https://github.com/user-attachments/assets/8ea96de1-317b-4227-8a27-93bab6ea3459" />

[![build-on-macOS-26](https://github.com/tomtastic/QSSTV/actions/workflows/macos-26.yml/badge.svg)](https://github.com/tomtastic/QSSTV/actions/workflows/macos-26.yml)

QSSTV is a program for receiving and transmitting SSTV and HAMDRM (sometimes called DSSTV).

It is compatible with most of MMSSTV and EasyPal

**NB: Video capture requires V4L, which is Linux-specific so excluded on MacOS**

This project is actively being modernized to Qt 6 and modern C++ standards.
</div>

---

## Installation

### Dependencies 

For apt based distros you can install dependencies as follows:

```
apt install pkg-config g++ libfftw3-dev qt6-base-dev qt6-svg-dev libhamlib++-dev libasound2-dev libpulse-dev libopenjp2-7 libopenjp2-7-dev libv4l-dev build-essential
```

For MacOS:
```
brew install fftw hamlib openjpeg qt pulseaudio
```
**Note:** You must have PulseAudio running for sound to work:
```bash
brew services start pulseaudio
```

### Compile and Install

For Linux:
```
	mkdir build
	cd build
	qmake ..
	make -j2
	sudo make install
```

For MacOS:
```
	mkdir build
	cd build
	qmake ../qsstv.pro
	make -j8
	sudo make install
    open /usr/local/bin/qsstv.app
```

Note: make -j2, 2 is the number of cores to be used for parallel compiling. If you have more cores, use a higher number.

### Debug Compile
If you have problems compiling the software, please give as much information as possible but at least:
* Linux Distribution and Version (e.g. Ubuntu 24.04)
* QT Version (e.g. Qt 6.x)
* Screen dump of the compile process showing the error

If you want to be able to debug the program, the simplest way is to install QtCreator and from within QtCreator open a new project and point to the qsstv.pro file. Note: you will need to install doxygen and libqwt

`sudo apt-get install doxygen libqwt-qt5-dev`

Note: libqwt may need to be built from source for Qt 6 compatibility.

You can also run qmake with the following attributes:

`qmake CONFIG+=debug`

and use an external debugger (such as gdb)

## Copying
QSSTV uses Qt see http://qt.digia.com/ if any restrictions apply.

As far as I know, QSSTV does not use any none-public software.
If there are doubts, please let me know by email: on4qz@telenet.be

If part of whole of this code is used, you have to include a statement to indicate that the program is based in whole or in part on QSSTV.
