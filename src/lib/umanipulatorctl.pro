#-------------------------------------------------
#
# Project created by QtCreator 2012-01-22T17:49:48
#
#-------------------------------------------------

QT       -= core gui

TARGET = umanipulatorctl
TEMPLATE = lib
CONFIG += staticlib

DEFINES += UMANIPULATORCTL_LIBRARY

SOURCES += umanipulatorctl.c \
    crc.c \
    extserialport.c

HEADERS += umanipulatorctl.h \
    common.h \
    crc.h \
    extserialport.h

windows: {
    LIBS += -shared
    addFiles.sources = umanipulatorctl.dll
    DEPLOYMENT += addFiles
}

unix:!symbian {
    maemo5 {
        target.path = /opt/usr/lib
    } else {
        target.path = /usr/local/lib
    }
    INSTALLS += target
}

mac: {
    DEFINES += WITHOUT_HI_SERIAL_PORT_SPEEDS
}
