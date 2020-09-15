#-------------------------------------------------
#
# Project created by QtCreator 2012-10-17T18:13:35
#
#-------------------------------------------------

QT       =

TARGET   = umlabvif
TEMPLATE = lib

DEFINES += UMLABVIF_LIBRARY
DEFINES += UMANIPULATORCTLSHARED_DO_NOT_EXPORT

SOURCES += umlabvif.c \
    ../lib/umanipulatorctl.c \
    ../lib/extserialport.c \
    ../lib/crc.c

HEADERS += umlabvif.h \
    ../lib/umanipulatorctl.h \
    ../lib/extserialport.h \
    ../lib/crc.h \
    ../lib/common.h

INCLUDEPATH += ../lib

# XXX debug features
DEFINES += DEBUG

windows: {
	LIBS += -shared
    TARGET = umlabvif
    addFiles.sources = umlabvif
	DEPLOYMENT += addFiles
}

mac: {
    DEFINES += WITHOUT_HI_SERIAL_PORT_SPEEDS
}

