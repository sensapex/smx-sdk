#-------------------------------------------------
#
# Project created by QtCreator 2012-06-23T09:19:45
#
#-------------------------------------------------

QT =
TARGET = cppsample
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

SOURCES += cppsample.cpp
LIBS += -L../lib -lumanipulatorctl
INCLUDEPATH += ../lib
HEADERS +=  customumctl.h

