#-------------------------------------------------
#
# Project created by QtCreator 2013-05-08T09:34:24
#
#-------------------------------------------------

QT       =

TARGET = umlabvif_sample2
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app

INCLUDEPATH += ..

LIBS += -L.. -lumlabvif

SOURCES += main.c

windows: LIBS += -L../dist -lwinmm

