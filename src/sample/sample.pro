TARGET = sample
QT =
CONFIG   += console
CONFIG   -= app_bundle
TEMPLATE = app
SOURCES += sample.c
LIBS    += -L../lib -lumanipulatorctl
INCLUDEPATH += ../lib
