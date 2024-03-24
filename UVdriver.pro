QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

win32{
INCLUDEPATH += C:\Boost\include\boost-1_83
LIBS += -LC:\Boost\lib  -llibboost_system-vc143-mt-gd-x64-1_83 -llibboost_serialization-vc143-mt-gd-x64-1_83 -llibboost_thread-vc143-mt-gd-x64-1_83
}

unix {
INCLUDEPATH += /home/nvidia/boost_1_83_0
LIBS += -L/home/nvidia/boost_1_83_0/stage/lib -lboost_thread -lboost_serialization -lboost_system -lpthread
}

DESTDIR = D:\QT\prjdir\OSGIMODULE\my_simulation_dir\build
# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
 connector_base.cpp\
 gpcsnode.cpp\
 mainnode.cpp\
 Publisher.cpp\
 session.cpp

HEADERS += \
 ALLconfig.h\
 can_msg_define.h\
 can_msg_name.h\
 connector_base.h\
 gpcsmat.h\
 gpcsnode.h\
 Publisher.h\
 session.h\
 state_cmd_struct.h\
 Subscriber.h
