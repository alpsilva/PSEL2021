# Qt libs to import
QT += core    \
      gui     \
      widgets \
      network \
      opengl

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

# Project libs
LIBS *= -lprotobuf -lQt5Core

# Project configs
TEMPLATE = app
DESTDIR  = ../bin
TARGET   = ProjetoPSEL
VERSION  = 1.0.0

CONFIG += c++14 console
CONFIG -= app_bundle

# Temporary dirs
OBJECTS_DIR = tmp/obj
MOC_DIR = tmp/moc
UI_DIR = tmp/moc
RCC_DIR = tmp/rc

# Compiling .proto files
system(echo "Compiling protobuf files" && cd protobufs/proto && protoc --cpp_out=../ *.proto && cd ../..)

SOURCES += \
        main.cpp \
        modules/actuator/actuator.cpp \
        modules/module.cpp \
        modules/vision/vision.cpp \
        protobufs/command.pb.cc \
        protobufs/common.pb.cc \
        protobufs/packet.pb.cc \
        protobufs/replacement.pb.cc

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

HEADERS += \
    modules/actuator/actuator.h \
    modules/module.h \
    modules/vision/vision.h \
    protobufs/command.pb.h \
    protobufs/common.pb.h \
    protobufs/packet.pb.h \
    protobufs/replacement.pb.h

