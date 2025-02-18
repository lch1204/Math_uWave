QT = core
QT += network

TEMPLATE = app

CONFIG += c++17 cmdline

SOURCES += \
        main.cpp \
        readerjson.cpp \
        sensormove.cpp \
        signalpropagation.cpp \
        MathAUV/configdata.cpp \
        MathAUV/kx_protocol.cpp \
        MathAUV/qkx_coeffs.cpp \
        MathAUV/qpiconfig.cpp \
        MathAUV/su_rov.cpp

HEADERS += \
    Navigation/NavigationFilter.h \
    Navigation/NavigationSystem.h \
    SensorIMU.h \
    SensorPressure.h \
    readerjson.h \
    sensormove.h \
    signalpropagation.h \
    stucturs.h \
    sensor_uwave.h \
    MathAUV/configdata.h \
    MathAUV/kx_protocol.h \
    MathAUV/qkx_coeffs.h \
    MathAUV/qpiconfig.h \
    MathAUV/su_rov.h

INCLUDEPATH += $$PWD/include

# Указываем, что бинарник будет собираться в папке config
DESTDIR = $$PWD/config

# Указываем путь установки (если нужно)
target.path = $$PWD/config
INSTALLS += target
