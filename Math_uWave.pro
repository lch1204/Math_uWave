QT = core gui charts
QT += network
QT += widgets
TEMPLATE = app

CONFIG += c++17 cmdline

SOURCES += \
        form.cpp \
        main.cpp \
        Navigation/navigationekf.cpp \
        readerjson.cpp \
        Sensor/sensormove.cpp \
        Sensor/signalpropagation.cpp \
        MathAUV/configdata.cpp \
        MathAUV/kx_protocol.cpp \
        MathAUV/qkx_coeffs.cpp \
        MathAUV/qpiconfig.cpp \
        MathAUV/su_rov.cpp

HEADERS += \
    Sensor/SensorIMU.h \
    Sensor/SensorPressure.h \
    form.h \
    Navigation/navigationekf.h \
    readerjson.h \
    Sensor/sensormove.h \
    Sensor/signalpropagation.h \
    stucturs.h \
    Sensor/sensor_uwave.h \
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

FORMS += \
    form.ui
