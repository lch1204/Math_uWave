QT = core gui charts
QT += network
QT += widgets
TEMPLATE = app

CONFIG += c++17

SOURCES += \
        form.cpp \
        graph.cpp \
        main.cpp \
        Navigation/navigationekf.cpp \
        readerjson.cpp \
        Sensor/sensormove.cpp \
        Sensor/signalpropagation.cpp \
        MathAUV/configdata.cpp \
        MathAUV/kx_protocol.cpp \
        MathAUV/qkx_coeffs.cpp \
        MathAUV/qpiconfig.cpp \
        MathAUV/su_rov.cpp \
        setvariable_t.cpp

HEADERS += \
    Sensor/SensorIMU.h \
    Sensor/SensorPressure.h \
    Sensor/positionanalyzer.h \
    form.h \
    Navigation/navigationekf.h \
    graph.h \
    readerjson.h \
    Sensor/sensormove.h \
    Sensor/signalpropagation.h \
    setvariable_t.h \
    stucturs.h \
    Sensor/sensor_uwave.h \
    MathAUV/configdata.h \
    MathAUV/kx_protocol.h \
    MathAUV/qkx_coeffs.h \
    MathAUV/qpiconfig.h \
    MathAUV/su_rov.h \
    Connection/pc_protocol.h \
    Connection/protocol.h \
    Connection/udp_protocol.h

INCLUDEPATH += $$PWD/include

# Указываем, что бинарник будет собираться в папке config
DESTDIR = $$PWD/config

# Указываем путь установки (если нужно)
target.path = $$PWD/config
INSTALLS += target

FORMS += \
    form.ui \
    graph.ui \
    setvariable_t.ui
