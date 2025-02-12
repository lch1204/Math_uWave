QT = core

CONFIG += c++17 cmdline

SOURCES += \
        main.cpp \
        readerjson.cpp \
        sensormove.cpp \
        signalpropagation.cpp \

HEADERS += \
    readerjson.h \
    sensormove.h \
    signalpropagation.h \
    stucturs.h \
    sensor_uwave.h

INCLUDEPATH += $$PWD/include

# Указываем, что бинарник будет собираться в папке config
DESTDIR = $$PWD/config

# Указываем путь установки (если нужно)
target.path = $$PWD/config
INSTALLS += target
