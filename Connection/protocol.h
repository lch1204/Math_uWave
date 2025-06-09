#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <QObject>

#pragma pack(push, 1) // Выравнивание по 1 байту

// Заголовок пакета
struct PacketHeader {
    quint8 sync_byte = 0xAA;   // Синхробайт
    quint8 packet_id;          // Идентификатор пакета (инкрементируется)
    quint8 packet_type = 0x01;  // Тип пакета (0x01 - настройки от ПУ)
};

// Основные данные пакета (настройки)
struct ControlSettings {
    struct Sensor {
        float positions[2][3];   // Позиции датчиков [x, y, z][]
        float noise_mean;        // Средний шум
        float noise_stddev;     // СКО шума
    } sensor;

    float sound_speed;           // Скорость звука
    float signal_loss_prob;      // Вероятность потери сигнала
    quint8 signal_loss_enabled;    // Флаг активации потерь
    float measurement_delay;     // Задержка измерения
    quint8 delay_enabled;          // Флаг активации задержки
    float timeout;               // Таймаут
    quint8 startAlgorithm;

    struct Initiator {
        quint8 is_mobile;          // Подвижность инициатора
        float velocity[3];       // Скорость [vx, vy, vz]
    } initiator;

    struct Simulation {
        float time_step;         // Шаг симуляции
        float duration;         // Длительность
        quint8 realtime_mode;     // Режим реального времени
    } simulation;

    struct AquariumWalls {
        quint8 enabled;            // Включение стен
        float left_x, right_x;   // Границы по X
        float front_y, back_y;  // Границы по Y
        float bottom_z, top_z;   // Границы по Z
        float reflection_loss;  // Потери при отражении
    } walls;

    struct Aquarium {
        float depth;             // Глубина
        float surface_z;         // Уровень поверхности
        quint8 bottom_type[16];    // Тип дна (песок, ил и т.д.)
    } aquarium;
};

// Полный пакет (заголовок + данные + CRC)
struct FromPult {
//    PacketHeader header;
    ControlSettings data;
    uint checksum;               // Контрольная сумма (CRC-16)
};


// Заголовок пакета (аналогичный)
struct TelemetryHeader {
    quint8 sync_byte = 0x55;   // Синхробайт (отличается от ПУ!)
    quint8 packet_id = 0;
    quint8 packet_type = 0x02; // Тип пакета (0x02 - телеметрия)
};

// Данные для графиков (координаты + EKF)
struct GraphData {
    double x;
    double ekf_x;            // Реальная и EKF-оценка X
    double y;
    double ekf_y;            // Реальная и EKF-оценка Y
    double z;
    double ekf_z;            // Реальная и EKF-оценка Z
    double timestamp;           // Время симуляции
};

// Данные для карты (положение + векторы)
struct MapData {
    double real_x;
    double real_y;      // Реальные координаты
    double ekf_x;
    double ekf_y;        // EKF-координаты
    double circle_radius;       // Радиус окружности (если нужен)
    double ekf_vx;
    double ekf_vy;      // EKF-скорость
    double real_vx;
    double real_vy;    // Реальная скорость
};

// Объединение для удобства
struct TelemetryPayload {
    GraphData graph;
    MapData map;
    bool start= false;
};


// Полный пакет телеметрии
struct ToPult {
//    TelemetryHeader header;
    TelemetryPayload payload;
    uint checksum;
};
#pragma pack(pop)

#endif // PROTOCOL_H
