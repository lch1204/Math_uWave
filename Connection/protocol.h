#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <QObject>

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;

#pragma pack(push, 1) // Выравнивание по 1 байту

// Заголовок пакета
struct PacketHeader {
    uint8_t sync_byte = 0xAA;   // Синхробайт
    uint16_t packet_id;          // Идентификатор пакета (инкрементируется)
    uint8_t packet_type = 0x01;  // Тип пакета (0x01 - настройки от ПУ)
};

// Основные данные пакета (настройки)
struct ControlSettings {
    ControlSettings() {memset(this, 0, sizeof(*this));}
    struct Sensor {
        float positions[2][3];   // Позиции датчиков [x, y, z][]
        float noise_mean;        // Средний шум
        float noise_stddev;     // СКО шума
    } sensor;

    float sound_speed;           // Скорость звука
    float signal_loss_prob;      // Вероятность потери сигнала
    bool signal_loss_enabled;    // Флаг активации потерь
    float measurement_delay;     // Задержка измерения
    bool delay_enabled;          // Флаг активации задержки
    float timeout;               // Таймаут
    bool startAlgorithm;

    struct Initiator {
        bool is_mobile;          // Подвижность инициатора
        float velocity[3];       // Скорость [vx, vy, vz]
    } initiator;

    struct Simulation {
        float time_step;         // Шаг симуляции
        float duration;         // Длительность
        bool realtime_mode;     // Режим реального времени
    } simulation;

    struct AquariumWalls {
        bool enabled;            // Включение стен
        float left_x, right_x;   // Границы по X
        float front_y, back_y;  // Границы по Y
        float bottom_z, top_z;   // Границы по Z
        float reflection_loss;  // Потери при отражении
    } walls;

    struct Aquarium {
        float depth;             // Глубина
        float surface_z;         // Уровень поверхности
        char bottom_type[16];    // Тип дна (песок, ил и т.д.)
    } aquarium;
};

// Полный пакет (заголовок + данные + CRC)
struct FromPult {
    PacketHeader header;
    ControlSettings data;
    uint checksum;               // Контрольная сумма (CRC-16)
};

#pragma pack(pop)

#pragma pack(push, 1)

// Заголовок пакета (аналогичный)
struct TelemetryHeader {
    uint8_t sync_byte = 0x55;   // Синхробайт (отличается от ПУ!)
    uint16_t packet_id = 0;
    uint8_t packet_type = 0x02; // Тип пакета (0x02 - телеметрия)
};

// Данные для графиков (координаты + EKF)
struct GraphData {
    GraphData() {memset(this, 0, sizeof(*this));}
    double x, ekf_x;            // Реальная и EKF-оценка X
    double y, ekf_y;            // Реальная и EKF-оценка Y
    double z, ekf_z;            // Реальная и EKF-оценка Z
    double timestamp;           // Время симуляции
};

// Данные для карты (положение + векторы)
struct MapData {
    MapData() {memset(this, 0, sizeof(*this));}
    double real_x, real_y;      // Реальные координаты
    double ekf_x, ekf_y;        // EKF-координаты
    double circle_radius;       // Радиус окружности (если нужен)
    double ekf_vx, ekf_vy;      // EKF-скорость
    double real_vx, real_vy;    // Реальная скорость
};

// Объединение для удобства
union TelemetryPayload {
    TelemetryPayload() {memset(this, 0, sizeof(*this));}
    GraphData graph;
    MapData map;
};

// Полный пакет телеметрии
struct ToPult {
    TelemetryHeader header;
    TelemetryPayload payload;
    uint checksum;
};

#pragma pack(pop)

#endif // PROTOCOL_H
