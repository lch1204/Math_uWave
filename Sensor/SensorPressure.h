#ifndef SENSORPRESSURE_H
#define SENSORPRESSURE_H

#pragma once
#include <random>
#include <cmath>
#include <vector>

class PressureSensor {
public:
    /**
     * @brief Конструктор датчика давления.
     * @param noise_std Стандартное отклонение шума измерения.
     * @param resolution Разрядность датчика (шаг квантования).
     * @param tau Постоянная времени звена (задержка датчика).
     * @param ma_window_size Размер окна для скользящего среднего фильтра.
     */
    PressureSensor(double measured_depth = 0, double noise_std = 1.5, double resolution = 0.01, double tau = 0.1, int ma_window_size = 5)
        : noise_std(noise_std),
          resolution(resolution),
          tau(tau),
          measured_depth(0.0),
          ma_window_size(ma_window_size),
          gen(rd()),
          dist(0.0, noise_std)
    {}

    /**
     * @brief Обновление измерения датчика давления.
     * @param true_depth Истинная глубина.
     * @param dt Время дискретизации.
     * @return Измеренная глубина с учетом динамики, шума, квантования и скользящего среднего фильтра.
     *
     * Математическая модель:
     *   1) Динамика: y[k+1] = y[k] + (dt/τ)·(true_depth - y[k])
     *   2) Добавление шума: y_noisy = y[k+1] + ε, где ε ~ N(0, noise_std²)
     *   3) Квантование: y_quantized = round(y_noisy/resolution)*resolution
     *   4) Скользящее среднее: y_filtered = (1/N) * Σ (последние N измерений)
     */
    void update(double true_depth, double dt) {
        // 1. Динамика датчика (задержка)
        measured_depth += (dt / tau) * (true_depth - measured_depth);

        // 2. Добавление шума
        double noisy_depth = measured_depth + dist(gen);

        // 3. Квантование результата
        double quantized_depth = quantize(noisy_depth, resolution);

        // 4. Применение скользящего среднего фильтра
        ma_buffer.push_back(quantized_depth);
        if (ma_buffer.size() > static_cast<size_t>(ma_window_size)) {
            // удаляем самый старый элемент
            ma_buffer.erase(ma_buffer.begin());
        }
        double sum = 0.0;
        for (double value : ma_buffer) {
            sum += value;
        }
        filtered_depth = sum / ma_buffer.size();
    }

    // Возвращает структуру с текущими измерениями
    double getOutput() const {
        return filtered_depth;
    }

private:
    double noise_std;      // СКО шума измерения
    double resolution;     // Разрядность датчика (шаг квантования)
    double tau;            // Постоянная времени (задержка датчика)
    double measured_depth; // Внутреннее состояние динамики датчика

    // Параметры скользящего среднего фильтра
    int ma_window_size;           // Размер окна фильтра
    std::vector<double> ma_buffer; // Буфер для хранения последних измерений

    std::random_device rd;
    std::mt19937 gen;
    std::normal_distribution<> dist;

    double filtered_depth; //итоговая глубина

    // Функция квантования: округление до ближайшего кратного resolution
    double quantize(double value, double res) {
        return std::round(value / res) * res;
    }
};

#endif // SENSORPRESSURE_H
