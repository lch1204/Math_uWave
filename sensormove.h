#ifndef SENSORMOVE_H
#define SENSORMOVE_H

#include "readerjson.h"
#include "sensor_uwave.h"

using namespace std;

class SensorMove
{
    ReaderJson config;
    vector<Sensor_uWave> sensors;
    vector<Obstacle> obstacles;
    double sim_time = 0.0;

public:
    SensorMove(const string& config_file);

    void run_simulation();

private:
    void process_delays(const vector<double>& delays, size_t sensor_id);
};

#endif // SENSORMOVE_H
