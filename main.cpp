#include "sensormove.h"

int main()
{
    SensorMove move;
    move.readConfig("config.json");
    move.run_simulation();
    // move.runMeasurementCycle();

    return 0;
}
