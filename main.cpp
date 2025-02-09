#include "sensormove.h"

int main()
{
    SensorMove move("config.json");
    move.run_simulation();
    // move.runMeasurementCycle();

    return 0;
}
