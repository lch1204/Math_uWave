#include "sensormove.h"

int main()
{
    SensorMove move("config.json");
    move.run_simulation();

    return 0;
}
