#include "visual.h"
#include "imu.h"
#include "imu_1750.h"

int main()
{
    imu_1750 imu("/dev/ttyUSB0");
    visualization vi;
    while (true)
    {
        vi.set_imu(imu.get_imu_data().front());
    }
}

