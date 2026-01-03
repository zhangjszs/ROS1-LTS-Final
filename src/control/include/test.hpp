#include "control.h"
namespace control
{
    class Test : public Car
    {
    public:
        Test()
        {
            pub_cmd = nh.advertise<common_msgs::HUAT_VehcileCmd>("vehcileCMDMsg", 1000);
        }

        int get_steering() override
        {
            if (now > 350)
                return 110;
            else
                return 110 + 40 * (sin((now % 60) * 6.0 * pi / 180.0));
        }

        int get_pedalling() override
        {
            if (now > 350)
                return 0;
            else
                return 10;
        }

        int get_braking() override
        {
            if (now > 350)
                return 80;
            else
                return 0;
        }

        int get_status() override
        {
            if (now > 350)
                return 4;
            else
                return 2;
        }
    };
}
