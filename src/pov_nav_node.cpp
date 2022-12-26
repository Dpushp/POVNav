#include <pov_nav.h>

/* DyParams */
double kp, kp1, kp2, kd, kd1, kd2, w1, w2;

void callback(pov_nav::pov_nav_dyparamConfig &config, uint32_t level) {
    kp = config.Kp;
    kp1 = config.Kp1;
    kp2 = config.Kp2;
    kd = config.Kd;
    kd1 = config.Kd1;
    kd2 = config.Kd2;
    w1 = config.w1;
    w2 = config.w2;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pov_nav");

    dynamic_reconfigure::Server<pov_nav::pov_nav_dyparamConfig> server;
    dynamic_reconfigure::Server<pov_nav::pov_nav_dyparamConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::NodeHandle nh;
    POVNav local_navigator(nh);

    while(ros::ok()) {
        local_navigator.updateConfig(kp, kd, kp1, kd1, kp2, kd2, w1, w2);
        ros::spinOnce();
    }

    return 0;
}
