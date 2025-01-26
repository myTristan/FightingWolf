
#include "datadefine.hpp"
 

//Main

SimulatorNode simu;
GUI_Set_S guiSet;


int main(int argc, char *argv[])
{
    ros::init (argc, argv, "cav_simulation_node");
    ros::NodeHandle nh;

    ros::NodeHandle priv_nh("~");
    priv_nh.getParam("x_0", guiSet.x_0);
    priv_nh.getParam("y_0", guiSet.y_0);
    priv_nh.getParam("heading_0", guiSet.heading_0);
    priv_nh.getParam("vehicle_type", guiSet.vehicle_type);

    printf("\n\n");
    printf("  getParam x_0 %f\n", guiSet.x_0);
    printf("  getParam y_0 %f\n", guiSet.y_0);
    printf("  getParam heading_0 %f\n", guiSet.heading_0);
    printf("  getParam vehicle_type %d\n", guiSet.vehicle_type);

    //recv
    simu.sub_cmd = nh.subscribe("/vehicle/control2bywire", 2, cmdCallback);

    //pub
    simu.pub_veh_state   = nh.advertise<cav_msgs::VehicleState>("/vehicle/vehicle_state", 2);
 
    //ini & loop
    ros::Rate loop_rate(FREQ); //50HZ
    simu.ini();
    while (ros::ok())
    {
        ros::spinOnce();
        simu.loop();
        loop_rate.sleep();        
    }

    ros::shutdown();
    return 0;
}
