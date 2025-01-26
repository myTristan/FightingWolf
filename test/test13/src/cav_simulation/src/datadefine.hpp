
#ifndef DATADEFINE_HPP
#define DATADEFINE_HPP

#include <iostream>
#include "ros/ros.h"

#include "cav_msgs/VehicleState.h"
#include "cav_msgs/Control.h"

#include "UTM.h"

#define  FREQ        (50)

namespace XM
{


    inline double Normalise_PI(double angle)
    {
        while (angle>PI)
            angle -= 2.0*PI;

        while (angle<=-PI)
            angle += 2.0*PI;

        return angle;
    }


    inline double Normalise_2PI(double angle)
    {
        while (angle >= 2.0*PI)
            angle -= 2.0*PI;

        while (angle < 0.0f)
            angle += 2.0*PI;

        return angle;
    }

}


//data models are for GUI
struct GUI_Set_S
{
    int     vehicle_type = 1;
    float   x_0 = 0;
    float   y_0 = 0;
    float   heading_0 = 0;
};


struct  VehState_Sim_S
{
    double x = 0;
    double y = 0;
    double z = 0;
    double heading = 0;
    double yawRate = 0;
    double speed_x = 0;
    double speed_y = 0;
    double acc_x = 0;
    double acc_y = 0;

    double steer_state_front_wheel = 0;
    double brake_pedal_output = 0;
    double throttle_pedal_output = 0;
    int    gear_position = 0;

    double latitude  = 0;
    double longitude = 0;

    double l_wheel_base = 1.94;
};

 

class SimulatorNode
{

public:
    void ini();
    void updateVehState();
    void loop();

    float initime, nowtime;
    long controlStep = 0;
 
    //recv control command
    ros::Subscriber sub_cmd;

    //pub vehicle states
    void pubVehState();
    ros::Publisher  pub_veh_state;
    VehState_Sim_S  vs;

    cav_msgs::Control ctrl_msg;
};

extern GUI_Set_S guiSet;
extern SimulatorNode simu;

void cmdCallback(const cav_msgs::Control::ConstPtr& msg);

#endif
