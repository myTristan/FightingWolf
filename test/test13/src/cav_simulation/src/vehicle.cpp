

#include "datadefine.hpp"


void SimulatorNode::ini()
{ ;
}


void SimulatorNode::updateVehState()
{
 
    if (controlStep < 10)
    {        
        vs.longitude = guiSet.x_0;
        vs.latitude = guiSet.y_0;
        vs.z = 0;
        vs.heading = guiSet.heading_0;

        if (guiSet.vehicle_type==1)
            vs.l_wheel_base = 1.94;//m Lyne：1.94 YHS:0.85
        else if (guiSet.vehicle_type==2)
            //vs.l_wheel_base = 0.85;
            vs.l_wheel_base = 0.7;

        //ini vehicle states
        UTM::LLtoUTM(vs.latitude, vs.longitude, vs.y, vs.x);
        vs.z = 202.0f;
        return;
    }


    float dt = 1.0/FREQ;

    //update vehicle speed
    vs.speed_x = vs.speed_x*0.9 + simu.ctrl_msg.speed_cmd*0.1;
    ROS_INFO_THROTTLE(2,"speedx= %f, speed_cmd= %f",
        vs.speed_x, simu.ctrl_msg.speed_cmd);

    //update control commands
    vs.brake_pedal_output     = simu.ctrl_msg.brake_cmd;
    vs.throttle_pedal_output  = simu.ctrl_msg.throttle_cmd;
    vs.gear_position          = simu.ctrl_msg.gear_cmd;
    vs.steer_state_front_wheel   = simu.ctrl_msg.steering_cmd; //rad



    //kinematics model
    if(true)
    {
        //update yaw rate
        //float R = vs.l_wheel_base/tan(vs.steer_state_front_wheel);
        float R = 0.7/tan(vs.steer_state_front_wheel);
        vs.yawRate = vs.speed_x/R;

        if (abs(vs.steer_state_front_wheel) < 0.0000001)
        {
            vs.yawRate = 0;
            //R = 1/0.0000001;
            R = 0.7/0.0000001;
        }            

        //lateral speed and acc
        vs.acc_y   = 0;
        vs.speed_y = 0;
        vs.heading += vs.yawRate * dt;
        vs.heading = XM::Normalise_2PI(vs.heading);


        //simulation of vehicle position
        vs.x  += ( vs.speed_x * cos(vs.heading) 
                        -vs.speed_y * sin(vs.heading) ) * dt;
        vs.y  += ( vs.speed_x * sin(vs.heading) 
                        +vs.speed_y * cos(vs.heading) ) * dt;
        
        UTM::UTMtoLL(vs.y, vs.x, UTM_NUM, vs.latitude, vs.longitude);
    }

    ROS_INFO_THROTTLE(2,
        "lon=%.8lf lat=%.8lf x=%.3f y=%.3f heading=%.2fdeg vx=%.2f vy=%.2f \n", 
        vs.longitude,vs.latitude,
        vs.x, vs.y, 
        vs.heading * 180.0f/M_PI,
        vs.speed_x, vs.speed_y
        );
}



void SimulatorNode::loop()
{
    controlStep ++;
    updateVehState();
    pubVehState();
}



void cmdCallback(const cav_msgs::Control::ConstPtr& msg)
{
    simu.ctrl_msg = *msg;
};


void SimulatorNode::pubVehState()
{
    cav_msgs::VehicleState msg;
 
    msg.timestamp           = ros::Time::now().toSec();
    msg.RTK_seq_num         = controlStep;
    msg.RTK_gps_status       = 2;
    msg.RTK_gps_service      = 3;

    //update gps and position
    msg.RTK_gps_longitude   = vs.longitude; 
    msg.RTK_gps_latitude    = vs.latitude;
    msg.RTK_gps_altitude    = vs.z;

    msg.x = vs.x;
    msg.y = vs.y;
    msg.z = vs.z;

    //update pose
    msg.heading         = vs.heading;
    msg.pitch           = 0.0f;
    msg.roll            = 0.0f;

    //update yaw rate
    msg.yaw_rate = vs.yawRate;

    //update acc and speed
    msg.acc_x = vs.acc_x;
    msg.acc_y = vs.acc_y;
    msg.acc_z = 0.0f;

    msg.speed_x = vs.speed_x;
    msg.speed_y = vs.speed_y;
    msg.speed_z = 0.0f;

    //update size
    msg.wheelbase = vs.l_wheel_base;

    if (guiSet.vehicle_type==1)
    {
        msg.size_x = 4;
        msg.size_y = 2.2;
        msg.size_z = 2.0;
        
    }
    else if (guiSet.vehicle_type==2)
    {
        msg.size_x = 1.6; //meter
        msg.size_y = 0.82; //meter
        msg.size_z = 1.0; //meter
    }

    msg.by_wire_enabled = true;

    msg.throttle_state  = vs.throttle_pedal_output;
    msg.brake_state = vs.brake_pedal_output;
    msg.steer_state = vs.steer_state_front_wheel;
    msg.gear_pos = vs.gear_position;

    pub_veh_state.publish(msg);
}
