// #define _GLIBCXX_USE_CXX11_ABI 0

// Main begins
// Defined all kinds of ros variables.
// Defined variables for setpoints.
// Segmentation fault

# include <iostream>
# include <string>

# include <ros/ros.h>

# include <vector>

# include <math.h>

# include <geometry_msgs/Pose.h>
# include <geometry_msgs/PoseStamped.h>
# include <geometry_msgs/Twist.h>
# include <geometry_msgs/TwistStamped.h>

# include <mavros_msgs/State.h>
# include <mavros_msgs/CommandBool.h>
# include <mavros_msgs/Param.h>
# include <mavros_msgs/ParamGet.h>
# include <mavros_msgs/ParamSet.h>
# include <mavros_msgs/SetMode.h>

using namespace std;

mavros_msgs::State currState;

geometry_msgs::PoseStamped currPose;

float precision = 0.07;

bool doneOnce = false;

vector<geometry_msgs::PoseStamped> PosSetpoints;

vector<geometry_msgs::TwistStamped> VelSetpoints;

// ::constptr
void stateCallBack(const mavros_msgs::State::ConstPtr& msg) // mavros_msgs::State msg)
{
    currState = *msg;
    if (!doneOnce) cout << "CallbackWorks I guess.\n";
    doneOnce = true;
}

// ::constptr
void posCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    currPose = *msg;
}

void addSetpoint(vector<float> position, vector<float> velocity)
{
    geometry_msgs::PoseStamped pos = geometry_msgs::PoseStamped();
    
    pos.pose.position.x = position[0];
    pos.pose.position.y = position[1];
    pos.pose.position.z = position[2];

    geometry_msgs::TwistStamped vel = geometry_msgs::TwistStamped();
    
    vel.twist.linear.x = velocity[0];
    vel.twist.linear.y = velocity[1];
    vel.twist.linear.z = velocity[2];

    PosSetpoints.emplace_back(pos);

    VelSetpoints.emplace_back(vel);
}

bool reachedSetpoint(geometry_msgs::PoseStamped target)
{

    float delX = target.pose.position.x - currPose.pose.position.x;
    float delY = target.pose.position.y - currPose.pose.position.y;
    float delZ = target.pose.position.z - currPose.pose.position.z;

    float del = sqrt(pow(delX, 2.0) + pow(delY, 2.0) + pow(delZ, 2.0));

    return del < precision;
}

int main0(int argc, char **argv)
{
    cout << "Hello frens." << endl;
    ros::init(argc, argv, "offboard_control_cpp");
    return 0;
}

int main(int argc, char **argv)
{

    cout << "Main begins\n";

    ros::init(argc, argv, "offboard_control_cpp");

    ros::NodeHandle nh;

    ros::Rate rate = ros::Rate(20);

    ros::Subscriber stateSubscriber = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, stateCallBack);
    ros::Subscriber posSubscriber = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, posCallBack);
    ros::Publisher localPosPublisher = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher localVelPublisher = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient armingClient = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient paramSetClient = nh.serviceClient<mavros_msgs::ParamSet>
            ("mavros/param/set");
    ros::ServiceClient paramGetClient = nh.serviceClient<mavros_msgs::ParamGet>
            ("mavros/param/get");

    cout << "Defined all kinds of ros variables.\n";

    vector<float> Pos {0, 0, 0};

    vector<float> Vel {0, 0, 0};

    cout << "Defined variables for setpoints.\n";

    // Mek pointers Fast
    addSetpoint(Pos, Vel); // 0th setpoint | Dummy setpoint

    cout << "Added 0th setpoint.\n";

    Pos.at(2) = 10.0;
    Vel.at(2) = 5.0;

    addSetpoint(Pos, Vel); // 1st setpoint | Take off setpoint

    Pos.at(0) = 10.0;
    Vel.at(2) = 0.0; Vel.at(0) = 5.0;

    addSetpoint(Pos, Vel); // 2nd setpoint 

    Pos.at(1) = 10.0;
    Vel.at(0) = 0.0; Vel.at(1) = 5.0;

    addSetpoint(Pos, Vel); // 3rd setpoint 

    Pos.at(0) = 0.0;
    Vel.at(1) = 0.0; Vel.at(0) = -5.0;

    addSetpoint(Pos, Vel); // 4th setpoint 

    Pos.at(1) = 0.0;
    Vel.at(0) = 0.0; Vel.at(1) = -5.0;

    addSetpoint(Pos, Vel); // 5th setpoint 

    Pos.at(2) = 0.0;
    Vel.at(1) = 0.0; Vel.at(2) = -5.0;

    addSetpoint(Pos, Vel); // Final setpoint | Landing setpoint

    cout << "Added all setpoints.\n";

    while (! (bool) currState.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    cout << "Connected Now.\n";

    cout << PosSetpoints.size() << ".\n";

    for (int i = 0; i < 100; i++)
    {
        localPosPublisher.publish(PosSetpoints.at(0));
        localVelPublisher.publish(VelSetpoints.at(0));
        
        ros::spinOnce();
        rate.sleep();
    }

    cout << "Published dummy setpoints.\n";

    // ROS_INFO("Published dummy setpoints.");

    mavros_msgs::CommandBool armMsg = mavros_msgs::CommandBool();
    armMsg.request.value = true;

    mavros_msgs::ParamSet paramSetMsg = mavros_msgs::ParamSet();
    paramSetMsg.request.param_id = "COM_RCL_EXCEPT";
    paramSetMsg.request.value.integer = (1 << 2);

    mavros_msgs::ParamGet paramGetMsg = mavros_msgs::ParamGet();
    paramGetMsg.request.param_id = "COM_RCL_EXCEPT";

    mavros_msgs::SetMode modeMsg = mavros_msgs::SetMode();
    modeMsg.request.base_mode = 0;
    modeMsg.request.custom_mode = "OFFBOARD";

    while (ros::ok())
    {
        paramGetClient.call(paramGetMsg);
        if (!currState.armed)
            armingClient.call(armMsg);
        
        if ((((paramGetMsg.response.value.integer) & (1 << 2)) >> 2) != 1)
            paramSetClient.call(paramSetMsg);
        
        if (currState.mode != "OFFBOARD")
            set_mode_client.call(modeMsg);

        ros::spinOnce();
        rate.sleep();
    }

    for (int i = 1; i < PosSetpoints.size(); i ++)
    {
        // ROS_INFO("next.");
        
        while (! ros::isShuttingDown())
        {
            if (reachedSetpoint(PosSetpoints[i]))
                break;
            
            localPosPublisher.publish(PosSetpoints.at(i));
            localVelPublisher.publish(VelSetpoints.at(i));
            
            ros::spinOnce();
            rate.sleep();
        }
    }

    armMsg.request.value = false;

    while (currState.armed)
    {
        armingClient.call(armMsg);
        
        ros::spinOnce();
        rate.sleep();
    }

    // ROS_INFO("Over");

    return 0;

}