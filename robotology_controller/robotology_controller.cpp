#include <stdio.h>
#include <signal.h>

// Basic ROS includes
#include <ros/init.h>
#include <ros/callback_queue.h>

#include <urdf/model.h>
#include <urdf_model/joint.h>

// Include messages
#include <sensor_msgs/JointState.h>
#include <robotology_msgs/controlBoardMsg.h>

// Include ROS controller interfaces
#include <hardware_interface/controller_info.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/hardware_interface.h>

// Include controller manager
#include <controller_manager/controller_manager.h>

#define APPLICATION_NAME "hw_test_1"

class MyRobot : public hardware_interface::RobotHW,
                public hardware_interface::HardwareInterface
{
public:
    MyRobot(): joint_num(-1), initted(false)
    {
        controlModeMap["position_controllers/JointTrajectoryController"] = robotology_msgs::controlBoardMsg::POSITION_DIRECT;
        controlModeMap["velocity_controllers/JointTrajectoryController"] = robotology_msgs::controlBoardMsg::VELOCITY_CMD;
        controlModeMap["effort_controllers/JointTrajectoryController"]   = robotology_msgs::controlBoardMsg::TORQUE_CMD;

    }

//     bool canSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
//                    const std::list<hardware_interface::ControllerInfo>& stop_list) const
//     {
//         return true;
//     }

    bool initInterfaces()
    {
        ROS_FATAL_NAMED(APPLICATION_NAME, "CIAOOOOOOOOOOOOOOOOO\n");
        int j;
        for(std::map<std::string,boost::shared_ptr<urdf::Joint> >::iterator joint = robot_model.joints_.begin();  joint != robot_model.joints_.end(); joint++)
        {
            printf("%s\n", joint->first.c_str());

            j = jointMap[joint->first];

            // connect and register the joint state interface
            hardware_interface::JointStateHandle state_handle_tmp(joint->first.c_str(), &measPos[j], &measVel[j], &measEff[j]);
            jnt_state_interface.registerHandle(state_handle_tmp);

            // connect and register the joint position interface
            hardware_interface::JointHandle pos_handle_tmp(jnt_state_interface.getHandle(joint->first.c_str()), &outCmd[j]);
            jnt_pos_interface.registerHandle(pos_handle_tmp);

            // connect and register the joint effort interface
            hardware_interface::JointHandle eff_handle_tmp(jnt_state_interface.getHandle(joint->first.c_str()), &outCmd[j]);
            eff_pos_interface.registerHandle(eff_handle_tmp);
        }
        registerInterface(&jnt_state_interface);
        registerInterface(&jnt_pos_interface);
        registerInterface(&eff_pos_interface);
    }

    bool init()
    {
        ROS_INFO_NAMED("init", "start");
        std::string param_name = "robot_description";
        std::string urdf_xml;

        if(!nodeHandle.getParam(param_name, urdf_xml) )
        {
            ROS_FATAL_NAMED(APPLICATION_NAME, "Could not load the parameter %s from param server\n", param_name.c_str());
            return false;
        }

//         std::cout << "\nurdf_xml before search id " << urdf_xml << std::endl;

        if(!robot_model.initString(urdf_xml) )
        {
            ROS_FATAL_NAMED(APPLICATION_NAME, "Could not load the 'robot_description' into a urdf::Model\n");
            return false;
        }

        std::cout << "****************************************" << std::endl;
        std::cout << "number of joints: " << robot_model.joints_.size() << std::endl;

        joint_num = 0;
        output_msg.names.clear();
        longestJointName_size = 0;
        std::pair<std::map<std::string, int>::iterator,bool> ret;

        for(std::map<std::string,boost::shared_ptr<urdf::Joint> >::iterator joint = robot_model.joints_.begin();  joint != robot_model.joints_.end(); joint++)
        {
            int jointType = joint->second.get()->type;
            if( (jointType != urdf::Joint::FIXED) && (jointType != urdf::Joint::UNKNOWN ))
            {
                ret = jointMap.insert(std::pair<std::string, int>(joint->first, joint_num) );
                if (ret.second==false)
                {
                    std::cout << "element " << joint->first <<  " already existed" << std::endl;
                }
                else
                    std::cout << "added element " << joint->first <<  " with value " << joint_num << std::endl;

                // set the name in the output message
//                 output_msg.names.push_back(joint->first);

                // increase the number of joints.
                joint_num++;
                if(joint->first.size() > longestJointName_size)
                    longestJointName_size = joint->first.size();
            }
        }

        std::cout << "number of joints NON FIXED: " << joint_num << std::endl;
        std::cout << "longestJointName_size: " << longestJointName_size << std::endl;
        std::cout << "****************************************" << std::endl;

        // assign will allocate the space, resize MUST not be done here
        measPos.assign(joint_num, 0);
        measVel.assign(joint_num, 0);
        measEff.assign(joint_num, 0);
        outCmd .assign(joint_num, 0);

        // Start the shared joint state subscriber
        jointState_subscriber = nodeHandle.subscribe<sensor_msgs::JointState>("/joint_states", 1,
                                                                              &MyRobot::stateCallback, this);

        // Start publishers
        std::string topicName;
        //     topicName = "/controller_cmd/joint_command";
        topicName = "/joint_states/cmd";
        yarpRosCmd_publisher = nodeHandle.advertise<robotology_msgs::controlBoardMsg>(topicName, 10);

        // name field will be allocated after using push_back
//         output_msg.reference.assign(joint_num, 0);
//         output_msg.referenceType.assign(joint_num, 0);

        ROS_INFO_NAMED("init", "end");

        initInterfaces();
        return true;
    }

    bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                       const std::list<hardware_interface::ControllerInfo>& stop_list)
    {

        std::cout << "Start list" << std::endl;
        std::vector<std::pair<std::string, int> >newControl_map;
        std::map<std::string, int>::iterator it_modeMap;

        // for all controllers
        for (std::list<hardware_interface::ControllerInfo>::const_iterator it=start_list.begin(); it != start_list.end(); ++it)
        {
            // for each resource (joint) required by the controller
            std::cout << "\t"<< it->name << "; type "<< it->type << "; interface "<< it->hardware_interface << std::endl;
            for (std::set<std::string>::iterator it_set=it->resources.begin(); it_set != it->resources.end(); ++it_set)
            {
                if( (it_modeMap = controlModeMap.find(it->type)) != controlModeMap.end())
                {
                    std::cout << ' ' << *it_set << "; type code is "<< controlModeMap.at(it->type) << std::endl;
                }
                else
                {
                    ROS_ERROR_NAMED(APPLICATION_NAME, "Desired controler type %s is not supported yet by this hardware interface", it->type.c_str());
                    return false;
                }

                newControl_map.push_back(std::make_pair<std::string, int>(*it_set, controlModeMap.at(it->type)));
            }
        }

        controlledJoints.insert(controlledJoints.end(), newControl_map.begin(), newControl_map.end());

        // for all controllers
        std::cout << "Stop list" << std::endl;
        for (std::list<hardware_interface::ControllerInfo>::const_iterator it=stop_list.begin(); it != stop_list.end(); ++it)
        {
            // for each resource (joint) required by the controller
            std::cout << "\t"<< it->name << "; type "<< it->type << "; interface "<< it->hardware_interface << std::endl;
            for (std::set<std::string>::iterator it_set=it->resources.begin(); it_set != it->resources.end(); ++it_set)
            {
                // search for it into the current controlled joints list and remove it.
                for(int i = 0; i != controlledJoints.size(); i++)
                {
                    std::cout << "i " << i << "; controlledJoint " << controlledJoints[i].first << "; *it_set " << *it_set << std::endl;
                    if(controlledJoints[i].first == *it_set)
                    {
                        std::cout << "Removing joint " << controlledJoints[i].first << std::endl;
                        controlledJoints.erase(controlledJoints.begin() + i);
                        break;
                    }
                }
            }
        }

        return canSwitch(start_list, stop_list);
    }

    bool read()
    {
        printf("In:\n");
        int msgLenght;
        if(initted)
        {
            mutex.lock();
            msgLenght = state_msg->name.size();
            if(msgLenght <= 0)
            {
                ROS_ERROR_NAMED(APPLICATION_NAME, "Received malformed message of size %d", msgLenght);
                return false;
            }

            // size of msg can be smaller than number of joints, but not bigger
            if(msgLenght > joint_num)
            {
                ROS_ERROR_NAMED(APPLICATION_NAME, "Size of input data is %d, which is bigger than numer of joints [%d]", msgLenght, joint_num);
                return false;
            }

            if(state_msg->position.size() != msgLenght)
            {
                ROS_ERROR_NAMED(APPLICATION_NAME, "Received malformed message. Size of 'names' is different from size of references.");
                return false;
            }

            // same check for other fields?? TBD: define this fucking msg!!!

            // Joints can be in any order, so we need a map to fill in the right spot.
            for(int i=0; i<msgLenght; i++)
            {
                // TODO: using '[]' operator may cause issues, use '.at()' instead
                measPos[jointMap[state_msg->name[i]]] = state_msg->position[i];
                measVel[jointMap[state_msg->name[i]]] = state_msg->velocity[i];
                measEff[jointMap[state_msg->name[i]]] = state_msg->effort[i];
                printf("[%2d] %-*s %8.4f\t%8.4f\t%8.4f\n", i, longestJointName_size+2, state_msg->name[i].c_str(), measPos[i], measVel[i], measEff[i]);
            }

            ROS_WARN_NAMED(APPLICATION_NAME, "TBS: Try to get the most recent reference target maybe??\n");

            mutex.unlock();
            fflush(stdout);
        }
        else
        {
            std::cout << "Not initted yet" << std::endl;
        }
        return true;
    }

    void write()
    {
        if(!initted)
        {
            ROS_WARN_NAMED(APPLICATION_NAME, "hw interface was not initted yet (not able to connect to the robot), cannot send command to the robot\n");
            return;
        }
        // write only meaningful values, if possible.
        printf("out: - size of controlledJoints is %d\n", (int) controlledJoints.size());

        if(controlledJoints.size() == 0)
            return;

        msg_clear();

        for(int i=0; i<controlledJoints.size(); i++)
        {
            output_msg.names.push_back(controlledJoints[i].first);
            output_msg.referenceType.push_back(controlledJoints[i].second);
            output_msg.reference.push_back(outCmd[jointMap.at(controlledJoints[i].first)]);
            printf("[%2d - %s] mode: %d; cmd: %f [%f]\n", i, controlledJoints[i].first.c_str(), output_msg.referenceType[i], output_msg.reference[i], outCmd[i]);
        }
        printf("\n"); fflush(stdout);
        yarpRosCmd_publisher.publish(output_msg);
    }

    void stateCallback(const sensor_msgs::JointStateConstPtr& msg)
    {
        mutex.lock();
        if(!initted)
            std::cout << "Initialization DONE" << std::endl;
        initted = true;
        state_msg = msg;
        mutex.unlock();
    }

    private:

    bool initted;
    // It's always good to have a mutex
    boost::mutex mutex;

    // To create publisher and subscriber.
    ros::NodeHandle nodeHandle;

    // handle urdf robot description
    urdf::Model robot_model;
    std::map<std::string, int> jointMap;
    std::map<std::string, int> controlModeMap;
    std::vector<std::pair<std::string, int> >controlledJoints;

    // Subscriber -- read the joint state from topic to update the controller
    ros::Subscriber jointState_subscriber;
    sensor_msgs::JointStateConstPtr state_msg;         // store the message

    // Publisher  -- send the command from controller to robot/simulator
    ros::Publisher  yarpRosCmd_publisher;
    robotology_msgs::controlBoardMsg output_msg;  //TBR -- just for a quick test

    int joint_num;
    int longestJointName_size;
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    hardware_interface::EffortJointInterface eff_pos_interface;
    std::vector<double>  measPos;
    std::vector<double>  measVel;
    std::vector<double>  measEff;
    std::vector<double>  outCmd;

    void msg_clear()
    {
        output_msg.names.clear();
        output_msg.referenceType.clear();
        output_msg.reference.clear();
        output_msg.reference_dot.clear();
        output_msg.torque_offset.clear();
    }

};

void mySigintHandler(int sig)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.

    // All the default sigint handler does is call shutdown()
    std::cout << "Got CRTL+C"<< std::endl;
    ros::shutdown();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, APPLICATION_NAME);

    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, mySigintHandler);

    ros::NodeHandle nh;
//     ros::CallbackQueue queue;
//     nh.setCallbackQueue(&queue);

    // Allow the action server to recieve and send ros messages
    ros::AsyncSpinner spinner(4);
    spinner.start();

    // To create publisher and subscriber... do we really need this node here??
//     ros::NodeHandle nodeHandle;

    std::cout << "Before MyRobot"<< std::endl;

    MyRobot robot;
    if(!robot.init() )
    {
        std::cout << "Error initializing robot model, quitting" << std::endl;
        return -1;
    }

    controller_manager::ControllerManager cm(&robot, nh);

    ros::Duration elapsed_time_(1,0);
    while (ros::ok())
    {

        std::cout << "Updating stuff" << std::endl;
        robot.read();
        cm.update(ros::Time::now(), elapsed_time_);
        robot.write();
        sleep(1);
        ros::spinOnce();
    }
    return 0;
}
