#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "custom_interfaces/msg/tank.hpp"

#include "tank_control/RoboteqDevice.h"
#include "tank_control/ErrorCodes.h"
#include "tank_control/Constants.h"

#include <iostream>

using namespace std::chrono_literals;

/*
This Class is a subscriber node that listens to the topic "move_tank_commands"
and sends the commands to the Roboteq motor controller.

Arguments:
    input_port: string
        The port to which the Roboteq motor controller is connected to.
        Example: "/dev/ttyACM1"

Attributes:
    subsciber: rclcpp::Subscription<custom_interfaces::msg::Tank>::SharedPtr
        The subscriber object that listens to the topic "move_tank_commands".
    device_: RoboteqDevice
        The Roboteq motor controller object.
    port: string
        The port to which the Roboteq motor controller is connected to.

*/


class MoveTank : public rclcpp::Node
{
  public:
    MoveTank(string input_port)
    : Node("move_tank"), device_()
    {
      subscriber = this->create_subscription<custom_interfaces::msg::Tank>(
      "move_tank_commands", 10, std::bind(&MoveTank::command_callback, this, std::placeholders::_1));

        port = input_port;
        int status = device_.Connect(port);

        if (status != RQ_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to connect to device, status: %d", status);
        }

        // Set to digital input
        cout<<"- SetConfig(_DINA, 1, 1)...";
        if((status = device_.SetConfig(_DINA, 1, 1)) != RQ_SUCCESS){
            RCLCPP_ERROR(this->get_logger(), "- SetConfig(_DINA, 1, 1)...failed --> %d", status);
            if (status == 3){
                cout << "Error occurred while transmitting data to device" << endl;
            }
        }
        else
            cout<<"succeeded."<<endl;
        
        sleepms(10);

        device_.Disconnect();
        
    }

  private:
    RoboteqDevice device_;
    string port;

    void command_callback(const custom_interfaces::msg::Tank::SharedPtr msg)
    {
      int status = device_.Connect(port);

        if (status != RQ_SUCCESS)
        {
            cout << "Error connecting to device: " << status << "." << endl;
        }

      RCLCPP_DEBUG(this->get_logger(), "I heard: Right: '%d', Left:'%d'", msg->right_speed, msg->left_speed);
      if (msg->right_speed == 0){
        cout<<"- SetCommand(_MS, 1)...";
        if((status = device_.SetCommand(_MS, 1)) != RQ_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "- SetCommand(_MS, 1)...failed --> %d", status);
            cout<<"failed --> "<<status<<endl;
            if (status == 3){
                cout << "Error occurred while transmitting data to device" << endl;
            }
        }
        else
            cout<<"succeeded."<<endl;
      }
        
      else{
        cout<<"- SetCommand(_GO, 1, "<< msg->right_speed <<")...";
        if((status = device_.SetCommand(_GO, 1, msg->right_speed)) != RQ_SUCCESS){
            RCLCPP_ERROR(this->get_logger(), "- SetCommand(_GO, 1, %d)...failed --> %d", msg->right_speed, status);        
            cout<<"failed --> "<<status<<endl;
        }
        else
            cout<<"succeeded."<<endl;
      }
          
      sleepms(10);
          
      
      if (msg->left_speed == 0){
        cout<<"- SetCommand(_MS, 2)...";
        if((status = device_.SetCommand(_MS, 2)) != RQ_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "- SetCommand(_MS, 2)...failed --> %d", status);
            cout<<"failed --> "<<status<<endl;
            if (status == 3){
                cout << "Error occurred while transmitting data to device" << endl;
            }
        }
        else
            cout<<"succeeded."<<endl;
      }
        
      else{
        cout<<"- SetCommand(_GO, 2, "<< msg->left_speed <<")...";
        if((status = device_.SetCommand(_GO, 2, msg->left_speed)) != RQ_SUCCESS){
            RCLCPP_ERROR(this->get_logger(), "- SetCommand(_GO, 2, %d)...failed --> %d", msg->right_speed, status);        
            cout<<"failed --> "<<status<<endl;
        }
        else
            cout<<"succeeded."<<endl;
      }
          
      sleepms(10);


        device_.Disconnect();
    }
    rclcpp::Subscription<custom_interfaces::msg::Tank>::SharedPtr subscriber;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveTank>("/dev/ttyACM1"));
  rclcpp::shutdown();
  return 0;
}