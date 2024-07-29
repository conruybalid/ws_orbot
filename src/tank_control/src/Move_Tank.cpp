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

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MoveTank : public rclcpp::Node
{
  public:
    MoveTank(string input_port)
    : Node("move_tank"), count_(0), device_()
    {
      subscriber = this->create_subscription<custom_interfaces::msg::Tank>(
      "move_commands", 10, std::bind(&MoveTank::command_callback, this, std::placeholders::_1));
      timer_ = this->create_wall_timer(
      1s, std::bind(&MoveTank::timer_callback, this));

        port = input_port;
        int status = device_.Connect(port);

        if (status != RQ_SUCCESS)
        {
            cout << "Error connecting to device: " << status << "." << endl;
        }

        // Set to digital input
        cout<<"- SetConfig(_DINA, 1, 1)...";
        if((status = device_.SetConfig(_DINA, 1, 1)) != RQ_SUCCESS){
            cout<<"failed --> "<<status<<endl;
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

    void timer_callback()
    {

    }
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;

    void command_callback(const custom_interfaces::msg::Tank::SharedPtr msg)
    {
      int status = device_.Connect(port);

        if (status != RQ_SUCCESS)
        {
            cout << "Error connecting to device: " << status << "." << endl;
        }

      RCLCPP_INFO(this->get_logger(), "I heard: Right: '%d', Left:'%d'", msg->right_speed, msg->left_speed);
      if (msg->right_speed == 0){
        cout<<"- SetCommand(_MS, 1)...";
        if((status = device_.SetCommand(_MS, 1)) != RQ_SUCCESS) {
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
        if((status = device_.SetCommand(_GO, 1, msg->right_speed)) != RQ_SUCCESS)
            cout<<"failed --> "<<status<<endl;
        else
            cout<<"succeeded."<<endl;
      }
          
      sleepms(10);
          
      
      if (msg->left_speed == 0){
        cout<<"- SetCommand(_MS, 2)...";
        if((status = device_.SetCommand(_MS, 2)) != RQ_SUCCESS) {
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
        if((status = device_.SetCommand(_GO, 2, msg->left_speed)) != RQ_SUCCESS)
            cout<<"failed --> "<<status<<endl;
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