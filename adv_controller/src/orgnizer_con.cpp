#include "rclcpp/rclcpp.hpp" //importing the ros2 lib if it shows a error open vs from the terminal
#include "rclcpp_action/rclcpp_action.hpp" //importing the action lib
#include "logistic_msg/action/move_robot.hpp" //importing the action file that we created
#include "logistic_msg/msg/orgniz_enable.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include"tf2/LinearMath/Quaternion.hpp"
#include"tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <thread>
#include <chrono>
#include <atomic>

using Twist = geometry_msgs::msg::Twist;
using FLoat64MulArray = std_msgs::msg::Float64MultiArray;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using BOOL = std_msgs::msg::Bool;
using OrgnizeEnable = logistic_msg::msg::OrgnizEnable;
using namespace std::placeholders;
using MoveRobot = logistic_msg::action::MoveRobot;
using MoveRobotGoalHandler = rclcpp_action::ServerGoalHandle<MoveRobot>;

class OrgnizerServerNode : public rclcpp::Node //create a class and inharet from the rclcpp
{
    public: //puplic
        OrgnizerServerNode() : Node("orgnizer_node") 
        {
            //assign the v and the methods
            assign_point(this->home,-3.0,0.0,0.0);
            assign_point(this->pickin,0.0,0.0,0.0);
            assign_point(this->before_pickin,-1.0,0.0,0.0);
            assign_point(this->pickoff,-5.0,13.0,-1.57);
            assign_point(this->before_pickoff,-5.0,10.0,-1.57);

            this->orgnizer_server = rclcpp_action::create_server<MoveRobot>(
                this,
                "cycle_perform",
                std::bind(&OrgnizerServerNode::goal_callback,  this, _1 , _2 ),
                std::bind(&OrgnizerServerNode::cancel_callback,this, _1 ),
                std::bind(&OrgnizerServerNode::handler_callback,this, _1)
            );

            this->enable_pub_ =this->create_publisher<OrgnizeEnable>("enable",10);

            this->done_getin_sub_ = this->create_subscription<BOOL>("/get_in_done",10,
            std::bind(&OrgnizerServerNode::getin_done_callback,this,_1));

            this->done_nav2_sub_ = this->create_subscription<BOOL>("/nav2_done",10,
            std::bind(&OrgnizerServerNode::nav2_done_callback,this,_1));

            this->nema_commander_ =this->create_publisher<FLoat64MulArray>
            ("/Nema_controller/commands",10);

            this->cmd_vel_pub_= this->create_publisher<Twist>("/cmd_vel",10);

            this->send_to_nav_pub_ = this->create_publisher<PoseStamped>("send_goal", 10);

            this->glob_getin_done_ =false;
            this->glob_nav2_done_ =false;

            this->go_back_slow_.linear.x = -0.8;

        }   
    private:
    //define the v of the class and the private methods
        

        //goal callback function
        rclcpp_action::GoalResponse goal_callback(
            const rclcpp_action::GoalUUID &uuid, std::shared_ptr< const MoveRobot::Goal> goal)
        {
            (void)uuid;
            (void)goal;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE ;
        }

        //cancel responce function 
        rclcpp_action::CancelResponse cancel_callback(
            const std::shared_ptr<MoveRobotGoalHandler> goal_handler )
        {
            (void)goal_handler;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        //goal_handler function 

        void handler_callback(
            const std::shared_ptr<MoveRobotGoalHandler> goal_handler )
        {
            std::thread{&OrgnizerServerNode::executegoal, this, goal_handler}.detach();
        }

        //goal execute like py (optional)
        void executegoal(
            const std::shared_ptr<MoveRobotGoalHandler> goal_handler)
        {
            auto income_command = goal_handler->get_goal()->action;
            RCLCPP_INFO(this->get_logger(),"Command is: %s",income_command.c_str());

            auto result = std::make_shared<MoveRobot::Result>();
            auto feedback = std::make_shared<MoveRobot::Feedback>();

            if (goal_handler->is_canceling())
            {
                RCLCPP_ERROR(this->get_logger(), "Canceling the goal");

                // result->pose_x = feedback->pose_x;
                // result->pose_y = feedback->pose_y;
                // result->theta  = feedback->theta ;
                goal_handler->canceled(result);
                return;
            }

            //first go to before the pickin
            this->before_pickin.header.stamp = this->get_clock()->now();
            this->glob_nav2_done_ =false;
            this->send_to_nav_pub_->publish(this->before_pickin);
            RCLCPP_INFO(this->get_logger(), "Going to before_pickin...");
            if (!wait_until(this->glob_nav2_done_, goal_handler))
            {
                goal_handler->canceled(result);
                return;
            }
            //reset the value so it can be updated again 
            this->glob_nav2_done_=false;

            this->glob_getin_done_ = false;
            enable_getin(true);
            RCLCPP_INFO(this->get_logger(), "Get-in enabled...");
            if (!wait_until(this->glob_getin_done_, goal_handler))
            {
                enable_getin(false);
                goal_handler->canceled(result);
                return;
            }
            enable_getin(false);

            //after this means the robot is under the shelf now lets lift up
            FLoat64MulArray value;
            value.data.push_back(0.154);
            this->nema_commander_->publish(value);
            value.data.clear();
            //to make sure that the weight is up
            rclcpp::sleep_for(std::chrono::seconds(8)); //simulation clock 

            this->pickoff.header.stamp = this->get_clock()->now();
            this->glob_nav2_done_ =false;
            this->send_to_nav_pub_->publish(this->pickoff);
            RCLCPP_INFO(this->get_logger(), "Going to before_pickin...");
            if (!wait_until(this->glob_nav2_done_, goal_handler))
            {
                goal_handler->canceled(result);
                return;
            }
            //reset the value so it can be updated again 
            this->glob_nav2_done_ =false;

            value.data.push_back(0.0);
            this->nema_commander_->publish(value);
            value.data.clear();
            //to make sure that the weight is up
            rclcpp::sleep_for(std::chrono::seconds(8));

            //it must go backword in strate line so we don't hit the shelf after we lift it down 
            // this->before_pickoff.header.stamp = this->get_clock()->now();
            // this->glob_nav2_done_ =false;
            // this->send_to_nav_pub_->publish(this->before_pickoff);
            // RCLCPP_INFO(this->get_logger(), "Going to before_pickin...");
            // if (!wait_until(this->glob_nav2_done_, goal_handler))
            // {
            //     goal_handler->canceled(result);
            //     return;
            // }
            // //reset the value so it can be updated again 
            // this->glob_nav2_done_ =false;
            // for(int i =0 ; i < 15; i++)
            // {
            //     this->cmd_vel_pub_->publish(this->go_back_slow_);
            //     rclcpp::sleep_for(std::chrono::seconds(1));
            // }
            // Twist Stop_; //delay till the robot go out
            // this->cmd_vel_pub_->publish(Stop_);

            this->before_pickoff.header.stamp = this->get_clock()->now();
            this->glob_nav2_done_ =false;
            this->send_to_nav_pub_->publish(this->before_pickoff);
            RCLCPP_INFO(this->get_logger(), "Going to before_pickin...");
            if (!wait_until(this->glob_nav2_done_, goal_handler))
            {
                goal_handler->canceled(result);
                return;
            }
            //reset the value so it can be updated again 
            this->glob_nav2_done_ =false;

            // go to home 
            this->home.header.stamp = this->get_clock()->now();
            this->glob_nav2_done_ =false;
            this->send_to_nav_pub_->publish(this->home);
            RCLCPP_INFO(this->get_logger(), "Going to before_pickin...");
            if (!wait_until(this->glob_nav2_done_, goal_handler))
            {
                goal_handler->canceled(result);
                return;
            }
            //reset the value so it can be updated again 
            this->glob_nav2_done_ =false;

            goal_handler->succeed(result); //update the result with the current pose
        }

        void getin_done_callback(const BOOL::SharedPtr state)
        {
            if (state->data)
            {
                this->glob_getin_done_ = state->data; 
            }
            
        }

        void nav2_done_callback(const BOOL::SharedPtr state)
        {
            if (state->data)
            {
                this->glob_nav2_done_ = state->data; 
            }
            
        }

        bool wait_until(std::atomic<bool> &flag, const std::shared_ptr<MoveRobotGoalHandler> &goal_handler)
        {
            while (rclcpp::ok() && !flag)
            {
                if (goal_handler->is_canceling())
                {
                    return false;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            return true;
        }

        void enable_getin(bool state)
        {
            OrgnizeEnable state_1;
            state_1.getin = state;
            this->enable_pub_->publish(state_1);
        }

        void assign_point(PoseStamped &point,float x, float y, float z)
        {
            tf2::Quaternion q;
             q.setRPY(0.0,0.0,z);
            point.header.frame_id = "map";
            point.header.stamp = this->get_clock()->now();
            point.pose.position.x = x;
            point.pose.position.y = y;
            point.pose.orientation = tf2::toMsg(q);
        }

        rclcpp_action::Server<MoveRobot>::SharedPtr orgnizer_server;
        rclcpp::Publisher<OrgnizeEnable>::SharedPtr enable_pub_;
        rclcpp::Subscription<BOOL>::SharedPtr done_getin_sub_;
        rclcpp::Subscription<BOOL>::SharedPtr done_nav2_sub_;
        rclcpp::Publisher<PoseStamped>::SharedPtr send_to_nav_pub_;
        rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::Publisher<FLoat64MulArray>::SharedPtr nema_commander_;

        std::atomic<bool> glob_getin_done_{false};
        std::atomic<bool> glob_nav2_done_{false};

        Twist go_back_slow_;
        

        PoseStamped home;
        PoseStamped pickin;
        PoseStamped before_pickin;
        PoseStamped pickoff;
        PoseStamped before_pickoff;
};

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<OrgnizerServerNode>(); //create the shared pointer of the my node
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}
