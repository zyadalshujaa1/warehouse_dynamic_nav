#include <rclcpp/rclcpp.hpp>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include <rclcpp_action/rclcpp_action.hpp>



using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigateToPoseGoalHandel = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using Goal = geometry_msgs::msg::PoseStamped;
using PoseInit = geometry_msgs::msg::PoseWithCovarianceStamped;

using BOOL = std_msgs::msg::Bool;

using namespace std::placeholders;


class Nav2ActionClinet : public rclcpp::Node
{
    public:
        Nav2ActionClinet() : Node("nav2_clint")
        {

            this->nav2_action_clinet_ = rclcpp_action::create_client<NavigateToPose>(this,"navigate_to_pose") ;
            this->goal_sub_ = this->create_subscription<Goal>("send_goal",10,
                std::bind(&Nav2ActionClinet::send_goal_request_callback,this,_1));
            
            this->feedback_pub = this->create_publisher<Goal>("current_pose",10);

            this->nav2_done_pub_ = this->create_publisher<BOOL>("nav2_done",10);

            RCLCPP_INFO(this->get_logger(),"Nav2 clinet is running...");
        }

    private:


    // Goal goal_stamped_maker(double x, double y, double roll, double pitch, double yaw)
    // {
    //     tf2::Quaternion q;
    //     q.setRPY(roll,pitch,yaw);

    //     Goal target_goal;
    //     target_goal.header.frame_id = "map";
    //     target_goal.header.stamp = this->get_clock()->now();
    //     target_goal.pose.position.x = x;
    //     target_goal.pose.position.y = y;
    //     target_goal.pose.orientation = tf2::toMsg(q);

    //     return target_goal;
    // }

    void send_goal_request_callback(Goal::SharedPtr msg)
    {
        this->nav2_action_clinet_->wait_for_action_server();

        auto target_goal = NavigateToPose::Goal();
        target_goal.pose.header.frame_id    = msg->header.frame_id;
        target_goal.pose.header.stamp       = msg->header.stamp;
        target_goal.pose.pose.position.x    = msg->pose.position.x ;
        target_goal.pose.pose.position.y    = msg->pose.position.y ;
        target_goal.pose.pose.orientation.w = msg->pose.orientation.w;
        target_goal.pose.pose.orientation.x = msg->pose.orientation.x;
        target_goal.pose.pose.orientation.y = msg->pose.orientation.y;
        target_goal.pose.pose.orientation.z = msg->pose.orientation.z;

        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        options.goal_response_callback = std::bind(&Nav2ActionClinet::goal_responce_callback,this,_1);
        options.result_callback = std::bind(&Nav2ActionClinet::result_goal_callback,this,_1);
        options.feedback_callback = std::bind(&Nav2ActionClinet::feedback_callback,this,_1,_2);


        RCLCPP_INFO(this->get_logger(),"Sending the Goal...");
        nav2_action_clinet_->async_send_goal(target_goal,options);
    }

    void goal_responce_callback(const NavigateToPoseGoalHandel::SharedPtr &goal_handel)
    {
        if (! goal_handel)
        {
            RCLCPP_ERROR(this->get_logger(),"Goal Has been rejected");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(),"Goal Has been Accepted");
        }
    }

    void result_goal_callback(const NavigateToPoseGoalHandel::WrappedResult &goal_result)
    {
        auto status = goal_result.code;
        if (status == rclcpp_action::ResultCode::SUCCEEDED)
        {

            RCLCPP_INFO(this->get_logger(),"Succeeded");
            // leave it here for now for first test 
            BOOL state;
            state.data =true;
            this->nav2_done_pub_->publish(state);
        }
        else if (status == rclcpp_action::ResultCode::CANCELED)
        {
            RCLCPP_WARN(this->get_logger(),"Canceled");
        }
        else if (status == rclcpp_action::ResultCode::ABORTED)
        {
            RCLCPP_ERROR(this->get_logger(),"Aborted");
        }
        else if (status == rclcpp_action::ResultCode::UNKNOWN)
        {
            RCLCPP_ERROR(this->get_logger(),"Unknown");
        }        
    }

    void feedback_callback (const NavigateToPoseGoalHandel::SharedPtr &goal_handel,
                            const std::shared_ptr<const NavigateToPose::Feedback> feedback )
    {
        (void)goal_handel;
        Goal currecnt_pose = feedback->current_pose;
        this->feedback_pub->publish(currecnt_pose);

        // RCLCPP_INFO(this->get_logger(),"current pose x:%f y:%f",currecnt_pose.pose.position.x,currecnt_pose.pose.position.y);
    }



    rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_action_clinet_ ;
    rclcpp::Subscription<Goal>::SharedPtr goal_sub_;
    rclcpp::Publisher<Goal>::SharedPtr feedback_pub;
    rclcpp::Publisher<BOOL>::SharedPtr nav2_done_pub_;


      
};



int main(int argc,char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Nav2ActionClinet>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
