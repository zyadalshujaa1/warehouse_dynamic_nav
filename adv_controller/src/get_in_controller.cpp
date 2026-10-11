#include "rclcpp/rclcpp.hpp" //importing the ros2 lib if it shows a error open vs from the terminal
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "logistic_msg/msg/orgniz_enable.hpp"
#include "std_msgs/msg/bool.hpp"

#include <algorithm>

#include "message_filters/subscriber.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"
#include "message_filters/synchronizer.hpp"

using Laser = sensor_msgs::msg::LaserScan;
using Twist = geometry_msgs::msg::Twist;
using OrgnizeEnable = logistic_msg::msg::OrgnizEnable;
using BOOL = std_msgs::msg::Bool;

using namespace std::placeholders;

using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    Laser,
    Laser>;

class AdvGetInController : public rclcpp::Node //create a class and inharet from the rclcpp
{
    public: //puplic
        AdvGetInController() : Node("get_in_controller") 
        {
            //assign the v and the methods

            this->enable_sub_ = this->create_subscription<OrgnizeEnable>("enable",10,
                                std::bind(&AdvGetInController::enable_callback,this,_1));

            this->ultra1_left_sub_.subscribe(this,"/ultra1_link_scan");
            this->ultra4_right_sub_.subscribe(this,"/ultra4_link_scan");

            this->cmd_vel_pub_= this->create_publisher<Twist>("/cmd_vel",10);
            this->done_pub_= this->create_publisher<BOOL>("/get_in_done",10);
            this->proximty_sub_ = this->create_subscription<Laser>("/proximity_sensor/scan",10,
            std::bind(&AdvGetInController::proximity_callback,this,_1));

            this->sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
                SyncPolicy(10), ultra1_left_sub_,ultra4_right_sub_);
            
            this->proximity_indecator_ = false;
            this->enable_glob_ = false;
            
            this->sync_->registerCallback(std::bind(&AdvGetInController::sync_callback,this,_1,_2));

            RCLCPP_INFO(this->get_logger(),"Adv_Controller is running...");
        }   
    private:
    //define the v of the class and the private methods

    void enable_callback(OrgnizeEnable::SharedPtr msg)
    {

        this->enable_glob_ = msg->getin;

    }

    void proximity_callback(const Laser::ConstSharedPtr &msg)
    {
        if (distance(msg) >= msg->range_max )
        {
            this->proximity_indecator_ = false;
        }
        else if (distance(msg) < msg->range_max)
        {
            this->proximity_indecator_ = true;
        }
        
    }

    void sync_callback(const Laser::ConstSharedPtr &left,const Laser::ConstSharedPtr &right)
    {
        if (enable_glob_)
        {
            auto left_dist = distance(left);
            auto right_dist = distance(right);

            if (left_dist < 0.0f && right_dist < 0.0f)
            {
                Twist stop;
                this->cmd_vel_pub_->publish(stop);
                BOOL state;
                state.data = true;
                this->done_pub_->publish(state);
                this->enable_glob_ = false;
                return;
            }

            if (this->proximity_indecator_)
            {
                Twist stop;
                this->cmd_vel_pub_->publish(stop);
                BOOL state;
                state.data = true;
                this->done_pub_->publish(state);
                this->enable_glob_ = false;
                return;

            }

            auto error = left_dist - right_dist;

            float Kp_ang = 0.4;                  //# rad/s per metre of difference
            float max_ang = 0.25;                //# hard limit so it never spins fast
            float max_lin = 0.22;

            float angular_z = error * Kp_ang;
            angular_z = std::max(-max_ang , std::min(max_ang,angular_z));
            
            float linear_x = max_lin * std::max(0.4 , 1.0 - std::abs(error) / 2.0); //max(0.4, 1.0 - abs(error) / 0.5)

            Twist cmd_vel_msg ;
            cmd_vel_msg.linear.x = linear_x;
            cmd_vel_msg.angular.z = angular_z;
            this->cmd_vel_pub_->publish(cmd_vel_msg);
        }
        else
        {
            return ;
        }
        


    }

    float distance(const Laser::ConstSharedPtr &msg)
    {
        if(msg->ranges.empty())
        {
            return 4.0;
        }

        int center_index = msg->ranges.size() / 2;
        float distance = msg->ranges[center_index];

        if (std::isinf(distance))
        {
            return msg->range_max;
        }

        return distance;
    }

    message_filters::Subscriber<Laser> ultra1_left_sub_;
    message_filters::Subscriber<Laser> ultra4_right_sub_;

    rclcpp::Subscription<Laser>::SharedPtr proximty_sub_;

    bool proximity_indecator_;

    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    bool enable_glob_;


    // rclcpp::Subscription<Laser>::SharedPtr ultra1_left_sub_;
    // rclcpp::Subscription<Laser>::SharedPtr ultra4_right_sub_;

    rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<OrgnizeEnable>::SharedPtr enable_sub_;
    rclcpp::Publisher<BOOL>::SharedPtr done_pub_;
};

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<AdvGetInController>(); //create the shared pointer of the my node
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}
