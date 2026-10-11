#include "rclcpp/rclcpp.hpp" //importing the ros2 lib if it shows a error open vs from the terminal
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "logistic_msg/msg/orgniz_enable.hpp"
#include "std_msgs/msg/bool.hpp"

#include <algorithm>

using Laser = sensor_msgs::msg::LaserScan;
using Twist = geometry_msgs::msg::Twist;
using OrgnizeEnable = logistic_msg::msg::OrgnizEnable;
using BOOL = std_msgs::msg::Bool;

using namespace std::placeholders;



class AdvGetInController : public rclcpp::Node //create a class and inharet from the rclcpp
{
    public: //puplic
        AdvGetInController() : Node("adv_getin") 
        {
            //assign the v and the methods

            this->enable_sub_ = this->create_subscription<OrgnizeEnable>("enable",10,
                                std::bind(&AdvGetInController::enable_callback,this,_1));

            this->cmd_vel_pub_= this->create_publisher<Twist>("/cmd_vel",10);
            this->done_pub_= this->create_publisher<BOOL>("/get_in_done",10);
            this->proximty_sub_ = this->create_subscription<Laser>("/proximity_sensor/scan",10,
            std::bind(&AdvGetInController::proximity_callback,this,_1));

            this->lidar_sub_ = this->create_subscription<Laser>("/scan",10,
            std::bind(&AdvGetInController::sync_callback,this,_1));

            
            
            this->proximity_indecator_ = false;
            this->enable_glob_ = false;
            
            

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

    void sync_callback(const Laser::ConstSharedPtr &scan)
    {
        if (enable_glob_)
        {
            auto left_dist = adv_distance(scan,true);
            auto right_dist = adv_distance(scan, false);

            if (left_dist < 0.0f && right_dist < 0.0f)
            {
                stop_and_finish();
                return;
            }

            if (this->proximity_indecator_)
            {
                stop_and_finish();
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

    float adv_distance(const Laser::ConstSharedPtr &msg , const bool left)
    {
        if (msg->ranges.empty())
        {
            return msg->range_max;
        }

        int center_index = msg->ranges.size() / 2;
        int total = msg->ranges.size();
        int start , end ;

        if (left)
        {
            start = center_index + 40; // this is 40 degree of view 
            end = center_index   + 183;
        }
        else
        {
            start = center_index - 183 ; // this is 40 degree of view 
            end = center_index   -40   ;
        }

        start = std::max(0 , start);
        end   = std::min(total - 1, end);

        float sum = 0.0f;
        int count = 0;

        for (int i = start; i<=end ; i++)
        {
            float r = msg->ranges[i];
            if (std::isfinite(r) && r > msg->range_min && r < msg->range_max)
            {
               sum += r;
               count ++; 
            }
        }

        if (count != 0)
        {
            return sum /count;
        }
        else
        {
            return msg->range_max;
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

    void stop_and_finish()
    {
        Twist stop;
        this->cmd_vel_pub_->publish(stop);
        BOOL state;
        state.data = true;
        this->done_pub_->publish(state);
        this->enable_glob_ = false;
    }

    rclcpp::Subscription<Laser>::SharedPtr lidar_sub_;

    rclcpp::Subscription<Laser>::SharedPtr proximty_sub_;

    bool proximity_indecator_;

    

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
