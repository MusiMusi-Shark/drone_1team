#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iostream>
#include <cmath>
#include <vector>
#include <array>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control")
    {
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        // VehicleOdometry 구독 (로컬 위치 정보)
        rclcpp::SensorDataQoS qos;
        local_position_subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos,
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
                current_position_[0] = msg->position[0];
                current_position_[1] = msg->position[1];
                current_position_[2] = msg->position[2];
            });

        // 정사각형 경로를 위한 waypoint 설정 (NED 좌표계: z는 음수 값이 고도를 의미)
        waypoints_ = {
            {0.0f, 0.0f, -5.0f},
            {5.0f, 0.0f, -5.0f},
            {5.0f, 5.0f, -5.0f},
            {0.0f, 5.0f, -5.0f}
        };
        current_waypoint_index_ = 0;
        waypoint_reached_ = false;
        landing_command_sent_ = false;

        offboard_setpoint_counter_ = 0;

        auto timer_callback = [this]() -> void {

            // 초기 10회 setpoint 전송 후 오프보드 모드 전환 및 아밍
            if (offboard_setpoint_counter_ == 10) {
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                this->arm();
            }
            if (offboard_setpoint_counter_ < 11) {
                offboard_setpoint_counter_++;
            }

            // 오프보드 컨트롤 모드 지속 발행
            publish_offboard_control_mode();

            // waypoint 비행 로직
            if (current_waypoint_index_ < waypoints_.size()) {
                // 현재 waypoint 목표와의 거리 계산
                const auto &target = waypoints_[current_waypoint_index_];
                float dx = current_position_[0] - target[0];
                float dy = current_position_[1] - target[1];
                float dz = current_position_[2] - target[2];
                float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
                const float tolerance = 0.5f; // 0.5m 내 도달로 판단

                if (distance < tolerance) {
                    if (!waypoint_reached_) {
                        waypoint_reached_ = true;
                        hover_start_time_ = this->now();
                        RCLCPP_INFO(this->get_logger(), "Waypoint %d 도달, 2초간 호버", current_waypoint_index_);
                    } else {
                        // 2초 경과 시 다음 waypoint로 진행
                        if ((this->now() - hover_start_time_).seconds() >= 2.0) {
                            RCLCPP_INFO(this->get_logger(), "다음 waypoint로 이동");
                            current_waypoint_index_++;
                            waypoint_reached_ = false;
                        }
                    }
                }
            } else {
                // 모든 waypoint 완료 시, 2초간 호버 후 랜딩
                if (!landing_command_sent_) {
                    if (!waypoint_reached_) {
                        waypoint_reached_ = true;
                        hover_start_time_ = this->now();
                        RCLCPP_INFO(this->get_logger(), "최종 waypoint 도달, 랜딩 전 2초 호버");
                    } else {
                        if ((this->now() - hover_start_time_).seconds() >= 2.0) {
                            RCLCPP_INFO(this->get_logger(), "랜딩 시작");
                            // 랜딩 명령 발행 (MAVLink의 MAV_CMD_NAV_LAND)
                            this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
                            landing_command_sent_ = true;
                        }
                    }
                }
            }

            // 현재 waypoint(또는 마지막 위치)로 setpoint 발행
            publish_trajectory_setpoint();
        };

        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

    std::array<float, 3> get_current_position() { return current_position_; }
    void arm();
    void disarm();

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr local_position_subscription_;

    std::array<float, 3> current_position_ = {0.0, 0.0, 0.0};
    std::atomic<uint64_t> timestamp_;
    uint64_t offboard_setpoint_counter_;

    // waypoint 관련 변수
    std::vector<std::array<float, 3>> waypoints_;
    int current_waypoint_index_;
    bool waypoint_reached_;
    rclcpp::Time hover_start_time_;
    bool landing_command_sent_;

    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

void OffboardControl::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "아밍 명령 전송");
}

void OffboardControl::disarm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "디스아밍 명령 전송");
}

void OffboardControl::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void OffboardControl::publish_trajectory_setpoint()
{
    TrajectorySetpoint msg{};
    if (current_waypoint_index_ < waypoints_.size()) {
        msg.position = waypoints_[current_waypoint_index_];
    } else {
        // 모든 waypoint가 완료된 경우 마지막 waypoint 위치로 setpoint 유지
        msg.position = waypoints_.back();
    }
    msg.yaw = -3.14; // 고정 yaw 값 (필요 시 조정)
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
    std::cout << "정사각형 비행 노드 시작..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}
