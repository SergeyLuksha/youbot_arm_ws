#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Dense>
#include <vector>
#include <brics_actuator/msg/joint_positions.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <sensor_msgs/msg/joy.hpp>


#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

using namespace std::chrono_literals;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(rclcpp::Node::SharedPtr n, QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_pushButton_calculate_clicked();
    void on_pushButton_move_clicked();

    void on_pushButton_calculate_traj_clicked();
    void on_pushButton_execute_traj_clicked();
    void updateTrajectoryExecution();

    void on_pushButton_joy_control_enable_clicked();
    void on_pushButton_base_position_clicked();

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    bool computeForwardKinematics(const std::vector<double>& joints, Eigen::Matrix4d& T_result);
    bool computeForwardKinematicsKDL(const std::vector<double>& joints, Eigen::Matrix4d& T_result);
    bool initKDLChain();

    bool computeInverseKinematicsManual(double x, double y, double z, std::vector<double>& joints_result);
    bool computeInverseKinematicsKDL(double x, double y, double z, std::vector<double>& joints_result, const std::vector<double>& seed_joints = {});

    Ui::MainWindow *ui;
    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    KDL::Chain kdl_chain_; // Сохраненная кинематическая цепь

    std::vector<double> calculated_ik_joints_; // Сохраненные углы после расчета IK
    bool is_ik_valid_ = false;                  // Флаг успешности расчета
    rclcpp::Publisher<brics_actuator::msg::JointPositions>::SharedPtr arm_pub_;
    bool initial_spinbox_set = false;

    std::vector<double> current_joint_values_ = {0.0, 0.0, 0.0, 0.0, 0.0};

    std::vector<std::vector<double>> planned_trajectory_joints_; // Список точек траектории (углы суставов)
    QTimer *traj_execution_timer_;
    size_t current_traj_index_ = 0;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<brics_actuator::msg::JointPositions>::SharedPtr gripper_pub_;
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    // Коэффициент скорости движения от стика (радианы за один такт)
    const double joy_sensitivity = 0.01;

    double current_gripper_pos_ = 0.0;
    const double gripper_step = 0.0005; // Скорость открытия/закрытия
    const double gripper_max = 0.0115;  // Макс. ход одного пальца

    bool joystick_control_enabled_ = false; // Флаг активности управления с джойстика
};

#endif // MAINWINDOW_H
