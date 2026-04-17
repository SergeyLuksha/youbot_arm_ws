#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "sensor_msgs/msg/joint_state.hpp" // Добавили сообщение ROS 2
#include "control_msgs/action/follow_joint_trajectory.hpp" // Action для управления
#include <std_msgs/msg/string.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <brics_actuator/msg/joint_positions.hpp> // Заголовок BRICS
#include "iksolver.h" // Ваш новый класс
#include "ui_mainwindow.h" // Создается автоматически из .ui
#include <brics_actuator/msg/joint_positions.hpp>
#include <sensor_msgs/msg/joy.hpp>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(rclcpp::Node::SharedPtr n, QWidget *parent = nullptr);

private slots:
    // Слот, который будет вызываться при перемещении любого слайдера
    void onSliderValueChanged(int value);

    void on_pushButton_move_to_point_clicked();

    void update_trajectory();

    void on_pushButton_Estop_clicked();

private:
    // Функция, которая будет вызываться при получении новых данных
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void send_arm_goal(); // Метод для отправки команды манипулятору

    void start_smooth_move(const std::vector<double>& target);

    Ui::MainWindow *ui;
    rclcpp::Node::SharedPtr node;

    // Объект подписчика
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    // Клиент для отправки команд движения
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr arm_action_client_;

    // В private:
    rclcpp::Publisher<brics_actuator::msg::JointPositions>::SharedPtr gripper_pub_;

    bool first_state_received_ = false;

    std::unique_ptr<IKSolver> ik_solver_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_desc_sub_;
    std::vector<double> current_joint_values_ = {0.0, 0.0, 0.0, 0.0, 0.0};

    QTimer *traj_timer_;
    std::vector<double> target_pose_;  // Куда идем
    std::vector<double> current_pose_; // Где сейчас находимся (внутри интерполятора)

    rclcpp::Publisher<brics_actuator::msg::JointPositions>::SharedPtr arm_pub_;

    // В private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    // Коэффициент скорости движения от стика (радианы за один такт)
    const double joy_sensitivity = 0.05;

    double current_gripper_pos_ = 0.0;
    const double gripper_step = 0.0005; // Скорость открытия/закрытия
    const double gripper_max = 0.0115;  // Макс. ход одного пальца

};

#endif // MAINWINDOW_H
