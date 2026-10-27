#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "sensor_msgs/msg/joint_state.hpp" // Добавили сообщение ROS 2
// #include "control_msgs/action/follow_joint_trajectory.hpp" // Action для управления
#include <std_msgs/msg/string.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <brics_actuator/msg/joint_positions.hpp> // Заголовок BRICS
#include "ui_mainwindow.h" // Создается автоматически из .ui
#include <brics_actuator/msg/joint_positions.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

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
    void onJointSelected(int index);         // Обработка смены сустава в ComboBox
    void onSetLinkClicked();                 // Кнопка "Задать"


    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
private:
    Ui::MainWindow *ui;
    rclcpp::Node::SharedPtr node;

    // Данные лимитов суставов (минимальные и максимальные значения в радианах)
    struct JointLimit {
        double min;
        double max;
        QString name;
    };

    QList<JointLimit> joint_limits_ = {
        {0.01,  5.89,  "arm_joint_1"},
        {0.01,  2.70,  "arm_joint_2"},
        {-5.18, -0.01, "arm_joint_3"},
        {0.01,  3.57,  "arm_joint_4"},
        {0.01,  5.84,  "arm_joint_5"}
    };

    int selected_joint_index_ = 0;
    std::vector<double> target_positions_ = {0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> current_pose_ = {0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> current_joint_values_ = {0.0, 0.0, 0.0, 0.0, 0.0};

    // ROS 2 подписчики и издатели
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<brics_actuator::msg::JointPositions>::SharedPtr arm_pub_;

    // График и серии данных
    QtCharts::QChart *chart_;
    QtCharts::QChartView *chart_view_;
    QtCharts::QLineSeries *series_target_;
    QtCharts::QLineSeries *series_actual_;
    QtCharts::QValueAxis *axis_x_;
    QtCharts::QValueAxis *axis_y_;

    bool is_plotting_ = false;          // Флаг активного построения графика
    double chart_time_elapsed_ = 0.0;   // Прошедшее время для оси X
    void setupChart();
};

#endif // MAINWINDOW_H
