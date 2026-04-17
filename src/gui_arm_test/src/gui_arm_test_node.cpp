#include <QApplication>
#include "mainwindow.h"
#include <thread>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("gui_node");
    auto m = std::make_shared<MainWindow>(node);

    m->show();

    // Запуск спиннера ROS в отдельном потоке
    std::thread ros_thread([node]() {
        rclcpp::spin(node);
    });
    int result = app.exec();

    rclcpp::shutdown();
    ros_thread.join();
    return result;
}
