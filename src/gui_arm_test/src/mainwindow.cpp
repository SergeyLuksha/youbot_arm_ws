#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(rclcpp::Node::SharedPtr n, QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), node(n)
{
    ui->setupUi(this);
    // Инициализация Action клиента
    arm_action_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        node, "/arm_1/arm_controller/follow_joint_trajectory");

    // Подписка на состояние
    joint_state_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&MainWindow::joint_state_callback, this, std::placeholders::_1));

    // Настройка слайдеров (диапазон 0-1000 для точности)
    QList<QSlider*> sliders = {ui->horizontalSlider_joint_1, ui->horizontalSlider_joint_2,
                                ui->horizontalSlider_joint_3, ui->horizontalSlider_joint_4,
                                ui->horizontalSlider_joint_5};

    for(auto slider : sliders) {
        slider->setRange(0, 1000);
        connect(slider, &QSlider::valueChanged, this, &MainWindow::onSliderValueChanged);
    }

    // В конструкторе MainWindow
    gripper_pub_ = node->create_publisher<brics_actuator::msg::JointPositions>(
        "/arm_1/gripper_controller/position_command", 10);

    connect(ui->horizontalSlider_gripper, &QSlider::valueChanged, this, [this](int value) {
        auto msg = brics_actuator::msg::JointPositions();

        // Создаем структуру для одного сустава (пальца)
        // Драйвер YouBot часто ожидает управление одним "виртуальным" суставом для захвата
        brics_actuator::msg::JointValue joint_val;
        joint_val.joint_uri = "gripper_finger_joint_l"; // Имя из вашего .cfg
        joint_val.unit = "m"; // Метры

        // Пересчет 0..1000 в 0..0.0115 (макс. ход одного пальца)
        joint_val.value = (value / 1000.0) * 0.0115;

        msg.positions.push_back(joint_val);

        joint_val.joint_uri = "gripper_finger_joint_r"; // Имя из вашего .cfg
        joint_val.unit = "m"; // Метры

        // Пересчет 0..1000 в 0..0.0115 (макс. ход одного пальца)
        joint_val.value = (value / 1000.0) * 0.0115;

        msg.positions.push_back(joint_val);

        gripper_pub_->publish(msg);
    });

    // В конструкторе MainWindow
    robot_desc_sub_ = node->create_subscription<std_msgs::msg::String>(
        "/robot_description",
        rclcpp::QoS(1).transient_local(), // Важно: описание часто публикуется как Transient Local
        [this](const std_msgs::msg::String::SharedPtr msg) {
            if (!ik_solver_) {
                ik_solver_ = std::make_unique<IKSolver>(msg->data);
                ui->label_status->setText("Статус: Модель робота загружена. ИК готова.");
            }
        }
        );

    // В конструкторе:
    arm_pub_ = node->create_publisher<brics_actuator::msg::JointPositions>(
        "/arm_1/arm_controller/position_command", 10);

    traj_timer_ = new QTimer(this);
    connect(traj_timer_, &QTimer::timeout, this, &MainWindow::update_trajectory);

    joy_sub_ = node->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&MainWindow::joy_callback, this, std::placeholders::_1));

}

void MainWindow::start_smooth_move(const std::vector<double>& target) {
    traj_timer_->stop(); // Прерываем текущее движение, если оно было
    target_pose_ = target;
    current_pose_ = current_joint_values_;
    traj_timer_->start(20);
}

void MainWindow::update_trajectory() {
    auto msg = brics_actuator::msg::JointPositions();
    bool all_reached = true;
    double step = 0.01; // Шаг в радианах (~0.5 градуса) за один тик таймера

    std::vector<std::string> joint_names = {"arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"};

    for (size_t i = 0; i < 5; ++i) {
        double diff = target_pose_[i] - current_pose_[i];

        if (std::abs(diff) > step) {
            current_pose_[i] += (diff > 0 ? step : -step);
            all_reached = false;
        } else {
            current_pose_[i] = target_pose_[i];
        }

        // Заполняем сообщение BRICS
        brics_actuator::msg::JointValue jv;
        jv.joint_uri = joint_names[i];
        jv.unit = "rad";
        jv.value = current_pose_[i];
        msg.positions.push_back(jv);
    }

    arm_pub_->publish(msg);

    if (all_reached) {
        traj_timer_->stop();
        ui->label_status->setText("Статус: Точка достигнута");
    }
}

void MainWindow::onSliderValueChanged(int /*value*/)
{
    // Как только любой слайдер сдвинулся, отправляем новую цель
    std::vector<double> target(5);
    target[0] = ui->horizontalSlider_joint_1->value() * 5.89 / 1000.0;
    target[1] = ui->horizontalSlider_joint_2->value() * 2.70 / 1000.0;
    target[2] = ui->horizontalSlider_joint_3->value() * -5.18 / 1000.0;
    target[3] = ui->horizontalSlider_joint_4->value() * 3.57 / 1000.0;
    target[4] = ui->horizontalSlider_joint_5->value() * 5.84 / 1000.0;

    // Запускаем плавное следование к этой позе
    start_smooth_move(target);
}

void MainWindow::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (msg->position.size() >= 7) {
        // 1. Обновляем текстовые метки (как и раньше)
        ui->label_joint_1->setText("Joint 1: " + QString::number(msg->position[0], 'f', 3));
        ui->label_joint_2->setText("Joint 2: " + QString::number(msg->position[1], 'f', 3));
        ui->label_joint_3->setText("Joint 3: " + QString::number(msg->position[2], 'f', 3));
        ui->label_joint_4->setText("Joint 4: " + QString::number(msg->position[3], 'f', 3));
        ui->label_joint_5->setText("Joint 5: " + QString::number(msg->position[4], 'f', 3));
        ui->label_gripper->setText("Gripper l: " + QString::number(msg->position[5], 'f', 3) + " r: " + QString::number(msg->position[6], 'f', 3));

        for (size_t i = 0; i < 5; ++i) {
            current_joint_values_[i] = msg->position[i];
        }

        if (ik_solver_ && msg->position.size() >= 5) {
            double x, y, z;
            std::vector<double> current_q = {msg->position[0], msg->position[1],
                                             msg->position[2], msg->position[3], msg->position[4]};

            if (ik_solver_->solveFK(current_q, x, y, z)) {
                ui->label_coords->setText(QString("X: %1, Y: %2, Z: %3")
                                              .arg(x, 0, 'f', 3).arg(y, 0, 'f', 3).arg(z, 0, 'f', 3));
            }
        }
        // 2. Синхронизируем слайдеры при первом запуске
        if (!first_state_received_) {
            // Блокируем сигналы, чтобы при установке значений слайдеров
            // программа не начала тут же отправлять команды «двигаться назад» роботу
            ui->horizontalSlider_joint_1->blockSignals(true);
            ui->horizontalSlider_joint_2->blockSignals(true);
            ui->horizontalSlider_joint_3->blockSignals(true);
            ui->horizontalSlider_joint_4->blockSignals(true);
            ui->horizontalSlider_joint_5->blockSignals(true);
            ui->horizontalSlider_gripper->blockSignals(true);

            // Обратный пересчет: (Радианы / Макс_Радиан) * 1000
            ui->horizontalSlider_joint_1->setValue(static_cast<int>((msg->position[0] / 5.89) * 1000.0));
            ui->horizontalSlider_joint_2->setValue(static_cast<int>((msg->position[1] / 2.70) * 1000.0));
            ui->horizontalSlider_joint_3->setValue(static_cast<int>((msg->position[2] / -5.18) * 1000.0));
            ui->horizontalSlider_joint_4->setValue(static_cast<int>((msg->position[3] / 3.57) * 1000.0));
            ui->horizontalSlider_joint_5->setValue(static_cast<int>((msg->position[4] / 5.84) * 1000.0));

            // Для гриппера (исходя из хода одного пальца 0.0115)
            ui->horizontalSlider_gripper->setValue(static_cast<int>((msg->position[5] / 0.0115) * 1000.0));

            // Разблокируем сигналы
            ui->horizontalSlider_joint_1->blockSignals(false);
            ui->horizontalSlider_joint_2->blockSignals(false);
            ui->horizontalSlider_joint_3->blockSignals(false);
            ui->horizontalSlider_joint_4->blockSignals(false);
            ui->horizontalSlider_joint_5->blockSignals(false);
            ui->horizontalSlider_gripper->blockSignals(false);

            first_state_received_ = true; // Запрещаем повторную синхронизацию
        }
    }
}

void MainWindow::send_arm_goal()
{
    if (!arm_action_client_->action_server_is_ready()) return;

    auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();

    trajectory_msgs::msg::JointTrajectoryPoint point;

    // Масштабируем значения слайдеров (0..1000) в примерные рабочие диапазоны радиан YouBot
    // Сустав 1: ~0.01 до 5.8, Сустав 2: ~0.01 до 2.6 и т.д.
    // Для теста возьмем упрощенный коэффициент (нужно сверить с вашим .urdf)
    point.positions.push_back(ui->horizontalSlider_joint_1->value() * 5.89 / 1000.0);
    point.positions.push_back(ui->horizontalSlider_joint_2->value() * 2.70 / 1000.0);
    point.positions.push_back(ui->horizontalSlider_joint_3->value() * -5.18 / 1000.0);
    point.positions.push_back(ui->horizontalSlider_joint_4->value() * 3.57 / 1000.0);
    point.positions.push_back(ui->horizontalSlider_joint_5->value() * 5.84 / 1000.0);

    // ОБЯЗАТЕЛЬНО: пустые скорости и ускорения, чтобы драйвер не отклонил цель
    point.velocities.resize(5, 0.0);
    point.accelerations.resize(5, 0.0);
    point.time_from_start.sec = 0;
    point.time_from_start.nanosec = 500000000; // 0.5 секунды на перемещение (быстрая реакция)

    goal_msg.trajectory.joint_names = {"arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"};
    goal_msg.trajectory.points.push_back(point);

    arm_action_client_->async_send_goal(goal_msg);
}

void MainWindow::on_pushButton_move_to_point_clicked()
{
    if (!ik_solver_) {
        ui->label_status->setText("Ошибка: Решатель ИК еще не инициализирован!");
        return;
    }

    // 1. Получаем целевые координаты из UI
    double x = ui->doubleSpinBox_X->value();
    double y = ui->doubleSpinBox_Y->value();
    double z = ui->doubleSpinBox_Z->value();

    // Для YouBot (5-DOF) обычно фиксируют Pitch = -PI/2 (захват смотрит вниз)
    double roll = 0.0;
    double pitch = -1.57;
    double yaw = ui->doubleSpinBox_Yaw->value();

    // 2. Получаем текущие углы из вашей системы (они уже приходят в joint_state_callback)
    // Допустим, вы сохраняете их в std::vector<double> current_joint_values_;
    std::vector<double> target_joints;

    // 3. Решаем задачу ИК
    if (ik_solver_->solveIK(x, y, z, roll, pitch, yaw, current_joint_values_, target_joints)) {

        for(int j = 0 ; j < target_joints.size(); j++) {
            RCLCPP_INFO_STREAM(node->get_logger(), "Target "<<j<<" joint: "<<target_joints.at(j));
        }



        // 4. Если решение найдено, обновляем слайдеры (блокируя сигналы)
        if (target_joints.size() >= 5) {
            // Список указателей на ваши слайдеры для удобного обхода в цикле
            QList<QSlider*> arm_sliders = {
                ui->horizontalSlider_joint_1,
                ui->horizontalSlider_joint_2,
                ui->horizontalSlider_joint_3,
                ui->horizontalSlider_joint_4,
                ui->horizontalSlider_joint_5
            };

            // Коэффициенты масштабирования (макс. значения из вашего кода/модели)
            // Важно: для Joint 3 мы используем abs(), так как у вас там отрицательный предел (-5.18)
            std::vector<double> limits = {5.89, 2.70, -5.18, 3.57, 5.84};

            for (int i = 0; i < arm_sliders.size(); ++i) {
                arm_sliders[i]->blockSignals(true);

                // Пересчет из радиан в диапазон слайдера (0..1000)
                double ratio = target_joints[i] / limits[i];
                int slider_value = static_cast<int>(ratio * 1000.0);

                // Ограничиваем значение, чтобы не выйти за пределы 0..1000
                slider_value = std::max(0, std::min(1000, slider_value));

                arm_sliders[i]->setValue(slider_value);
                arm_sliders[i]->blockSignals(false);
            }
        }

        // 5. Отправляем команду роботу (используя ваш существующий метод)
        start_smooth_move(target_joints);

        ui->label_status->setText("Статус: Решение найдено, движение...");
    } else {
        ui->label_status->setText("Статус: Точка недостижима!");
    }
}


void MainWindow::on_pushButton_Estop_clicked()
{
    traj_timer_->stop();
}

void MainWindow::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {

    // Опять используем invokeMethod, так как это поток ROS!
    QMetaObject::invokeMethod(this, [this, msg](){
        // Предположим раскладку для Logitech (Mode выключен):
        // Левый стик: оси 0 (X) и 1 (Y)
        // Правый стик: оси 3 (X) и 4 (Y)
        // В ROS оси стиков обычно в диапазоне [-1.0, 1.0]

        std::vector<double> target = current_joint_values_;

        // Управляем суставами 1-4 с помощью двух стиков
        target[0] += msg->axes[0] * joy_sensitivity; // Поворот основания
        target[1] += msg->axes[1] * joy_sensitivity; // Плечо
        target[2] += msg->axes[4] * joy_sensitivity; // Локоть
        target[3] += msg->axes[3] * joy_sensitivity; // Кисть (pitch)

        // Кнопки (L1/R1) для управления 5-м суставом (поворот кисти)
        if (msg->buttons[4]) target[4] += joy_sensitivity; // L1
        if (msg->buttons[5]) target[4] -= joy_sensitivity; // R1

        // Запускаем плавное движение к вычисленной точке
        start_smooth_move(target);

        // --- Управление захватом ---
        // Кнопка 0 (X / A) - Закрыть
        // Кнопка 1 (O / B) - Открыть
        bool gripper_changed = false;

        if (msg->buttons[0] == 1) {
            current_gripper_pos_ -= gripper_step;
            gripper_changed = true;
        } else if (msg->buttons[1] == 1) {
            current_gripper_pos_ += gripper_step;
            gripper_changed = true;
        }

        // Ограничиваем ход
        current_gripper_pos_ = std::max(0.0, std::min(gripper_max, current_gripper_pos_));

        if (gripper_changed) {
            auto g_msg = brics_actuator::msg::JointPositions();

            // Левый палец
            brics_actuator::msg::JointValue jv_l;
            jv_l.joint_uri = "gripper_finger_joint_l";
            jv_l.unit = "m";
            jv_l.value = current_gripper_pos_;
            g_msg.positions.push_back(jv_l);

            // Правый палец
            brics_actuator::msg::JointValue jv_r;
            jv_r.joint_uri = "gripper_finger_joint_r";
            jv_r.unit = "m";
            jv_r.value = current_gripper_pos_;
            g_msg.positions.push_back(jv_r);

            gripper_pub_->publish(g_msg);

            // Синхронизируем слайдер гриппера в UI (блокируя сигналы)
            ui->horizontalSlider_gripper->blockSignals(true);
            ui->horizontalSlider_gripper->setValue(static_cast<int>((current_gripper_pos_ / gripper_max) * 1000.0));
            ui->horizontalSlider_gripper->blockSignals(false);
        }
    });

}

