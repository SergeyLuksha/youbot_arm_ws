#include "../include/mainwindow.h"
#include "ui_mainwindow.h"
#include <cmath>

MainWindow::MainWindow(rclcpp::Node::SharedPtr n, QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , node(n)
{
    ui->setupUi(this);

    // 2. Кнопка "Повернуть" изначально неактивна
    ui->pushButton_move->setEnabled(false);

    // 3. Подключаем кнопки к слотам
    connect(ui->pushButton_calculate, &QPushButton::clicked, this, &MainWindow::on_pushButton_calculate_clicked);
    connect(ui->pushButton_move, &QPushButton::clicked, this, &MainWindow::on_pushButton_move_clicked);

    // Инициализация таймера траектории
    traj_execution_timer_ = new QTimer(this);
    connect(traj_execution_timer_, &QTimer::timeout, this, &MainWindow::updateTrajectoryExecution);

    // Кнопка выполнения изначально неактивна, пока траектория не рассчитана
    ui->pushButton_execute_traj->setEnabled(false);

    // Подключаем новые кнопки траектории
    connect(ui->pushButton_calculate_traj, &QPushButton::clicked, this, &MainWindow::on_pushButton_calculate_traj_clicked);
    connect(ui->pushButton_execute_traj, &QPushButton::clicked, this, &MainWindow::on_pushButton_execute_traj_clicked);

    // Подключаем новые кнопки управления с джойстика и базового положения
    connect(ui->pushButton_joy_control_enable, &QPushButton::clicked, this, &MainWindow::on_pushButton_joy_control_enable_clicked);
    connect(ui->pushButton_base_position, &QPushButton::clicked, this, &MainWindow::on_pushButton_base_position_clicked);

    ui->pushButton_joy_control_enable->setCheckable(true);
    ui->pushButton_joy_control_enable->setText("Включить управление с джойстика");


    // Откладываем инициализацию KDL на 1 секунду, давая ROS 2 время на запуск узлов
    QTimer::singleShot(1000, this, &MainWindow::initKDLChain);

    // 1. Инициализируем паблишер для отправки команд на манипулятор youBot
    arm_pub_ = node->create_publisher<brics_actuator::msg::JointPositions>(
        "/arm_1/arm_controller/position_command", 10);
    // Подписка на текущее состояние суставов робота
    joint_state_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&MainWindow::joint_state_callback, this, std::placeholders::_1));
    gripper_pub_ = node->create_publisher<brics_actuator::msg::JointPositions>(
        "/arm_1/gripper_controller/position_command", 10);
    joy_sub_ = node->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&MainWindow::joy_callback, this, std::placeholders::_1));
}


MainWindow::~MainWindow()
{
    delete ui;
}

// Функция расчёта прямой кинематики через матрицы преобразования Eigen для 5-DOF youBot  --
//ПРОВЕРЕНА и НАСТРОЕНА
bool MainWindow::computeForwardKinematics(const std::vector<double>& joints, Eigen::Matrix4d& T_result) {
    if (joints.size() < 5) return false;

    // «Сырые» углы из топика /joint_states
    double raw_q1 = joints[0];
    double raw_q2 = joints[1];
    double raw_q3 = joints[2];
    double raw_q4 = joints[3];
    double raw_q5 = joints[4];

    // Геометрические параметры манипулятора KUKA youBot (в метрах)
    double d1 = 0.147; // Высота базы до 2-го сустава
    double a2 = 0.155; // Длина плеча
    double a3 = 0.135; // Длина предплечья
    double d5 = 0.218; // Полный вылет до кончика захвата

    // Смещения (offsets) и знаки для перевода сырых углов ROS в геометрические углы сочленений:
    // У youBot суставы 2 и 3 имеют конструктивные смещения относительно вертикали/горизонтали
    double q1 = -raw_q1 + 2.9;
    double q2 = raw_q2 - 1.134 - M_PI_2;   // Компенсация начального угла плеча (~65°)
    double q3 = raw_q3 + 2.500;   // Компенсация локтевого сустава
    double q4 = raw_q4 - 1.745 + M_PI_2;   // Компенсация запястья
    double q5 = -raw_q5 + 2.9;
    // RCLCPP_INFO_STREAM(node->get_logger(), "Angles : "<<q1 <<" : "<<q2<<" : "<<q3<<" : "<<q4<<" : "<<q5);

    //Итоговые поворот и перемещение
    Eigen::Matrix3d R_final = Eigen::Matrix3d::Identity();
    Eigen::Vector3d P_final = Eigen::Vector3d::Zero();

    // --- 1. Звено 1 (Основание) ---
    Eigen::Matrix3d R1;
    R1 << cos(q1), -sin(q1), 0,
        sin(q1),  cos(q1), 0,
        0,        0, 1;
    // Локальный вектор смещения базы по оси Z на высоту d1
    Eigen::Vector3d p1_local(0.024, 0.0, d1);
    R_final = R_final * R1;
    P_final = P_final + R_final * p1_local;


    // --- 2. Звено 2 (Плечо) ---
    Eigen::Matrix3d R2;
    R2 << cos(q2), 0,  sin(q2),
        0, 1,        0,
        -sin(q2), 0,  cos(q2);
    // Звено плеча длиной a2 лежит вдоль локальной оси X
    Eigen::Vector3d p2_local(a2, 0.0, 0.0);
    R_final = R_final * R2;
    P_final = P_final + R_final * p2_local;


    // --- 3. Звено 3 (Предплечье) ---
    Eigen::Matrix3d R3;
    R3 << cos(q3), 0,  sin(q3),
        0, 1,        0,
        -sin(q3), 0,  cos(q3);
    // Звено предплечья длиной a3 лежит вдоль локальной оси X
    Eigen::Vector3d p3_local(a3, 0.0, 0.0);
    R_final = R_final * R3;
    P_final = P_final + R_final * p3_local;

    // --- 4. Звено 4 (Запястье) ---
    Eigen::Matrix3d R4;
    R4 << cos(q4), 0,  sin(q4),
        0, 1,        0,
        -sin(q4), 0,  cos(q4);
    // Сустав запястья не имеет линейного смещения (вектор равен нулю)
    Eigen::Vector3d p4_local(0.0, 0.0, 0.0);
    R_final = R_final * R4;
    P_final = P_final + R_final * p4_local;


    // --- 5. Звено 5 (Кисть и вылет захвата) ---
    Eigen::Matrix3d R5;
    R5 << cos(q5), -sin(q5), 0,
        sin(q5),  cos(q5), 0,
        0,        0, 1;
    // Вылет d5 направлен вдоль локальной оси Z кисти
    Eigen::Vector3d p5_local(0.0, 0.0, d5);
    R_final = R_final * R5;
    P_final = P_final + R_final * p5_local;


    // Итоговое перемножение матриц трансформации 4x4
    // T_result = T5;
    T_result.block<3,3>(0, 0) = R_final;
    T_result.block<3,1>(0, 3) = P_final;
    return true;
}

bool MainWindow::computeForwardKinematicsKDL(const std::vector<double>& joints, Eigen::Matrix4d& T_result) {

    if (joints.size() < 5 || kdl_chain_.getNrOfJoints() == 0) return false;

    // Создаем решатель на основе уже готовой и сохраненной цепи
    KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain_);

    KDL::JntArray q(kdl_chain_.getNrOfJoints());
    for (size_t i = 0; i < joints.size() && i < q.rows(); ++i) {
        q(i) = joints[i];
    }

    KDL::Frame ee_frame;
    if (fk_solver.JntToCart(q, ee_frame) >= 0) {
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                T_result(r, c) = ee_frame(r, c);
            }
        }
        return true;
    }

    return false;
}

void MainWindow::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->position.size() < 5) return;

    current_joint_values_.resize(5);
    for (size_t i = 0; i < 5; ++i) {
        current_joint_values_[i] = msg->position[i];
    }

    std::vector<double> joints = {
        msg->position[0],
        msg->position[1],
        msg->position[2],
        msg->position[3],
        msg->position[4]
    };

    Eigen::Matrix4d T_eigen = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_kdl = Eigen::Matrix4d::Identity();

    // Выполняем расчёт двумя методами
    bool eigen_ok = computeForwardKinematics(joints, T_eigen);
    bool kdl_ok   = computeForwardKinematicsKDL(joints, T_kdl);

    if (eigen_ok && kdl_ok) {
        // 1. Извлекаем координаты ручного метода (Eigen)
        double x_e = T_eigen(0, 3);
        double y_e = T_eigen(1, 3);
        double z_e = T_eigen(2, 3);

        // 2. Извлекаем координаты метода KDL
        double x_k = T_kdl(0, 3);
        double y_k = T_kdl(1, 3);
        double z_k = T_kdl(2, 3);

        // 3. Считаем абсолютную погрешность (разницу) между методами
        double diff_x = std::abs(x_e - x_k);
        double diff_y = std::abs(y_e - y_k);
        double diff_z = std::abs(z_e - z_k);

        // Извлекаем ориентацию (Roll, Pitch, Yaw) для ручного метода (для примера)
        Eigen::Vector3d rpy_e = T_eigen.block<3,3>(0,0).eulerAngles(0, 1, 2);

        // Формируем подробный отчет для QLabel
        QString pose_text = QString(
                                "=== СРАВНЕНИЕ FK (Eigen vs KDL) ===\n"
                                "Ручной (Eigen) -> X: %1, Y: %2, Z: %3 м\n"
                                "KDL (URDF)    -> X: %4, Y: %5, Z: %6 м\n"
                                "-------------------------------------\n"
                                "Разница (dX, dY, dZ): %7, %8, %9 м\n"
                                "Ориентация RPY (Eigen): [%10, %11, %12]"
                                )
                                .arg(x_e, 0, 'f', 4).arg(y_e, 0, 'f', 4).arg(z_e, 0, 'f', 4)
                                .arg(x_k, 0, 'f', 4).arg(y_k, 0, 'f', 4).arg(z_k, 0, 'f', 4)
                                .arg(diff_x, 0, 'e', 2).arg(diff_y, 0, 'e', 2).arg(diff_z, 0, 'e', 2) // экспоненциальный формат для малых погрешностей
                                .arg(rpy_e[0], 0, 'f', 2).arg(rpy_e[1], 0, 'f', 2).arg(rpy_e[2], 0, 'f', 2);

        ui->label_current_pose->setText(pose_text);
        if (!initial_spinbox_set) {
            ui->doubleSpinBox_target_x->setValue(x_k);
            ui->doubleSpinBox_target_y->setValue(y_k);
            ui->doubleSpinBox_target_z->setValue(z_k);

            ui->doubleSpinBox_traj_target_x->setValue(x_k);
            ui->doubleSpinBox_traj_target_y->setValue(y_k);
            ui->doubleSpinBox_traj_target_z->setValue(z_k);
            initial_spinbox_set = true;
        }
    }
    else if (eigen_ok) {
        // Если KDL не смог инициализироваться, выводим хотя бы ручной метод
        double x = T_eigen(0, 3);
        double y = T_eigen(1, 3);
        double z = T_eigen(2, 3);
        QString pose_text = QString("Ручной метод (Eigen) — X: %1, Y: %2, Z: %3 м\n(KDL метод не отвечает)")
                                .arg(x, 0, 'f', 3).arg(y, 0, 'f', 3).arg(z, 0, 'f', 3);
        ui->label_current_pose->setText(pose_text);


    }


}

bool MainWindow::initKDLChain() {
    std::string robot_desc_string;

    // Используем AsyncParametersClient для получения параметра из robot_state_publisher
    auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "robot_state_publisher");

    if (!parameters_client->wait_for_service(std::chrono::seconds(3))) {
        RCLCPP_ERROR(node->get_logger(), "Service /robot_state_publisher/get_parameters not available during init!");
        return false;
    }

    auto future = parameters_client->get_parameters({"robot_description"});
    if (future.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
        try {
            auto results = future.get();
            if (!results.empty()) {
                robot_desc_string = results[0].as_string();
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node->get_logger(), "Exception while getting robot_description: %s", e.what());
            return false;
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "Timeout waiting for robot_description parameter in constructor!");
        return false;
    }

    if (robot_desc_string.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Retrieved robot_description string is empty!");
        return false;
    }

    // Парсим URDF и собираем KDL дерево
    urdf::Model robot_model;
    if (!robot_model.initString(robot_desc_string)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to parse URDF string.");
        return false;
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(robot_model, tree)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to construct KDL tree.");
        return false;
    }

    if (!tree.getChain("arm_link_0", "gripper_palm_link", kdl_chain_)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to get KDL chain from arm_link_0 to arm_link_5.");
        return false;
    }

    return true;
}

bool MainWindow::computeInverseKinematicsManual(double x, double y, double z, std::vector<double>& joints_result) {
    joints_result.assign(5, 0.0);

    // Геометрические параметры манипулятора KUKA youBot (в метрах)
    double d1 = 0.147;
    double a2 = 0.155;
    double a3 = 0.135;
    double d5 = 0.218;
    double base_offset_x = 0.024;

    // 1. Угол основания (q1)
    double x_adjusted = x - base_offset_x;
    double raw_q1 = std::atan2(y, x_adjusted);
    double q1 = 2.9 - raw_q1;

    // 2. Радиус и эффективная высота
    double R = std::sqrt(x_adjusted * x_adjusted + y * y);
    double Z_eff = z - d1;

    // Фиксированный угол наклона захвата (захват направлен перпендикулярно вниз)
    double target_pitch = M_PI_2;

    double xw = R - d5 * std::sin(target_pitch);
    double zw = Z_eff - d5 * std::cos(target_pitch);

    // 3. Теорема косинусов для плеча (a2) и предплечья (a3)
    double D_sq = xw * xw + zw * zw;
    double cos_q3 = (D_sq - (a2 * a2 + a3 * a3)) / (2.0 * a2 * a3);

    // Проверка на достижимость точки
    if (cos_q3 < -1.0 || cos_q3 > 1.0) return false;

    double q3_geom = std::acos(cos_q3);
    double q3 = -q3_geom; // Конфигурация "локтем вверх"

    double beta = std::atan2(zw, xw);
    double alpha = std::atan2(a3 * std::sin(q3_geom), a2 + a3 * std::cos(q3_geom));
    double q2 = beta - alpha;

    // 4. Запястье и кисть (q4, q5)
    double q4 = target_pitch - (q2 + q3);
    double q5 = 0.0;

    joints_result = {q1, q2, q3, q4, q5};
    RCLCPP_INFO_STREAM(node->get_logger(), "Result: "<<q1<<" : "<<q2<<" : "<<q3<<" : "<<q4<<" : "<<q5);

    // 5. Проверка физических лимитов суставов KUKA youBot из URDF:
    // arm_joint_1: [0, 5.899]
    // arm_joint_2: [0, 2.705]
    // arm_joint_3: [-5.183, 0]
    // arm_joint_4: [0, 3.577]
    // arm_joint_5: [0, 5.846]
    if (q1 < 0.0 || q1 > 5.899) return false;
    if (q2 < 0.0 || q2 > 2.705) return false;
    if (q3 < -5.183 || q3 > 0.0) return false;
    if (q4 < 0.0 || q4 > 3.577) return false;
    if (q5 < 0.0 || q5 > 5.846) return false;

    return true;
}

void MainWindow::on_pushButton_calculate_clicked() {
    double x = ui->doubleSpinBox_target_x->value();
    double y = ui->doubleSpinBox_target_y->value();
    double z = ui->doubleSpinBox_target_z->value();

    std::vector<double> ik_joints;

    // Вызываем KDL решатель обратной кинематики
    if (computeInverseKinematicsKDL(x, y, z, ik_joints)) {
        calculated_ik_joints_ = ik_joints;
        is_ik_valid_ = true;

        QString result_str = QString(
                                 "УСПЕХ (KDL IK): Решение найдено!\n"
                                 "q1: %1 | q2: %2 | q3: %3\n"
                                 "q4: %4 | q5: %5 (рад)"
                                 )
                                 .arg(ik_joints[0], 0, 'f', 3)
                                 .arg(ik_joints[1], 0, 'f', 3)
                                 .arg(ik_joints[2], 0, 'f', 3)
                                 .arg(ik_joints[3], 0, 'f', 3)
                                 .arg(ik_joints[4], 0, 'f', 3);

        ui->label_IK_result->setText(result_str);
        ui->pushButton_move->setEnabled(true);
    } else {
        is_ik_valid_ = false;
        calculated_ik_joints_.clear();
        ui->label_IK_result->setText("ОШИБКА: Точка вне досягаемости KDL или нарушает лимиты!");
        ui->pushButton_move->setEnabled(false);
    }
}

void MainWindow::on_pushButton_move_clicked() {
    if (!is_ik_valid_ || calculated_ik_joints_.size() < 5) return;

    auto msg = brics_actuator::msg::JointPositions();
    std::vector<std::string> joint_names = {
        "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"
    };

    for (size_t i = 0; i < 5; ++i) {
        brics_actuator::msg::JointValue jv;
        jv.joint_uri = joint_names[i];
        jv.unit = "rad";
        jv.value = calculated_ik_joints_[i]; // Передаем углы напрямую из KDL IK
        msg.positions.push_back(jv);
    }

    if (arm_pub_) {
        arm_pub_->publish(msg);
        ui->label_IK_result->setText(ui->label_IK_result->text() + "\n[Команда отправлена роботу]");
    }
}

bool MainWindow::computeInverseKinematicsKDL(double x, double y, double z, std::vector<double>& joints_result, const std::vector<double>& seed_joints) {
    if (kdl_chain_.getNrOfJoints() == 0) return false;

    // Создаем LMA решатель с настроенной точностью и числом итераций
    KDL::ChainIkSolverPos_LMA ik_solver(kdl_chain_, 1e-3, 1500);

    KDL::JntArray q_init(kdl_chain_.getNrOfJoints());

    // ЕСЛИ переданы внешние начальные условия (например, предыдущая точка траектории) — используем их.
    // Иначе берем текущие реальные углы робота (для простого расчета в точку).
    if (!seed_joints.empty() && seed_joints.size() >= q_init.rows()) {
        for (size_t i = 0; i < q_init.rows(); ++i) {
            q_init(i) = seed_joints[i];
        }
    } else {
        for (size_t i = 0; i < q_init.rows() && i < current_joint_values_.size(); ++i) {
            q_init(i) = current_joint_values_[i];
        }
    }

    // Вычисляем ориентацию на основе выбранного начального приближения
    KDL::ChainFkSolverPos_recursive temp_fk(kdl_chain_);
    KDL::Frame current_frame;
    temp_fk.JntToCart(q_init, current_frame);

    KDL::Frame target_frame(current_frame.M, KDL::Vector(x, y, z));
    KDL::JntArray q_out(kdl_chain_.getNrOfJoints());

    int status = ik_solver.CartToJnt(q_init, target_frame, q_out);

    // Защита от локальных минимумов (небольшой случайный шум при неудаче)
    if (status < 0) {
        for (size_t i = 0; i < q_init.rows(); ++i) {
            double noise = ((double)rand() / RAND_MAX - 0.5) * 0.2;
            q_init(i) += noise;
        }
        status = ik_solver.CartToJnt(q_init, target_frame, q_out);
    }

    if (status >= 0) {
        joints_result.clear();
        for (size_t i = 0; i < q_out.rows(); ++i) {
            joints_result.push_back(q_out(i));
        }

        // Проверка физических лимитов суставов KUKA youBot из URDF
        if (joints_result.size() >= 5) {
            if (joints_result[0] < 0.0 || joints_result[0] > 5.899) return false;
            if (joints_result[1] < 0.0 || joints_result[1] > 2.705) return false;
            if (joints_result[2] < -5.183 || joints_result[2] > 0.0) return false;
            if (joints_result[3] < 0.0 || joints_result[3] > 3.577) return false;
            if (joints_result[4] < 0.0 || joints_result[4] > 5.846) return false;
        }

        return true;
    }

    return false;
}
void MainWindow::on_pushButton_calculate_traj_clicked() {
    planned_trajectory_joints_.clear();
    ui->pushButton_execute_traj->setEnabled(false);

    if (kdl_chain_.getNrOfJoints() == 0) {
        RCLCPP_ERROR(node->get_logger(), "[TRAJ] Error: KDL chain is not initialized!");
        ui->label_traj_result->setText("Ошибка: KDL цепь не инициализирована!");
        return;
    }

    // 1. Получаем стартовую позицию траектории через FK
    KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain_);
    KDL::JntArray q_current(kdl_chain_.getNrOfJoints());
    for (size_t i = 0; i < q_current.rows() && i < current_joint_values_.size(); ++i) {
        q_current(i) = current_joint_values_[i];
    }
    KDL::Frame start_frame;
    if (fk_solver.JntToCart(q_current, start_frame) < 0) {
        RCLCPP_ERROR(node->get_logger(), "[TRAJ] Error: Failed to compute FK for start position!");
        ui->label_traj_result->setText("Ошибка: Не удалось получить текущую позицию FK!");
        return;
    }

    double x_start = start_frame.p.x();
    double y_start = start_frame.p.y();
    double z_start = start_frame.p.z();

    // 2. Целевая позиция из UI
    double x_target = ui->doubleSpinBox_traj_target_x->value();
    double y_target = ui->doubleSpinBox_traj_target_y->value();
    double z_target = ui->doubleSpinBox_traj_target_z->value();

    RCLCPP_INFO_STREAM(node->get_logger(), "[TRAJ] === Расчет траектории (по точкам отрезка) ===");
    RCLCPP_INFO_STREAM(node->get_logger(), "[TRAJ] Старт -> X: " << x_start << ", Y: " << y_start << ", Z: " << z_start);
    RCLCPP_INFO_STREAM(node->get_logger(), "[TRAJ] Цель  -> X: " << x_target << ", Y: " << y_target << ", Z: " << z_target);

    // 3. Вычисление шагов (максимум 0.01 м на шаг)
    double dx = x_target - x_start;
    double dy = y_target - y_start;
    double dz = z_target - z_start;
    double total_distance = std::sqrt(dx*dx + dy*dy + dz*dz);

    double max_step = 0.01; // 1 см
    int num_steps = static_cast<int>(std::ceil(total_distance / max_step));
    if (num_steps < 1) num_steps = 1;

    RCLCPP_INFO_STREAM(node->get_logger(), "[TRAJ] Дистанция: " << total_distance << " м | Шагов: " << num_steps);

    // 4. Цикл дискретизации: передаем решение предыдущей точки как seed для текущей
    std::vector<double> last_valid_joints = current_joint_values_;
    bool success = true;
    std::vector<std::vector<double>> temp_trajectory;

    for (int i = 1; i <= num_steps; ++i) {
        double factor = static_cast<double>(i) / num_steps;
        double x_i = x_start + dx * factor;
        double y_i = y_start + dy * factor;
        double z_i = z_start + dz * factor;

        std::vector<double> point_joints;

        // Вызываем IK, передавая last_valid_joints в качестве начального условия
        if (computeInverseKinematicsKDL(x_i, y_i, z_i, point_joints, last_valid_joints)) {
            temp_trajectory.push_back(point_joints);
            last_valid_joints = point_joints; // Запоминаем как базу для следующего шага
        } else {
            RCLCPP_WARN_STREAM(node->get_logger(), "[TRAJ] Ошибка IK на шаге " << i << "/" << num_steps
                                                                               << " (X: " << x_i << ", Y: " << y_i << ", Z: " << z_i << ")");
            success = false;
            break;
        }
    }

    // 5. Результат
    if (success) {
        planned_trajectory_joints_ = temp_trajectory;
        RCLCPP_INFO_STREAM(node->get_logger(), "[TRAJ] Траектория успешно построена. Точек: " << planned_trajectory_joints_.size());
        ui->label_traj_result->setText(QString("УСПЕХ: Траектория построена.\nКоличество точек: %1").arg(planned_trajectory_joints_.size()));
        ui->pushButton_execute_traj->setEnabled(true);
    } else {
        RCLCPP_ERROR_STREAM(node->get_logger(), "[TRAJ] Расчет траектории прерван.");
        ui->label_traj_result->setText("ОШИБКА: Решение IK не найдено на одном из шагов траектории");
        ui->pushButton_execute_traj->setEnabled(false);
    }
}
void MainWindow::on_pushButton_execute_traj_clicked() {
    if (planned_trajectory_joints_.empty()) return;

    current_traj_index_ = 0;
    // Запускаем таймер с интервалом 30 мс для плавной выдачи точек манипулятору
    traj_execution_timer_->start(100);
    ui->label_traj_result->setText(ui->label_traj_result->text() + "\n[Выполнение траектории...]");
    ui->pushButton_execute_traj->setEnabled(false);
}

void MainWindow::updateTrajectoryExecution() {
    if (current_traj_index_ < planned_trajectory_joints_.size()) {
        const auto& joints = planned_trajectory_joints_[current_traj_index_];

        auto msg = brics_actuator::msg::JointPositions();
        std::vector<std::string> joint_names = {
            "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"
        };

        for (size_t i = 0; i < 5; ++i) {
            brics_actuator::msg::JointValue jv;
            jv.joint_uri = joint_names[i];
            jv.unit = "rad";
            jv.value = joints[i];
            msg.positions.push_back(jv);
        }

        if (arm_pub_) {
            arm_pub_->publish(msg);
        }

        current_traj_index_++;
    } else {
        // Траектория полностью пройдена
        traj_execution_timer_->stop();
        ui->label_traj_result->setText(ui->label_traj_result->text() + "\n[Траектория успешно выполнена!]");
        ui->pushButton_execute_traj->setEnabled(true);
    }
}

void MainWindow::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    QMetaObject::invokeMethod(this, [this, msg](){
        // Если управление с джойстика не включено кнопкой — ничего не делаем
        if (!joystick_control_enabled_) {
            return;
        }

        // Статические переменные для хранения текущего декартова положения захвата и угла последнего сустава
        static double target_x = 0.0;
        static double target_y = 0.0;
        static double target_z = 0.0;
        static double target_joint_5 = 0.0;
        static bool target_initialized = false;

        // При первом вызове инициализируем целевые координаты и угол сустава из текущего положения робота
        if (!target_initialized) {
            KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain_);
            KDL::JntArray q_curr(kdl_chain_.getNrOfJoints());
            for (size_t i = 0; i < q_curr.rows() && i < current_joint_values_.size(); ++i) {
                q_curr(i) = current_joint_values_[i];
            }
            KDL::Frame curr_frame;
            if (fk_solver.JntToCart(q_curr, curr_frame) >= 0) {
                target_x = curr_frame.p.x();
                target_y = curr_frame.p.y();
                target_z = curr_frame.p.z();
                if (!current_joint_values_.empty() && current_joint_values_.size() >= 5) {
                    target_joint_5 = current_joint_values_[4];
                }
                target_initialized = true;
            } else {
                return; // Ждем, пока поступят валидные данные о суставах
            }
        }

        // Коэффициенты чувствительности
        double cart_sensitivity = 0.001; // Шаг перемещения по декартовым координатам (м)
        double joint_sensitivity = 0.05;  // Шаг вращения для последнего сочленения (рад)

        // Изменяем целевые декартовы координаты с помощью стиков геймпада
        target_x += msg->axes[1] * cart_sensitivity;
        target_y += msg->axes[0] * cart_sensitivity;
        target_z += msg->axes[4] * cart_sensitivity;

        // Управляем поворотом последнего сочленения (индекс 4 / arm_joint_5) с помощью оси с индексом 3
        target_joint_5 += msg->axes[3] * joint_sensitivity;

        // Вычисляем обратную кинематику для новой декартовой точки
        std::vector<double> ik_joints;
        if (computeInverseKinematicsKDL(target_x, target_y, target_z, ik_joints, current_joint_values_)) {
            // Применяем накопленный с джойстика угол для последнего сочленения
            if (ik_joints.size() >= 5) {
                ik_joints[4] = target_joint_5;
            }

            // Если решение найдено и лимиты соблюдены — отправляем команду роботу
            auto p_msg = brics_actuator::msg::JointPositions();
            std::vector<std::string> joint_names = {
                "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"
            };

            for (size_t i = 0; i < 5; ++i) {
                brics_actuator::msg::JointValue jv;
                jv.joint_uri = joint_names[i];
                jv.unit = "rad";
                jv.value = ik_joints[i];
                p_msg.positions.push_back(jv);
            }

            if (arm_pub_) {
                arm_pub_->publish(p_msg);
            }
        } else {
            // Если точка недостижима, откатываем изменения координат и угла сустава назад,
            // чтобы стики не "застревали" в недостижимой зоне
            target_x -= msg->axes[1] * cart_sensitivity;
            target_y -= msg->axes[0] * cart_sensitivity;
            target_z -= msg->axes[4] * cart_sensitivity;
            target_joint_5 -= msg->axes[3] * joint_sensitivity;
        }

        // --- Управление захватом ---
        bool gripper_changed = false;
        if (msg->buttons[0] == 1) {
            current_gripper_pos_ -= gripper_step;
            gripper_changed = true;
        } else if (msg->buttons[1] == 1) {
            current_gripper_pos_ += gripper_step;
            gripper_changed = true;
        }

        current_gripper_pos_ = std::max(0.0, std::min(gripper_max, current_gripper_pos_));

        if (gripper_changed) {
            auto g_msg = brics_actuator::msg::JointPositions();

            brics_actuator::msg::JointValue jv_l;
            jv_l.joint_uri = "gripper_finger_joint_l";
            jv_l.unit = "m";
            jv_l.value = current_gripper_pos_;
            g_msg.positions.push_back(jv_l);

            brics_actuator::msg::JointValue jv_r;
            jv_r.joint_uri = "gripper_finger_joint_r";
            jv_r.unit = "m";
            jv_r.value = current_gripper_pos_;
            g_msg.positions.push_back(jv_r);

            gripper_pub_->publish(g_msg);
        }
    });
}

void MainWindow::on_pushButton_joy_control_enable_clicked() {
    // Переключаем состояние в зависимости от того, нажата ли кнопка
    joystick_control_enabled_ = ui->pushButton_joy_control_enable->isChecked();

    if (joystick_control_enabled_) {
        ui->pushButton_joy_control_enable->setText("Выключить управление с джойстика");
        RCLCPP_INFO(node->get_logger(), "Управление с джойстика ВКЛЮЧЕНО.");
    } else {
        ui->pushButton_joy_control_enable->setText("Включить управление с джойстика");
        RCLCPP_INFO(node->get_logger(), "Управление с джойстика ВЫКЛЮЧЕНО.");
    }
}

void MainWindow::on_pushButton_base_position_clicked() {
    // Базовые позиции сочленений для управления: 3.626, 1.380, -1.090, 3.240, 2.820 рад
    std::vector<double> base_positions = {3.626, 1.380, -1.090, 3.240, 2.820};

    auto msg = brics_actuator::msg::JointPositions();
    std::vector<std::string> joint_names = {
        "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"
    };

    for (size_t i = 0; i < 5; ++i) {
        brics_actuator::msg::JointValue jv;
        jv.joint_uri = joint_names[i];
        jv.unit = "rad";
        jv.value = base_positions[i];
        msg.positions.push_back(jv);
    }

    if (arm_pub_) {
        arm_pub_->publish(msg);
        RCLCPP_INFO(node->get_logger(), "Команда отправлена: перевод манипулятора в базовое положение.");
        ui->label_traj_result->setText("Статус: Манипулятор переводится в базовое положение...");
    }
}

// RCLCPP_INFO_STREAM(node->get_logger(), "Result: "<<joints_result[0]
//                                                   <<" : "<<joints_result[1]
//                                                   <<" : "<<joints_result[2]
//                                                   <<" : "<<joints_result[3]
//                                                   <<" : "<<joints_result[4]);
