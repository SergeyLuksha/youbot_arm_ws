#include "../joint_regulator_config/mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(rclcpp::Node::SharedPtr n, QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , node(n)
{
    ui->setupUi(this);
    // 1. Заполняем ComboBox суставами
    ui->comboBox_joint->clear();
    for (int i = 0; i < joint_limits_.size(); ++i) {
        ui->comboBox_joint->addItem(joint_limits_[i].name);
    }

    // Связываем выбор в ComboBox со слотом
    connect(ui->comboBox_joint, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onJointSelected);

    // Связываем нажатие кнопки "Задать"
    connect(ui->pushButton_set_value, &QPushButton::clicked,
            this, &MainWindow::onSetLinkClicked);

    // Инициализируем SpinBox для первого выбранного сустава по умолчанию
    onJointSelected(0);

    setupChart(); // Инициализация графика

    // Подписка на состояние суставов
    joint_state_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&MainWindow::joint_state_callback, this, std::placeholders::_1));

    arm_pub_ = node->create_publisher<brics_actuator::msg::JointPositions>(
        "/arm_1/arm_controller/position_command", 10);

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onJointSelected(int index) {
    if (index < 0 || index >= joint_limits_.size()) return;
    selected_joint_index_ = index;

    auto limit = joint_limits_[index];

    // Устанавливаем допустимый диапазон и текущее значение для SpinBox
    ui->doubleSpinBox_set_value->setRange(limit.min, limit.max);
    ui->doubleSpinBox_set_value->setSingleStep(0.01);
    ui->doubleSpinBox_set_value->setDecimals(3);
    ui->doubleSpinBox_set_value->setValue(current_joint_values_[index]);

    // Выводим лимиты в QLabel
    ui->label_limits->setText(QString("Ограничения: [%1, %2] рад")
                                  .arg(limit.min, 0, 'f', 2)
                                  .arg(limit.max, 0, 'f', 2));
}

void MainWindow::onSetLinkClicked() {
    // 1. Получаем новое значение из SpinBox для выбранного сустава
    double new_val = ui->doubleSpinBox_set_value->value();

    // 2. Обновляем целевое значение в локальном массиве
    target_positions_[selected_joint_index_] = new_val;

    // 3. Формируем сообщение BRICS для отправки прямой команды на все 5 суставов
    auto msg = brics_actuator::msg::JointPositions();

    std::vector<std::string> joint_names = {
        "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"
    };

    for (size_t i = 0; i < 5; ++i) {
        brics_actuator::msg::JointValue jv;
        jv.joint_uri = joint_names[i];
        jv.unit = "rad";
        // Если это выбранный сустав — берем новое значение из SpinBox,
        // для остальных суставов отправляем текущее реальное положение (или последнюю цель),
        // чтобы они не сбросились в ноль при отправке пакета.
        jv.value = target_positions_[i];
        msg.positions.push_back(jv);
    }

    // 4. Публикуем напрямую в топик управления позицией
    arm_pub_->publish(msg);

    // 5. Выводим статус в интерфейс
    ui->label_status->setText(QString("Статус: Прямая команда для %1 = %2 рад")
                                  .arg(QString::fromStdString(joint_names[selected_joint_index_]))
                                  .arg(new_val, 0, 'f', 3));

    // --- СТАРТ ПОСТРОЕНИЯ ГРАФИКА ---
    if (series_target_ && series_actual_) {
        series_target_->clear();
        series_actual_->clear();
    }
    chart_time_elapsed_ = 0.0;
    is_plotting_ = true; // Включаем запись точек в график

}

// Колбэк получения реального состояния робота
void MainWindow::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (msg->position.size() >= 5) {
        for (size_t i = 0; i < 5; ++i) {
            current_joint_values_[i] = msg->position[i];
        }

        // Вывод текущего положения выбранного звена в интерфейс
        double current_val = current_joint_values_[selected_joint_index_];
        double target_val = target_positions_[selected_joint_index_];
        ui->label_current_pose->setText(QString("Текущее положение: %1 рад")
                                            .arg(current_val, 0, 'f', 3));

        //--------------------------Работа с графиком--------------------------
        // Если активен режим построения графика
        if (is_plotting_ && series_target_ && series_actual_) {
            // Шаг времени (примерно соответствует частоте публикации /joint_states)
            chart_time_elapsed_ += 0.05;

            // Добавляем точки на серии
            series_target_->append(chart_time_elapsed_, target_val);
            series_actual_->append(chart_time_elapsed_, current_val);

            // Автоматическое масштабирование осей графика
            chart_->axes(Qt::Horizontal).first()->setRange(std::max(0.0, chart_time_elapsed_ - 10.0), chart_time_elapsed_ + 1.0);
            auto limit = joint_limits_[selected_joint_index_];
            chart_->axes(Qt::Vertical).first()->setRange(limit.min - 0.2, limit.max + 0.2);

            // Условие достижения цели (погрешность менее 0.01 радиана)
            if (std::abs(current_val - target_val) < 0.01) {
                is_plotting_ = false; // Останавливаем построение
                ui->label_status->setText(QString("Статус: Цель для сустава %1 успешно достигнута!")
                                              .arg(joint_limits_[selected_joint_index_].name));
            }
        }
        //---------------------------------------------------------------------
        // RCLCPP_INFO_STREAM(node->get_logger(), "Receive joint position "<<current_val);

        // Если идет процесс движения, пишем лог в CSV файл для выбранного сустава
        // if (is_logging_) {
        //     double elapsed_time = (node->now() - start_log_time_).seconds();
        //     double target_val = target_positions_[selected_joint_index_];
        //     log_stream_ << elapsed_time << "," << target_val << "," << current_val << "\n";
        // }
    }
}

void MainWindow::setupChart() {
    series_target_ = new QtCharts::QLineSeries();
    series_actual_ = new QtCharts::QLineSeries();

    series_target_->setName("Заданное (Target)");
    series_actual_->setName("Текущее (Actual)");

    chart_ = new QtCharts::QChart();
    chart_->addSeries(series_target_);
    chart_->addSeries(series_actual_);
    chart_->setTitle("График отслеживания положения сустава");
    chart_->createDefaultAxes();

    // Настройка осей (примерные начальные диапазоны)
    chart_->axes(Qt::Horizontal).first()->setTitleText("Время (с)");
    chart_->axes(Qt::Vertical).first()->setTitleText("Положение (рад)");

    chart_view_ = new QtCharts::QChartView(chart_);
    chart_view_->setRenderHint(QPainter::Antialiasing);

    // Добавляем график в layout, предусмотренный в .ui файле
    ui->horizontalLayout_chat->addWidget(chart_view_);
}
