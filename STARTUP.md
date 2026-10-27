# KUKA youBot — Инструкция по развертыванию и запуску (ROS 2 Humble / Ubuntu 22.04)

## 1. Системные требования и зависимости ROS 2

Убедитесь, что у вас установлена ОС **Ubuntu 22.04** и фреймворк **ROS 2 Humble**. Установите необходимые пакеты для работы с URDF, джойстиком и сообщениями управления:

```bash
sudo apt update
sudo apt install ros-humble-control-msgs ros-humble-xacro ros-humble-joy

```

> **Важно по Python `setuptools`:**
> Избегайте установки пользовательских версий `setuptools` через `pip install --user setuptools` в локальное окружение `~/.local/lib/python3.10/...`, так как они вызывают ошибку совместимости `canonicalize_version() got an unexpected keyword argument 'strip_trailing_zero'` при сборке пакетов ROS 2. Используйте системную версию пакетного менеджера (`sudo apt install python3-setuptools`).

---

## 2. Подготовка сетевого интерфейса (EtherCAT)

Протокол EtherCAT работает на канальном уровне (L2) и конфликтует со службами DHCP.

1. Подключите робота кабелем Ethernet к компьютеру.
2. Узнайте имя вашего сетевого интерфейса (например, `enp5s1`):
```bash
ip link

```


3. Настройте сетевое соединение в системе: интерфейс должен быть активен, но **без назначенного IP-адреса** (режим *Link-Local Only / Ignore*).

---

## 3. Настройка конфигурации драйвера

В конфигурационном файле драйвера EtherCAT (обычно `youbot-ethercat.cfg` в пакете `youbot_driver` или в папке установки) укажите актуальное имя вашего сетевого интерфейса:

```ini
[EtherCAT]
EthernetDevice = enp5s1  # Замените на имя вашего интерфейса
EtherCATUpdateRate_[usec] = 2000

```

---

## 4. Настройка привилегий (Capabilities)

Для того чтобы драйвер мог напрямую работать с «сырыми» пакетами EtherCAT без прав суперпользователя (`sudo`), выдайте права исполняемому файлу:

```bash
sudo setcap cap_net_raw+ep ~/ROS2/youbot_arm_ws/install/youbot_driver_ros_interface/lib/youbot_driver_ros_interface/youbot_driver_ros_interface

```

Поскольку пакеты с `setcap` игнорируют переменную `LD_LIBRARY_PATH`, убедитесь, что системные библиотеки ROS 2 и ваши воркспейсы прописаны в конфигурации динамического линковщика Linux:

1. Отредактируйте (или создайте) файл:
```bash
sudo nano /etc/ld.so.conf.d/youbot_libs.conf

```


2. Добавьте пути к библиотекам:
```text
/home/ls/ROS2/youbot_arm_ws/install/brics_actuator/lib
/home/ls/ROS2/youbot_arm_ws/install/mir_interfaces/lib
/opt/ros/humble/lib

```


3. Обновите кэш линковщика:
```bash
sudo ldconfig

```



---

## 5. Сборка проекта

Выполняйте сборку из корневой директории вашего рабочего пространства:

```bash
cd ~/ROS2/youbot_arm_ws
colcon build --symlink-install

```

*Если требуется собрать только конкретный пакет (например, драйвер):*

```bash
colcon build --packages-select youbot_driver --symlink-install

```

---

## 6. Поэтапный запуск узлов

### Шаг 6.1. Активация окружения

В каждом новом терминале перед запуском команд активируйте окружение ROS 2 и вашего воркспейса:

```bash
source /opt/ros/humble/setup.bash
source ~/ROS2/youbot_arm_ws/install/setup.bash

```

### Шаг 6.2. Запуск низкоуровневого драйвера (EtherCAT)

Запустите лаунч-файл драйвера манипулятора:

```bash
ros2 launch youbot_driver_ros_interface youbot_driver.launch.py

```

*Успешный запуск:* в логах должно появиться обнаружение подчиненных устройств (slaves) шины.

### Шаг 6.3. Проверка топиков состояния

В новом терминале проверьте поступление данных с датчиков робота:

```bash
ros2 topic echo /joint_states

```

### Шаг 6.4. Запуск интерфейса управления (Qt GUI / ИК / Джойстик)

1. Запустите разработанное Qt-приложение (с поддержкой интерполяции траектории, ИК-решателя и подписчика на джойстик).
2. Для телеоперации с геймпада (Logitech) в отдельном терминале запустите стандартный узел `joy`:
```bash
ros2 run joy joy_node

```


3. Используйте оси стиков для плавного инкрементального перемещения суставов через топик `/arm_1/arm_controller/position_command` и кнопки для управления захватом.
