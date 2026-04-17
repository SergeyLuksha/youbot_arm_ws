#include "iksolver.h"
#include <rclcpp/rclcpp.hpp>

IKSolver::IKSolver(const std::string& urdf_content) {
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(urdf_content, tree)) {
        RCLCPP_ERROR(rclcpp::get_logger("ik_solver"), "Failed to construct KDL tree");
        return;
    }

    // В ваших файлах база — это arm_link_0, конец — arm_link_5
    if (!tree.getChain("arm_link_0", "arm_link_5", chain_)) {
        RCLCPP_ERROR(rclcpp::get_logger("ik_solver"), "Failed to get KDL chain");
        return;
    }

    // Создаем решатель.
    // Для 5-DOF YouBot LMA (Levenberg-Marquardt) стабильнее, чем Newton-Raphson
    ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_);
}

bool IKSolver::solveIK(double x, double y, double z, double roll, double pitch, double yaw,
                       const std::vector<double>& current_joints, std::vector<double>& target_joints) {

    // 1. Подготовка целевого фрейма (Position + Orientation)
    KDL::Frame target_frame(KDL::Rotation::RPY(roll, pitch, yaw), KDL::Vector(x, y, z));

    // 2. Входные углы (начальное приближение — текущие углы робота)
    KDL::JntArray q_init(chain_.getNrOfJoints());
    for (size_t i = 0; i < current_joints.size(); i++) {
        q_init(i) = current_joints[i];
    }

    // 3. Выходные углы
    KDL::JntArray q_out(chain_.getNrOfJoints());

    // 4. Расчет
    int status = ik_solver_->CartToJnt(q_init, target_frame, q_out);

    if (status >= 0) {
        target_joints.clear();
        for (unsigned int i = 0; i < q_out.rows(); i++) {
            target_joints.push_back(q_out(i));
        }
        return true;
    }

    return false;
}

bool IKSolver::solveFK(const std::vector<double>& joints, double &x, double &y, double &z) {
    KDL::ChainFkSolverPos_recursive fk_solver(chain_);
    KDL::JntArray q(joints.size());
    for(size_t i=0; i<joints.size(); ++i) q(i) = joints[i];

    KDL::Frame ee_frame;
    if (fk_solver.JntToCart(q, ee_frame) >= 0) {
        x = ee_frame.p.x();
        y = ee_frame.p.y();
        z = ee_frame.p.z();
        return true;
    }
    return false;
}
