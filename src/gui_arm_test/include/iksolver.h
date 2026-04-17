#ifndef IKSOLVER_H
#define IKSOLVER_H

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <vector>
#include <string>

class IKSolver {
public:
    IKSolver(const std::string& urdf_content);

    // Решение обратной задачи кинематики
    // x, y, z — координаты, r, p, y — ориентация в радианах
    bool solveIK(double x, double y, double z, double roll, double pitch, double yaw,
                 const std::vector<double>& current_joints, std::vector<double>& target_joints);
    // В public:
    bool solveFK(const std::vector<double>& joints, double &x, double &y, double &z);

private:
    KDL::Chain chain_;
    // Решатель Левенберга-Марквардта (хорошо подходит для 5-DOF)
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
};

#endif
