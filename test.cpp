#include <iostream>
#include <math.h>
#include <Eigen/Core>
#include <ceres/ceres.h>
#include <Eigen/Geometry>
using namespace ceres;
// Eigen::Matrix<double,3,1> A(1.0,1.0,1.0);
// Eigen::Matrix<double,3,1> B(4.0,0.0,2.0);
// Eigen::Matrix<double,3,1> point_m(1.0,2.0,3.0);
// Eigen::Matrix<double,3,1> normal_m(1.0,1.0,1.0);
// Eigen::Matrix<double,3,1> center_m(1.0,0.0,0.0);
struct point2plant
{
    point2plant(Eigen::Matrix<double,3,1> normal,Eigen::Matrix<double,3,1> center,Eigen::Matrix<double,3,1>A,Eigen::Matrix<double,3,1>B):normal_(normal),
                                                                                                                                         center_(center),
                                                                                                                                         A_(A),
                                                                                                                                         B_(B)
    {
        unit_normal=normal_/normal_.norm();
    }
    template<typename T>
    bool operator()(const T* q,const T* t,T* resudial)const
    {
        // T r[3];
        // T t[3];
        // r[0]=T(trans[0]);
        // r[1]=T(trans[1]);
        // r[2]=T(trans[2]);
        // t[0]=T(trans[3]);
        // t[1]=T(trans[4]);
        // t[2]=T(trans[5]);
        Eigen::Quaternion<T>rot_q{q[3],q[0],q[1],q[2]};
        Eigen::Matrix<T,3,1>rot_t{t[0],t[1],t[2]};
        Eigen::Matrix<T,3,1>_A{T(A_.x()),T(A_.y()),T(A_.z())};
        Eigen::Matrix<T,3,1>_B{T(B_.x()),T(B_.y()),T(B_.z())};
        Eigen::Matrix<T,3,1>_normal{T(normal_.x()),T(normal_.y()),T(normal_.z())};
        Eigen::Matrix<T,3,1>_center{T(center_.x()),T(center_.y()),T(center_.z())};
        Eigen::Matrix<T,3,1> A1;
        A1=rot_q*_A+rot_t;
        Eigen::Matrix<T,3,1> B1;
        B1=rot_q*_B+rot_t;
        resudial[0]=abs((A1-_center).dot(unit_normal));
        resudial[1]=abs((B1-_center).dot(unit_normal));
        return 1;
     }
     const Eigen::Matrix<double,3,1> normal_,center_,A_,B_;
     Eigen::Matrix<double,3,1> unit_normal;
};

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    double initial_q[4]={0.0,0.0,0.0,0.0};
    double q[4]={1,0.0,0.0,0.0};
    double initial_t[3]={0.0,0.0,1.0};
    double t[3]={0.0,0.0,1.0};
    // ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
    ceres::LocalParameterization *q_parameterization =
    new ceres::EigenQuaternionParameterization();
    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);
    problem.AddParameterBlock(q, 4, q_parameterization);
    problem.AddParameterBlock(t, 3);
    Eigen::Matrix<double,3,1>normal(1,1,1);
    Eigen::Matrix<double,3,1>center(1,0,0);
    Eigen::Matrix<double,3,1>A(1,1,1);
    Eigen::Matrix<double,3,1>B(4,3,1);
    CostFunction* cost_function =new AutoDiffCostFunction<point2plant, 2, 4,3>(new point2plant(normal,center,A,B));
    problem.AddResidualBlock(cost_function,NULL, q,t);

  // Run the solver!
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "q : " << initial_q[0]<< " -> " << q[0] << "\n";
    std::cout << "q : " << initial_q[1]<< " -> " << q[1] << "\n";
    std::cout << "q : " << initial_q[2]<< " -> " << q[2] << "\n";
    std::cout << "q : " << initial_q[3]<< " -> " << q[3] << "\n";
    std::cout << "t : " << initial_t[0]<< " -> " << t[0] << "\n";
    std::cout << "t : " << initial_t[1]<< " -> " << t[1] << "\n";
    std::cout << "t : " << initial_t[2]<< " -> " << t[2] << "\n";
    return 0;

    // Eigen::Matrix3d R;
    // R<<1,0,0,
    //     0,1,0,
    //     0,0,1;
    // Eigen::Quaterniond q(R);
    // Eigen::Vector3d t(0,0,1);
    // std::cout<<"旋转矩阵计算结果"<<R*A+t<<std::endl;
    // std::cout<<"四元数计算结果"<<q*A+t<<std::endl;

}