#ifndef EKF_H
#define EKF_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

class EKF {
public:
    EKF() {
        // 初始化状态：x, y, z ,vx, vy, vz ,ax, ay ,az
        x = Eigen::VectorXf(9);
        x << 0, 0, 0, 0, 0, 0, 0, 0, -9.796;
        
        // 初始协方差矩阵
        P = Eigen::MatrixXf::Identity(9, 9) * 1000;

        dt=0.033;

        // 状态转移矩阵 F
        F = Eigen::MatrixXf(9, 9);
        F << 1, 0, 0, dt, 0, 0, 0, 0, 0,
             0, 1, 0, 0, dt, 0, 0, 0, 0,
             0, 0, 1, 0, 0, dt, 0, 0, 0.5*dt*dt,
             0, 0, 0, 1, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 1, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 1, 0, 0, dt, 
             0, 0, 0, 0, 0, 0, 1, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 1, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 1;

        // 观测矩阵 H
        H = Eigen::MatrixXf(3, 9);
        H << 1, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 1, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 1, 0, 0, 0, 0, 0, 0;

        // 观测噪声协方差 R
        R = Eigen::MatrixXf(3, 3);
        // R = Eigen::MatrixXf.onnx /path/to/image.jpg
        // 请确保 InferCoreYolov8Wrapper 类已经正确实现了模型加载、推理和后处理的功能，并(3, 3);
        R << 2, 0, 0,
             0, 2, 0,
             0, 0, 2;

        // 过程噪声协方差 Q
        Q = Eigen::MatrixXf(9, 9);
        Q << 0.1, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0.1, 0, 0, 0, 0, 0, 0, 0,
             0, 0, 0.1, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0.1, 0, 0, 0, 0, 0,
             0, 0, 0, 0, 0.1, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0.1, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0.1, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0.1, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 0.1;
    }

    void predict() {
        // 根据当前状态调整状态转移矩阵 F
        F(3, 3) = 1 - 0.000175 * std::abs(x(3));
        F(4, 4) = 1 - 0.00006 * std::abs(x(4));
        F(5, 5) = 1 - 0.000175 * std::abs(x(5));
        //F(1, 7) = 0.5*dt*dt;

        // 状态预测
        x = F * x;

        // 协方差预测
        P = F * P * F.transpose() + Q;
    }

    void update(const Eigen::VectorXf &z) {
        // 计算卡尔曼增益
        Eigen::MatrixXf Ht = H.transpose();
        Eigen::MatrixXf S = H * P * Ht + R;
        Eigen::MatrixXf K = P * Ht * S.inverse();

        // 更新状态估计
        Eigen::VectorXf y = z - H * x;
        x = x + K * y;

        // 更新协方差矩阵
        int size = x.size();
        P = (Eigen::MatrixXf::Identity(size, size) - K * H) * P;
    }
    void set_state(float x, float y, float z,float v_x,float v_y,float v_z)
    {
        this->x << x,y,z,v_x,v_y,v_z,0,0,-9.796;
    }

    void set_Ft(float t)
    {
        dt=t;
    }
    void clear()
    {
        x << 0,0,0,0,0,0,0,0,-9.796;
    }

    // 获取当前状态估计
    Eigen::VectorXf getState() {
        return x;
    }

private:
    Eigen::VectorXf x;        // 状态向量 [x, y, vx, vy, ax, ay]
    Eigen::MatrixXf P;        // 协方差矩阵
    Eigen::MatrixXf F;        // 状态转移矩阵
    Eigen::MatrixXf H;        // 观测矩阵
    Eigen::MatrixXf R;        // 观测噪声协方差矩阵
    Eigen::MatrixXf Q;        // 过程噪声协方差矩阵
    float dt;
};

class cauclate_EKF
{
public:
    cauclate_EKF()
    {
        result_point<<0,0,0;
    }
    void update_point(MyKalmanFilter& kf)
    {
        ekf.set_state(kf.get_message()[0],kf.get_message()[1],kf.get_message()[2],kf.get_message()[3],kf.get_message()[4],kf.get_message()[5]);
        ekf.set_Ft(kf.get_dt());
        if(kf.pos_init()&&kf.speed_init())
        {
            for(int i=0;i<1000;i++)
            {
                ekf.predict();
                Eigen::VectorXf predicted_point=ekf.getState();
                if(predicted_point[2]<-0.875)
                {
                    result_point<<predicted_point[0],predicted_point[1],predicted_point[2];
                    break;
                }
                else
                {
                    result_point<<predicted_point[0],predicted_point[1],predicted_point[2];
                }
            }
        }
    }
    Eigen::Vector3f getresult()
    {
        return result_point;
    }

    void clear()
    {
        ekf.clear();
        result_point<<0,0,0;
    }
private:
    Eigen::Vector3f result_point;
    EKF ekf;
};

#endif