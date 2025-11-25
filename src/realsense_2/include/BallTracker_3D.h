#ifndef KALMAN_H
#define KALMAN_H

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>

//#include <openvino/openvino.hpp> //openvino header file
#include <opencv2/opencv.hpp>    //opencv header file

using namespace Eigen;

class MyKalmanFilter {
public:
    MyKalmanFilter()
    {
        is_set=0;
        dt=0.033;
        // 初始化状态向量
        x << 0, 0, 0, 0, 0, 0 ,0;  // x, y, z, v_x, v_y, v_z
        
        // 初始化协方差矩阵
        P << 0.42528 , 0       , 0       , 2.7335 , 0      , 0      , 0      ,
             0       , 0.42528 , 0       , 0      , 2.7335 , 0      , 0      ,
             0       , 0       , 0.43261 , 0      , 0      , 4.7561 , 18.356 ,
             2.7335  , 0       , 0       , 777.9  , 0      , 0      , 0      ,
             0       , 2.7335  , 0       , 0      , 777.9  , 0      , 0      ,
             0       , 0       , 4.7561  , 0      , 0      , 1338   , 5244.3 ,
             0       , 0       , 18.356  , 0      , 0      , 5244.3 , 64777  ; 
        
        // 状态转移矩阵 (假设每个时间步长为 dt = 0.02)
        F << 1, 0, 0, dt, 0, 0, 0,
             0, 1, 0, 0, dt, 0, 0,
             0, 0, 1, 0, 0, dt, 0.5*dt*dt,
             0, 0, 0, 1, 0, 0, 0,
             0, 0, 0, 0, 1, 0, 0,
             0, 0, 0, 0, 0, 1, dt,
             0, 0, 0, 0, 0, 0, 1;
        
        // 观测矩阵 (假设我们只观测到位置 x, y, z)
        H = Matrix<float, 3, 7>::Zero();
        H(0, 0) = 1;
        H(1, 1) = 1;
        H(2, 2) = 1;
        
        // 观测噪声协方差矩阵 (这里假设观测噪声是 0.5)
        R = Matrix3f::Identity() * 0.5;
        
        // 过程噪声协方差矩阵 (这里假设过程噪声是 2.0)
        Q = MatrixXf::Identity(7, 7) * 2.0;
        Q(3,3)=100.0;
        Q(4,4)=100.0;
        Q(5,5)=100.0;
        Q(6,6)=1000.0;
    }

    VectorXf predict() {
            F(0,3)=dt;
            F(1,4)=dt;
            F(2,5)=dt;
            F(2,6)=0.5*dt*dt;
            F(5,6)=dt;
            //F(4,6)=dt;
            // 预测状态
            x = F * x;
            // 预测协方差
            P = F * P * F.transpose() + Q;
            return x;
    }

    VectorXf update(const Vector3f& measurement) {
        // 计算卡尔曼增益
        MatrixXf S = H * P * H.transpose() + R;
        MatrixXf K = P * H.transpose() * S.inverse();
        
        // 更新状态
        x = x + K * (measurement - H * x);
        
        // 更新协方差
        P = (MatrixXf::Identity(7, 7) - K * H) * P;
        return x;
    }

    VectorXf get_message() {
        return x;
    }

    void change_state(float x, float y, float z, float v_x = 0, float v_y = 0, float v_z = 0)
    {
        this->x << x, y, z, v_x, v_y, v_z ,0;
    }

    void clear(float time=0)
    {
        is_set=0;
        x << 0, 0, 0, 0, 0, 0 ,0;
        if(time!=0)
        {
            this->flash_time=time;
        }
    }

    int pos_init()
    {
        if(x(0)==0&&x(1)==0&&x(2)==0)
        {
            return 0;
        }
        else
        {
            return 1;
        }
    }

    int speed_init()
    {
        if(x(3)==0&&x(4)==0&&x(5)==0)
        {
            return 0;
        }
        else
        {
            return 1;
        }
    }
    float get_dt()
    {
        return dt;
    }

    void change_dt(float t)
    {
        dt=t;
    }

    void flash(float frame_time)
    {
        if(flash_time==0)
        {
            flash_time=frame_time;
            this->clear();
        }
        else
        {
            if((frame_time-flash_time)>2000)
            {
                this->clear();
                flash_time=frame_time;
                printf("============================clear!!!!\n");
            }
        }
    }
public:
    VectorXf x = VectorXf(7); // 状态向量 (x, y, z, v_x, v_y, v_z)
    MatrixXf P = MatrixXf(7, 7); // 协方差矩阵
    MatrixXf F = MatrixXf(7, 7); // 状态转移矩阵
    Matrix<float, 3, 7> H; // 观测矩阵, 修改为3x6，观测到3个维度的状态
    Matrix3f R; // 观测噪声协方差矩阵
    MatrixXf Q = MatrixXf(7, 7); // 过程噪声协方差矩阵
    float dt;
    int is_set;
    float flash_time=0;
};

#endif