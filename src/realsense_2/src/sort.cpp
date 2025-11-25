#include "sort.h"
#include <iostream>
#include <vector>
#include <fstream>
#include <queue>

using namespace cv;

std::queue<double> q;

// void change_kalman(MyKalmanFilter& kalman_filter)
// {
//     // 从文件读取 JSON 对象
//     std::ifstream file2("/home/heyanjie/rc_admin_cpp/json/config_kalman.json");
//     json j2;
//     file2 >> j2;

//     // 从 JSON 对象转换为 Eigen 矩阵
//     MatrixXd matrix_P = json_to_matrix(j2["matrix_P"]);
//     MatrixXd matrix_R = json_to_matrix(j2["matrix_R"]);
//     MatrixXd matrix_Q = json_to_matrix(j2["matrix_Q"]);
//     kalman_filter.change_dt(j2["dt"]);
//     kalman_filter.P=matrix_P;
//     kalman_filter.R=matrix_R;
//     kalman_filter.Q=matrix_Q;
// }

Eigen::VectorXf cauclate_sort(BallPredictorInputType& input, MyKalmanFilter& kalman_filter)
{
    //change_kalman(kalman_filter);
    //VectorXd curr_state=kalman_filter.get_message();
    //if(!results.indices.empty())
    //{
    //    if(results.indices.size()==1)
    //    {
    //        cv::Point2d point1=results.boxes[results.indices[0]].tl();
    //        cv::Point2d point2=results.boxes[results.indices[0]].br();
    //        float detect_point[2] = {float((point1.x+point2.x)/2), float((point1.y+point2.y)/2)}; // 改为浮点数
    //        float distance = depth_frame.get_distance(detect_point[0], detect_point[1]);
    //        float cam_coord[3];  // 保持这个为数组类型
    //        rs2_deproject_pixel_to_point(cam_coord, &depth_intrin, detect_point, distance);
//
    //        if(kalman_filter.is_init())
    //        {
    //            double detect_distance=sqrt(pow(cam_coord[0]-kalman_filter.get_message()[0],2)+pow(cam_coord[1]-kalman_filter.get_message()[1],2)+pow(cam_coord[2]-kalman_filter.get_message()[2],2));
    //            if(detect_distance<3)
    //            {
    //                if(cam_coord[2]!=0)
    //                {
    //                    VectorXd predicted_state=kalman_filter.predict();
    //                    Vector3d measure_state=Vector3d(cam_coord[0],cam_coord[1],cam_coord[2]);
    //                    kalman_filter.update(measure_state);
    //                    // Vector4d predicted_state_2D=kf_2D.predict();
    //                    // kf_2D.update(Vector2d((point1.x+point2.x)/2,(point1.y+point2.y)/2));
    //                    
    //                }
//
    //                else if(cam_coord[2]==0)
    //                {
    //                    VectorXd predicted_state=kalman_filter.predict();
    //                    // Vector4d predicted_state_2D=kf_2D.predict();
    //                }
    //            }
    //        }
    //        else
    //        {
    //            kalman_filter.change_state(cam_coord[0],cam_coord[1],cam_coord[2]);
    //            // kf_2D.change_state((point1.x+point2.x)/2,(point1.y+point2.y)/2);
    //        }
    //    }
    //    else if(results.indices.size()>1&&kalman_filter.is_init())
    //    {
    //        double min_Distance=10000000;
    //        Vector3d measure_state;
    //        // Vector2d measure_state_2D;
    //        for(int i=0;i<results.indices.size();i++)
    //        {
    //            cv::Point2d point1=results.boxes[results.indices[i]].tl();
    //            cv::Point2d point2=results.boxes[results.indices[i]].br();
    //            float detect_point[2] = {float((point1.x+point2.x)/2), float((point1.y+point2.y)/2)}; // 改为浮点数
    //            float distance = depth_frame.get_distance(detect_point[0], detect_point[1]);
    //            float cam_coord[3];  // 保持这个为数组类型
    //            rs2_deproject_pixel_to_point(cam_coord, &depth_intrin, detect_point, distance);
//
    //            double detect_distance=sqrt(pow(cam_coord[0]-kalman_filter.get_message()[0],2)+pow(cam_coord[1]-kalman_filter.get_message()[1],2)+pow(cam_coord[2]-kalman_filter.get_message()[2],2));
    //            if(detect_distance<min_Distance&&detect_distance<3)
    //            {
    //                min_Distance=detect_distance;
    //                measure_state=Vector3d(cam_coord[0],cam_coord[1],cam_coord[2]);
    //                // measure_state_2D=Vector2d((point1.x+point2.x)/2,(point1.y+point2.y)/2);
    //            }           
    //        }
    //        if(min_Distance!=10000000)
    //        {
    //            // Vector4d predicted_state_2D=kf_2D.predict();
    //            // kf_2D.update(measure_state_2D);
    //            VectorXd predicted_state=kalman_filter.predict();
    //            kalman_filter.update(measure_state);
    //        }
    //        else
    //        {
    //            // kf_2D.predict();
    //            kalman_filter.predict();
    //        }
    //    }
    //}
    //else if(results.indices.empty()&&kalman_filter.is_init())
    //{
    //    VectorXd predected_state=kalman_filter.predict();
    //    // kf_2D.predict();
    //}
//
    ////将输出写入log.txt
    //if(kalman_filter.is_init())
    //{
    //    std::ofstream outfile("/home/heyanjie/rc_admin_cpp/src/log.txt",std::ios::app);
    //    if(!outfile)
    //    {
    //        std::cout<<"open log.txt failed"<<std::endl;
    //    }
    //    else
    //    {
    //        outfile<<"kalman_filter_current_state:"<<kalman_filter.get_message()<<std::endl;
    //        outfile.close();
    //    }
    //}
    //else
    //{
    //    std::ofstream outfile("/home/heyanjie/rc_admin_cpp/src/log.txt",std::ios::out | std::ios::trunc);
    //    if(!outfile)
    //    {
    //        std::cout<<"open log.txt failed"<<std::endl;
    //    }
    //    else
    //    {
    //        outfile<<"kalman_filter_current_state:"<<kalman_filter.get_message()<<std::endl;
    //        outfile.close();
    //    }
    //}
//
    //float reprojected_pixel[2]; // 存储重投影后的像素坐标
    //float cam_coord[3];  // 保持这个为数组类型
    //cam_coord[0] = kalman_filter.get_message()[0];
    //cam_coord[1] = kalman_filter.get_message()[1];
    //cam_coord[2] = kalman_filter.get_message()[2];
    //rs2_project_point_to_pixel(reprojected_pixel, &depth_intrin, cam_coord);  // 从相机坐标到像素坐标
    //cv::circle(frame,cv::Point(reprojected_pixel[0],reprojected_pixel[1]),5,cv::Scalar(0,255,0),-1);
    //return frame;
    if(q.size()==0||q.size()==1)
    {
        q.push(input.frame_time_stamp);
    }
    else
    {
        q.push(input.frame_time_stamp);
        q.pop();
        kalman_filter.change_dt((q.back()-q.front())/1000);
        printf("dt is changed into:%f\n",kalman_filter.get_dt());
    }
    cauclate_EKF list;
    if(!kalman_filter.pos_init())
    {
        if(kalman_filter.is_set<4)
        {   
            kalman_filter.is_set+=1;
            printf("-----------------------------------leaf init!-----------------------------------------\n");
            //break;
        }
        else
        {
            kalman_filter.change_state(input.abs_ball_crd(0),input.abs_ball_crd(1),input.abs_ball_crd(2));
            list.update_point(kalman_filter);
            printf("+++++++++++++++++exe_time:%f\n",(input.frame_time_stamp-kalman_filter.flash_time)/1000);
    
        }
    }
    else if(kalman_filter.pos_init()&&!kalman_filter.speed_init())
    {
        float temp[3]={input.abs_ball_crd(0),input.abs_ball_crd(1),input.abs_ball_crd(2)};
        printf("-------old_pos:%f  %f  %f\n",kalman_filter.get_message()[0],kalman_filter.get_message()[1],kalman_filter.get_message()[2]);
        printf("-------new_pos:%f  %f  %f\n",temp[0],temp[1],temp[2]);
        printf("-------dt:%f\n",kalman_filter.get_dt());
        float vx=(input.abs_ball_crd(0)-kalman_filter.get_message()[0])/kalman_filter.get_dt();
        float vy=(input.abs_ball_crd(1)-kalman_filter.get_message()[1])/kalman_filter.get_dt();
        float vz=2*(input.abs_ball_crd(2)-kalman_filter.get_message()[2])/kalman_filter.get_dt();
        printf("-------speed:%f  %f  %f\n",vx,vy,vz);
        list.clear();
        kalman_filter.change_state(temp[0],temp[1],temp[2],vx/1.2,vy/1.2,vz/1.2);
        list.update_point(kalman_filter);
        printf("+++++++++++++++++exe_time:%f\n",(input.frame_time_stamp-kalman_filter.flash_time)/1000);
    }
    else
    {
        // Eigen::VectorXf old_pos=kalman_filter.get_message();
        // Eigen::VectorXf new_pos=input.abs_ball_crd;
        // float vx=(input.abs_ball_crd(0)-kalman_filter.get_message()[0])/kalman_filter.get_dt();
        // float vy=(input.abs_ball_crd(1)-kalman_filter.get_message()[1])/kalman_filter.get_dt();
        // printf("************************getold:%f  %f  %f\n",kalman_filter.get_message()[0],kalman_filter.get_message()[1],kalman_filter.get_message()[2]);
        // printf("************************getnew:%f  %f  %f\n",input.abs_ball_crd(0),input.abs_ball_crd(1),input.abs_ball_crd(2));
        float old_vx=kalman_filter.get_message()[3];
        float old_vy=kalman_filter.get_message()[4];
        kalman_filter.predict();
        kalman_filter.update(input.abs_ball_crd);
        float new_vx=kalman_filter.get_message()[3];
        float new_vy=kalman_filter.get_message()[4];
        float threshold=0.5;
        if(abs(new_vx-old_vx)>threshold||abs(new_vy-old_vy)>threshold)
        {
            kalman_filter.clear(input.frame_time_stamp);
            kalman_filter.change_state(input.abs_ball_crd(0),input.abs_ball_crd(1),input.abs_ball_crd(2));
            list.clear();
            list.update_point(kalman_filter);
            printf("||||||||||||||||||||||vx or vy changed,has cleared");
        }
        else
        {
            //kalman_filter.predict();
            //kalman_filter.update(input.abs_ball_crd);
            list.clear();
            list.update_point(kalman_filter);
            printf("++++++++++++++++exe_time:%f\n",(input.frame_time_stamp-kalman_filter.flash_time)/1000);
        }
    }
    //printf("-------------------------%.3f",list.getresult());
    return list.getresult();
}
