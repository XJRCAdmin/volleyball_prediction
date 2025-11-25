#include "pred.h"
#include <iostream>
#include <vector>
#include <fstream>
#include <queue>

using namespace cv;

std::queue<double> q;

Eigen::Vector3f cauclate_sort(BallPredictorInputType& input, MyKalmanFilter& kalman_filter)
{
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
    if(input.abs_ball_crd[2]<0.5)//当小于某个值时认为球已经掉落，结束预测
    {
        kalman_filter.over_time=input.frame_time_stamp;
        kalman_filter.clear(input.frame_time_stamp);
    }
    if(kalman_filter.over_time!=0&&(input.frame_time_stamp-kalman_filter.over_time<1000))//在一秒之内不接受信息
    {
        return kalman_filter.pre_pos;
    }
    else if(kalman_filter.over_time!=0&&(input.frame_time_stamp-kalman_filter.over_time>1000))
    {
        kalman_filter.over_time=0;
    }
    cauclate_EKF list;
    if(!kalman_filter.pos_init())
    {
        if(kalman_filter.is_set<4)
        {   
            kalman_filter.is_set+=1;
            printf("-----------------------------------leaf init!-----------------------------------------");
            //break;
        }
        else
        {
            kalman_filter.change_state(input.abs_ball_crd(0),input.abs_ball_crd(1),input.abs_ball_crd(2));
            kalman_filter.flash_time=input.frame_time_stamp;
            // list.update_point(kalman_filter);
            // printf("+++++++++++++++++exe_time:%f\n",(input.frame_time_stamp-kalman_filter.flash_time)/1000);
        }
    }
    else if(kalman_filter.pos_init()&&!kalman_filter.speed_init())
    {
        float temp[3]={input.abs_ball_crd(0),input.abs_ball_crd(1),input.abs_ball_crd(2)};
        // printf("-------old_pos:%f  %f  %f\n",kalman_filter.get_message()[0],kalman_filter.get_message()[1],kalman_filter.get_message()[2]);
        // printf("-------new_pos:%f  %f  %f\n",temp[0],temp[1],temp[2]);
        // printf("-------dt:%f\n",kalman_filter.get_dt());
        float vx=(input.abs_ball_crd(0)-kalman_filter.get_message()[0])/kalman_filter.get_dt();
        float vy=(input.abs_ball_crd(1)-kalman_filter.get_message()[1])/kalman_filter.get_dt();
        float vz=(input.abs_ball_crd(2)-kalman_filter.get_message()[2])/kalman_filter.get_dt();
        if(vz>0.1) //防止误差
        {
            kalman_filter.change_state(input.abs_ball_crd(0),input.abs_ball_crd(1),input.abs_ball_crd(2));
            //list.update_point(kalman_filter);
            //printf("+++++++++++++++++exe_time:%f\n",(input.frame_time_stamp-kalman_filter.flash_time)/1000);
            kalman_filter.flash_time=input.frame_time_stamp;
        }
        else
        {
            // kalman_filter.space+=1;
            // if(kalman_filter.space>3)//把误差分到4帧内
            // {
                printf("-------old_pos:%f  %f  %f\n",kalman_filter.get_message()[0],kalman_filter.get_message()[1],kalman_filter.get_message()[2]);
                printf("-------new_pos:%f  %f  %f\n",temp[0],temp[1],temp[2]);
                // printf("-------dt:%f\n",kalman_filter.get_dt());
                // printf("-------speed:%f  %f  %f\n",vx,vy,vz);
                list.clear();
                kalman_filter.change_state(temp[0],temp[1],temp[2],vx,vy,vz);
                list.update_point(kalman_filter);
                printf("+++++++++++++++++exe_time:%f\n",(input.frame_time_stamp-kalman_filter.flash_time)/1000);
            //}
        }
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
        float threshold=5;
        if(abs(new_vx-old_vx)>threshold||abs(new_vy-old_vy)>threshold)
        {
            kalman_filter.clear(input.frame_time_stamp);
            kalman_filter.change_state(input.abs_ball_crd(0),input.abs_ball_crd(1),input.abs_ball_crd(2));
            list.clear();
            list.update_point(kalman_filter);
            printf("|||||||||||||| ||||||||vx or vy changed,has cleared");
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
    //printf("-------old_pos:%f  %f  %f\n",kalman_filter.get_message()[0],kalman_filter.get_message()[1],kalman_filter.get_message()[2]);
    // printf("-------------------------old_pose:%.3f   %.3f     %.3f",list.getresult()[0],list.getresult()[1],list.getresult()[2]);
    if(kalman_filter.pre_pos[0]==0&&kalman_filter.pre_pos[1]==0&&kalman_filter.pre_pos[2]==0)
    {
        if(!(list.getresult()[0]==0&&list.getresult()[1]==0&&list.getresult()[2]==0))
        {
            kalman_filter.pre_pos=list.getresult();
        }
    }
    else
    {
         //printf("-------------------------old_pose:%.3f   %.3f     %.3f",list.getresult()[0],list.getresult()[1],list.getresult()[2]);
        if(abs(list.getresult()[0]-kalman_filter.pre_pos[0])<5.0&&abs(list.getresult()[1]-kalman_filter.pre_pos[1])<5.0)//当预测的点与上一次预测的点距离小于0.2时，认为预测的点是正确的
        {
            kalman_filter.pre_pos=list.getresult();
            // printf("-------------------------old_pose:%.3f   %.3f     %.3f",list.getresult()[0],list.getresult()[1],list.getresult()[2]);
        }
    }
   // printf("-------old_pos:%f  %f  %f\n",kalman_filter.get_message()[0],kalman_filter.get_message()[1],kalman_filter.get_message()[2]);
    return kalman_filter.pre_pos;
}
