#include <ocs2_mobile_manipulator/MobileManipulatorPreComputation.h>
#include <ocs2_mobile_manipulator/constraint/EndEffectorObtConstraint.h>
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <ros/subscriber.h>

#include <string>

namespace ocs2{
namespace mobile_manipulator{

            using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;

void obstacleCallback(const geometry_msgs::Point::ConstPtr& msg)  ;
float obs1_position[3]={0,0,0};
std::vector<std::tuple<scalar_t,vector3_t>> obstacle_positions_1;//
bool cheek_obs_cons=false;
EndEffectorObtConstraint::EndEffectorObtConstraint(std::vector<std::tuple<scalar_t,vector3_t>> obstacles,const EndEffectorKinematics<scalar_t>& endEffectorKinematics, const ReferenceManager& referenceManager,const scalar_t d_safe):
        StateConstraint(ConstraintOrder::Linear),   
        obstacle_positions_(obstacles),
        endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
        referenceManagerPtr_(&referenceManager),
        d_safe_(d_safe ){

    v_d_safe_<<d_safe,d_safe,d_safe;
    if (endEffectorKinematics.getIds().size() != 1) {
        throw std::runtime_error("[EndEffectorConstraint] endEffectorKinematics has wrong number of end effector IDs.");
    }

    pinocchioEEKinPtr_ = dynamic_cast<PinocchioEndEffectorKinematics*>(endEffectorKinematicsPtr_.get());
    if(bos_topic==true)
    {
        bos_topic=false;
    ::ros::NodeHandle obs_hd;  
     sub = obs_hd.subscribe("/obstacle_position", 1, &obstacleCallback);

    }
}

size_t EndEffectorObtConstraint::getNumConstraints(scalar_t time)const{ // 改为所有关节的避障

ROS_INFO("TEST1111111111111!!!!!!!!!!!!!!!!!!!");

    return obstacle_positions_.size();//只有position 约束
}
void obstacleCallback(const geometry_msgs::Point::ConstPtr& msg)  
{  

    // 在这里处理接收到的消息 
cheek_obs_cons=true;
obs1_position[0]=msg->x;
obs1_position[1]=msg->y;
obs1_position[2]=msg->z;
//  obstacle_positions_1.empty;
vector3_t obstacle_pos(obs1_position[0],obs1_position[1],obs1_position[2]);//位置
  std::tuple<scalar_t,vector3_t> obstacles{0.225,obstacle_pos};//半径
//    obstacle_positions_1.clear();
  obstacle_positions_1.push_back(obstacles);

}  
vector_t EndEffectorObtConstraint::getValue(scalar_t time,const vector_t& state,const PreComputation& preComputation)const{
    if(pinocchioEEKinPtr_!=nullptr){
        const auto& preCompMM=cast<MobileManipulatorPreComputation>(preComputation);
        pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());      
    }

// obstacle_positions_=obstacle_positions_1;

    //考虑障碍
    int i=0;
if(cheek_obs_cons==true)
{
                    // std::cout<<obstacle_positions_1.size()<<std::endl;
    vector_t constraint(obstacle_positions_1.size());

    for(auto obs:obstacle_positions_1){
            // std::cout<<obs<<std::endl;
        // ROS_INFO("TEST!!!!!!!!!!!!!!!!!!!");

        scalar_t radios_ob=std::get<0>(obs);
        Eigen::Matrix<scalar_t, 3, 1> obstacle_i=std::get<1>(obs);
        constraint(i)=(endEffectorKinematicsPtr_->getPosition(state).front()-obstacle_i).norm()-d_safe_-radios_ob;
        i++;
    }
        return constraint;

}
else{
        //设置constraints //假设是立方体约束 first step 指针对第一个障 所以下面长度为 1
        vector_t constraint(obstacle_positions_.size());
        for(auto obs:obstacle_positions_){
            // std::cout<<obs<<std::endl;
        // ROS_INFO("TEST!!!!!!!!!!!!!!!!!!!");

        scalar_t radios_ob=std::get<0>(obs);
        Eigen::Matrix<scalar_t, 3, 1> obstacle_i=std::get<1>(obs);
        constraint(i)=(endEffectorKinematicsPtr_->getPosition(state).front()-obstacle_i).norm()-d_safe_-radios_ob;
        i++;
    }
    return constraint;


}

}

VectorFunctionLinearApproximation EndEffectorObtConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation& preComputation) const{
    //hx 函数：||eepos-obspos||^2-d_safe^2 =(eepos_x-obspos_x)^2+(eepos_y-obspos_y)^2+(eepos_z-obspos_z)^2-d_safe^2
    //first oder derivative of state: 二范数的求导 结果应该是长度为state 所有关节角度的列向量 
    if(pinocchioEEKinPtr_!=nullptr){
        const auto& preCompMM=cast<MobileManipulatorPreComputation>(preComputation);
        pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());      
    }
    auto approximation= VectorFunctionLinearApproximation(obstacle_positions_.size(),state.rows(),0);
    const auto eePosition = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
    // vector_t input(6);
    // vector3_t state_dot=endEffectorKinematicsPtr_->getVelocity(state,input.setZero()).front();
    
    //获取雅可比的上三行
    matrix_t J_pos=eePosition.dfdx;//这里时不是要换成endEffectorKinematicsPtr_
    vector_t h_dot=J_pos.transpose()*(state.head<3>()-std::get<1>(obstacle_positions_.front()));
    Eigen::Matrix<scalar_t, 3, 1> obstacle_1=std::get<1>(obstacle_positions_.front());
    scalar_t norm_down=(endEffectorKinematicsPtr_->getPosition(state).front()-obstacle_1).norm();
    approximation.f=this->getValue(time,state,preComputation);
    //fill the dfdx matrix with obs_row x state_clo
    matrix_t obs_dfx(obstacle_positions_.size(),state.rows());
    for(int i=0;i<obstacle_positions_.size();i++){
        obs_dfx.row(i)=(h_dot*(1/norm_down)).transpose();
    }
    approximation.dfdx=obs_dfx;
    approximation.dfdu=approximation.dfdx;
    approximation.dfdu.setZero();
    // std::cout<<"obs!! linear approximaion \n"<<approximation.dfdx<<std::endl;
    return approximation;
}



}
}