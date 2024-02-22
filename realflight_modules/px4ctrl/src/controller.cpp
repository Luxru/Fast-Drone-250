#include "controller.h"

using namespace std;

// 该函数用于将四元数转换为yaw角
// 原理：四元数q = [w, x, y, z]，则yaw角为atan2(2 * (x*y + w*z), w*w + x*x - y*y
// - z*z)
double LinearControl::fromQuaternion2yaw( Eigen::Quaterniond q ) {
    double yaw = atan2( 2 * ( q.x() * q.y() + q.w() * q.z() ), q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z() );
    return yaw;
}

LinearControl::LinearControl( Parameter_t& param ) : param_( param ) {
    resetThrustMapping();
}

/*
  compute u.thrust and u.q, controller gains and other parameters are in param_
*/
quadrotor_msgs::Px4ctrlDebug LinearControl::calculateControl( const Desired_State_t& des, 
                                                              const Odom_Data_t& odom, 
                                                              const Imu_Data_t& imu, 
                                                              Controller_Output_t& u ) {
    /* WRITE YOUR CODE HERE */
    // compute disired acceleration in world frame
    Eigen::Vector3d des_acc( 0.0, 0.0, 0.0 );
    // 此处的计算控制只可以控制无人机的位置和姿态，需要提供三轴的加速度
    Eigen::Vector3d Kp, Kv;
    Kp << param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2;
    Kv << param_.gain.Kv0, param_.gain.Kv1, param_.gain.Kv2;
    des_acc = des.a + Kv.asDiagonal() * ( des.v - odom.v ) + Kp.asDiagonal() * ( des.p - odom.p );
    //des_acc need to add gravity, because the thrust is the sum of gravity and des_acc
    des_acc += Eigen::Vector3d( 0, 0, param_.gra );

    u.thrust = computeDesiredCollectiveThrustSignal( des_acc );
    double roll, pitch, yaw, yaw_imu;
    double yaw_odom = fromQuaternion2yaw( odom.q );
    double sin      = std::sin( yaw_odom );
    double cos      = std::cos( yaw_odom );
    roll            = ( des_acc( 0 ) * sin - des_acc( 1 ) * cos ) / param_.gra;
    pitch           = ( des_acc( 0 ) * cos + des_acc( 1 ) * sin ) / param_.gra;
    // yaw = fromQuaternion2yaw(des.q);
    yaw_imu = fromQuaternion2yaw( imu.q );
    // Eigen::Quaterniond q = Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())
    //   * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())
    //   * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY());
    /* 
      Odometry的pose定义为前x，左y，上z，飞机机头指向x轴正向，油门拉力方向为z轴正向，一定要严格对齐！
      Odometry的q 是将 wRb，R*body => world(体坐标系下的表示)
      u.q 是 期望的机体姿态
    */
    Eigen::Quaterniond q = Eigen::AngleAxisd( des.yaw, Eigen::Vector3d::UnitZ() ) * 
    Eigen::AngleAxisd( pitch, Eigen::Vector3d::UnitY() ) * 
    Eigen::AngleAxisd( roll, Eigen::Vector3d::UnitX() );
    u.q                  = imu.q * odom.q.inverse() * q;

    /* WRITE YOUR CODE HERE */

    // used for debug
    //  debug_msg_.des_p_x = des.p(0);
    //  debug_msg_.des_p_y = des.p(1);
    //  debug_msg_.des_p_z = des.p(2);

    debug_msg_.des_v_x = des.v( 0 );
    debug_msg_.des_v_y = des.v( 1 );
    debug_msg_.des_v_z = des.v( 2 );

    debug_msg_.des_a_x = des_acc( 0 );
    debug_msg_.des_a_y = des_acc( 1 );
    debug_msg_.des_a_z = des_acc( 2 );

    debug_msg_.des_q_x = u.q.x();
    debug_msg_.des_q_y = u.q.y();
    debug_msg_.des_q_z = u.q.z();
    debug_msg_.des_q_w = u.q.w();

    debug_msg_.des_thr = u.thrust;

    // Used for thrust-accel mapping estimation
    timed_thrust_.push( std::pair< ros::Time, double >( ros::Time::now(), u.thrust ) );
    while ( timed_thrust_.size() > 100 ) {
        timed_thrust_.pop();
    }
    return debug_msg_;
}

/*
  compute throttle percentage
  des_acc: desired acceleration in world frame
  return: throttle percentage
*/
double LinearControl::computeDesiredCollectiveThrustSignal( const Eigen::Vector3d& des_acc ) {
    double throttle_percentage( 0.0 );

    /* compute throttle, thr2acc has been estimated before */
    throttle_percentage = des_acc( 2 ) / thr2acc_;

    return throttle_percentage;
}

bool LinearControl::estimateThrustModel( const Eigen::Vector3d& est_a, const Parameter_t& param ) {
    ros::Time t_now = ros::Time::now();
    while ( timed_thrust_.size() >= 1 ) {
        // Choose data before 35~45ms ago
        std::pair< ros::Time, double > t_t         = timed_thrust_.front();
        double                         time_passed = ( t_now - t_t.first ).toSec();
        if ( time_passed > 0.045 )  // thrust data is too old, more than 45ms
        {
            // printf("continue, time_passed=%f\n", time_passed);
            timed_thrust_.pop();
            continue;
        }
        if ( time_passed < 0.035 )  //thrust data is too new, less than 35ms 
        {
            // printf("skip, time_passed=%f\n", time_passed);
            return false;
        }

        /***********************************************************/
        /* Recursive least squares algorithm with vanishing memory */
        /***********************************************************/
        double thr = t_t.second;
        timed_thrust_.pop();

        /************************************/
        /* Model: est_a(2) = thr2acc_ * thr */
        /* thr = des_acc(2) / thra2acc_     */
        /************************************/
        double gamma = 1 / ( rho2_ + thr * P_ * thr );
        double K     = gamma * P_ * thr;
        thr2acc_     = thr2acc_ + K * ( est_a( 2 ) - thr * thr2acc_ );
        P_           = ( 1 - K * thr ) * P_ / rho2_;
        // printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
        // fflush(stdout);

        // debug_msg_.thr2acc = thr2acc_;
        return true;
    }
    return false;
}
  /* 
    thr2acc 是一个常数，用于将油门值转换为加速度
    let des_acc be x, gravity be g, hover_percentage be h, thrust be t, then
    t = (1+x/g)*h = h + (h/g)*x, h need to be estimated
    简易推力模型是一个线性模型，认为油门值和产生的加速度是一个线性关系， 会在线根据期望机体z轴加速度和实际机体z轴加速度估计线性模型的斜率.
   */
void LinearControl::resetThrustMapping( void ) {
    thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
    P_       = 1e6;
}
