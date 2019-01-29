

#include <assert.h>
#include <math.h>
#include <iostream>

#include <boost/format.hpp>

#include "microstrain_3dmgx2_imu/3dmgx2.h"


using namespace std;

typedef struct 
{
  uint64_t time;
  double accel[3];
  double angrate[3];
  double orientation[9];
} ImuData;


class ImuNode 
{
public:
  microstrain_3dmgx2_imu::IMU imu;
//   sensor_msgs::Imu reading;

  string port;

  microstrain_3dmgx2_imu::IMU::cmd cmd;

  bool running;

  bool autocalibrate_;
//   bool calibrate_requested_;
  bool calibrated_;
  
  int error_count_;
  
  double offset_;
    
  double bias_x_;
  double bias_y_;
  double bias_z_;

  double angular_velocity_stdev_, angular_velocity_covariance_;
  double linear_acceleration_covariance_, linear_acceleration_stdev_;
  double orientation_covariance_, orientation_stdev_;

  double max_drift_rate_;

  
  ImuNode() :  
  error_count_(0)
  {
    
//     private_node_handle_.param("autocalibrate", autocalibrate_, true);
       autocalibrate_=true;
//     private_node_handle_.param("assume_calibrated", calibrated_, false);
       calibrated_=false;
//     private_node_handle_.param("port", port, string("/dev/ttyUSB0"));
       port="/dev/ttyACM0";
//     private_node_handle_.param("max_drift_rate", max_drift_rate_, 0.0002);
       max_drift_rate_=0.0002;

    cmd = microstrain_3dmgx2_imu::IMU::CMD_ACCEL_ANGRATE_ORIENT;
    
    running = false;

    bias_x_ = bias_y_ = bias_z_ = 0;

//     private_node_handle_.param("frame_id", frameid_, string("imu"));
//     reading.header.frame_id = frameid_;

//     private_node_handle_.param("time_offset", offset_, 0.0);
       offset_=0.0;

//     private_node_handle_.param("linear_acceleration_stdev", linear_acceleration_stdev_, 0.098); 
       linear_acceleration_stdev_=0.098;
//     private_node_handle_.param("orientation_stdev", orientation_stdev_, 0.035); 
       orientation_stdev_=0.035;
//     private_node_handle_.param("angular_velocity_stdev", angular_velocity_stdev_, 0.012);
       angular_velocity_stdev_=0.012;

    double angular_velocity_covariance = angular_velocity_stdev_ * angular_velocity_stdev_;
    double orientation_covariance = orientation_stdev_ * orientation_stdev_;
    double linear_acceleration_covariance = linear_acceleration_stdev_ * linear_acceleration_stdev_;
    
//     reading.linear_acceleration_covariance[0] = linear_acceleration_covariance;
//     reading.linear_acceleration_covariance[4] = linear_acceleration_covariance;
//     reading.linear_acceleration_covariance[8] = linear_acceleration_covariance;
// 
//     reading.angular_velocity_covariance[0] = angular_velocity_covariance;
//     reading.angular_velocity_covariance[4] = angular_velocity_covariance;
//     reading.angular_velocity_covariance[8] = angular_velocity_covariance;
//     
//     reading.orientation_covariance[0] = orientation_covariance;
//     reading.orientation_covariance[4] = orientation_covariance;
//     reading.orientation_covariance[8] = orientation_covariance;
    

  }

  ~ImuNode()
  {
    stop();
  }


 
  void doCalibrate()
  { 
    // Expects to be called with the IMU stopped.
    printf("\nCalibrating IMU gyros.");
    imu.initGyros(&bias_x_, &bias_y_, &bias_z_);
    
    // check calibration
    if (!imu.setContinuous(microstrain_3dmgx2_imu::IMU::CMD_ACCEL_ANGRATE_ORIENT))
    {
      printf("\nCould not start streaming data to verify calibration");
      exit(0);
    } 
    else 
    {
      double x_rate = 0.0;
      double y_rate = 0.0;
      double z_rate = 0.0;
      size_t count = 0;
      ImuData idata;
      getData(idata);
      uint64_t start_time = idata.time;

      while(idata.time - start_time < 2*1000000000){
        getData(idata);
        x_rate += idata.angrate[0];
        y_rate += idata.angrate[1];
        z_rate += idata.angrate[2];
        ++count;
      }

      double average_rate = sqrt(x_rate*x_rate + y_rate*y_rate + z_rate*z_rate) / count;
      std::cout<<"count:\t"<<count<<std::endl;
      if (count < 200){
        printf("\nWARN:\tImu: calibration check acquired fewer than 200 samples.");
      }
      
      // calibration succeeded
      if (average_rate < max_drift_rate_) {
        printf("\nImu: calibration check succeeded: average angular drift %f mdeg/sec < %f mdeg/sec", average_rate*180*1000/M_PI, max_drift_rate_*180*1000/M_PI);
        calibrated_ = true;
        printf("\nIMU gyro calibration completed.");
      }
      // calibration failed
      else
      {
        calibrated_ = false;
        printf("\n average angular drift = %f mdeg/sec > %f mdeg/sec", average_rate*180*1000/M_PI, max_drift_rate_*180*1000/M_PI);
	std::cerr<<"\nImu: calibration check failed:"<<std::endl;
	exit(0);
      }
      imu.stopContinuous();
    }
  }

  int start()
  {
    stop();

    try
    {
      try
      {
        imu.openPort(port.c_str());
      } catch (microstrain_3dmgx2_imu::Exception& e) {
        error_count_++;
        std::cout<<e.what()<<std::endl;;
        return -1;
      }


      printf("Initializing IMU time with offset %f.", offset_);

      imu.initTime(offset_);

      if (autocalibrate_ )
      {
        doCalibrate();
//         calibrate_requested_ = false;
        autocalibrate_ = false; // No need to do this each time we reopen the device.
      }
      else
      {
        printf("Not calibrating the IMU sensor. Use the calibrate service to calibrate it before use.");
      }

      printf("IMU sensor initialized.");

      imu.setContinuous(cmd);

//       freq_diag_.clear();

      running = true;

    } catch (microstrain_3dmgx2_imu::Exception& e) {
      error_count_++;
      usleep(100000); // Give isShuttingDown a chance to go true.
        printf("Exception thrown while starting IMU. This sometimes happens if you are not connected to an IMU or if another process is trying to access the IMU port. You may try 'lsof|grep %s' to see if other processes have the port open. %s", port.c_str(), e.what());
      
      return -1;
    }

    return(0);
  }
 
  
  int stop()
  {
    if(running)
    {
      try
      {
        imu.closePort();
      } catch (microstrain_3dmgx2_imu::Exception& e) {
        error_count_++;
        std::cout<<"Exception thrown while stopping IMU. "<<e.what()<<std::endl;;
      }
      running = false;
    }

    return(0);
  }

  void getData(ImuData &idata)
  {
    uint64_t time;
    double accel[3];
    double angrate[3];
    double orientation[9];

    imu.receiveAccelAngrateOrientation(&time, accel, angrate, orientation);
//     data.linear_acceleration.x = accel[0];
//     data.linear_acceleration.y = accel[1];
//     data.linear_acceleration.z = accel[2];
//     std::cout<<"linear_acceleration:\t"<<accel[0]<<","<<accel[1]<<","<<accel[2]<<std::endl;
    for(int i=0;i<3;i++)
      idata.accel[i]=accel[i];
 
//     data.angular_velocity.x = angrate[0];
//     data.angular_velocity.y = angrate[1];
//     data.angular_velocity.z = angrate[2];
//     std::cout<<"angular_velocity:\t"<<angrate[0]<<","<<angrate[1]<<","<<angrate[2]<<std::endl;
    for(int i=0;i<3;i++)
      idata.angrate[i]=angrate[i];
      
//     tf::Quaternion quat;
//     (tf::Matrix3x3(-1,0,0,
// 		 0,1,0,
// 		 0,0,-1)*
//     tf::Matrix3x3(orientation[0], orientation[3], orientation[6],
// 		 orientation[1], orientation[4], orientation[7],
// 		 orientation[2], orientation[5], orientation[8])).getRotation(quat);
//     
//     tf::quaternionTFToMsg(quat, data.orientation);
    for(int i=0;i<9;i++)
      idata.orientation[i]=orientation[i];
    
      
//     data.header.stamp = ros::Time::now().fromNSec(time);
//     printf("time:\t%ld",time);
    idata.time=time;
    //ROS_INFO(data.header.stamp);
  }
};

int main(int argc, char** argv)
{
  printf("microstrain_3dmgx2_node");

  ImuNode in;
  
  while(1)
  {
    if(in.start()==0)
      break;
    else
    {
      printf("in.start failed ,usleep(1000000) and restart...\n\n");
      usleep(1000000);
    }
  }
  
  ImuData idata;
  
  while(1)
  {
    in.getData(idata);
    std::cout<<"linear_acceleration:\t"<<idata.accel[0]<<","<<idata.accel[1]<<","<<idata.accel[2]<<std::endl;
    std::cout<<"angular_velocity:\t"<<idata.angrate[0]<<","<<idata.angrate[1]<<","<<idata.angrate[2]<<std::endl;
    std::cout<<"time:\t"<<idata.time<<std::endl;
  }
  
  in.stop();
  
  return(0);
}
