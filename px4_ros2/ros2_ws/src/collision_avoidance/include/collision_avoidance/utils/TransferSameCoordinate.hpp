/*
해당 기능
: 지금 vehicle_odometry을 그냥 subscribe 하는 경우에 지금 원점이 통일되지 않은 값이 등장
  즉 맨 처음 ekf가 켜졌을 때의 지점을 원점으로 진행하는 좌표계의 값이 등장함으로
  이것을 해결하기 위해서 지금 기존의 vehicle_odometry의 값에 초기조건의 위치를 평행이동 시켜서 진행을 함
  참고로 지금 그냥 정확하게 위치만을 수정하는 것을 의미
*/

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>


namespace Transfer_coordinate
{

 /*스폰 위치를 전달 받을 구조체 정의*/
 struct SpawnPosition {
      float x;
      float y;
      float z;
  };


class TransferSameCoordinate : public rclcpp::Node
{

    public:
        TransferSameCoordinate();

    private:
       /*서브스크라이버에 대한 맴버변수*/
       std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr> raw_odom_subs_;

       /*퍼블리셔에 대한 맴버변수*/
       std::vector<rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr> trans_odom_pubs_;

       /*원본 odometry 저장용*/
       std::vector<px4_msgs::msg::VehicleOdometry> m_raw_odometry;

       /*스폰 오프셋 구조체*/
       std::vector<SpawnPosition> m_spawn_offsets;

       /*파라미터 변수*/

       int m_total_agent_num{0};
       std::vector<double> m_spawn_offset_x;
       std::vector<double> m_spawn_offset_y;
       std::vector<double> m_spawn_offset_z;

};


}
