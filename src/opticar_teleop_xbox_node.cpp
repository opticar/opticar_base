#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <ros/console.h>

class OpticarTeleopXBox
{
public:
  OpticarTeleopXBox();

protected:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle m_NH;

  int m_AxisLinearForward;
  int m_AxisLinearSideways;
  int m_AxisAngularTurn;  
  int m_BtnEnable;
  double m_LinearScale;
  double m_AngularScale;
  double m_DeadZone;

  ros::Publisher m_VelPub;
  ros::Subscriber m_JoySub;
};

OpticarTeleopXBox::OpticarTeleopXBox() :
  m_AxisLinearForward(4),
  m_AxisLinearSideways(0),
  m_AxisAngularTurn(3),
  m_BtnEnable(5),
  m_LinearScale(1),
  m_AngularScale(1),
  m_DeadZone(0.1)
{
  m_NH.param("axis_linear_forward", m_AxisLinearForward, m_AxisLinearForward);
  m_NH.param("axis_linear_sideways", m_AxisLinearSideways, m_AxisLinearSideways);
  m_NH.param("axis_angular_turn", m_AxisAngularTurn, m_AxisAngularTurn);
  m_NH.param("button_enable", m_BtnEnable, m_BtnEnable);
  m_NH.param("scale_linear", m_LinearScale, m_LinearScale);
  m_NH.param("scale_angular", m_AngularScale, m_AngularScale);
  m_NH.param("deadzone", m_DeadZone, m_DeadZone);
  
  m_VelPub = m_NH.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  
  m_JoySub = m_NH.subscribe<sensor_msgs::Joy>("joy", 10, &OpticarTeleopXBox::joyCallback, this);
  
  ROS_INFO("Joystick config: fw %d, sw: %d, at: %d, btn: %d, lin: %f, turn: %f, dz: %f", m_AxisLinearForward, m_AxisLinearSideways, m_AxisAngularTurn, m_BtnEnable, m_LinearScale, m_AngularScale, m_DeadZone);
}

void OpticarTeleopXBox::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  
  auto az = joy->axes[m_AxisAngularTurn];
  auto lx = joy->axes[m_AxisLinearForward];
  auto ly = joy->axes[m_AxisLinearSideways];
  
  if (std::abs(az) < m_DeadZone) az = 0;
  if (std::abs(lx) < m_DeadZone) lx = 0;
  if (std::abs(ly) < m_DeadZone) ly = 0;
  
  if (joy->buttons[m_BtnEnable])
  {    
    twist.angular.z = m_AngularScale * az;
    twist.linear.x  = m_LinearScale  * lx;
    twist.linear.y  = m_LinearScale  * ly;
  }
  
  ROS_DEBUG("Joystick data: %f, %f, %f, %d", joy->axes[m_AxisAngularTurn], joy->axes[m_AxisLinearForward], joy->axes[m_AxisLinearSideways], joy->buttons[m_BtnEnable]);
  
  m_VelPub.publish(twist);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "opticar_teleop_xbox");
  
  OpticarTeleopXBox opticar_teleop_xbox;
  
  ros::spin();
}