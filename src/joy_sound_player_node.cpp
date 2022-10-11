#include "joy_sound_player.h"
#include <sensor_msgs/Joy.h>

std::unique_ptr<JoySoundPlayer> jsp;
sensor_msgs::Joy previous_joy_msg = sensor_msgs::Joy();
sensor_msgs::Joy current_joy_msg = sensor_msgs::Joy();
double joy_speed_scale = 1.0;
int speaker_volume = 20;
bool enable_mono_cam_data_record = false;
bool enable_stereo_cam_data_record = false;
bool enable_lidar_data_record = false;
bool data_record_started = false;
bool joy_stick_activated = true;
std::string hid_id = "";

void callback_joystick(const sensor_msgs::Joy &joy_msg)
{
  // std::cout<<joy_msg<<std::endl;
  //store joystick msg to verify if a button is pressed and released or not
  previous_joy_msg = current_joy_msg;
  current_joy_msg = joy_msg;

  if (previous_joy_msg.buttons.size() ==0)
    return;

  // Activate joystick or disable
  if (previous_joy_msg.buttons[13] == 1 && current_joy_msg.buttons[13] == 0) //test if the buttons is pressed and released
  {
    joy_stick_activated = (joy_stick_activated+1)%2;
    std::vector<std::string> joy_status {"Disactivated", "Activated"};
    std::cout<< "Joystick: "<<joy_status[joy_stick_activated]<<std::endl;
    jsp->play(joy_stick_activated);
  }

  if(!joy_stick_activated)
  { 
    //if disactivated during bag record
    if(data_record_started)
    {
      jsp->play_data_collect(JoySoundPlayer::DataCollect::Interrupt);
      jsp->play_data_collect(JoySoundPlayer::DataCollect::InterruptSave);
      data_record_started = false;
    }
    return;
  }

  // send velocity
  if (previous_joy_msg.buttons[6] == 1 && current_joy_msg.buttons[6] == 0) // L2 descreased speed, min speed limited to 0.2
    joy_speed_scale = std::max(joy_speed_scale - 0.2, 0.2);
  if (previous_joy_msg.buttons[7] == 1 && current_joy_msg.buttons[7] == 0) // R2 increased speed,  max speed limited to 2.0
    joy_speed_scale = std::min(joy_speed_scale + 0.2, 2.0);

  // double seq = joy_msg.header.seq / sin_period_;
  double speed = joy_msg.axes[1] * joy_speed_scale;
  double steer = joy_msg.axes[2] * 0.5; //0.5 is the max steer
  // if (speed != 0.0 && steer == 0.0)
  //   steer +=  std::cos(seq)/sin_amp_;
  // std::cout<<"sending speed: "<<speed<<" steer: "<<steer<<std::endl;

  // volume up
  if (previous_joy_msg.buttons[4] == 1 && current_joy_msg.buttons[4] == 0)
  {
    speaker_volume = std::max(speaker_volume - 20, 0);
    jsp->set_volume((int8_t)speaker_volume);
  }

  // volume down
  if (previous_joy_msg.buttons[5] == 1 && current_joy_msg.buttons[5] == 0)
  {
    speaker_volume = std::min(speaker_volume + 20, 100);
    jsp->set_volume((int8_t)speaker_volume);
  }

  //modify mono record status
  if (previous_joy_msg.buttons[0] == 1 && current_joy_msg.buttons[0] == 0)
  {
    enable_mono_cam_data_record = (enable_mono_cam_data_record+1)%2;
    if(enable_mono_cam_data_record)
      jsp->play_data_collect(JoySoundPlayer::DataCollect::MonoOn);
    else
      jsp->play_data_collect(JoySoundPlayer::DataCollect::MonoOff);
  }

  //modify stereo record status
  if (previous_joy_msg.buttons[1] == 1 && current_joy_msg.buttons[1] == 0)
  {
    enable_stereo_cam_data_record = (enable_stereo_cam_data_record+1)%2;
    if(enable_stereo_cam_data_record)
      jsp->play_data_collect(JoySoundPlayer::DataCollect::StereoOn);
    else
      jsp->play_data_collect(JoySoundPlayer::DataCollect::StereoOff);
  }

  //modify lidar record status
  if (previous_joy_msg.buttons[2] == 1 && current_joy_msg.buttons[2] == 0)
  {
    enable_lidar_data_record = (enable_lidar_data_record+1)%2;
    if(enable_lidar_data_record)
      jsp->play_data_collect(JoySoundPlayer::DataCollect::LidarOn);
    else
      jsp->play_data_collect(JoySoundPlayer::DataCollect::LidarOff);
  }

  //start bag record
  if (previous_joy_msg.buttons[9] == 1 && current_joy_msg.buttons[9] == 0)
  {
    if (!data_record_started)
    {
      jsp->play_data_collect(JoySoundPlayer::DataCollect::Start);
      data_record_started = true;
    }
    else
    {
      jsp->play_data_collect(JoySoundPlayer::DataCollect::AlreadyStarted);
    }
  }

  //stop bag record
  if (previous_joy_msg.buttons[8] == 1 && current_joy_msg.buttons[8] == 0)
  {
    if (data_record_started)
    {
      jsp->play_data_collect(JoySoundPlayer::DataCollect::Wait);
      jsp->play_data_collect(JoySoundPlayer::DataCollect::Finished);
      data_record_started = false;
    }
    else
    {
      jsp->play_data_collect(JoySoundPlayer::DataCollect::NotStarted);
    }
  }

}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "joy_sound_player_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Subscriber sub_joystick;
  pnh.param("hid_id", hid_id, std::string("/dev/hidraw0"));
  std::cout<< "hid_id: "<< hid_id << std::endl;
  jsp = std::make_unique<JoySoundPlayer>(hid_id);
  sub_joystick = nh.subscribe("joy", 1, &callback_joystick);
  ros::spin();

  return 0;
};