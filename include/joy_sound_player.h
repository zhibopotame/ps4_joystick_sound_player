#ifndef JOY_SOUND_PLAYER_H_
#define JOY_SOUND_PLAYER_H_

/*
    https://gitlab.com/ryochan7/dual-pod-shock/-/tree/ryochan-changes
*/
#include <stdint.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <ros/ros.h>
#include <ros/package.h>

#define CRCPOLY         0xedb88320
/* S E T T I N G S  D U A L - P O D - S H O C K  4 */
#define _protocolID 0x15 /* Protocol ID */
#define _modeType 0xc0 | 0x04 /* c0 Bluetooth Mode / a0 USB Mode. Add 4 ms poll rate */
//#define _transactionType 0xa0 /* Transaction Type is DATA (0xa0). Report Type is OUTPUT (0x02) */
#define _transactionType 0xa2 /* Transaction Type is DATA (0xa0). Report Type is OUTPUT (0x02) */
#define _featuresSwitch 0xf3 /* 0xf0 Disables LED and Rumble Motors. 0xf3 Enables All of Them */
#define _powerRumbleRight 0x00 /* Power Rumble Right */
#define _powerRumbleLeft 0x00 /* Power Rumble Left */
#define _flashON 0x00 /* LED Flash On */
#define _flashOFF 0x00 /* LED Flash Off */
#define _volLeft 0x30 /* Volume Headset Speaker Left */
#define _volRight 0x30 /* Volume Headset Speaker Right */
#define _volMic	0x00 /* Volume Mic */
#define _volSpeaker 0x50 /* Volume Built-in Speaker / 0x4d == Uppercase M (Mute?) */
#define _R 0x00 /* Color Red */
#define _G 0x50 /* Color Green */
#define _B 0x00 /* Color Blue */

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t crc32_le ( uint32_t seed, const void *data, int len );

#ifdef __cplusplus
}
#endif

#endif



class JoySoundPlayer
{
public:
    JoySoundPlayer(std::string hid_id);
    ~JoySoundPlayer(){};
    enum DataCollect
    {
        Start,
        Wait,
        Finished,
        AlreadyStarted,
        NotStarted,
        LidarOn,
        LidarOff,
        MonoOn,
        MonoOff,
        StereoOn,
        StereoOff,
        Interrupt,
        InterruptSave
    };
    void play(bool mode);
    void play_data_collect(DataCollect);
    void set_volume(char volume);
    

private:
    void power_up();
    void power_down();
    void play_sound(std::string SBCFile, std::vector<char> RGB, char volume);

    std::string DS4="";
    std::string current_pkg_path="";
    char speaker_volume_ = 0x20; 
};