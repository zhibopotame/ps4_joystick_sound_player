#include "joy_sound_player.h"
#include <iostream>

uint32_t crc32_le ( uint32_t seed, const void *data, int len )
{
 uint32_t crc = seed;
 const uint8_t *src = static_cast<uint8_t const*> (data);
 uint32_t mult;
 int i;

 while ( len-- ) {
         crc ^= *src++;
         for ( i = 0; i < 8; i++ ) {
                 mult = ( crc & 1 ) ? CRCPOLY : 0;
                 crc = ( crc >> 1 ) ^ mult;
         }
 }

 return crc;
}

long diff_in_us(struct timespec t1, struct timespec t2)
{
    struct timespec diff;
    if (t2.tv_nsec-t1.tv_nsec < 0) {
        diff.tv_sec  = t2.tv_sec - t1.tv_sec - 1;
        diff.tv_nsec = t2.tv_nsec - t1.tv_nsec + 1000000000;
    } else {
        diff.tv_sec  = t2.tv_sec - t1.tv_sec;
        diff.tv_nsec = t2.tv_nsec - t1.tv_nsec;
    }
    return (diff.tv_sec * 1000000.0 + diff.tv_nsec / 1000.0);
}

JoySoundPlayer::JoySoundPlayer(std::string hid_id):DS4(hid_id)
{
	current_pkg_path= ros::package::getPath("ps4_joystick_sound_player");
}

void JoySoundPlayer::power_up()
{
    std::string SBCFile = current_pkg_path + "/data/power_up.sbc";
	std::vector<char> RGB = {0x00, 0x50, 0x00};
    char volume = 0x50;
    play_sound(SBCFile, RGB, volume);
}

void JoySoundPlayer::power_down()
{
    std::string SBCFile = current_pkg_path + "/data/power_down.sbc";
    std::vector<char> RGB = {0x50, 0x00, 0x00};
    char volume = 0x30;
    play_sound(SBCFile, RGB, volume);
}

void JoySoundPlayer::play(bool mode)
{
    if (mode)
        power_up();
    else
        power_down();
}
void JoySoundPlayer::set_volume(char volume)
{
	speaker_volume_ = volume;
	std::string SBCFile = current_pkg_path + "/data/volume.sbc";
	play_sound(SBCFile, {0x00, 0x50, 0x00}, speaker_volume_);
}

void JoySoundPlayer::play_data_collect(DataCollect msg)
{
	std::vector<char> RGB = {0x00, 0x50, 0x00};
	char volume = speaker_volume_;
	switch(msg)
	{
		case Start:
		{
			std::string SBCFile = current_pkg_path + "/data/dc_start.sbc";
			play_sound(SBCFile, RGB, volume);
			break;
		}
		case Wait:
		{
			std::string SBCFile = current_pkg_path + "/data/dc_wait.sbc";
			play_sound(SBCFile, RGB, volume);
			break;
		}
		case Finished:
		{
			std::string SBCFile = current_pkg_path + "/data/dc_finished.sbc";
			play_sound(SBCFile, RGB, volume);
			break;
		}
		case AlreadyStarted:
		{
			std::string SBCFile = current_pkg_path + "/data/dc_already_started.sbc";
			play_sound(SBCFile, RGB, volume);
			break;
		}
		case NotStarted:
		{
			std::string SBCFile = current_pkg_path + "/data/dc_not_started.sbc";
			play_sound(SBCFile, RGB, volume);
			break;
		}
		case LidarOn:
		{
			std::string SBCFile = current_pkg_path + "/data/lidar_on.sbc";
			play_sound(SBCFile, RGB, volume);
			break;
		}
		case LidarOff:
		{
			std::string SBCFile = current_pkg_path + "/data/lidar_off.sbc";
			play_sound(SBCFile, RGB, volume);
			break;
		}
		case MonoOn:
		{
			std::string SBCFile = current_pkg_path + "/data/mono_camera_on.sbc";
			play_sound(SBCFile, RGB, volume);
			break;
		}
		case MonoOff:
		{
			std::string SBCFile = current_pkg_path + "/data/mono_camera_off.sbc";
			play_sound(SBCFile, RGB, volume);
			break;
		}
		case StereoOn:
		{
			std::string SBCFile = current_pkg_path + "/data/stereo_camera_on.sbc";
			play_sound(SBCFile, RGB, volume);
			break;
		}
		case StereoOff:
		{
			std::string SBCFile = current_pkg_path + "/data/stereo_camera_off.sbc";
			play_sound(SBCFile, RGB, volume);
			break;
		}
		case Interrupt:
		{
			std::string SBCFile = current_pkg_path + "/data/dc_interrupted.sbc";
			RGB = {0x50, 0x00, 0x00};
			play_sound(SBCFile, RGB, volume);
			break;
		}
		case InterruptSave:
		{
			std::string SBCFile = current_pkg_path + "/data/dc_interrupted_saved.sbc";
			RGB = {0x50, 0x00, 0x00};
			play_sound(SBCFile, RGB, volume);
			break;
		}
	}
}

void JoySoundPlayer::play_sound(std::string SBCFile, std::vector<char> RGB, char volume)
{
	int bufferSize = 334;
	size_t bytesRead = 0;
	char buf[bufferSize];
	char audioData[224];
	int fd, res;
    struct timespec tstart={0,0}, tend={0,0};

	fd = open(DS4.c_str(), O_RDWR|O_NONBLOCK);

	if (fd < 0) {
		perror("Unable to open device");
		return;
	}

	/* P L A Y  T R A C K */
	FILE *file = NULL;
	file = fopen(SBCFile.c_str(), "rb");

	if (file != NULL){
		int lilEndianCounter = 0;
        bytesRead = fread(audioData, 1, sizeof(audioData), file);
		while (bytesRead > 0)
        {
			int indexBuffer = 81;
			int indexAudioData = 0;
			if(lilEndianCounter > 0xffff){
				lilEndianCounter = 0;
			}

			buf[0] = _protocolID; /* Protocol ID */
			buf[1] = _modeType; /* c0 Blueooth Mode, a0 USB Mode */
			buf[2] = _transactionType; /* Transaction Type is DATA (0xa0), Report Type is OUTPUT (0x02) */
			buf[3] = _featuresSwitch; /* 0xf0 disables the LEDs and rumble motors, 0xf3 enables them */
			buf[4] = 0x00; /* Unknown */
			buf[5] = 0x00; /* Unknown */
			buf[6] = _powerRumbleRight;/* Rumble Power Right */
			buf[7] = _powerRumbleLeft; /* Rumble Power Left */
			buf[8] = RGB[0]; /* Red */
			buf[9] = RGB[1]; /* Green */
			buf[10] = RGB[2]; /* Blue */
			buf[11] = _flashON; /* LED Flash On */
			buf[12] = _flashOFF; /* LED Flash Off */
			buf[13] = 0x00; buf[14] = 0x00; buf[15] = 0x00; buf[16] = 0x00; /* Start Empty Frames */
			buf[17] = 0x00; buf[18] = 0x00; buf[19] = 0x00; buf[20] = 0x00; /* End Empty Frames */
			buf[21] = _volLeft; /* Vol Left */
			buf[22] = _volRight; /* Vol Right */
			buf[23] = _volMic; /* Vol Mic */
			buf[24] = volume; /* Vol Built-in Speaker / 0x4d == Uppercase M (Mute?) */
			buf[25] = 0x85; 											/* Unknown */
			buf[26] = 0x00; buf[27] = 0x00; buf[28] = 0x00; buf[29] = 0x00; buf[30] = 0x00; buf[31] = 0x00;	/* Start Empty Frames */
			buf[32] = 0x00; buf[33] = 0x00; buf[34] = 0x00;	buf[35] = 0x00; buf[36] = 0x00; buf[37] = 0x00;
			buf[38] = 0x00; buf[39] = 0x00; buf[40] = 0x00; buf[41] = 0x00; buf[42] = 0x00; buf[43] = 0x00;
			buf[44] = 0x00; buf[45] = 0x00; buf[46] = 0x00; buf[47] = 0x00; buf[48] = 0x00; buf[49] = 0x00;
			buf[50] = 0x00; buf[51] = 0x00; buf[52] = 0x00; buf[53] = 0x00; buf[54] = 0x00; buf[55] = 0x00;
			buf[56] = 0x00; buf[57] = 0x00; buf[58] = 0x00; buf[59] = 0x00; buf[60] = 0x00; buf[61] = 0x00;
			buf[62] = 0x00; buf[63] = 0x00; buf[64] = 0x00; buf[65] = 0x00; buf[66] = 0x00; buf[67] = 0x00;
			buf[68] = 0x00; buf[69] = 0x00; buf[70] = 0x00; buf[71] = 0x00; buf[72] = 0x00; buf[73] = 0x00;
			buf[74] = 0x00; buf[75] = 0x00; buf[76] = 0x00; buf[77] = 0x00; /* End Empty Frames */
			buf[78] = lilEndianCounter & 255; /* Audio frame counter (endian 1)*/
			buf[79] = (lilEndianCounter / 256) & 255; /* Audio frame counter (endian 2) */
			buf[80] = 0x02; /* 0x02 Speaker Mode On / 0x24 Headset Mode On*/
			//buf[80] = 0x24; /* 0x02 Speaker Mode On / 0x24 Headset Mode On*/

			// A U D I O  D A T A
			for(indexAudioData = 0; indexAudioData < sizeof(audioData); indexAudioData++){
				buf[indexBuffer++] = audioData[indexAudioData] & 255;
			}

    		buf[306] = 0x00; buf[307] = 0x00; buf[308] = 0x00; buf[309] = 0x00; buf[310] = 0x00; buf[311] = 0x00; /*Start Empty Frames */
			buf[312] = 0x00; buf[313] = 0x00; buf[314] = 0x00; buf[315] = 0x00; buf[316] = 0x00; buf[317] = 0x00;
			buf[318] = 0x00; buf[319] = 0x00; buf[320] = 0x00; buf[321] = 0x00; buf[322] = 0x00; buf[323] = 0x00;
			buf[324] = 0x00; buf[325] = 0x00; buf[326] = 0x00; buf[327] = 0x00; buf[328] = 0x00; buf[329] = 0x00; /* End Empty Frames */
			buf[330] = 0x00; buf[331] = 0x00; buf[332] = 0x00; buf[333] = 0x00; /* CRC-32 */

			// Generate CRC32 data for output buffer and add it to output report
			uint8_t bthdr = 0xA2;
			uint32_t crc = 0;
			crc = crc32_le(0xFFFFFFFF, &bthdr, 1);
			crc = ~crc32_le(crc, buf, 334-4);
			buf[330] = crc;
			buf[331] = crc >> 8;
			buf[332] = crc >> 16;
			buf[333] = crc >> 24;

			res = write(fd, buf, bufferSize);

			if (res < 0) {
				printf("Error: %d\n", errno);
				perror("write");
			} else {
				lilEndianCounter += 2;
                bytesRead = fread(audioData, 1, sizeof(audioData), file);
			}

            clock_gettime(CLOCK_MONOTONIC, &tend);
            long timediff = diff_in_us(tstart, tend);
            useconds_t sleepTime = 5000 - timediff;
            sleepTime = (sleepTime > 5000) ? 5000 : ((sleepTime < 0) ? 0 : sleepTime);
            if (sleepTime > 0)
            {
                usleep(sleepTime);
            }

            // Get new end time
            clock_gettime(CLOCK_MONOTONIC, &tend);
            timediff = diff_in_us(tstart, tend);
            while (timediff < 7995)
            {
                // Need to find better wait to perform spin wait
                //printf("HUP: %d\n", timediff);
                clock_gettime(CLOCK_MONOTONIC, &tend);
                timediff = diff_in_us(tstart, tend);
            }
            clock_gettime(CLOCK_MONOTONIC, &tstart);
		}
	}
}