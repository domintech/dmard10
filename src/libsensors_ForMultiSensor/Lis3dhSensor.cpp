/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/select.h>
#include <cutils/log.h>

#include "Lis3dhSensor.h"

#define D06_ACC_DEV_PATH_NAME    "/dev/dmard06"
#define D06_ACC_INPUT_NAME  "DMT_accel" 
char path_en[]="/sys/class/accelemeter/dmard06/enable_acc";
char path_de[]="/sys/class/accelemeter/dmard06/delay_acc";
#define	LIS3DH_ACC_IOCTL_BASE 77
/** The following define the IOCTL command values via the ioctl macros */
#define	LIS3DH_ACC_IOCTL_SET_DELAY		_IOW(LIS3DH_ACC_IOCTL_BASE, 0, int)
#define	LIS3DH_ACC_IOCTL_GET_DELAY		_IOR(LIS3DH_ACC_IOCTL_BASE, 1, int)
#define	LIS3DH_ACC_IOCTL_SET_ENABLE		_IOW(LIS3DH_ACC_IOCTL_BASE, 2, int)
#define	LIS3DH_ACC_IOCTL_GET_ENABLE		_IOR(LIS3DH_ACC_IOCTL_BASE, 3, int)

#define LIS3DH_LAYOUT_NEGX    0x01
#define LIS3DH_LAYOUT_NEGY    0x02
#define LIS3DH_LAYOUT_NEGZ    0x04
 
#define DMT_UNIT_CONVERSION(value)		((value) * (GRAVITY_EARTH / (32.0f)))  //1g = 256codes
//#define LIS3DH_UNIT_CONVERSION(value) ((value) * GRAVITY_EARTH / (32.0f))
int write_sys_attribute(const char *path, const char *value, int bytes)
{
    int ifd, amt;

	ifd = open(path, O_WRONLY);
    if (ifd < 0) {
        LOGE("Write_attr failed to open %s (%s)",path, strerror(errno));
        return -1;
	}

    amt = write(ifd, value, bytes);
	amt = ((amt == -1) ? -errno : 0);
	LOGE_IF(amt < 0, "Write_int failed to write %s (%s)",
		path, strerror(errno));
    close(ifd);
	return amt;
}
/*****************************************************************************/
Lis3dhSensor::Lis3dhSensor()
    : SensorBase(D06_ACC_DEV_PATH_NAME, D06_ACC_INPUT_NAME),
      mEnabled(1),
      mDelay(10000),
      mLayout(1),
      mInputReader(32),
      mHasPendingEvent(false)
{
    mPendingEvent.version = sizeof(sensors_event_t);
    mPendingEvent.sensor = ID_A;
    mPendingEvent.type = SENSOR_TYPE_ACCELEROMETER;
    memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));

    //mLayout =  getLayout();
   
    open_device();
}

Lis3dhSensor::~Lis3dhSensor() {
    if (mEnabled) {
        setEnable(0, 0);
    }

    close_device();
}

int Lis3dhSensor::setInitialState() {
    struct input_absinfo absinfo;

	if (mEnabled) {
		if (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_ACCEL_X), &absinfo)) {
			mPendingEvent.acceleration.x = DMT_UNIT_CONVERSION(absinfo.value);
		}
		if (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_ACCEL_Y), &absinfo)) {
			mPendingEvent.acceleration.y = DMT_UNIT_CONVERSION(absinfo.value);
		}
		if (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_ACCEL_Z), &absinfo)) {
			mPendingEvent.acceleration.z = DMT_UNIT_CONVERSION(absinfo.value);
		}
	}
	return 0;
}
/*
int Lis3dhSensor::getLayout() {
    return 1;
}
*/
bool Lis3dhSensor::hasPendingEvents() const {
    return mHasPendingEvent;
}

int Lis3dhSensor::setEnable(int32_t handle, int enabled) {
	char value[2]={0,0};
	value[0]= enabled ? '1':'0';
	LOGE("DmtSensor: %s:enable=%s\n",__func__,value);
	write_sys_attribute(path_en,value, 1);
	return 0;

}

int Lis3dhSensor::setDelay(int32_t handle, int64_t delay_ns)
{
    int err = 0;
    int ms; 

    /* handle check */
    /*if (handle != ID_A) {
        LOGE("Lis3dhSensor: Invalid handle (%d)", handle);
        return -EINVAL;
    }

    if (mDelay != delay_ns) {
        ms = delay_ns / 1000000;
	LOGE("LisdhSensor: set delay = %d", ms);

        if (ioctl(dev_fd, LIS3DH_ACC_IOCTL_SET_DELAY, &ms)) {
            return -errno;
        }
        mDelay = delay_ns;
    }
    */
	char value[11]={0};
	//int bytes;
	ms = delay_ns / 1000;
	//gsensor_delay_time = delay_ns / 1000;
	err = sprintf(value,"%u",ms);
	LOGE("@@@Dmt: value=%s ,gsensor=%u, bytes=%d\n",value,ms,err);	
	write_sys_attribute(path_de,value, err);
	mDelay = delay_ns;
	
    return err;
}

int64_t Lis3dhSensor::getDelay(int32_t handle)
{
    return (handle == ID_A) ? mDelay : 0;
}

int Lis3dhSensor::getEnable(int32_t handle)
{
    return (handle == ID_A) ? mEnabled : 0;
}

int Lis3dhSensor::readEvents(sensors_event_t* data, int count)
{
    if (count < 1)
        return -EINVAL;

    if (mHasPendingEvent) {
        mHasPendingEvent = false;
        mPendingEvent.timestamp = getTimestamp();
        *data = mPendingEvent;
        return mEnabled ? 1 : 0;
    }

    ssize_t n = mInputReader.fill(data_fd);
    if (n < 0)
        return n;

    int numEventReceived = 0;
    input_event const* event;

    while (count && mInputReader.readEvent(&event)) {
		int type = event->type;
		if (type == EV_ABS) {
			float value = event->value;
			if (event->code == EVENT_TYPE_ACCEL_X) {
				mPendingEvent.acceleration.x = DMT_UNIT_CONVERSION(value);
			} else if (event->code == EVENT_TYPE_ACCEL_Y) {
				mPendingEvent.acceleration.y = DMT_UNIT_CONVERSION(value);
			} else if (event->code == EVENT_TYPE_ACCEL_Z) {
				mPendingEvent.acceleration.z = DMT_UNIT_CONVERSION(value);
			}
		} else if (type == EV_SYN) {
			mPendingEvent.timestamp = timevalToNano(event->time);
			if (mEnabled) {
				*data++ = mPendingEvent;
				count--;
				numEventReceived++;
			}
		} else {
			LOGE("DmtSensor: unknown event (type=%d, code=%d)",
					type, event->code);
		}
        mInputReader.next();
    }

    return numEventReceived;
}
