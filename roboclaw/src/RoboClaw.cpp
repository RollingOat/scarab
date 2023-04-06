#include "RoboClaw.h"

#include <vector>
#include <ros/ros.h>

#define MAXRETRY 2
#define SetDWORDval(arg) (uint8_t)(((uint32_t)arg)>>24),(uint8_t)(((uint32_t)arg)>>16),(uint8_t)(((uint32_t)arg)>>8),(uint8_t)arg
#define SetWORDval(arg) (uint8_t)(((uint16_t)arg)>>8),(uint8_t)arg

using namespace std;


//! Macro for throwing an exception with a message, passing args
#define SERIAL_EXCEPT(msg, ...)                                         \
  {                                                                     \
    char buf[1000];                                                     \
    snprintf(buf, 1000, msg " (in USBSerial::%s)" , ##__VA_ARGS__, __FUNCTION__); \
    throw USBSerial::Exception(buf);                                    \
  }

void USBSerial::Open(const char *port) {
  if (IsOpen()) {
    Close();
  }

  fd_ = open(port, O_RDWR | O_NONBLOCK | O_NOCTTY);
  buf_start_ = buf_end_ = 0;
  if (fd_ == -1) {
    SERIAL_EXCEPT("Failed to open port: %s. %s (errno = %d)",
                  port, strerror(errno), errno);
  }

  try {
    struct flock fl;
    fl.l_type = F_WRLCK;
    fl.l_whence = SEEK_SET;
    fl.l_start = 0;
    fl.l_len = 0;
    fl.l_pid = getpid();

    if (fcntl(fd_, F_SETLK, &fl) != 0) {
      SERIAL_EXCEPT("Device %s is already locked", port);
    }

    struct termios newtio;
    tcgetattr(fd_, &newtio);
    memset(&newtio.c_cc, 0, sizeof(newtio.c_cc)); // Clear special characters
    newtio.c_cflag = CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    tcflush(fd_, TCIFLUSH);
    if (tcsetattr(fd_, TCSANOW, &newtio) < 0) {
      SERIAL_EXCEPT("Couldn't set serial port attributes: %s", port);
    }

    usleep(200000);
    Flush();
  } catch (USBSerial::Exception &e) {
    if (fd_ != -1) {
      close(fd_);
    }
    fd_ = -1;
    throw e;
  }
}

bool USBSerial::IsOpen() {
  return fd_ != -1;
}

void USBSerial::Close() {
  if (IsOpen()) {
    try {
      Flush();
    } catch (USBSerial::Exception &e) {
      ROS_WARN("While closing, error while flushing=%s", e.what());
    }
  }
  int rv = close(fd_);
  if (rv != 0) {
    SERIAL_EXCEPT("Error closing port: %s (%d)", strerror(errno), errno);
  }
}

int USBSerial::Flush() {
  int retval = tcflush(fd_, TCIOFLUSH);
  if (retval != 0)
    SERIAL_EXCEPT("tcflush failed %i", retval);
  buf_start_ = 0;
  buf_end_ = 0;
  return retval;
}

int USBSerial::Write(const unsigned char *data, int len) {
  // TODO: Because this is synchronous, if the device behaves weirdly we can
  // hang here
  int origflags = fcntl(fd_, F_GETFL, 0);
  fcntl(fd_, F_SETFL, origflags & ~O_NONBLOCK);
  // fprintf(stderr, "Writing ");
  // for (int i = 0; i < len; ++i) {
  //   fprintf(stderr, "%02x ", data[i]);
  // }
  // fprintf(stderr, "\n");
  ssize_t retval = write(fd_, data, len);
  int fputserrno = errno;
  fcntl(fd_, F_SETFL, origflags | O_NONBLOCK);
  errno = fputserrno;
  if (retval != -1 ) {
    return retval;
  } else {
    SERIAL_EXCEPT("write() failed: %s (%d)", strerror(errno), errno);
  }
}

int USBSerial::Read(char *buf, int len, int timeout, bool translate) {
  int current = 0;
  struct pollfd ufd[1];
  int retval;
  ufd[0].fd = fd_;
  ufd[0].events = POLLIN;

  while (true) {
    if (buf_start_ == buf_end_) {
      if ((retval = poll(ufd, 1, timeout)) < 0) {
        SERIAL_EXCEPT("Poll failed %s (%d)", strerror(errno), errno);
      } else if (retval == 0) {
        SERIAL_EXCEPT("Timeout reached");
      } else if (ufd[0].revents & POLLERR) {
        SERIAL_EXCEPT("Error on socket");
      } else if (ufd[0].revents & POLLIN) {
        int bytes = read(fd_, read_buf_, sizeof(read_buf_));
        buf_start_ = 0;
        buf_end_ = bytes;
        if (buf_end_ == 0) {
          SERIAL_EXCEPT("Read 0 bytes");
        }
      } else {
        SERIAL_EXCEPT("Unhandled case!");
      }
    }

    while (buf_start_ != buf_end_) {
      if (translate) {
        if (current == len - 1) {
          buf[current] = 0;
          SERIAL_EXCEPT("Buffer filled without end of line being found");
        }
        buf[current] = read_buf_[buf_start_];
        buf_start_++;
        if (read_buf_[current++] == '\n') {
          buf[current] = 0;
          return current;
        }
      } else {
        buf[current] = read_buf_[buf_start_];
        buf_start_++;
        current++;
        if (current == len) {
          return current;
        }
      }
    }
  }
}

int roboclaw_restart_usb() {
  const int16_t ROBOCLAW_PRODUCT = 0x2404;
  const int16_t ROBOCLAW_VENDOR = 0x3eb;

  libusb_init(NULL);
  libusb_device **list;
  ssize_t cnt = libusb_get_device_list(NULL, &list);
  if (cnt < 0) {
    ROS_WARN("Couldn't get device list: %s\n", libusb_error_name(cnt));
    return -1;
  }

  size_t problems = 0;
  int err = 0;
  for (int i = 0; i < cnt; i++) {
    libusb_device *device = list[i];
    struct libusb_device_descriptor desc;

    if (libusb_get_device_descriptor(device, &desc) != 0) {
      ROS_WARN("Couldn't get descriptor: %s", libusb_error_name(cnt));
      continue;
    }

    if (desc.idProduct == ROBOCLAW_PRODUCT &&
        desc.idVendor == ROBOCLAW_VENDOR) {
      libusb_device_handle *handle;
      if ((err = libusb_open(device, &handle)) != 0) {
        ROS_WARN("Couldn't open device: %s\n", libusb_error_name(err));
        problems |= 1;
      } else {
        if ((err = libusb_reset_device(handle)) != 0) {
          ROS_WARN("Couldn't reset device: %s\n", libusb_error_name(err));
          problems |= 1;
        } else {
          libusb_close(handle);
        }
      }
    }
  }
  libusb_free_device_list(list, 1);
  libusb_close(NULL);
  return problems;
}

//
// Constructor
//
RoboClaw::RoboClaw(USBSerial *ser) {
  timeout = 100; //From example, can be reduced
  setSerial(ser);

}

//
// Destructor
//
RoboClaw::~RoboClaw() {

}

void RoboClaw::setSerial(USBSerial *ser) {
  if (ser == NULL || !ser->IsOpen()) {
    throw invalid_argument("RoboClaw needs open USBSerial device");
  }
  ser_ = ser;
}

size_t RoboClaw::write(uint8_t byte)
{
  // fprintf(stderr, "Sending: 0x%02x\n", byte);
  return ser_->Write(&byte, 1);
}

uint8_t RoboClaw::read()
{
  char d[1];
  int rv = ser_->Read(d, sizeof(d), 100, false);
  if (rv != 1) {
     fprintf(stderr, "1Rv = %i\n", rv);
    throw std::runtime_error("RoboClaw::write_n() Didn't get byte from read()");
  } else {
     //fprintf(stderr, "1Read 0x%02x\n", d[0]);
  }
  return static_cast<uint8_t>(d[0]);
}


void RoboClaw::flush()
{
	ser_->Flush();
}

/*
uint8_t RoboClaw::readbyte() {
  char d[1];
  int rv = ser_->Read(d, sizeof(d), 100, false);
  if (rv != 1) {
    throw std::runtime_error("RoboClaw::write_n() Didn't get byte from read()");
  } else {
     fprintf(stderr, "Read 0x%02x\n", d[0]);
  }
  return static_cast<uint8_t>(d[0]);
}

uint16_t RoboClaw::readword(){
	uint8_t val1 = readbyte();
	uint8_t val2 = readbyte();
	uint16_t val = val1<<8|val2;
	return val;
}

uint32_t RoboClaw::readlong(){
	uint8_t val1 = readbyte();
	uint8_t val2 = readbyte();
	uint8_t val3 = readbyte();
	uint8_t val4 = readbyte();
	uint32_t val = val1<<24|val2<<16|val3<<8|val4;
 	return (0,0)
}

uint32_t RoboClaw::readslong(){
	val = readlong();
	if(val&0x80000000)
		return val-0x100000000;
	return val;
}
*/

uint8_t RoboClaw::read(uint32_t timeout)
{
  char d[1];
  int rv = ser_->Read(d, sizeof(d), timeout, false);
  if (rv != 1) {
     fprintf(stderr, "Rv = %i\n", rv);
    throw std::runtime_error("RoboClaw::write_n() Didn't get byte from read()");
  } else {
     //fprintf(stderr, "Read 0x%02x\n", d[0]);
  }
  return static_cast<uint8_t>(d[0]);
}

void RoboClaw::crc_clear()
{
	crc = 0;
}

void RoboClaw::crc_update (uint8_t data)
{
	int i;
	crc = crc ^ ((uint16_t)data << 8);
	for (i=0; i<8; i++)
	{
		if (crc & 0x8000)
			crc = (crc << 1) ^ 0x1021;
		else
			crc <<= 1;
	}
}

uint16_t RoboClaw::crc_get()
{
	return crc;
}

bool RoboClaw::write_n(uint8_t cnt, ... )
{
	uint8_t trys=MAXRETRY;
	do{
		crc_clear();
		//send data with crc
		va_list marker;
		va_start( marker, cnt );     /* Initialize variable arguments. */
		for(uint8_t index=0;index<cnt;index++){
			uint8_t data = va_arg(marker, int);
			crc_update(data);
			write(data);
		}
		va_end( marker );              /* Reset variable arguments.      */
		uint16_t crc = crc_get();
		write(crc>>8);
		write(crc);
		if(read(timeout)==0xFF)
			return true;
	}while(trys--);
	return false;
}

bool RoboClaw::read_n(uint8_t cnt,uint8_t address,uint8_t cmd,...)
{
	uint32_t value=0;
	uint8_t trys=MAXRETRY;
	int16_t data;
	do{
		flush();

		data=0;
		crc_clear();
		write(address);
		crc_update(address);
		write(cmd);
		crc_update(cmd);

		//send data with crc
		va_list marker;
		va_start( marker, cmd );     /* Initialize variable arguments. */
		for(uint8_t index=0;index<cnt;index++){
			uint32_t *ptr = va_arg(marker, uint32_t *);

			if(data!=-1){
				data = read(timeout);
				crc_update(data);
				value=(uint32_t)data<<24;
			}
			else{
				break;
			}

			if(data!=-1){
				data = read(timeout);
				crc_update(data);
				value|=(uint32_t)data<<16;
			}
			else{
				break;
			}

			if(data!=-1){
				data = read(timeout);
				crc_update(data);
				value|=(uint32_t)data<<8;
			}
			else{
				break;
			}

			if(data!=-1){
				data = read(timeout);
				crc_update(data);
				value|=(uint32_t)data;
			}
			else{
				break;
			}

			*ptr = value;
		}
		va_end( marker );              /* Reset variable arguments.      */

		if(data!=-1){
			uint16_t ccrc;
			data = read(timeout);
			if(data!=-1){
				ccrc = data << 8;
				data = read(timeout);
				if(data!=-1){
					ccrc |= data;
					return crc_get()==ccrc;
				}
			}
		}
	}while(trys--);

	return false;
}

uint8_t RoboClaw::Read1(uint8_t address,uint8_t cmd,bool *valid){

	if(valid)
		*valid = false;

	uint8_t value=0;
	uint8_t trys=MAXRETRY;
	int16_t data;
	do{
		flush();

		crc_clear();
		write(address);
		crc_update(address);
		write(cmd);
		crc_update(cmd);

		data = read(timeout);
		crc_update(data);
		value=data;

		if(data!=-1){
			uint16_t ccrc;
			data = read(timeout);
			if(data!=-1){
				ccrc = data << 8;
				data = read(timeout);
				if(data!=-1){
					ccrc |= data;
					if(crc_get()==ccrc){
						*valid = true;
						return value;
					}
				}
			}
		}
	}while(trys--);

	return false;
}

uint16_t RoboClaw::Read2(uint8_t address,uint8_t cmd,bool *valid){

	if(valid)
		*valid = false;

	uint16_t value=0;
	uint8_t trys=MAXRETRY;
	int16_t data;
	do{
		flush();

		crc_clear();
		write(address);
		crc_update(address);
		write(cmd);
		crc_update(cmd);

		data = read(timeout);
		crc_update(data);
		value=(uint16_t)data<<8;

		if(data!=-1){
			data = read(timeout);
			crc_update(data);
			value|=(uint16_t)data;
		}

		if(data!=-1){
			uint16_t ccrc;
			data = read(timeout);
			if(data!=-1){
				ccrc = data << 8;
				data = read(timeout);
				if(data!=-1){
					ccrc |= data;
					if(crc_get()==ccrc){
						*valid = true;
						return value;
					}
				}
			}
		}
	}while(trys--);

	return false;
}

uint32_t RoboClaw::Read4(uint8_t address, uint8_t cmd, bool *valid){

	if(valid)
		*valid = false;

	uint32_t value=0;
	uint8_t trys=MAXRETRY;
	int16_t data;
	do{
		flush();

		crc_clear();
		write(address);
		crc_update(address);
		write(cmd);
		crc_update(cmd);

		data = read(timeout);
		crc_update(data);
		value=(uint32_t)data<<24;

		if(data!=-1){
			data = read(timeout);
			crc_update(data);
			value|=(uint32_t)data<<16;
		}

		if(data!=-1){
			data = read(timeout);
			crc_update(data);
			value|=(uint32_t)data<<8;
		}

		if(data!=-1){
			data = read(timeout);
			crc_update(data);
			value|=(uint32_t)data;
		}

		if(data!=-1){
			uint16_t ccrc;
			data = read(timeout);
			if(data!=-1){
				ccrc = data << 8;
				data = read(timeout);
				if(data!=-1){
					ccrc |= data;
					if(crc_get()==ccrc){
						*valid = true;
						return value;
					}
				}
			}
		}
	}while(trys--);

	return false;
}

uint32_t RoboClaw::Read4_1(uint8_t address, uint8_t cmd, uint8_t *status, bool *valid){

	if(valid)
		*valid = false;

	uint32_t value=0;
	uint8_t trys=MAXRETRY;
	int16_t data;
	do{
		flush();

		crc_clear();
		write(address);
		crc_update(address);
		write(cmd);
		crc_update(cmd);

		data = read(timeout);
		crc_update(data);
		value=(uint32_t)data<<24;

		if(data!=-1){
			data = read(timeout);
			crc_update(data);
			value|=(uint32_t)data<<16;
		}

		if(data!=-1){
			data = read(timeout);
			crc_update(data);
			value|=(uint32_t)data<<8;
		}

		if(data!=-1){
			data = read(timeout);
			crc_update(data);
			value|=(uint32_t)data;
		}

		if(data!=-1){
			data = read(timeout);
			crc_update(data);
			if(status)
				*status = data;
		}

		if(data!=-1){
			uint16_t ccrc;
			data = read(timeout);
			if(data!=-1){
				ccrc = data << 8;
				data = read(timeout);
				if(data!=-1){
					ccrc |= data;
					if(crc_get()==ccrc){
						*valid = true;
						return value;
					}
				}
			}
		}
	}while(trys--);

	return false;
}

bool RoboClaw::ForwardM1(uint8_t address, uint8_t speed){
	return write_n(3,address,M1FORWARD,speed);
}

bool RoboClaw::BackwardM1(uint8_t address, uint8_t speed){
	return write_n(3,address,M1BACKWARD,speed);
}

bool RoboClaw::SetMinVoltageMainBattery(uint8_t address, uint8_t voltage){
	return write_n(3,address,SETMINMB,voltage);
}

bool RoboClaw::SetMaxVoltageMainBattery(uint8_t address, uint8_t voltage){
	return write_n(3,address,SETMAXMB,voltage);
}

bool RoboClaw::ForwardM2(uint8_t address, uint8_t speed){
	return write_n(3,address,M2FORWARD,speed);
}

bool RoboClaw::BackwardM2(uint8_t address, uint8_t speed){
	return write_n(3,address,M2BACKWARD,speed);
}

bool RoboClaw::ForwardBackwardM1(uint8_t address, uint8_t speed){
	return write_n(3,address,M17BIT,speed);
}

bool RoboClaw::ForwardBackwardM2(uint8_t address, uint8_t speed){
	return write_n(3,address,M27BIT,speed);
}

bool RoboClaw::ForwardMixed(uint8_t address, uint8_t speed){
	return write_n(3,address,MIXEDFORWARD,speed);
}

bool RoboClaw::BackwardMixed(uint8_t address, uint8_t speed){
	return write_n(3,address,MIXEDBACKWARD,speed);
}

bool RoboClaw::TurnRightMixed(uint8_t address, uint8_t speed){
	return write_n(3,address,MIXEDRIGHT,speed);
}

bool RoboClaw::TurnLeftMixed(uint8_t address, uint8_t speed){
	return write_n(3,address,MIXEDLEFT,speed);
}

bool RoboClaw::ForwardBackwardMixed(uint8_t address, uint8_t speed){
	return write_n(3,address,MIXEDFB,speed);
}

bool RoboClaw::LeftRightMixed(uint8_t address, uint8_t speed){
	return write_n(3,address,MIXEDLR,speed);
}

uint32_t RoboClaw::ReadEncM1(uint8_t address, uint8_t *status,bool *valid){
	return Read4_1(address,GETM1ENC,status,valid);
}

uint32_t RoboClaw::ReadEncM2(uint8_t address, uint8_t *status,bool *valid){
	return Read4_1(address,GETM2ENC,status,valid);
}

uint32_t RoboClaw::ReadSpeedM1(uint8_t address, uint8_t *status,bool *valid){
	return Read4_1(address,GETM1SPEED,status,valid);
}

uint32_t RoboClaw::ReadSpeedM2(uint8_t address, uint8_t *status,bool *valid){
	return Read4_1(address,GETM2SPEED,status,valid);
}

bool RoboClaw::ResetEncoders(uint8_t address){
	return write_n(2,address,RESETENC);
}

bool RoboClaw::ReadVersion(uint8_t address,char *version){
	int8_t data;
	uint8_t trys=MAXRETRY;
	do{
		flush();

		data = 0;

		crc_clear();
		write(address);
		crc_update(address);
		write(GETVERSION);
		crc_update(GETVERSION);

		uint8_t i;
		for(i=0;i<48;i++){
			if(data!=-1){
				data=read(timeout);
				version[i] = data;
				crc_update(version[i]);
				if(version[i]==0){
					uint16_t ccrc;
					data = read(timeout);
					if(data!=-1){
						ccrc = data << 8;
						data = read(timeout);
						if(data!=-1){
							ccrc |= data;
							return crc_get()==ccrc;
						}
					}
					break;
				}
			}
			else{
				break;
			}
		}
	}while(trys--);

	return false;
}

bool RoboClaw::SetEncM1(uint8_t address, int32_t val){
	return write_n(6,address,SETM1ENCCOUNT,SetDWORDval(val));
}

bool RoboClaw::SetEncM2(uint8_t address, int32_t val){
	return write_n(6,address,SETM2ENCCOUNT,SetDWORDval(val));
}

uint16_t RoboClaw::ReadMainBatteryVoltage(uint8_t address,bool *valid){
	return Read2(address,GETMBATT,valid);
}

uint16_t RoboClaw::ReadLogicBatteryVoltage(uint8_t address,bool *valid){
	return Read2(address,GETLBATT,valid);
}

bool RoboClaw::SetMinVoltageLogicBattery(uint8_t address, uint8_t voltage){
	return write_n(3,address,SETMINLB,voltage);
}

bool RoboClaw::SetMaxVoltageLogicBattery(uint8_t address, uint8_t voltage){
	return write_n(3,address,SETMAXLB,voltage);
}

bool RoboClaw::SetM1VelocityPID(uint8_t address, float kp_fp, float ki_fp, float kd_fp, uint32_t qpps){
	uint32_t kp = kp_fp*65536;
	uint32_t ki = ki_fp*65536;
	uint32_t kd = kd_fp*65536;
	return write_n(18,address,SETM1PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps));
}

bool RoboClaw::SetM2VelocityPID(uint8_t address, float kp_fp, float ki_fp, float kd_fp, uint32_t qpps){
	uint32_t kp = kp_fp*65536;
	uint32_t ki = ki_fp*65536;
	uint32_t kd = kd_fp*65536;
	return write_n(18,address,SETM2PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps));
}

uint32_t RoboClaw::ReadISpeedM1(uint8_t address,uint8_t *status,bool *valid){
	return Read4_1(address,GETM1ISPEED,status,valid);
}

uint32_t RoboClaw::ReadISpeedM2(uint8_t address,uint8_t *status,bool *valid){
	return Read4_1(address,GETM2ISPEED,status,valid);
}

bool RoboClaw::DutyM1(uint8_t address, uint16_t duty){
	return write_n(4,address,M1DUTY,SetWORDval(duty));
}

bool RoboClaw::DutyM2(uint8_t address, uint16_t duty){
	return write_n(4,address,M2DUTY,SetWORDval(duty));
}

bool RoboClaw::DutyM1M2(uint8_t address, uint16_t duty1, uint16_t duty2){
	return write_n(6,address,MIXEDDUTY,SetWORDval(duty1),SetWORDval(duty2));
}

bool RoboClaw::SpeedM1(uint8_t address, uint32_t speed){
	return write_n(6,address,M1SPEED,SetDWORDval(speed));
}

bool RoboClaw::SpeedM2(uint8_t address, uint32_t speed){
	return write_n(6,address,M2SPEED,SetDWORDval(speed));
}

bool RoboClaw::SpeedM1M2(uint8_t address, uint32_t speed1, uint32_t speed2){
	return write_n(10,address,MIXEDSPEED,SetDWORDval(speed1),SetDWORDval(speed2));
}

bool RoboClaw::SpeedAccelM1(uint8_t address, uint32_t accel, uint32_t speed){
	return write_n(10,address,M1SPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed));
}

bool RoboClaw::SpeedAccelM2(uint8_t address, uint32_t accel, uint32_t speed){
	return write_n(10,address,M2SPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed));
}
bool RoboClaw::SpeedAccelM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t speed2){
	return write_n(14,address,MIXEDSPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed1),SetDWORDval(speed2));
}

bool RoboClaw::SpeedDistanceM1(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag){
	return write_n(11,address,M1SPEEDDIST,SetDWORDval(speed),SetDWORDval(distance),flag);
}

bool RoboClaw::SpeedDistanceM2(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag){
	return write_n(11,address,M2SPEEDDIST,SetDWORDval(speed),SetDWORDval(distance),flag);
}

bool RoboClaw::SpeedDistanceM1M2(uint8_t address, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag){
	return write_n(19,address,MIXEDSPEEDDIST,SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

bool RoboClaw::SpeedAccelDistanceM1(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag){
	return write_n(15,address,M1SPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag);
}

bool RoboClaw::SpeedAccelDistanceM2(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag){
	return write_n(15,address,M2SPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag);
}

bool RoboClaw::SpeedAccelDistanceM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag){
	return write_n(23,address,MIXEDSPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

bool RoboClaw::ReadBuffers(uint8_t address, uint8_t &depth1, uint8_t &depth2){
	bool valid;
	uint16_t value = Read2(address,GETBUFFERS,&valid);
	if(valid){
		depth1 = value>>8;
		depth2 = value;
	}
	return valid;
}

bool RoboClaw::ReadPWMs(uint8_t address, int16_t &pwm1, int16_t &pwm2){
	bool valid;
	uint32_t value = Read4(address,GETPWMS,&valid);
	if(valid){
		pwm1 = value>>16;
		pwm2 = value&0xFFFF;
	}
	return valid;
}

bool RoboClaw::ReadCurrents(uint8_t address, int16_t &current1, int16_t &current2){
	bool valid;
	uint32_t value = Read4(address,GETCURRENTS,&valid);
	if(valid){
		current1 = value>>16;
		current2 = value&0xFFFF;
	}
	return valid;
}

bool RoboClaw::SpeedAccelM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t accel2, uint32_t speed2){
	return write_n(18,address,MIXEDSPEED2ACCEL,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(accel2),SetDWORDval(speed2));
}

bool RoboClaw::SpeedAccelDistanceM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t distance1, uint32_t accel2, uint32_t speed2, uint32_t distance2, uint8_t flag){
	return write_n(27,address,MIXEDSPEED2ACCELDIST,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(accel2),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

bool RoboClaw::DutyAccelM1(uint8_t address, uint16_t duty, uint32_t accel){
	return write_n(8,address,M1DUTYACCEL,SetWORDval(duty),SetDWORDval(accel));
}

bool RoboClaw::DutyAccelM2(uint8_t address, uint16_t duty, uint32_t accel){
	return write_n(8,address,M2DUTYACCEL,SetWORDval(duty),SetDWORDval(accel));
}

bool RoboClaw::DutyAccelM1M2(uint8_t address, uint16_t duty1, uint32_t accel1, uint16_t duty2, uint32_t accel2){
	return write_n(14,address,MIXEDDUTYACCEL,SetWORDval(duty1),SetDWORDval(accel1),SetWORDval(duty2),SetDWORDval(accel2));
}

bool RoboClaw::ReadM1VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps){
	uint32_t Kp,Ki,Kd;
	bool valid = read_n(4,address,READM1PID,&Kp,&Ki,&Kd,&qpps);
	Kp_fp = ((float)Kp)/65536;
	Ki_fp = ((float)Ki)/65536;
	Kd_fp = ((float)Kd)/65536;
	return valid;
}

bool RoboClaw::ReadM2VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps){
	uint32_t Kp,Ki,Kd;
	bool valid = read_n(4,address,READM2PID,&Kp,&Ki,&Kd,&qpps);
	Kp_fp = ((float)Kp)/65536;
	Ki_fp = ((float)Ki)/65536;
	Kd_fp = ((float)Kd)/65536;
	return valid;
}

bool RoboClaw::SetMainVoltages(uint8_t address,uint16_t min,uint16_t max){
	return write_n(6,address,SETMAINVOLTAGES,SetWORDval(min),SetWORDval(max));
}

bool RoboClaw::SetLogicVoltages(uint8_t address,uint16_t min,uint16_t max){
	return write_n(6,address,SETLOGICVOLTAGES,SetWORDval(min),SetWORDval(max));
}

bool RoboClaw::ReadMinMaxMainVoltages(uint8_t address,uint16_t &min,uint16_t &max){
	bool valid;
	uint32_t value = Read4(address,GETMINMAXMAINVOLTAGES,&valid);
	if(valid){
		min = value>>16;
		max = value&0xFFFF;
	}
	return valid;
}

bool RoboClaw::ReadMinMaxLogicVoltages(uint8_t address,uint16_t &min,uint16_t &max){
	bool valid;
	uint32_t value = Read4(address,GETMINMAXLOGICVOLTAGES,&valid);
	if(valid){
		min = value>>16;
		max = value&0xFFFF;
	}
	return valid;
}

bool RoboClaw::SetM1PositionPID(uint8_t address,float kp_fp,float ki_fp,float kd_fp,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max){
	uint32_t kp=kp_fp*1024;
	uint32_t ki=ki_fp*1024;
	uint32_t kd=kd_fp*1024;
	return write_n(30,address,SETM1POSPID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(kiMax),SetDWORDval(deadzone),SetDWORDval(min),SetDWORDval(max));
}

bool RoboClaw::SetM2PositionPID(uint8_t address,float kp_fp,float ki_fp,float kd_fp,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max){
	uint32_t kp=kp_fp*1024;
	uint32_t ki=ki_fp*1024;
	uint32_t kd=kd_fp*1024;
	return write_n(30,address,SETM2POSPID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(kiMax),SetDWORDval(deadzone),SetDWORDval(min),SetDWORDval(max));
}

bool RoboClaw::ReadM1PositionPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max){
	uint32_t Kp,Ki,Kd;
	bool valid = read_n(7,address,READM1POSPID,&Kp,&Ki,&Kd,&KiMax,&DeadZone,&Min,&Max);
	Kp_fp = ((float)Kp)/1024;
	Ki_fp = ((float)Ki)/1024;
	Kd_fp = ((float)Kd)/1024;
	return valid;
}

bool RoboClaw::ReadM2PositionPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max){
	uint32_t Kp,Ki,Kd;
	bool valid = read_n(7,address,READM2POSPID,&Kp,&Ki,&Kd,&KiMax,&DeadZone,&Min,&Max);
	Kp_fp = ((float)Kp)/1024;
	Ki_fp = ((float)Ki)/1024;
	Kd_fp = ((float)Kd)/1024;
	return valid;
}

bool RoboClaw::SpeedAccelDeccelPositionM1(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag){
	return write_n(19,address,M1SPEEDACCELDECCELPOS,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(deccel),SetDWORDval(position),flag);
}

bool RoboClaw::SpeedAccelDeccelPositionM2(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag){
	return write_n(19,address,M2SPEEDACCELDECCELPOS,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(deccel),SetDWORDval(position),flag);
}

bool RoboClaw::SpeedAccelDeccelPositionM1M2(uint8_t address,uint32_t accel1,uint32_t speed1,uint32_t deccel1,uint32_t position1,uint32_t accel2,uint32_t speed2,uint32_t deccel2,uint32_t position2,uint8_t flag){
	return write_n(35,address,MIXEDSPEEDACCELDECCELPOS,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(deccel1),SetDWORDval(position1),SetDWORDval(accel2),SetDWORDval(speed2),SetDWORDval(deccel2),SetDWORDval(position2),flag);
}

bool RoboClaw::SetM1DefaultAccel(uint8_t address, uint32_t accel){
	return write_n(6,address,SETM1DEFAULTACCEL,SetDWORDval(accel));
}

bool RoboClaw::SetM2DefaultAccel(uint8_t address, uint32_t accel){
	return write_n(6,address,SETM2DEFAULTACCEL,SetDWORDval(accel));
}

bool RoboClaw::SetPinFunctions(uint8_t address, uint8_t S3mode, uint8_t S4mode, uint8_t S5mode){
	return write_n(5,address,SETPINFUNCTIONS,S3mode,S4mode,S5mode);
}

bool RoboClaw::GetPinFunctions(uint8_t address, uint8_t &S3mode, uint8_t &S4mode, uint8_t &S5mode){
	uint8_t val1,val2,val3;
	uint8_t trys=MAXRETRY;
	int16_t data;
	do{
		flush();

		crc_clear();
		write(address);
		crc_update(address);
		write(GETPINFUNCTIONS);
		crc_update(GETPINFUNCTIONS);

		data = read(timeout);
		crc_update(data);
		val1=data;

		if(data!=-1){
			data = read(timeout);
			crc_update(data);
			val2=data;
		}

		if(data!=-1){
			data = read(timeout);
			crc_update(data);
			val3=data;
		}

		if(data!=-1){
			uint16_t ccrc;
			data = read(timeout);
			if(data!=-1){
				ccrc = data << 8;
				data = read(timeout);
				if(data!=-1){
					ccrc |= data;
					if(crc_get()==ccrc){
						S3mode = val1;
						S4mode = val2;
						S5mode = val3;
						return true;
					}
				}
			}
		}
	}while(trys--);

	return false;
}

bool RoboClaw::SetDeadBand(uint8_t address, uint8_t Min, uint8_t Max){
	return write_n(4,address,SETDEADBAND,Min,Max);
}

bool RoboClaw::GetDeadBand(uint8_t address, uint8_t &Min, uint8_t &Max){
	bool valid;
	uint16_t value = Read2(address,GETDEADBAND,&valid);
	if(valid){
		Min = value>>8;
		Max = value;
	}
	return valid;
}

bool RoboClaw::ReadEncoders(uint8_t address,uint32_t &enc1,uint32_t &enc2){
	bool valid = read_n(2,address,GETENCODERS,&enc1,&enc2);
	return valid;
}

bool RoboClaw::ReadISpeeds(uint8_t address,uint32_t &ispeed1,uint32_t &ispeed2){
	bool valid = read_n(2,address,GETISPEEDS,&ispeed1,&ispeed2);
	return valid;
}

bool RoboClaw::RestoreDefaults(uint8_t address){
	return write_n(2,address,RESTOREDEFAULTS);
}

bool RoboClaw::ReadTemp(uint8_t address, uint16_t &temp){
	bool valid;
	temp = Read2(address,GETTEMP,&valid);
	return valid;
}

bool RoboClaw::ReadTemp2(uint8_t address, uint16_t &temp){
	bool valid;
	temp = Read2(address,GETTEMP2,&valid);
	return valid;
}

uint16_t RoboClaw::ReadError(uint8_t address,bool *valid){
	return Read2(address,GETERROR,valid);
}

bool RoboClaw::ReadEncoderModes(uint8_t address, uint8_t &M1mode, uint8_t &M2mode){
	bool valid;
	uint16_t value = Read2(address,GETENCODERMODE,&valid);
	if(valid){
		M1mode = value>>8;
		M2mode = value;
	}
	return valid;
}

bool RoboClaw::SetM1EncoderMode(uint8_t address,uint8_t mode){
	return write_n(3,address,SETM1ENCODERMODE,mode);
}

bool RoboClaw::SetM2EncoderMode(uint8_t address,uint8_t mode){
	return write_n(3,address,SETM2ENCODERMODE,mode);
}

bool RoboClaw::WriteNVM(uint8_t address){
	return write_n(6,address,WRITENVM, SetDWORDval(0xE22EAB7A) );
}

bool RoboClaw::ReadNVM(uint8_t address){
	return write_n(2,address,READNVM);
}

bool RoboClaw::SetConfig(uint8_t address, uint16_t config){
	return write_n(4,address,SETCONFIG,SetWORDval(config));
}

bool RoboClaw::GetConfig(uint8_t address, uint16_t &config){
	bool valid;
	uint16_t value = Read2(address,GETCONFIG,&valid);
	if(valid){
		config = value;
	}
	return valid;
}

bool RoboClaw::SetM1MaxCurrent(uint8_t address,uint32_t max){
	return write_n(10,address,SETM1MAXCURRENT,SetDWORDval(max),SetDWORDval(0));
}

bool RoboClaw::SetM2MaxCurrent(uint8_t address,uint32_t max){
	return write_n(10,address,SETM2MAXCURRENT,SetDWORDval(max),SetDWORDval(0));
}

bool RoboClaw::ReadM1MaxCurrent(uint8_t address,uint32_t &max){
	uint32_t tmax,dummy;
	bool valid = read_n(2,address,GETM1MAXCURRENT,&tmax,&dummy);
	if(valid)
		max = tmax;
	return valid;
}

bool RoboClaw::ReadM2MaxCurrent(uint8_t address,uint32_t &max){
	uint32_t tmax,dummy;
	bool valid = read_n(2,address,GETM2MAXCURRENT,&tmax,&dummy);
	if(valid)
		max = tmax;
	return valid;
}

bool RoboClaw::SetPWMMode(uint8_t address, uint8_t mode){
	return write_n(3,address,SETPWMMODE,mode);
}

bool RoboClaw::GetPWMMode(uint8_t address, uint8_t &mode){
	bool valid;
	uint8_t value = Read1(address,GETPWMMODE,&valid);
	if(valid){
		mode = value;
	}
	return valid;
}
