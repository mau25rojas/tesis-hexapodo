#include "ros/ros.h"
#include "std_msgs/String.h"
#include "comm.hpp"
#include "camina/commData.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <fcntl.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <termios.h>
#include <string>
#include <vector>
#include <stdint.h>
#include <boost/function.hpp>
#include <boost/thread/thread.hpp>

#define MAX_LENGTH 128

namespace cereal
{
	//! Macro for defining an exception with a given parent (std::runtime_error should be top parent)
	#define DEF_EXCEPTION(name, parent) \
	class name : public parent { \
  		public: \
    	name(const char* msg) : parent(msg) {} \
  	}

  	//! A standard exception
  	DEF_EXCEPTION(Exception, std::runtime_error);

	//! An exception for use when a timeout is exceeded
  	DEF_EXCEPTION(TimeoutException, Exception);

	#undef DEF_EXCEPTION

	/*! \class CerealPort CerealPort.h "inc/CerealPort.h"
	 *  \brief C++ serial port class for ROS.
	 *
	 * This class was based on the serial port code found on the hokuyo_node as suggested by Blaise Gassend on the ros-users mailling list.
	 */
	class CerealPort
	{
		public:
		//! Constructor
		CerealPort();
		//! Destructor
		~CerealPort();

		//! Open the serial port
		/*!
		* This opens the serial port for communication. Wrapper for open.
		*
		* \param port_name   A null terminated character array containing the name of the port.
		* \param baud_rate   Baud rate of the serial port. Defaults to 115200.
		*
		*/
		void open(const char * port_name, int baud_rate = 115200);

		//! Close the serial port
		/*!
		* This call essentially wraps close.
		*/
		void close();

		//! Check whether the port is open or not
		bool portOpen() { return fd_ != -1; }

		//! Get the current baud rate
		int baudRate() { return baud_; }

		//! Write to the port
		/*!
		*  This function allows to send data through the serial port. Wraper for write.
		*
		*  \param data    Data to send in a character array or null terminated c string.
		*  \param length  Number of bytes being sent. Defaults to -1 if sending a c string.
		*
		*  \return Number of bytes writen.
		*/
		int write(const char * data, int length = -1);

		//! Read from the port
		/*!
		*  This function allows to read data from the serial port. Simple wrapper for read.
		*
		*  \param data    		Data coming from the serial port.
		*  \param max_length  	Maximum length of the incoming data.
		*  \param timeout 		Timeout in milliseconds.
		*
		*  \return Number of bytes read.
		*/
	    int read(char * data, int max_length, int timeout = -1);

		//! Read a fixed number of bytes from the serial port
		/*!
		*  This function allows to read a fixed number of data bytes from the serial port, no more, no less.
		*
		*  \param data    Data coming from the serial port.
		*  \param length  Fixed length of the incoming data.
		*  \param timeout Timeout in milliseconds.
		*
		*  \sa read()
		*
		*  \return Number of bytes read.
		*/
	    int readBytes(char * data, int length, int timeout = -1);

	   	//! Read a line from the serial port
		/*!
		*  This function allows to read a line from the serial port. Data is return as char*
		*
		*  \param data    	Data coming from the serial port.
		*  \param length  	Length of the incoming data.
		*  \param timeout	Timeout in milliseconds.
		*
		*  \sa readLine(std::string*, int)
		*
		*  \return Number of bytes read.
		*/
	    int readLine(char * data, int length, int timeout = -1);

	    //! Read a line from the serial port
		/*!
		*  This function allows to read a line from the serial port. Data is return as std::string
		*
		*  \param data    	Data coming from the serial port.
		*  \param timeout	Timeout in milliseconds.
		*
		*  \sa readLine(char*, int, int)
		*
		*  \return Whether the read was successful or not.
		*/
	    bool readLine(std::string * data, int timeout = -1);

	    //! Read from the serial port between a start char and an end char
		/*!
		*  This function allows to read data from the serial port between a start and an end char.
		*
		*  \param data    	Data coming from the serial port.
		*  \param start		Start character of the incoming data stream.
		*  \param end		End character of the incoming data stream.
		*  \param timeout	Timeout in milliseconds.
		*
		*  \return Whether the read was successful or not.
		*/
		//TODO: int readBetween(char * data, int length, char start, char end, int timeout = -1);
	    bool readBetween(std::string * data, char start, char end, int timeout = -1);

	    //! Wrapper around tcflush
	    int flush();

	    //*** Stream functions ***

	    //! Start a stream of read()
		/*!
		*  Stream version of the read function.
		*
		*  \param f    		Callback boost function to receive the data.
		*
		*  \sa read()
		*
		*  \return True if successful false if a stream is already running.
		*/
	    bool startReadStream(boost::function<void(char*, int)> f);

	    //! Start a stream of readLine(std::string*, int)
		/*!
		*  Stream version of the readLine(std::string*, int) function.
		*
		*  \param f    		Callback boost function to receive the data.
		*
		*  \sa readLine(std::string*, int)
		*
		*  \return True if successful false if a stream is already running.
		*/
	    bool startReadLineStream(boost::function<void(std::string*)> f);

	    //! Start a stream of readBetween()
		/*!
		*  Stream version of the readBetween() function.
		*
		*  \param f    		Callback boost function to receive the data.
		*  \param start		Start character of the incoming data stream.
		*  \param end		End character of the incoming data stream.
		*
		*  \sa readBetween()
		*
		*  \return True if successful false if a stream is already running.
		*/
	    bool startReadBetweenStream(boost::function<void(std::string*)> f, char start, char end);

	    //! Stop streaming
		void stopStream();
		//! Pause streaming
		void pauseStream();
		//! Resume streaming
		void resumeStream();

		private:
		//! File descriptor
	   	int fd_;
	   	//! Baud rate
		int baud_;

		//std::vector<char> leftovers;

		//! Thread for a stream of read()
		/*!
		*  Stream version of the read function.
		*/
		void readThread();

		//! Thread for a stream of readLine(std::string*, int)
		/*!
		*  Stream version of the readLine function.
		*/
	    void readLineThread();

	    //! Thread for a stream of readBetween()
		/*!
		*  Stream version of the readBetween function.
		*
		*  \param start		Start character of the incoming data stream.
		*  \param end		End character of the incoming data stream.
		*
		*/
	    void readBetweenThread(char start, char end);

		//! Stream thread
		boost::thread * stream_thread_;

		//! Stream read callback boost function
		boost::function<void(char*, int)> readCallback;
		//! Stream readLine callback boost function
		boost::function<void(std::string*)> readLineCallback;
		//! Stream readBetween callback boost function
		boost::function<void(std::string*)> readBetweenCallback;

		//! Whether streaming is paused or not
		bool stream_paused_;
		//! Whether streaming is stopped or not
		bool stream_stopped_;
	};

}


//! Macro for throwing an exception with a message, passing args
#define CEREAL_EXCEPT(except, msg, ...) \
{ \
    char buf[1000]; \
    snprintf(buf, 1000, msg " (in cereal::CerealPort::%s)" , ##__VA_ARGS__, __FUNCTION__); \
    throw except(buf); \
}

cereal::CerealPort::CerealPort() : fd_(-1)
{
	stream_thread_ = NULL;
}

cereal::CerealPort::~CerealPort()
{
	if(portOpen()) close();
}

void cereal::CerealPort::open(const char * port_name, int baud_rate)
{
	if(portOpen()) close();

	// Make IO non blocking. This way there are no race conditions that
	// cause blocking when a badly behaving process does a read at the same
	// time as us. Will need to switch to blocking for writes or errors
	// occur just after a replug event.
	fd_ = ::open(port_name, O_RDWR | O_NONBLOCK | O_NOCTTY);

	if(fd_ == -1)
	{
		const char *extra_msg = "";
		switch(errno)
		{
			case EACCES:
			extra_msg = "You probably don't have premission to open the port for reading and writing.";
			break;

			case ENOENT:
			extra_msg = "The requested port does not exist. Is the hokuyo connected? Was the port name misspelled?";
			break;
		}
		CEREAL_EXCEPT(cereal::Exception, "Failed to open port: %s. %s (errno = %d). %s", port_name, strerror(errno), errno, extra_msg);
	}

	try
	{
		struct flock fl;
		fl.l_type = F_WRLCK;
		fl.l_whence = SEEK_SET;
		fl.l_start = 0;
		fl.l_len = 0;
		fl.l_pid = getpid();

		if(fcntl(fd_, F_SETLK, &fl) != 0)
			CEREAL_EXCEPT(cereal::Exception, "Device %s is already locked. Try 'lsof | grep %s' to find other processes that currently have the port open.", port_name, port_name);

		// Settings for USB?
		struct termios newtio;
		tcgetattr(fd_, &newtio);
		memset (&newtio.c_cc, 0, sizeof (newtio.c_cc));
		newtio.c_cflag = CS8 | CLOCAL | CREAD;
		newtio.c_iflag = IGNPAR;
		newtio.c_oflag = 0;
		newtio.c_lflag = 0;
		cfsetspeed(&newtio, baud_rate);
		baud_ = baud_rate;

		// Activate new settings
		tcflush(fd_, TCIFLUSH);
		if(tcsetattr(fd_, TCSANOW, &newtio) < 0)
			CEREAL_EXCEPT(cereal::Exception, "Unable to set serial port attributes. The port you specified (%s) may not be a serial port.", port_name); /// @todo tcsetattr returns true if at least one attribute was set. Hence, we might not have set everything on success.
		usleep (200000);
	}
	catch(cereal::Exception& e)
	{
		// These exceptions mean something failed on open and we should close
		if(fd_ != -1) ::close(fd_);
		fd_ = -1;
		throw e;
	}
}

void cereal::CerealPort::close()
{
	int retval = 0;

  	retval = ::close(fd_);

  	fd_ = -1;

  	if(retval != 0)
    		CEREAL_EXCEPT(cereal::Exception, "Failed to close port properly -- error = %d: %s\n", errno, strerror(errno));
}

int cereal::CerealPort::write(const char * data, int length)
{
	int len = length==-1 ? strlen(data) : length;

	// IO is currently non-blocking. This is what we want for the more cerealon read case.
	int origflags = fcntl(fd_, F_GETFL, 0);
	fcntl(fd_, F_SETFL, origflags & ~O_NONBLOCK); // TODO: @todo can we make this all work in non-blocking?
	int retval = ::write(fd_, data, len);
	fcntl(fd_, F_SETFL, origflags | O_NONBLOCK);

	if(retval == len) return retval;
	else CEREAL_EXCEPT(cereal::Exception, "write failed");
}

int cereal::CerealPort::read(char * buffer, int max_length, int timeout)
{
	int ret;

	struct pollfd ufd[1];
	int retval;
	ufd[0].fd = fd_;
	ufd[0].events = POLLIN;

	if(timeout == 0) timeout = -1; // For compatibility with former behavior, 0 means no timeout. For poll, negative means no timeout.

	if((retval = poll(ufd, 1, timeout)) < 0) CEREAL_EXCEPT(cereal::Exception, "poll failed -- error = %d: %s", errno, strerror(errno));

	if(retval == 0) CEREAL_EXCEPT(cereal::TimeoutException, "timeout reached");

	if(ufd[0].revents & POLLERR) CEREAL_EXCEPT(cereal::Exception, "error on socket, possibly unplugged");

    ret = ::read(fd_, buffer, max_length);

	if(ret == -1 && errno != EAGAIN && errno != EWOULDBLOCK) CEREAL_EXCEPT(cereal::Exception, "read failed");

	return ret;
}

int cereal::CerealPort::readBytes(char * buffer, int length, int timeout)
{
	int ret;
	int current = 0;

	struct pollfd ufd[1];
	int retval;
	ufd[0].fd = fd_;
	ufd[0].events = POLLIN;

	if(timeout == 0) timeout = -1; // For compatibility with former behavior, 0 means no timeout. For poll, negative means no timeout.

	while(current < length)
	{
		if((retval = poll(ufd, 1, timeout)) < 0) CEREAL_EXCEPT(cereal::Exception, "poll failed -- error = %d: %s", errno, strerror(errno));

		if(retval == 0) CEREAL_EXCEPT(cereal::TimeoutException, "timeout reached");

		if(ufd[0].revents & POLLERR) CEREAL_EXCEPT(cereal::Exception, "error on socket, possibly unplugged");

      	ret = ::read(fd_, &buffer[current], length-current);

		if(ret == -1 && errno != EAGAIN && errno != EWOULDBLOCK) CEREAL_EXCEPT(cereal::Exception, "read failed");

		current += ret;
	}
	return current;
}

int cereal::CerealPort::readLine(char * buffer, int length, int timeout)
{
	int ret;
	int current = 0;

	struct pollfd ufd[1];
	int retval;
	ufd[0].fd = fd_;
	ufd[0].events = POLLIN;

	if(timeout == 0) timeout = -1; // For compatibility with former behavior, 0 means no timeout. For poll, negative means no timeout.

	while(current < length-1)
	{
		if(current > 0)
			if(buffer[current-1] == '\n')
				return current;

		if((retval = poll(ufd, 1, timeout)) < 0) CEREAL_EXCEPT(cereal::Exception, "poll failed -- error = %d: %s", errno, strerror(errno));

		if(retval == 0) CEREAL_EXCEPT(cereal::TimeoutException, "timeout reached");

		if(ufd[0].revents & POLLERR) CEREAL_EXCEPT(cereal::Exception, "error on socket, possibly unplugged");

      	ret = ::read(fd_, &buffer[current], length-current);

		if(ret == -1 && errno != EAGAIN && errno != EWOULDBLOCK) CEREAL_EXCEPT(cereal::Exception, "read failed");

		current += ret;
	}
	CEREAL_EXCEPT(cereal::Exception, "buffer filled without end of line being found");
}

bool cereal::CerealPort::readLine(std::string * buffer, int timeout)
{
	int ret;

	struct pollfd ufd[1];
	int retval;
	ufd[0].fd = fd_;
	ufd[0].events = POLLIN;

	if(timeout == 0) timeout = -1; // For compatibility with former behavior, 0 means no timeout. For poll, negative means no timeout.

	buffer->clear();
	while(buffer->size() < buffer->max_size()/2)
	{
		// Look for the end char
		ret = buffer->find_first_of('\n');
		if(ret > 0)
		{
			// If it is there clear everything after it and return
			buffer->erase(ret+1, buffer->size()-ret-1);
			return true;
		}

		if((retval = poll(ufd, 1, timeout)) < 0) CEREAL_EXCEPT(cereal::Exception, "poll failed -- error = %d: %s", errno, strerror(errno));

		if(retval == 0) CEREAL_EXCEPT(cereal::TimeoutException, "timeout reached");

		if(ufd[0].revents & POLLERR) CEREAL_EXCEPT(cereal::Exception, "error on socket, possibly unplugged");

      	char temp_buffer[128];
      	ret = ::read(fd_, temp_buffer, 128);

		if(ret == -1 && errno != EAGAIN && errno != EWOULDBLOCK) CEREAL_EXCEPT(cereal::Exception, "read failed");

		// Append the new data to the buffer
      	buffer->append(temp_buffer, ret);
	}
	CEREAL_EXCEPT(cereal::Exception, "buffer filled without end of line being found");
}

bool cereal::CerealPort::readBetween(std::string * buffer, char start, char end, int timeout)
{
	int ret;

	struct pollfd ufd[1];
	int retval;
	ufd[0].fd = fd_;
	ufd[0].events = POLLIN;

	if(timeout == 0) timeout = -1; // For compatibility with former behavior, 0 means no timeout. For poll, negative means no timeout.

	// Clear the buffer before we start
	buffer->clear();
	while(buffer->size() < buffer->max_size()/2)
	{
		if((retval = poll(ufd, 1, timeout)) < 0) CEREAL_EXCEPT(cereal::Exception, "poll failed -- error = %d: %s", errno, strerror(errno));

		if(retval == 0) CEREAL_EXCEPT(cereal::TimeoutException, "timeout reached");

		if(ufd[0].revents & POLLERR) CEREAL_EXCEPT(cereal::Exception, "error on socket, possibly unplugged");

		char temp_buffer[128];
  		ret = ::read(fd_, temp_buffer, 128);

  		if(ret == -1 && errno != EAGAIN && errno != EWOULDBLOCK) CEREAL_EXCEPT(cereal::Exception, "read failed");

  		// Append the new data to the buffer
  		buffer->append(temp_buffer, ret);

      	// Look for the start char
      	ret = buffer->find_first_of(start);
      	// If it is not on the buffer, clear it
      	if(ret == -1) buffer->clear();
      	// If it is there, but not on the first position clear everything behind it
      	else if(ret > 0) buffer->erase(0, ret);

		// Look for the end char
		ret = buffer->find_first_of(end);
		if(ret > 0)
		{
			// If it is there clear everything after it and return
			buffer->erase(ret+1, buffer->size()-ret-1);
			return true;
		}
	}
	CEREAL_EXCEPT(cereal::Exception, "buffer filled without reaching end of data stream");
}

int cereal::CerealPort::flush()
{
	  int retval = tcflush(fd_, TCIOFLUSH);
	  if(retval != 0) CEREAL_EXCEPT(cereal::Exception, "tcflush failed");

	  return retval;
}

bool cereal::CerealPort::startReadStream(boost::function<void(char*, int)> f)
{
	if(stream_thread_ != NULL) return false;

	stream_stopped_ = false;
	stream_paused_ = false;

	readCallback = f;

	stream_thread_ = new boost::thread(boost::bind(&cereal::CerealPort::readThread, this));
	return true;
}

void cereal::CerealPort::readThread()
{
	char data[MAX_LENGTH];
	int ret;

	struct pollfd ufd[1];
	ufd[0].fd = fd_;
	ufd[0].events = POLLIN;

	while(!stream_stopped_)
	{
		if(!stream_paused_)
		{
			if(poll(ufd, 1, 10) > 0)
			{
				if(!(ufd[0].revents & POLLERR))
				{
			  		ret = ::read(fd_, data, MAX_LENGTH);
			  		if(ret>0)
			  		{
			  			readCallback(data, ret);
			  		}
			  	}
			}
		}
	}
}

bool cereal::CerealPort::startReadLineStream(boost::function<void(std::string*)> f)
{
	if(stream_thread_ != NULL) return false;

	stream_stopped_ = false;
	stream_paused_ = false;

	readLineCallback = f;

	stream_thread_ = new boost::thread(boost::bind(&cereal::CerealPort::readLineThread, this));
	return true;
}

void cereal::CerealPort::readLineThread()
{
	std::string data;
	bool error = false;

	while(!stream_stopped_)
	{
		if(!stream_paused_)
		{
			error = false;
			try{ readLine(&data, 100); }
			catch(cereal::Exception& e)
			{
				error = true;
			}

			if(!error && data.size()>0) readLineCallback(&data);
		}
	}
}

bool cereal::CerealPort::startReadBetweenStream(boost::function<void(std::string*)> f, char start, char end)
{
	if(stream_thread_ != NULL) return false;

	stream_stopped_ = false;
	stream_paused_ = false;

	readBetweenCallback = f;

	stream_thread_ = new boost::thread(boost::bind(&cereal::CerealPort::readBetweenThread, this, start, end));
	return true;
}

void cereal::CerealPort::readBetweenThread(char start, char end)
{
	std::string data;
	bool error = false;

	while(!stream_stopped_)
	{
		if(!stream_paused_)
		{
			error = false;
			try{ readBetween(&data, start, end, 100); }
			catch(cereal::Exception& e)
			{
				error = true;
			}

			if(!error && data.size()>0) readBetweenCallback(&data);
		}
	}
}

void cereal::CerealPort::stopStream()
{
	stream_stopped_ = true;
	stream_thread_->join();

	delete stream_thread_;
	stream_thread_ = NULL;
}

void cereal::CerealPort::pauseStream()
{
	stream_paused_ = true;
}

void cereal::CerealPort::resumeStream()
{
	stream_paused_ = false;
}

/******************************************************************************/

cereal::CerealPort cp;

void spinThread()
{
	ros::spin();
}

void commTxCallback(const camina::commData trama)
{
	char dato;
	ROS_INFO("COMM: recibo dato\n");
		dato = trama.data_hexapodo;
		cp.write(&dato, 1);

	ROS_INFO("COMM:\n");
	ROS_INFO("[%s]: 0x%X.", COMM_MSG_TX, dato);
}

int main(int argc, char **argv)
{
	//char data;
	ros::init(argc, argv, "comm");
	ros::NodeHandle nx;

	// Abro el puerto serial
	ROS_INFO("Abriendo puerto [%s] a [%d] bps", COMM_PORT, COMM_BAUDRATE);
	ROS_INFO("Publicando en: [%s]", COMM_MSG_RX);
	ROS_INFO("Subscrito  a : [%s]", COMM_MSG_TX);
	cp.open(COMM_PORT, COMM_BAUDRATE);

	ros::Subscriber sub        = nx.subscribe("COMM_MSG_TX", 50, commTxCallback);
	ros::Publisher chatter_pub = nx.advertise<camina::commData> ("COMM_MSG_RX", 50);

//	boost::thread spin_thread(&spinThread);

	camina::commData msg;
	while (ros::ok())
	{	char data;
//		cp.readBytes(&data, 1, 0);
//		msg.data_hexapodo = data;
//		ROS_INFO("[%s]: 0x%x ", COMM_MSG_RX, data);
//		ROS_INFO("comm: hola");
		chatter_pub.publish(msg);
		ros::spinOnce();
	}
	ros::shutdown();
//	spin_thread.join();

	return 0;
}
