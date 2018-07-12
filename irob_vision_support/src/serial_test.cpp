
#include <ros/ros.h>

#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>


ros::Publisher pub;



int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "serial_test");
  ros::NodeHandle nh;

  const char *device = "/dev/ttyUSB0";

  struct termios tio;
       struct termios stdio;
       int tty_fd;
       fd_set rdset;

       unsigned char c='D';

       memset(&stdio,0,sizeof(stdio));
       stdio.c_iflag=0;
       stdio.c_oflag=0;
       stdio.c_cflag=0;
       stdio.c_lflag=0;
       stdio.c_cc[VMIN]=1;
       stdio.c_cc[VTIME]=0;
       tcsetattr(STDOUT_FILENO,TCSANOW,&stdio);
       tcsetattr(STDOUT_FILENO,TCSAFLUSH,&stdio);
       fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);       // make the reads non-blocking




       memset(&tio,0,sizeof(tio));
       tio.c_iflag=0;
       tio.c_oflag=0;
       tio.c_cflag=CS8|CREAD|CLOCAL;           // 8n1, see termios.h for more information
       tio.c_lflag=0;
       tio.c_cc[VMIN]=1;
       tio.c_cc[VTIME]=5;

       tty_fd=open(device, O_RDWR | O_NONBLOCK);        // O_NONBLOCK might override VMIN and VTIME, so read() may return immediately.
       cfsetospeed(&tio,B115200);            // 115200 baud
       cfsetispeed(&tio,B115200);            // 115200 baud

       tcsetattr(tty_fd,TCSANOW,&tio);
       while (c!='q')
       {
               if (read(tty_fd,&c,1)>0)  {

                 write(tty_fd,&c,1);
                 ROS_INFO_STREAM(c);
                 }// if new data is available on the console, send it to the serial port
       }

       close(tty_fd);


  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub = nh.subscribe("/saf/stereo/points2", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  //pub = nh.advertise<sensor_msgs::PointCloud2>("/saf/vision/cylinders", 1);

  // Spin
  ros::spin ();
}
