#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <unistd.h>

int BAUDRATE = B115200;

int open_serial(const char *device_name)
{
    int fd1 = open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    fcntl(fd1, F_SETFL, 0);
    //load configuration
    struct termios conf_tio;
    tcgetattr(fd1, &conf_tio);
    //set baudrate
    //speed_t BAUDRATE = B115200;
    cfsetispeed(&conf_tio, BAUDRATE);
    cfsetospeed(&conf_tio, BAUDRATE);
    //non canonical, non echo back
    conf_tio.c_lflag &= ~(ECHO | ICANON);
    conf_tio.c_oflag &= ~(ONLCR | OCRNL);
    //non blocking
    conf_tio.c_cc[VMIN] = 0;
    conf_tio.c_cc[VTIME] = 0;
    //store configuration
    tcsetattr(fd1, TCSANOW, &conf_tio);
    return fd1;
}

int fd1 = 0;
char endmsg = '\n';
bool subflag = false;
int sleeptime = 5000; //us
int sub_loop_rate = 200;
char *floattochar;
int floatdatasize = 0;


void sub_callback(const std_msgs::Float32MultiArray &serial_msg)
{
    if (subflag)
    {
        usleep(sleeptime);
        subflag == false;
    }
    delete[] floattochar;
    floatdatasize = serial_msg.data.size();
    floattochar = new char[floatdatasize * 4 + 6];
    floattochar[0] = 'f';
    *(int *)(&floattochar[1]) = floatdatasize;
    //memcpy(&floattochar[1], &datasize, 4);
    for (int i = 0; i < floatdatasize; i++)
    {
        *(float *)(&floattochar[i * 4 + 5]) = serial_msg.data[i];
        //memcpy(&floattochar[i * 4 + 5], &serial_msg.data[i], 4);
    }
    floattochar[floatdatasize * 4 + 5] = endmsg;

    subflag = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rogi_serial_node");
    ros::NodeHandle n;

    //Publisher
    ros::Publisher serial_pub = n.advertise<std_msgs::Float32MultiArray>("Serial_pub", 1000);

    //Subscriber
    ros::Subscriber serial_sub = n.subscribe("Serial_sub", 100, sub_callback);

    // Parameter
    ros::NodeHandle arg_n("~");
    std::string port_name = "/dev/ttyACM0";
    arg_n.getParam("port", port_name);
    arg_n.getParam("baudrate", BAUDRATE);
    arg_n.getParam("looprate", sub_loop_rate);

    fd1 = open_serial(port_name.c_str());

    while (ros::ok())
    {
        fd1 = open_serial(port_name.c_str());
        //ROS_ERROR("Serial Fail: cound not open %s", port_name.c_str());
        //ros::shutdown();
        printf("Serial Connecting\n");
        sleep(1);
        if (fd1 >= 0)
            break;
    }

    ROS_INFO("Serial Success");

    char buf_pub[256] = {0};
    int recv_data_size = 0;
    int arraysize = 0;
    int rec;
    ros::Rate loop_rate(sub_loop_rate);

    // remove initial_buff_data
    for (int i = 0; i < 1000; i++)
    {
        read(fd1, &buf_pub[0], sizeof(buf_pub));
        usleep(1000);
    }

    while (ros::ok())
    {
        int recv_data = read(fd1, &buf_pub[recv_data_size], sizeof(buf_pub));

        if (recv_data > 0)
        {
            recv_data_size += recv_data;
            if (recv_data_size >= 256)
            {
                recv_data_size = 0;
            }
            else if (buf_pub[recv_data_size - 1] == endmsg)
            {
                arraysize = *(int *)(&buf_pub[1]);
                //memcpy(&arraysize, &(buf_pub[1]), 4);

                if(buf_pub[0]==0xFF)
                {
                    if (recv_data_size == arraysize * 4 + 6)
                    {
                        std_msgs::Float32MultiArray pub_float;
                        pub_float.data.resize(arraysize);
                        for (int i = 0; i < arraysize; i++)
                        {
                            pub_float.data[i] = *(float *)(&buf_pub[i * 4 + 5]);
                            //memcpy(&pub_float.data[i], &buf_pub[i * 4 + 5], 4);
                        }
                        serial_pub.publish(pub_float);
                    }
                    else
                    {
                        ROS_INFO("Datasize Error Float");
                    }
                }


                else    ROS_INFO("ERROR");
            
                recv_data_size = 0;
            }
        }

        // publish
        if (subflag)
        {
            rec = write(fd1, floattochar, floatdatasize * 4 + 6);
            if (rec < 0)
            {
                ROS_ERROR_ONCE("Serial Fail: cound not write float");
            }
            subflag = false;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}