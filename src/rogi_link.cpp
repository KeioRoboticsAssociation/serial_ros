#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Char.h"

#include <rogi_link_msgs/RogiLink.h>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstdio>

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
char *sending_message;

const int datasize = 2;


void sub_callback(const rogi_link_msgs::RogiLink &serial_msg)
{
    if (subflag)
    {
        usleep(sleeptime);
        subflag == false;
    }
    delete[] sending_message;
    // datasize = serial_msg.data.size(); //ここら辺場合に依ってしまう、他ノードでどこまで指定するか…
    sending_message = new char[11];
    sending_message[0] = 0xFF; //start flag
    *(char *)(&sending_message[1]) = serial_msg.id; //canID
    //memcpy(&floattochar[1], &datasize, 4);
    for (int i = 0; i < datasize; i++)
    {
        *(float *)(&sending_message[i * 4 + 2]) = serial_msg.data[i+1];
        //memcpy(&floattochar[i * 4 + 5], &serial_msg.data[i], 4);
    }
    sending_message[datasize * 4 + 5] = endmsg;

    subflag = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rogi_link_node");
    ros::NodeHandle n;
    // ROS_INFO("wtf");

    //Publisher
    ros::Publisher serial_pub[32];
    for(int i=0;i<32;i++){
        char hard_id[20];
        sprintf(hard_id,"%02x",i);
        std::string node_name = "rcv_serial_";
        node_name += hard_id;
        serial_pub[i] = n.advertise<std_msgs::Float32MultiArray>(node_name, 1000);
    }

    ros::Publisher connection_status = n.advertise<std_msgs::Empty>("connection_status", 1);

    //Subscriber
    ros::Subscriber serial_sub = n.subscribe("send_serial", 100, sub_callback);

    // Parameter
    ros::NodeHandle arg_n("~");
    std::string port_name = "/dev/ttyACM0";
    arg_n.getParam("port", port_name);
    arg_n.getParam("baudrate", BAUDRATE);
    arg_n.getParam("looprate", sub_loop_rate);

    //opening serial port
    fd1 = open_serial(port_name.c_str());

    while (ros::ok())
    {
        fd1 = open_serial(port_name.c_str());
        //ROS_ERROR("Serial Fail: cound not open %s", port_name.c_str());
        //ros::shutdown();
        ROS_WARN_ONCE("Serial Connecting\n");

        if (fd1 >= 0)   break;
    }

    ROS_INFO("Serial Success");

    char buf_pub[256] = {0};
    int recv_data_size = 0;
    int arraysize = 0;
    int rec;
    int trash = 0;
    short canid=0;
    ros::Rate loop_rate(sub_loop_rate);

    // remove initial_buff_data
    for (int i = 0; i < 1000; i++)
    {
        trash=read(fd1, &buf_pub[0], sizeof(buf_pub));
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
                // arraysize = *(int *)(&buf_pub[1]);
                //memcpy(&arraysize, &(buf_pub[1]), 4);

                if(buf_pub[0]==0xFF) //checking start flag
                {
                    canid = *(short*)(&buf_pub[2]);

                    if (recv_data_size == 11) //data length 11byte
                    {
                        std_msgs::Float32MultiArray pub_float;
                        pub_float.data.resize(2);
                        for (int i = 0; i < arraysize; i++)
                        {
                            pub_float.data[i] = *(float *)(&buf_pub[i * 4 + 2]);
                            //memcpy(&pub_float.data[i], &buf_pub[i * 4 + 5], 4);
                        }
                        serial_pub[canid>>6 & 0b0000000000011111].publish(pub_float);
                        //現在は受信はfloatに限定、他用途ができた場合はfloat以外の処理も導入する必要あり
                    }
                    else
                    {
                        ROS_WARN("Datasize Error");
                    }
                    //ただしバッファーに複数回分の受信データがストックされてしまった場合についてもDatasize Errorになってしまっている
                }

                else    ROS_WARN("START FLAG ERROR");
            
                recv_data_size = 0;
            }
        }

        // publish
        if (subflag)
        {
            rec = write(fd1, sending_message, datasize * 4 + 6);
            if (rec < 0)
            {
                ROS_ERROR("Serial Fail: cound not write");
            }
            subflag = false;
        }

        std_msgs::Empty status_msg;
        connection_status.publish(status_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}