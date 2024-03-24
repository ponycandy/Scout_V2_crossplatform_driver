#include <iostream>
#include "test_connect.h"
#include <iostream>
#include "gpcsnode.h"
// #include "QDebug"
struct motioncommand
{
    float linear;
    float omega;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar& linear;
        ar& omega;
    }
};
int counter=0;
double deltat=0;
test_connect ioport(500);
void getremotecmd(const std::string& data)
{
    std::cout<<"第i次接收数据"<<counter++<<std::endl;

    motioncommand control = gpcs::struct_load<motioncommand>(data);
    ioport.SetMotionCommand(control.linear,0, control.omega, ioport.current_motion_cmd_.fault_clear_flag);

    ioport.SendMotionCmd();//看来得持续发，不然就会冲掉为0
}


int main(int argc, char **argv)
{
    //parse input，大车用的是"192.168.1.10",4001
    // if (argc < 3) {
    //     std::cerr << "Usage: " << argv[0] << " <IP address> <port>" << std::endl;
    //     return 1; // Exit with an error status
    // }

    // Parse the IP address and port from the command-line arguments
    // std::string ipAddress = argv[1];
    // int port = std::stoi(argv[2]); // Convert the port argument to an integer
    // //
    std::cout<<"Scout_V2_driver_initializing!"<<std::endl;
    std::string ipAddress ="192.168.1.10";
    int port = 4001;
    gpcs::gpcsnode *nh = new gpcs::gpcsnode;

    nh->init("Scout_V2");
    nh->subscribe("motion_command", getremotecmd);
 if(ioport.init(ipAddress, port))
 {
        // qDebug()<<"开始循环!";

        std::cout<<"开始循环!"<<std::endl;
 while(true)
    {
     nh->spinonce(20);
     // qDebug()<<"spin多少的时候出错:"<<counter++;
     //为什么崩溃的是你呢...
    }
 }
 else
 {
     std::cout<<("init failed");
 }
 
}
