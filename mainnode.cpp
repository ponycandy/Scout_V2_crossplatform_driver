#include <iostream>
#include "test_connect.h"
#include <iostream>
#include "gpcsnode.h"

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

double deltat=0;
test_connect ioport(500);
void getremotecmd(const std::string& data)
{
    motioncommand control = gpcs::struct_load<motioncommand>(data);
     ioport.SetMotionCommand(control.linear,0, control.omega, ioport.current_motion_cmd_.fault_clear_flag);
     ioport.SendMotionCmd();
}


int main(int argc, char **argv)
{
    //parse input，大车用的是"192.168.1.10",4001
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <IP address> <port>" << std::endl;
        return 1; // Exit with an error status
    }

    // Parse the IP address and port from the command-line arguments
    std::string ipAddress = argv[1];
    int port = std::stoi(argv[2]); // Convert the port argument to an integer
    //
    gpcs::gpcsnode *nh = new gpcs::gpcsnode;
    nh->init("Scout_V2");
    nh->subscribe("motion_command", getremotecmd);
 if(ioport.init(ipAddress, port))
 {
      
 while(true)
    {
     nh->spinonce(20);
    }
 }
 else
 {
     std::cout<<("init failed");
 }
 
}
