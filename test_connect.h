#ifndef TEST_CONNECY_H
#define TEST_CONNECT_H

#include<connector_base.h>
#include <boost/asio.hpp>
class test_connect : public Connector
{
private:
    /* data */
public:
    boost::asio::io_context io_context_;
    boost::asio::ip::tcp::socket* Core_socket;
    enum { max_length = 1024 };
    char data_[max_length];
    test_connect(int baud);

    // 向对端发送报文
    int  Send(const void *buf,const int buflen);


    int init(std::string Ipaddress, int port);
    void handleRead(const boost::system::error_code& error, std::size_t bytes_transferred);
    void unpack_all();
    void printall();
    void runIoContext();
    void startreadthreading();
    

    
    
    void cmd_test();
};






#endif
