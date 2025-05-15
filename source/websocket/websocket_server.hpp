#ifndef WEBSOCKET_SERVER_HPP
#define WEBSOCKET_SERVER_HPP

#include "concurrentqueue/concurrentqueue.h"
#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include "Calculation/sensor_db.h"
#include <mutex>
#include <string>
#include <thread>
#include <vector>
//
namespace beast     = boost::beast;
namespace http      = beast::http;
namespace websocket = beast::websocket;
namespace net       = boost::asio;
using tcp           = net::ip::tcp;
//
struct NET_PTR
{
    websocket::stream< tcp::socket >* connection_;
    std::thread*                      thread_;
};
//
class WebSocketServer
{
public:
    WebSocketServer();
    ~WebSocketServer();
    void setPort( const std::string& port );
    void start();
    void stop();
private:
    void acceptConnections();
    void handleReceive( websocket::stream< tcp::socket >& ws );

    std::string            port_;
    net::io_context        ioc_;
    tcp::acceptor          acceptor_;
    std::thread            serverThread_;
    std::vector< NET_PTR > net_ptrs_;
    std::mutex             connectionsMutex_;
public:
    std::string str_fusion_config = "";
    void        handleSend( std::string message );
public:
    int64_t     start_time = 0;
    std::string commond_;
    bool        running_;
private:
};

#endif