#ifndef WEBSOCKET_SERVER_HPP
#define WEBSOCKET_SERVER_HPP

#include "concurrentqueue/concurrentqueue.h"
#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
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
    void setPort( const std::string& port);
    void start();
    void stop();
private:
    void acceptConnections();
    void handleReceive( websocket::stream< tcp::socket >& ws );
    void handleSend( std::string message );

    std::string            port_;
    net::io_context        ioc_;
    tcp::acceptor          acceptor_;
    std::thread            serverThread_;
    bool                   running_;
    std::vector< NET_PTR > net_ptrs_;
    std::mutex             connectionsMutex_;
public:

};

#endif