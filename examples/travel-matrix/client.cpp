#include <zmqpp/zmqpp.hpp>
#include <string>
#include <iostream>

using namespace std;

int main(int argc, char *argv[]) {
  const string endpoint = "tcp://localhost:4242";

  // initialize the 0MQ context
  zmqpp::context context;

  // generate a push socket
  zmqpp::socket_type type = zmqpp::socket_type::request;
  zmqpp::socket socket (context, type);

  // open the connection
  socket.connect(endpoint);

  {
    // send a message
    zmqpp::message message;
    // compose a message from a string and a number
    message << "{\"departs\": [[9.0396988, 7.3708228]], \"arrives\": [[9.0929029, 7.4223216], [9.1393948, 7.3687402], [52.0, 4.0]]}";
    socket.send(message);

    socket.receive(message);
    string receive;
    message >> receive;
    cout << receive;
  }

  {
    // send a message
    zmqpp::message message;
    // compose a message from a string and a number
    message << "{\"departs\": [[\"a\", 7.3708228]], \"arrives\": [[9.0929029, 7.4223216], [9.1393948, 7.3687402], [52.0, 4.0]]}";
    socket.send(message);

    socket.receive(message);
    string receive;
    message >> receive;
    cout << receive;
  }

}
