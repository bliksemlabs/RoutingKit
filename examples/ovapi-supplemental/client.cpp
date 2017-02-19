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

  // send a message
  zmqpp::message message;
  // compose a message from a string and a number
  // message << "{\"lon\": 4.8574, \"lat\": 52.3379, \"geojson\": true, \"distance\": 500}";
  // message << "{\"lon\": 4.8993, \"lat\": 52.3778, \"geojson\": true, \"distance\": 2000}";
  message << "{\"debug:pedestrian\": 161490}";
  socket.send(message);

  socket.receive(message);
  string receive;
  message >> receive;
  cout << receive;
}
