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

  /* test a normal situation */
  {
    zmqpp::message message;
    message << "{\"departs\": [[52.08497,4.33394]], \"arrives\": [[52.01355,4.35433]]}";
    socket.send(message);

    socket.receive(message);
    string receive;
    message >> receive;
    cout << receive << "\n";
  }

  /* test a shared depart and arrive node-id, same geographical location */
  {
    zmqpp::message message;
    message << "{\"departs\": [[52.08497,4.33394]], \"arrives\": [[52.08497,4.33394]]}";
    socket.send(message);

    socket.receive(message);
    string receive;
    message >> receive;
    cout << receive << "\n";
  }

  /* test a shared depart and arrive node-id, different geographical location */
  {
    zmqpp::message message;
    message << "{\"departs\": [[52.08497,4.33394]], \"arrives\": [[52.08497,4.334]]}";
    socket.send(message);

    socket.receive(message);
    string receive;
    message >> receive;
    cout << receive << "\n";
  }

  /* test two locations not geocoded to a node-id */
  {
    zmqpp::message message;
    message << "{\"departs\": [[4.1514,52.0860]], \"arrives\": [[52.1465,4.1407]]}";
    socket.send(message);

    socket.receive(message);
    string receive;
    message >> receive;
    cout << receive << "\n";
  }

  /* test invalid input */
  {
    zmqpp::message message;
    message << "{\"departs\": [""], \"arrives\": [[52.1465,4.1407]]}";
    socket.send(message);

    socket.receive(message);
    string receive;
    message >> receive;
    cout << receive << "\n";
  }

  /* combine the queries */
  {
    zmqpp::message message;
    message << "{\"departs\": [[52.08497,4.33394],[4.1514,52.0860]], \"arrives\": [[52.01355,4.35433],[52.1465,4.1407],[52.08497,4.33394],[52.08497,4.334]]}";
    socket.send(message);

    socket.receive(message);
    string receive;
    message >> receive;
    cout << receive << "\n";
  }
}
