#include <routingkit/osm_simple.h>
#include <routingkit/contraction_hierarchy.h>
#include <routingkit/inverse_vector.h>
#include <routingkit/timer.h>
#include <routingkit/geo_position_to_node.h>
#include <routingkit/osm_profile.h>
#include <iostream>
#include <string>
#include <zmqpp/zmqpp.hpp>

using namespace RoutingKit;
using namespace std;

#include "json.hpp"

using json = nlohmann::json;

int main (int argc, char *argv[]) {
    if (argc != 2) return -1;

    // Load a car routing graph from OpenStreetMap-based data
    auto graph = simple_load_osm_car_routing_graph_from_pbf(argv[1]);
    auto tail = invert_inverse_vector(graph.first_out);

    // Build the shortest path index
    auto ch = ContractionHierarchy::build(
        graph.node_count(),
        tail, graph.head,
        graph.travel_time
    );

    // Build the index to quickly map latitudes and longitudes
    GeoPositionToNode map_geo_position(graph.latitude, graph.longitude);

    const string endpoint = "tcp://*:4242";

    // initialize the 0MQ context
    zmqpp::context context;

    // generate a pull socket
    zmqpp::socket_type type = zmqpp::socket_type::reply;
    zmqpp::socket socket (context, type);

    // bind to the socket
    cout << "Binding to " << endpoint << "..." << endl;
    socket.bind(endpoint);

    while (1) {
        cout << "Receiving message..." << endl;
        zmqpp::message zmq_receive;

        // decompose the message
        socket.receive(zmq_receive);
        string receive;
        zmq_receive >> receive;

        // auto input = json::parse("{\"source\": [[4.3822, 52.0773]], \"target\": [[4.2764, 52.1051],[4.3111, 51.9742]]}");
        auto input = json::parse(receive);

        auto source = input["source"];
        auto target = input["target"];

        zmqpp::message zmq_reply;

        if (!source.is_array() || source.size() == 0 ||
            !target.is_array() || target.size() == 0) {

            zmq_reply << "{\"distances\":[]}";

        } else {
            std::vector<unsigned>source_list;
            for (json::iterator it = source.begin(); it != source.end(); ++it) {
                auto coordinate = it.value();
                if (coordinate.size() == 2 &&
                    coordinate[0].is_number() &&
                    coordinate[1].is_number()) {

                    unsigned result = map_geo_position.find_nearest_neighbor_within_radius(coordinate[1], coordinate[0], 1000).id;
                    if (result != invalid_id) {
                        source_list.push_back(result);
                    }
                }
            }

            std::vector<unsigned>target_list;
            for (json::iterator it = target.begin(); it != target.end(); ++it) {
                auto coordinate = it.value();
                if (coordinate.size() == 2 &&
                    coordinate[0].is_number() &&
                    coordinate[1].is_number()) {

                    unsigned result = map_geo_position.find_nearest_neighbor_within_radius(coordinate[1], coordinate[0], 1000).id;
                    if (result != invalid_id) {
                        target_list.push_back(result);
                    }
                }
            }

            // Besides the CH itself we need a query object.
            ContractionHierarchyQuery ch_query(ch);

            json reply = {{"type", "FeatureCollection"}, {"features", nullptr}};

            ch_query.reset().pin_targets(target_list);
            for(auto s:source_list) {
                for(auto t:target_list) {
                    ch_query.reset().add_source(s).add_target(t).run();

					#if 0
					std::vector<unsigned>nodes = ch_query.get_node_path();
                    for (auto n:nodes) {
                        json lonlat;
                        lonlat.push_back(graph.longitude[n]);
                        lonlat.push_back(graph.latitude[n]);
                        coordinates.push_back(lonlat);

						cout << graph.longitude[n] << "," << graph.latitude[n] << " ";
                    }

					cout << "\n";

					for (auto a:arcs) {
						cout "Arc: " << a << "\n" << graph.longitude[graph.head[a]] << "," << graph.latitude[graph.head[a]] << "\n";
					}

					cout << "\n";
					#endif

                    json coordinates;
					std::vector<unsigned>arcs = ch_query.get_arc_path();
					if (arcs.size() > 0) {
							json distances;
							json way_class;
							json way_max_speed;

							for (unsigned a:arcs) {
								distances.push_back(graph.geo_distance[a]);
								way_class.push_back(get_osm_way_class_string(graph.way_class[a]));
								way_max_speed.push_back(graph.way_max_speed[a]);
							}
							json feature = {{"distances", distances}, {"way_class", way_class}, {"way_max_speed", way_max_speed}};
							reply["features"].push_back(feature);
					}
                }
            }

            zmq_reply << reply.dump();
        }

        socket.send(zmq_reply);
    }
}
