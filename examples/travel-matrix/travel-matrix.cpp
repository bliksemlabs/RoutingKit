#include <routingkit/osm_simple.h>
#include <routingkit/contraction_hierarchy.h>
#include <routingkit/inverse_vector.h>
#include <routingkit/timer.h>
#include <routingkit/geo_position_to_node.h>
#include <routingkit/geo_dist.h>
#include <iostream>
#include <string>
#include <zmqpp/zmqpp.hpp>
#include <cmath>

using namespace RoutingKit;
using namespace std;

#include "json.hpp"

using json = nlohmann::json;

json triple(json traveltime, json distance, json ascrowflies) {
    json t;
    t.push_back(traveltime);
    t.push_back(distance);
    t.push_back(ascrowflies);
    return t;
}

/* a coordinate is a pair of two valid numbers,
 * with the domain:
 *  -90.0f <  latitude < 90.0f
 * -180.0f < longitude < 180.0f
 */
bool validcoordinate(json coordinate) {
	return (coordinate.size() == 2 &&
            coordinate[0].is_number() &&
            coordinate[1].is_number() &&
			coordinate[0] >  -90.0f && coordinate[0] <  90.0f &&
			coordinate[1] > -180.0f && coordinate[0] < 180.0f);
}

json validateinput(json input) {
	auto source = input["departs"];
	auto target = input["arrives"];

	json reply = nullptr;

	/* Validate the input contains two arrays with at least two elements */
	if (source.is_array() && source.size() > 0 &&
		target.is_array() && target.size() > 0) {

		/* validate that each element in the departure array consists of a coordinate */
		for (json::iterator it = source.begin(); it != source.end(); ++it) {
			auto coordinate = it.value();
			if (!validcoordinate(coordinate)) {
				reply["error"]["coordinate"].push_back(coordinate);
			}
		}

		/* The same as above, but for the arrival positions */
		for (json::iterator it = target.begin(); it != target.end(); ++it) {
			auto coordinate = it.value();
			if (!validcoordinate(coordinate)) {
				reply["error"]["coordinate"].push_back(coordinate);
			}
		}

		/* If we have errors terminate in our input coordinates, terminate */
		if (reply["error"] != nullptr) {
			reply["error"]["message"] = "Invalid coordinates found.";
		}
	} else {
		reply["error"]["message"] = "Missing departure and/or arrival coordinates.";
	}

	return reply;
}

int main (int argc, char *argv[]) {
    if (argc != 2) return -1;

    /* Load a car routing graph from OpenStreetMap-based data */
    auto graph = simple_load_osm_car_routing_graph_from_pbf(argv[1]);
    auto tail = invert_inverse_vector(graph.first_out);

    /* Build the shortest path index using geo_distance as weight for min-dist
     * alternatives would be travel_time for earliest arrival
     */
    auto ch = ContractionHierarchy::build(
        graph.node_count(),
        tail, graph.head,
        graph.geo_distance
    );

    /* Build the index to quickly map latitudes and longitudes */
    GeoPositionToNode map_geo_position(graph.latitude, graph.longitude);

    const string endpoint = "tcp://*:4242";

    /* initialize the 0MQ context */
    zmqpp::context context;

    /* generate a pull socket */
    zmqpp::socket_type type = zmqpp::socket_type::reply;
    zmqpp::socket socket (context, type);

    /* bind to the socket */
    cout << "Binding to " << endpoint << "..." << endl;
    socket.bind(endpoint);

    while (1) {
        cout << "Blocking until a message has been received." << endl;
        zmqpp::message zmq_receive;

        /* decompose the message */
        socket.receive(zmq_receive);
        string receive;
        zmq_receive >> receive;

        auto input = json::parse(receive);

        auto source = input["departs"];
        auto target = input["arrives"];

        zmqpp::message zmq_reply;

		json reply = validateinput(input);
		if (reply != nullptr) {
            /* cache the map_targets, otherwise we are geocoding them every time */
            std::vector<std::vector<GeoPositionToNode::NearestNeighborhoodQueryResult>> map_targets;

            for (json::iterator it = target.begin(); it != target.end(); ++it) {
                auto coordinate = it.value();
                map_targets.push_back(map_geo_position.find_all_nodes_within_radius(coordinate[0], coordinate[1], 1000));
            }

            /* Besides the CH itself we need a query object. */
            ContractionHierarchyQuery ch_query(ch);

            reply["matrix"] = json::array();

            /* for all input departure points */
            for (json::iterator it = source.begin(); it != source.end(); ++it) {
                auto coordinate = it.value();
                std::vector<GeoPositionToNode::NearestNeighborhoodQueryResult> map_sources = map_geo_position.find_all_nodes_within_radius(coordinate[0], coordinate[1], 1000);

                if (map_sources.size() > 0) {
                    /* clear the query and assign the source */
                    ch_query.reset();

                    /* add the intersections around the source, to start searching from */
                    for (unsigned si = 0; si < map_sources.size(); si++) {
                        ch_query.add_source(map_sources[si].id, map_sources[si].distance * 2);
                    }

                    /* for all input arrival points */
                    for (unsigned ti = 0; ti < map_targets.size(); ti++) {

                        if (map_targets[ti].size() > 0) {
                            ch_query.reset_target();
                            for (unsigned tj = 0; tj < map_targets[ti].size(); tj++) {
                                /* assign the target, and run the query. */
                                ch_query.add_target(map_targets[ti][tj].id, map_targets[ti][tj].distance * 2);
                            }
                            ch_query.run();

                            std::vector<unsigned>nodes = ch_query.get_node_path();
                            if (nodes.size() > 1) {
                                /* when a route is found */
                                unsigned ascrowflies = geo_dist(coordinate[0], coordinate[1], graph.latitude[nodes[0]], graph.longitude[nodes[0]]) +
                                                       geo_dist(graph.latitude[nodes[nodes.size() - 1]], graph.longitude[nodes[nodes.size() - 1]], target[ti][0], target[ti][1]);

                                unsigned distance    = 0;
                                unsigned travel_time = 0;

                                std::vector<unsigned>arcs = ch_query.get_arc_path();
                                for (auto a:arcs) {
                                    distance    += graph.geo_distance[a];
                                    travel_time += graph.travel_time[a];
                                }

                                reply["matrix"].push_back(triple(travel_time, distance, ascrowflies));

                            } else {
                                /* when there is no route found or the source and target snap to the same origin */
                                unsigned ascrowflies = geo_dist(coordinate[0], coordinate[1], target[ti][0], target[ti][1]);
                                reply["matrix"].push_back(triple(nullptr, nullptr, ascrowflies));
                            }
                        } else {
                            /* when the arrival point did not return an intersection */
                            unsigned ascrowflies = geo_dist(coordinate[0], coordinate[1], target[ti][0], target[ti][1]);
                            reply["matrix"].push_back(triple(nullptr, nullptr, ascrowflies));
                        }
                    }
                } else {
                   /* when the source is invalid, we can't compute any
                    * source-target pair for that source. Instead we
                    * could compute the great circle distance.
                    */
                    for (unsigned ti = 0; ti < map_targets.size(); ti++) {
                        unsigned ascrowflies = geo_dist(coordinate[0], coordinate[1], target[ti][0], target[ti][1]);
                        reply["matrix"].push_back(triple(nullptr, nullptr, ascrowflies));
                    }
                }
            }
        }

        zmq_reply << reply.dump();
        socket.send(zmq_reply);
    }
}
