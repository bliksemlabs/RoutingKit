#include <routingkit/osm_simple.h>
#include <routingkit/contraction_hierarchy.h>
#include <routingkit/inverse_vector.h>
#include <routingkit/timer.h>
#include <routingkit/geo_position_to_node.h>
#include <iostream>
#include <string>
#include <zmqpp/zmqpp.hpp>
#include <cmath>

using namespace RoutingKit;
using namespace std;

#include "json.hpp"

using json = nlohmann::json;


/* Haversine; great circle distance */

#define R 6371.0088
#define TO_RAD (3.1415926536f / 180.0f)

double dist(double th1, double ph1, double th2, double ph2)
{
    double dx, dy, dz;
    ph1 -= ph2;
    ph1 *= TO_RAD, th1 *= TO_RAD, th2 *= TO_RAD;

    dz = sin(th1) - sin(th2);
    dx = cos(ph1) * cos(th1) - cos(th2);
    dy = sin(ph1) * cos(th1);
    return asin(sqrt(dx * dx + dy * dy + dz * dz) / 2) * 2 * R * 1000;
}

json distance(json coordinate_from, json coordinate_to) {
    json pair;
    unsigned distance = dist (coordinate_from[0], coordinate_from[1], coordinate_to[0], coordinate_to[1]);
    pair.push_back (distance);
    pair.push_back (distance / 12);

    return pair;
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
        cout << "Receiving message..." << endl;
        zmqpp::message zmq_receive;

        // decompose the message
        socket.receive(zmq_receive);
        string receive;
        zmq_receive >> receive;

        auto input = json::parse(receive);

        auto source = input["departs"];
        auto target = input["arrives"];

        zmqpp::message zmq_reply;
        json reply;
        reply["error"];
        reply["matrix"] = json::array();

        /* Validate the input contains two arrays with at least two elements */
        if (source.is_array() && source.size() > 0 &&
            target.is_array() && target.size() > 0) {

            /* create a list of node ids to be used as input for the search
             * we assume that if we can't find any node within 1000m the result
             * should be reevaluated and will result invalid_id.
             */
            std::vector<unsigned>source_list;
            for (json::iterator it = source.begin(); it != source.end(); ++it) {

                /* validate that the array consists of pairs of two numbers */
                auto coordinate = it.value();
                if (coordinate.size() == 2 &&
                    coordinate[0].is_number() &&
                    coordinate[1].is_number()) {

                    /* Geocode the input */
                    unsigned result = map_geo_position.find_nearest_neighbor_within_radius(coordinate[0], coordinate[1], 1000).id;

                    /* Store the result as input for the planner */
                    source_list.push_back(result);

                    /* Errors while using this function are reported back */
                    if (result == invalid_id) {
                        reply["error"].push_back(coordinate);
                    }

                /* If the array consist of invalid values terminate */
                } else {
                    reply["error"] = "Invalid source coordinates. " + coordinate.dump();
                    goto end;
                }
            }

            /* The same as above, but for the arrival positions */
            std::vector<unsigned>target_list;
            for (json::iterator it = target.begin(); it != target.end(); ++it) {
                auto coordinate = it.value();
                if (coordinate.size() == 2 &&
                    coordinate[0].is_number() &&
                    coordinate[1].is_number()) {

                    unsigned result = map_geo_position.find_nearest_neighbor_within_radius(coordinate[0], coordinate[1], 1000).id;
                    target_list.push_back(result);

                    if (result == invalid_id) {
                        reply["error"].push_back(coordinate);
                    }
                } else {
                    reply["error"] = "Invalid target coordinates. " + coordinate.dump();
                    goto end;
                }
            }

            /* Besides the CH itself we need a query object. */
            ContractionHierarchyQuery ch_query(ch);

            /* We keep an index, so we can fall back to the coordinate list */
            unsigned si = 0;

            for(auto s:source_list) {
                if (s == invalid_id) {
                    /* when the source is invalid, we can't compute any
                     * source-target pair for that source. Instead we
                     * compute a great circle distance.
                     */
                    auto coordinate_from = source.at(si);
                    for (unsigned ti = 0; ti < target.size(); ti++) {
                        auto coordinate_to = target.at(ti);
                        json pair = distance (coordinate_from, coordinate_to);
                        reply["matrix"].push_back (pair);
                    }

                } else {
                    /* clear the query and assign the source */
                    ch_query.reset();
                    ch_query.add_source(s);

                    unsigned ti = 0;
                    for (auto t:target_list) {
                        json pair;
                        if (t != invalid_id) {
                            /* assign the target, and run the query. */
                            ch_query.reset_target().add_target(t).run();

                            /* when succesful, the distance is available */
                            unsigned distance = ch_query.get_distance();
                            if (distance != 0) {

                                /* the travel time should be computed by a
                                 * sum over all arcs.
                                 */
                                unsigned travel_time = 0;
                                std::vector<unsigned>arcs = ch_query.get_arc_path();
                                for (auto a:arcs) travel_time += graph.travel_time[a];

                                /* we pair the result */
                                pair.push_back (distance);
                                pair.push_back (travel_time);
                            }
                        }
                        if (pair.is_null()) {
                            /* if the query failed we will compute the great
                             * circle distance
                             */
                            auto coordinate_from = source.at(si);
                            auto coordinate_to = target.at(ti);
                            pair = distance (coordinate_from, coordinate_to);
                        }
                        reply["matrix"].push_back (pair);
                        ti++;
                    }
                }
                si++;
            }
        } else {
            reply["error"] = "Invalid JSON presentation.";
        }

end:
        zmq_reply << reply.dump();
        socket.send(zmq_reply);
    }
}
