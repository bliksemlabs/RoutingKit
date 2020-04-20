#include <cmath>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <routingkit/contraction_hierarchy.h>
#include <routingkit/osm_graph_builder.h>
#include <routingkit/geo_dist.h>
#include <routingkit/timer.h>
#include <routingkit/sort.h>
#include <routingkit/inverse_vector.h>
#include <routingkit/permutation.h>
#include <routingkit/graph_util.h>
#include <routingkit/bit_vector.h>
#include <routingkit/filter.h>
#include <routingkit/id_mapper.h>

#include <vector>
#include <stdint.h>
#include <string>
#include <stdio.h>
#include <memory>
#include "wkb.hpp"
#include <pqxx/pqxx> 

#include <routingkit/geo_position_to_node.h>
#include <routingkit/geo_dist.h>
#include <routingkit/osm_profile.h>
#include <zmqpp/zmqpp.hpp>
#include "json.hpp"

using json = nlohmann::json;
using namespace RoutingKit;
using namespace std;

void modelling_nodes_from_wkb(std::string const &wkb_hex, std::vector<float> &modelling_node_latitude, std::vector<float> &modelling_node_longitude) {
    auto wkb_c = ewkb::parser_t::wkb_from_hex(wkb_hex);
    auto wkb = ewkb::parser_t(wkb_c);

    if (wkb.read_header() == ewkb::wkb_line) {
        auto sz = wkb.read_length();
        modelling_node_latitude.reserve(sz);
        modelling_node_longitude.reserve(sz);

        for (size_t i = 0; i < sz; ++i) {
            auto coordinate = wkb.read_point();
            modelling_node_latitude.push_back(coordinate.y);
            modelling_node_longitude.push_back(coordinate.x);
        }
    }
}

void transform_nwb_node(pqxx::result &R, RoutingKit::OSMRoutingGraph &routing_graph, RoutingKit::BitVector &is_routing_node) {
	bool distinct = true;

	for (pqxx::result::const_iterator c = R.begin(); c != R.end(); ++c) {
        int global_id   = c[0].as<int>();
        float longitude = c[1].as<float>();
        float latitude  = c[2].as<float>();

        is_routing_node.make_large_enough_for(global_id);
        if (!is_routing_node.is_set(global_id)) {
            is_routing_node.set(global_id);
            routing_graph.latitude.push_back(latitude);
            routing_graph.longitude.push_back(longitude);
        } else {
			if (distinct) {
				distinct = false;
				std::cerr << "The node query doesn't have distinct global ids." << std::endl;
			}
		}
	}
}

void transform_nwb_edge(pqxx::result &R, RoutingKit::OSMRoutingGraph &routing_graph, RoutingKit::IDMapper &routing_node,
						std::vector<unsigned> &tail, RoutingKit::OSMRoadGeometry geometry_to_be_extracted) {

    /* List down all the records */
    for (pqxx::result::const_iterator c = R.begin(); c != R.end(); ++c) {
        int routing_way_id = c[0].as<int>();
        int global_x       = c[1].as<int>();
        int global_y       = c[2].as<int>();
        int geo_distince   = c[4].as<int>();

        std::vector<float> modelling_node_latitude;
        std::vector<float> modelling_node_longitude;
        modelling_nodes_from_wkb(c[3].as<std::string>(), modelling_node_latitude, modelling_node_longitude);

        tail.push_back(routing_node.to_local(global_x));
        routing_graph.head.push_back(routing_node.to_local(global_y));
        routing_graph.geo_distance.push_back(geo_distince);
        routing_graph.way.push_back(routing_way_id);

        if(geometry_to_be_extracted == RoutingKit::OSMRoadGeometry::uncompressed){
            routing_graph.first_modelling_node.push_back(routing_graph.modelling_node_latitude.size());
			if (modelling_node_latitude.size() > 2) {
				routing_graph.modelling_node_latitude.insert(
					routing_graph.modelling_node_latitude.end(),
					modelling_node_latitude.begin() + 1, modelling_node_latitude.end() - 1
				);
				routing_graph.modelling_node_longitude.insert(
					routing_graph.modelling_node_longitude.end(),
					modelling_node_longitude.begin() + 1, modelling_node_longitude.end() - 1
				);
			}
		}

        tail.push_back(routing_node.to_local(global_y));
        routing_graph.head.push_back(routing_node.to_local(global_x));
        routing_graph.geo_distance.push_back(geo_distince);
        routing_graph.way.push_back(routing_way_id);

        std::reverse(modelling_node_latitude.begin(), modelling_node_latitude.end());
        std::reverse(modelling_node_longitude.begin(), modelling_node_longitude.end());

        if(geometry_to_be_extracted == RoutingKit::OSMRoadGeometry::uncompressed){
            routing_graph.first_modelling_node.push_back(routing_graph.modelling_node_latitude.size());
			if (modelling_node_latitude.size() > 2) {
				routing_graph.modelling_node_latitude.insert(
					routing_graph.modelling_node_latitude.end(),
					modelling_node_latitude.begin() + 1, modelling_node_latitude.end() - 1
				);
				routing_graph.modelling_node_longitude.insert(
					routing_graph.modelling_node_longitude.end(),
					modelling_node_longitude.begin() + 1, modelling_node_longitude.end() - 1
				);
			}
		}
    }
}

RoutingKit::OSMRoutingGraph load_routing_graph_from_postgis(
    const std::string&connection_string,
    const std::string&sql_node,
    const std::string&sql_edge,
    std::function<void(pqxx::result&, RoutingKit::OSMRoutingGraph&, RoutingKit::BitVector&)>transform_row_node,
    std::function<void(pqxx::result&, RoutingKit::OSMRoutingGraph&, RoutingKit::IDMapper&, std::vector<unsigned>&, RoutingKit::OSMRoadGeometry)>transform_row_edge) {
   
	RoutingKit::OSMRoutingGraph routing_graph;
	
	RoutingKit::BitVector is_routing_node;
   
	std::vector<unsigned>tail;

    try {
        pqxx::connection C(connection_string);
        if (C.is_open()) {
            std::cout << "Opened database successfully: " << C.dbname() << std::endl;
        } else {
            std::cout << "Can't open database" << std::endl;
			return routing_graph;
        }

        /* Create a non-transactional object. */
        pqxx::nontransaction N(C);
         
        /* Execute SQL query */
        pqxx::result R_node( N.exec( sql_node ));
        
		transform_row_node(R_node, routing_graph, is_routing_node);
		
        
		/* Execute SQL query */
        pqxx::result R_edge( N.exec( sql_edge ));

    	RoutingKit::IDMapper routing_node(is_routing_node);

        transform_row_edge(R_edge, routing_graph, routing_node, tail, RoutingKit::OSMRoadGeometry::uncompressed);
	} catch (const std::exception &e) {
		std::cerr << e.what() << std::endl;
	}
    
	/* How to use a single IDMapper in try-catch dependent on is_routing_node and beyond */	
	RoutingKit::IDMapper routing_node(is_routing_node);

    {
        unsigned node_count = routing_node.local_id_count();

		std::cout << "Node count: " << node_count << std::endl;
		std::cout << "Head count: " << routing_graph.head.size() << std::endl;
		std::cout << "Tail count: " << tail.size() << std::endl;
		std::cout << "Geo distance count: " << routing_graph.geo_distance.size() << std::endl;
		std::cout << "Way count: " << routing_graph.way.size() << std::endl;

        auto p = RoutingKit::compute_inverse_sort_permutation_first_by_tail_then_by_head_and_apply_sort_to_tail(node_count, tail, routing_graph.head);
        routing_graph.head = RoutingKit::apply_inverse_permutation(p, std::move(routing_graph.head));
        routing_graph.geo_distance = RoutingKit::apply_inverse_permutation(p, std::move(routing_graph.geo_distance));
        routing_graph.way = RoutingKit::apply_inverse_permutation(p, std::move(routing_graph.way));
        // routing_graph.is_arc_antiparallel_to_way = RoutingKit::apply_inverse_permutation(p, std::move(routing_graph.is_arc_antiparallel_to_way));
        routing_graph.first_out = RoutingKit::invert_vector(tail, node_count);

        // if(geometry_to_be_extracted == OSMRoadGeometry::uncompressed){
        if(true){
            routing_graph.first_modelling_node.push_back(routing_graph.modelling_node_latitude.size());

            std::vector<unsigned>first_modelling_node;
            std::vector<float>modelling_node_latitude;
            std::vector<float>modelling_node_longitude;

            first_modelling_node.reserve(routing_graph.first_modelling_node.size());
            modelling_node_latitude.reserve(routing_graph.modelling_node_latitude.size());
            modelling_node_longitude.reserve(routing_graph.modelling_node_longitude.size());

            auto new_arc_id_to_old_arc_id = RoutingKit::invert_permutation(p);
            for(auto old_arc_id : new_arc_id_to_old_arc_id){
                first_modelling_node.push_back(modelling_node_latitude.size());

                int first = routing_graph.first_modelling_node[old_arc_id];
                int last = routing_graph.first_modelling_node[old_arc_id + 1];

                modelling_node_latitude.insert(
                    modelling_node_latitude.end(),
                    routing_graph.modelling_node_latitude.begin() + first,
                    routing_graph.modelling_node_latitude.begin() + last
                );
                modelling_node_longitude.insert(
                    modelling_node_longitude.end(),
                    routing_graph.modelling_node_longitude.begin() + first,
                    routing_graph.modelling_node_longitude.begin() + last
                );
            }

            first_modelling_node.push_back(modelling_node_latitude.size());

            routing_graph.first_modelling_node = std::move(first_modelling_node);
            routing_graph.modelling_node_latitude = std::move(modelling_node_latitude);
            routing_graph.modelling_node_longitude = std::move(modelling_node_longitude);
        }
    }

	return routing_graph;
}
    

int main() {
    const string endpoint = "tcp://*:4246";

    cout << "Startup..." << endl;

    // initialize the 0MQ context
    zmqpp::context context;

    // generate a pull socket
    zmqpp::socket_type type = zmqpp::socket_type::reply;
    zmqpp::socket socket (context, type);

    // bind to the socket
    cout << "Binding to " << endpoint << "..." << endl;
    socket.bind(endpoint);

	std::string connection_string = "dbname = top10nl user = postgres password = postgres hostaddr = 127.0.0.1 port = 5432";

    std::string sql_node = "SELECT row_number, ST_X(ST_Transform(node, 4326)), ST_Y(ST_Transform(node, 4326)) FROM top10_nodes;";

    std::string sql_edge = "SELECT lokaalid, x, y, ST_Transform(geom, 4326), distance FROM top10nl_edges;";

	RoutingKit::OSMRoutingGraph routing_graph = load_routing_graph_from_postgis(connection_string, sql_node, sql_edge, transform_nwb_node, transform_nwb_edge); 
        
    auto tail = invert_inverse_vector(routing_graph.first_out);

	auto ch = RoutingKit::ContractionHierarchy::build(
		routing_graph.node_count(),
		tail, routing_graph.head,
		routing_graph.geo_distance
		);

    GeoPositionToNode map_geo_position(routing_graph.latitude, routing_graph.longitude);

    ContractionHierarchyQuery ch_query(ch);

    while (1) {
        cout << "Receiving message..." << endl;
        /* Stap 6; ontvang de query van ZeroMQ */
        zmqpp::message zmq_receive;
        // decompose the message
        socket.receive(zmq_receive);
        string receive;
        zmq_receive >> receive;

        json reply = {{"type", "FeatureCollection"}, {"features", nullptr}};

        std::vector<unsigned> route_nodes;
        try {
            auto input = json::parse(receive);
            auto route = input["route"];
            if (route.is_array() && route.size() > 1) {
                for (json::iterator it = route.begin(); it != route.end(); ++it) {
                    auto coordinate = it.value();
                    unsigned mapped = map_geo_position.find_nearest_neighbor_within_radius(coordinate[1], coordinate[0], 1000).id;
                    if (mapped != invalid_id) route_nodes.push_back(mapped);
                }
            }
        } catch (int e) {
            goto clean;
        }

        if (route_nodes.size() > 1) {
            ch_query.reset().add_source(route_nodes[0]).add_target(route_nodes[1]).run();
            unsigned distance = ch_query.get_distance();

            if (distance != inf_weight) {
                std::vector<unsigned>arcs = ch_query.get_arc_path();
                std::vector<unsigned>shape_distances;
                json coordinates;

                float total_distance = 0.0f;

                {
                    json lonlat_from;
                    lonlat_from.push_back(routing_graph.longitude[tail[arcs[0]]]);
                    lonlat_from.push_back(routing_graph.latitude[tail[arcs[0]]]);
                    coordinates.push_back(lonlat_from);
                    shape_distances.push_back(total_distance);
                }

                for (auto xy : arcs) {
                    unsigned x = tail[xy];
                    unsigned y = routing_graph.head[xy];
                    // xy is the arc from node x to node y

                    float total_distance_intermediate = total_distance;

                    float prev_longitude = routing_graph.longitude[x];
                    float prev_latitude = routing_graph.latitude[x];

                    for (unsigned m = routing_graph.first_modelling_node[xy]; m < routing_graph.first_modelling_node[xy + 1]; ++m) {
                        float this_longitude = routing_graph.modelling_node_longitude[m];
                        float this_latitude = routing_graph.modelling_node_latitude[m];

                        total_distance_intermediate += geo_dist(prev_latitude, prev_longitude, this_latitude, this_longitude);

                        json lonlat_inter;
                        lonlat_inter.push_back(this_longitude);
                        lonlat_inter.push_back(this_latitude);
                        coordinates.push_back(lonlat_inter);
                        shape_distances.push_back(total_distance_intermediate);

                        prev_longitude = this_longitude;
                        prev_latitude = this_latitude;
                    }

                    total_distance += routing_graph.geo_distance[xy]; /* prevent accumulative routing errors */

                    json lonlat_to;
                    lonlat_to.push_back(routing_graph.longitude[y]);
                    lonlat_to.push_back(routing_graph.latitude[y]);
                    coordinates.push_back(lonlat_to);
                    shape_distances.push_back(total_distance);
                }

                json feature = {{"type", "Feature"}, {"properties", {{ "distance", distance }, {"shape_distances", shape_distances} }}, {"geometry", { {"type", "LineString"}, {"coordinates", coordinates} } }};
                reply["features"].push_back(feature);
            }
        }
clean:
        zmqpp::message zmq_reply;
        zmq_reply << reply.dump();
        socket.send(zmq_reply);
    }
}
