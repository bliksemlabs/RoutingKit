#include <routingkit/osm_graph_builder.h>
#include <routingkit/osm_profile.h>
#include <routingkit/osm_simple.h>

#include <vector>
#include <stdint.h>
#include <string>

namespace RoutingKit{

SimpleOSMCarRoutingGraph simple_load_osm_car_routing_graph_from_pbf(
	const std::string&pbf_file,
	const std::function<void(const std::string&)>&log_message,
	bool file_is_ordered_even_though_file_header_says_that_it_is_unordered
){
	auto mapping = load_osm_id_mapping_from_pbf(
		pbf_file,
		nullptr,
		[&](uint64_t osm_way_id, const TagMap&tags){
			return is_osm_way_used_by_cars(osm_way_id, tags, log_message);
		},
		log_message
	);

	unsigned routing_way_count = mapping.is_routing_way.population_count();
	std::vector<uint32_t>way_speed(routing_way_count);
	std::vector<unsigned>way_max_speed(routing_way_count);
	std::vector<OSMWayClass>way_class(routing_way_count);

	auto routing_graph = load_osm_routing_graph_from_pbf(
		pbf_file,
		mapping,
		[&](uint64_t osm_way_id, unsigned routing_way_id, const TagMap&way_tags){
			way_speed[routing_way_id] = get_osm_way_speed(osm_way_id, way_tags, log_message);
			way_class[routing_way_id] = get_osm_way_class(osm_way_id, way_tags, log_message);
			way_max_speed[routing_way_id] = get_osm_max_speed(osm_way_id, way_tags, log_message);

			return get_osm_car_direction_category(osm_way_id, way_tags, log_message);
		},
		log_message,
		true
	);

	mapping = OSMRoutingIDMapping(); // release memory

	SimpleOSMCarRoutingGraph ret;
	ret.first_out = std::move(routing_graph.first_out);
	ret.head = std::move(routing_graph.head);
	ret.geo_distance = std::move(routing_graph.geo_distance);
	ret.latitude = std::move(routing_graph.latitude);
	ret.longitude = std::move(routing_graph.longitude);
	ret.travel_time = ret.geo_distance;
	ret.way_class.reserve(ret.geo_distance.size());
	ret.way_speed.reserve(ret.geo_distance.size());
	ret.way_max_speed.reserve(ret.geo_distance.size());

	for(unsigned a=0; a<ret.travel_time.size(); ++a){
		ret.travel_time[a] *= 18;
		ret.travel_time[a] /= way_speed[routing_graph.way[a]];
		ret.travel_time[a] /= 5;

		ret.way_class[a] = way_class[routing_graph.way[a]];
		ret.way_speed[a] = way_speed[routing_graph.way[a]];
		ret.way_max_speed[a] = way_max_speed[routing_graph.way[a]];
	}

	return ret;
}

} // RoutingKit

