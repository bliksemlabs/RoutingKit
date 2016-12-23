#include <routingkit/osm_profile.h>

namespace RoutingKit{

namespace{
	bool str_eq(const char*l, const char*r){
		return !strcmp(l, r);
	}

	bool str_wild_char_eq(const char*l, const char*r){
		while(*l != '\0' && *r != '\0'){
			if(*l != '?' && *r != '?' && *l != *r)
				return false;
			++l;
			++r;
		}
		return *l == '\0' && *r == '\0';
	}

	bool starts_with(const char*prefix, const char*str){
		while(*prefix != '\0' && *str == *prefix){
			++prefix;
			++str;
		}
		return *prefix == '\0';
	}


	void copy_str_and_make_lower_case(const char*in, char*out, unsigned out_size){
		char*out_end = out + out_size-1;
		while(*in && out != out_end){
			if('A' <= *in && *in <= 'Z')
				*out = *in - 'A' + 'a';
			else
				*out = *in;
			++in;
			++out;
		}
		*out = '\0';
	}

	// Splits the string at some separators such as ; and calls f(str) for each part.
	// The ; is replaced by a '\0'. Leading spaces are removed
	template<class F>
	void split_str_at_osm_value_separators(char*in, const F&f){
		while(*in == ' ')
			++in;
		const char*value_begin = in;
		for(;;){
			while(*in != '\0' && *in != ';')
				++in;
			if(*in == '\0'){
				f(value_begin);
				return;
			}else{
				*in = '\0';
				f(value_begin);
				++in;
				while(*in == ' ')
					++in;
				value_begin = in;
			}
		}
	}
}

bool is_osm_way_used_by_cars(uint64_t osm_way_id, const TagMap&tags, std::function<void(const std::string&)>log_message){
	const char* junction = tags["junction"];
	if(junction != nullptr)
		return true;

	const char* highway = tags["highway"];
	if(highway == nullptr)
		return false;

	const char*motor_vehicle = tags["motor_vehicle"];
	if(motor_vehicle && str_eq(motor_vehicle, "no"))
		return false;

	const char*access = tags["access"];
	if(access){
		if(!(str_eq(access, "yes") || str_eq(access, "permissive") || str_eq(access, "delivery")|| str_eq(access, "designated") || str_eq(access, "destination")))
			return false;
	}

	if(
		str_eq(highway, "motorway") ||
		str_eq(highway, "trunk") ||
		str_eq(highway, "primary") ||
		str_eq(highway, "secondary") ||
		str_eq(highway, "tertiary") ||
		str_eq(highway, "unclassified") ||
		str_eq(highway, "residential") ||
		str_eq(highway, "service") ||
		str_eq(highway, "motorway_link") ||
		str_eq(highway, "trunk_link") ||
		str_eq(highway, "primary_link") ||
		str_eq(highway, "secondary_link") ||
		str_eq(highway, "motorway_junction") ||
		str_eq(highway, "living_street") ||
		str_eq(highway, "residential") ||
		str_eq(highway, "ferry")
	)
		return true;

	if(str_eq(highway, "bicycle_road")){
		auto motorcar = tags["motorcar"];
		if(motorcar != nullptr)
			if(str_eq(motorcar, "yes"))
				return true;
		return false;
	}

	if(
		str_eq(highway, "construction") ||
		str_eq(highway, "path") ||
		str_eq(highway, "footway") ||
		str_eq(highway, "cycleway") ||
		str_eq(highway, "bridleway") ||
		str_eq(highway, "pedestrian") ||
		str_eq(highway, "bus_guideway") ||
		str_eq(highway, "raceway") ||
		str_eq(highway, "escape") ||
		str_eq(highway, "steps") ||
		str_eq(highway, "conveying")
	)
		return false;

	const char* oneway = tags["oneway"];
	if(oneway != nullptr){
		if(str_eq(oneway, "reversible")) {
			return false;
		}
	}

	const char* maxspeed = tags["maxspeed"];
	if(maxspeed != nullptr)
		return true;

	return false;
}


OSMWayClass get_osm_way_class(uint64_t osm_way_id, const TagMap&tags, std::function<void(const std::string&)>log_message) {
	const char
		*junction = tags["junction"],
		*highway = tags["highway"]
	;

	if (highway != nullptr) {
		if (str_eq(highway, "primary")) return OSMWayClass::primary;
		if (str_eq(highway, "primary_link")) return OSMWayClass::primary_link;
		if (str_eq(highway, "secondary")) return OSMWayClass::secondary;
		if (str_eq(highway, "secondary_link")) return OSMWayClass::secondary_link;
		if (str_eq(highway, "tertiary")) return OSMWayClass::tertiary;
		if (str_eq(highway, "tertiary_link")) return OSMWayClass::tertiary_link;
		if (str_eq(highway, "trunk")) return OSMWayClass::trunk;
		if (str_eq(highway, "trunk_link")) return OSMWayClass::trunk_link;
		if (str_eq(highway, "motorway")) return OSMWayClass::motorway;
		if (str_eq(highway, "motorway_link")) return OSMWayClass::motorway_link;
		if (str_eq(highway, "motorway_junction")) return OSMWayClass::motorway_junction;
		if (str_eq(highway, "unclassified")) return OSMWayClass::unclassified;
		if (str_eq(highway, "service")) return OSMWayClass::service;
		if (str_eq(highway, "living_street")) return OSMWayClass::living_street;
		if (str_eq(highway, "residential")) return OSMWayClass::residential;
		if (str_eq(highway, "bicycle_road")) return OSMWayClass::bicycle_road;
		if (str_eq(highway, "ferry")) return OSMWayClass::ferry;
		if (str_eq(highway, "construction")) return OSMWayClass::construction;
		if (str_eq(highway, "path")) return OSMWayClass::path;
		if (str_eq(highway, "footway")) return OSMWayClass::footway;
		if (str_eq(highway, "cycleway")) return OSMWayClass::cycleway;
		if (str_eq(highway, "bridleway")) return OSMWayClass::bridleway;
		if (str_eq(highway, "pedestian")) return OSMWayClass::pedestian;
		if (str_eq(highway, "bus_guideway")) return OSMWayClass::bus_guideway;
		if (str_eq(highway, "raceway")) return OSMWayClass::raceway;
		if (str_eq(highway, "escape")) return OSMWayClass::escape;
		if (str_eq(highway, "steps")) return OSMWayClass::steps;
		if (str_eq(highway, "conveying")) return OSMWayClass::conveying;

	} else
	if (junction != nullptr) {
		return OSMWayClass::junction;
	}

	return OSMWayClass::other;
}


const char* get_osm_way_class_string(OSMWayClass way_class) {
		if (way_class == OSMWayClass::primary) { return "primary"; } else
		if (way_class == OSMWayClass::primary_link) { return "primary_link"; } else
		if (way_class == OSMWayClass::secondary) { return "secondary"; } else
		if (way_class == OSMWayClass::secondary_link) { return "secondary_link"; } else
		if (way_class == OSMWayClass::tertiary) { return "tertiary"; } else
		if (way_class == OSMWayClass::tertiary_link) { return "tertiary_link"; } else
		if (way_class == OSMWayClass::trunk) { return "trunk"; } else
		if (way_class == OSMWayClass::trunk_link) { return "trunk_link"; } else
		if (way_class == OSMWayClass::motorway) { return "motorway"; } else
		if (way_class == OSMWayClass::motorway_link) { return "motorway_link"; } else
		if (way_class == OSMWayClass::motorway_junction) { return "motorway_junction"; } else
		if (way_class == OSMWayClass::unclassified) { return "unclassified"; } else
		if (way_class == OSMWayClass::service) { return "service"; } else
		if (way_class == OSMWayClass::living_street) { return "living_street"; } else
		if (way_class == OSMWayClass::residential) { return "residential"; } else
		if (way_class == OSMWayClass::bicycle_road) { return "bicycle_road"; } else
		if (way_class == OSMWayClass::ferry) { return "ferry"; } else
		if (way_class == OSMWayClass::junction) { return "junction"; } else
		if (way_class == OSMWayClass::construction) { return "construction"; } else
		if (way_class == OSMWayClass::path) { return "path"; } else
		if (way_class == OSMWayClass::footway) { return "footway"; } else
		if (way_class == OSMWayClass::cycleway) { return "cycleway"; } else
		if (way_class == OSMWayClass::bridleway) { return "bridleway"; } else
		if (way_class == OSMWayClass::pedestian) { return "pedestian"; } else
		if (way_class == OSMWayClass::bus_guideway) { return "bus_guideway"; } else
		if (way_class == OSMWayClass::raceway) { return "raceway"; } else
		if (way_class == OSMWayClass::escape) { return "escape"; } else
		if (way_class == OSMWayClass::steps) { return "steps"; } else
		if (way_class == OSMWayClass::conveying) { return "conveying"; }

		return "other";
}


OSMWayDirectionCategory get_osm_car_direction_category(uint64_t osm_way_id, const TagMap&tags, std::function<void(const std::string&)>log_message){
	const char
		*oneway = tags["oneway"],
		*junction = tags["junction"],
		*highway = tags["highway"]
	;
	if(oneway != nullptr){
		if(str_eq(oneway, "-1") || str_eq(oneway, "reverse") || str_eq(oneway, "backward")) {
			return OSMWayDirectionCategory::only_open_backwards;
		} else if(str_eq(oneway, "yes") || str_eq(oneway, "true") || str_eq(oneway, "1")) {
			return OSMWayDirectionCategory::only_open_forwards;
		} else if(str_eq(oneway, "no") || str_eq(oneway, "false") || str_eq(oneway, "0")) {
			return OSMWayDirectionCategory::open_in_both;
		} else if(str_eq(oneway, "reversible")) {
			return OSMWayDirectionCategory::closed;
		} else {
			log_message("Warning: OSM way "+std::to_string(osm_way_id)+" has unknown oneway tag value \""+oneway+"\" for \"oneway\". Way is closed.");
		}
	} else if(junction != nullptr && str_eq(junction, "roundabout")) {
		return OSMWayDirectionCategory::only_open_forwards;
	} else if(highway != nullptr && (str_eq(highway, "motorway") || str_eq(highway, "motorway_link"))) {
		return OSMWayDirectionCategory::only_open_forwards;
	}
	return OSMWayDirectionCategory::open_in_both;
}

	unsigned parse_maxspeed_value(uint64_t osm_way_id, const char*maxspeed, std::function<void(const std::string&)>log_message){
		if(str_eq(maxspeed, "signals") || str_eq(maxspeed, "variable"))
			return inf_weight;

		if(str_eq(maxspeed, "none") || str_eq(maxspeed, "unlimited"))
			return 130;

		if(str_eq(maxspeed, "walk") || str_eq(maxspeed, "foot") || str_wild_char_eq(maxspeed, "??:walk"))
			return 5;

		if(str_wild_char_eq(maxspeed, "??:urban") || str_eq(maxspeed, "urban"))
			return 40;

		if(str_wild_char_eq(maxspeed, "??:living_street") || str_eq(maxspeed, "living_street"))
			return 10;

		if(str_eq(maxspeed, "de:rural") || str_eq(maxspeed, "at:rural") || str_eq(maxspeed, "ro:rural") || str_eq(maxspeed, "rural"))
			return 100;
		if(str_eq(maxspeed, "ru:rural") || str_eq(maxspeed, "fr:rural") || str_eq(maxspeed, "ua:rural"))
			return 90;

		if(str_eq(maxspeed, "ru:motorway"))
			return 110;
		if(str_eq(maxspeed, "at:motorway") || str_eq(maxspeed, "ro:motorway"))
			return 130;

		if(str_eq(maxspeed, "national"))
			return 100;

		if(str_eq(maxspeed, "ro:trunk"))
			return 100;
		if(str_eq(maxspeed, "dk:rural") || str_eq(maxspeed, "ch:rural"))
			return 80;
		if(str_eq(maxspeed, "it:rural") || str_eq(maxspeed, "hu:rural"))
			return 90;
		if(str_eq(maxspeed, "de:zone:30"))
			return 30;


		if('0' <= *maxspeed && *maxspeed <= '9'){
			unsigned speed = 0;
			while('0' <= *maxspeed && *maxspeed <= '9'){
				speed *= 10;
				speed += *maxspeed - '0';
				++maxspeed;
			}
			while(*maxspeed == ' ')
				++maxspeed;
			if(*maxspeed == '\0' || str_eq(maxspeed, "km/h") || str_eq(maxspeed, "kmh") || str_eq(maxspeed, "kph")){
				return speed;
			}else if(str_eq(maxspeed, "mph")){
				return speed * 1609 / 1000;
			}else if(str_eq(maxspeed, "knots")){
				return speed * 1852 / 1000;
			}else{
				log_message("Warning: OSM way "+std::to_string(osm_way_id) +" has an unknown unit \""+maxspeed+"\" for its \"maxspeed\" tag -> assuming \"km/h\".");
				return speed;
			}
		}else{
			log_message("Warning: OSM way "+std::to_string(osm_way_id) +" has an unrecognized value of \""+maxspeed+"\" for its \"maxspeed\" tag.");
		}

		return inf_weight;
	}

unsigned get_osm_way_speed(uint64_t osm_way_id, const TagMap&tags, std::function<void(const std::string&)>log_message){
	auto maxspeed = tags["maxspeed"];
	if(maxspeed != nullptr){
		char lower_case_maxspeed[1024];
		copy_str_and_make_lower_case(maxspeed, lower_case_maxspeed, sizeof(lower_case_maxspeed)-1);

		unsigned speed = inf_weight;

		split_str_at_osm_value_separators(
			lower_case_maxspeed,
			 [&](const char*maxspeed){
				min_to(speed, parse_maxspeed_value(osm_way_id, maxspeed, log_message));
			}
		);

		if(speed == 0){
			speed = 1;
			log_message("Warning: OSM way "+std::to_string(osm_way_id)+" has speed 0 km/h, setting it to 1 km/h");
		}

		if(speed != inf_weight)
			return speed;

	}

	auto junction = tags["junction"];
	if(junction){
		return 20;
	}

	auto highway = tags["highway"];
	if(highway){
		if(str_eq(highway, "motorway"))
			return 90;
		if(str_eq(highway, "motorway_link"))
			return 45;
		if(str_eq(highway, "trunk"))
			return 85;
		if(str_eq(highway, "trunk_link"))
			return 40;
		if(str_eq(highway, "primary"))
			return 65;
		if(str_eq(highway, "primary_link"))
			return 30;
		if(str_eq(highway, "secondary"))
			return 55;
		if(str_eq(highway, "secondary_link"))
			return 25;
		if(str_eq(highway, "tertiary"))
			return 40;
		if(str_eq(highway, "tertiary_link"))
			return 20;
		if(str_eq(highway, "unclassified"))
			return 25;
		if(str_eq(highway, "residential"))
			return 25;
		if(str_eq(highway, "living_street"))
			return 10;
		if(str_eq(highway, "service"))
			return 1;
		if(str_eq(highway, "ferry"))
			return 5;
	}

	if(maxspeed && highway)
		log_message("Warning: OSM way "+std::to_string(osm_way_id) +" has an unrecognized \"maxspeed\" tag of \""+maxspeed+"\" and an unrecognized \"highway\" tag of \""+highway+"\" and an no junction tag -> assuming 50km/h.");
	if(!maxspeed && highway)
		log_message("Warning: OSM way "+std::to_string(osm_way_id) +" has no \"maxspeed\" and an unrecognized \"highway\" tag of \""+highway+"\" and an no junction tag -> assuming 50km/h.");
	if(!maxspeed && !highway)
		log_message("Warning: OSM way "+std::to_string(osm_way_id) +" has no \"maxspeed\" and no \"highway\" tag of \""+highway+"\" and an no junction tag -> assuming 50km/h.");
	if(maxspeed && !highway)
		log_message("Warning: OSM way "+std::to_string(osm_way_id) +" has an unrecognized \"maxspeed\" tag of \""+maxspeed+"\" and no \"highway\" tag and an no junction tag -> assuming 50km/h.");
	return 50;
}

std::string get_osm_way_name(uint64_t osm_way_id, const TagMap&tags, std::function<void(const std::string&)>log_message){
	auto
		name = tags["name"],
		ref = tags["ref"];

	if(name != nullptr && ref != nullptr)
		return std::string(name) + ";"+ref;
	else if(name != nullptr)
		return std::string(name);
	else if(ref != nullptr)
		return std::string(ref);
	else
		return std::string();
}

} // RoutingKit
