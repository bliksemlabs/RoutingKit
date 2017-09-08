#include <routingkit/osm_simple.h>
#include <routingkit/contraction_hierarchy.h>
#include <routingkit/inverse_vector.h>
#include <routingkit/timer.h>
#include <routingkit/osm_profile.h>
#include <iostream>
#include <string>
#include <cmath>
#include <thread>
#include <time.h>

using namespace RoutingKit;
using namespace std;

int main (int argc, char *argv[]) {
    if (argc != 2) return -1;
    cout << "Loading: " << endl;

    SimpleOSMCarRoutingGraph graph = simple_load_osm_car_routing_graph_from_pbf(argv[1]);

	srand(time(NULL));

    auto tail = invert_inverse_vector(graph.first_out);

    auto contraction_hierarchy = ContractionHierarchy::build(
            graph.node_count(),
            tail, graph.head,
            graph.travel_time,
            graph.geo_distance
            );

    ContractionHierarchyQuery ch_query(contraction_hierarchy);

	std::vector<unsigned> targets;
	/* create a random source */
	unsigned source = (rand() * 1103515245 + 12345) % graph.latitude.size();

	/* create a random set of targets points */
	for (int i = 0; i < 1000; i++) {
		unsigned target = (rand() * 1103515245 + 12345) % graph.latitude.size();
		targets.push_back(target);
	}
	
	clock_t tStart;

	std::vector<unsigned> distances;
	std::vector<unsigned> alts;
	unsigned runs = 10000;

	tStart  = clock();

	/* benchmark 1: run_to_pinned_targets + get_distances_to_targets + get_alts_to_targets */
	for (unsigned run = 0; run < runs; run++) {
		distances.clear();
		alts.clear();
		distances.reserve(targets.size());
		alts.reserve(targets.size());

		ch_query.reset().add_source(source).pin_targets(targets).run_to_pinned_targets();
		distances = ch_query.get_distances_to_targets();
		alts = ch_query.get_alts_to_targets();
	}

	printf("Time taken get_alts_to_targets (%d times %d): %.2fs\n", targets.size(), runs, (double)(clock() - tStart)/CLOCKS_PER_SEC);

	cout << "Distance: ";
	for (auto distance : distances) {
		cout << distance << " ";
	}
	cout << endl;

	cout << "     Alt: ";
	for (auto alt : alts) {
		cout << alt << " ";
	}
	cout << endl;
	
	/* benchmark 2: run + get_distance + get_arc_path + geo_distance[x] */
	for (unsigned run = 0; run < runs; run++) {
		distances.clear();
		alts.clear();
		distances.reserve(targets.size());
		alts.reserve(targets.size());

		ch_query.reset().add_source(source);
		for(auto target : targets){
			ch_query.reset_target().add_target(target).run();

			/* when succesful, the distance is available */
			unsigned weight = ch_query.get_distance();
			if (weight != 0 && weight != inf_weight) {

				/* the travel time and distances should be computed by a sum over all arcs.
				* If you use the weight from get_distance() the target weight adjustments are
				* taken along. You don't want that as snap_weight will affect the distance returned
				*/
				unsigned travel_time = 0;
				unsigned geo_distance = 0;

				std::vector<unsigned>arcs = ch_query.get_arc_path();
				for (auto a:arcs) {
					travel_time += graph.travel_time[a];
					geo_distance += graph.geo_distance[a];
				}

				// returns a list of distance, time
				alts.push_back(geo_distance);
				distances.push_back(travel_time);
			} else {
				alts.push_back(inf_weight);
				distances.push_back(inf_weight);
			}
		}
	}

	printf("Time taken verbose method (%d times %d): %.2fs\n", targets.size(), runs, (double)(clock() - tStart)/CLOCKS_PER_SEC);

	cout << "Distance: ";
	for (auto distance : distances) {
		cout << distance << " ";
	}
	cout << endl;

	cout << "     Alt: ";
	for (auto alt : alts) {
		cout << alt << " ";
	}
	cout << endl;
}
