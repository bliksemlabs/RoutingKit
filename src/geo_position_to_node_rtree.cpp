#include <routingkit/geo_position_to_node.h>
#include <routingkit/geo_dist.h>
#include <routingkit/geo_position_to_node_rtree.h>
#include <vector>
#include <iostream>
#include <iomanip>

// TODO: This is not supposed to, any better w
// MyTree tree;

namespace RoutingKit{

namespace {
    bool MySearchCallback(unsigned id, void *results) { 
        std::vector<unsigned> &vector = *static_cast<std::vector<unsigned>*>(results);
        vector.push_back(id);
        return true; // keep going
    }
}

GeoPositionToNodeRTree::GeoPositionToNodeRTree(const std::vector<float>&latitude, const std::vector<float>&longitude, const std::vector<unsigned>&first_out, const std::vector<unsigned>&head, const std::vector<unsigned>&tail, const std::vector<unsigned>&first_modelling_node, const std::vector<float>&modelling_node_latitude, const std::vector<float>&modelling_node_longitude):
    imin(head.size()), imax(head.size()) {
    assert(latitude.size() == longitude.size());

    this->head = std::move(head);
    this->tail = std::move(tail);
    this->latitude = std::move(latitude);
    this->longitude = std::move(longitude);
    this->first_modelling_node = std::move(first_modelling_node);
    this->modelling_node_latitude = std::move(modelling_node_latitude);
    this->modelling_node_longitude = std::move(modelling_node_longitude);


    for (unsigned x = 0; x < (first_out.size() - 1); ++x) {
        // xy is the arc from node x to node y
        for(unsigned xy = first_out[x];
                     xy < first_out[x+1]; ++xy) {
            unsigned y = head[xy];
            float min[2], max[2];

            if (longitude[x] > longitude[y]) {
                max[0] = longitude[x];
                min[0] = longitude[y];
            } else {
                max[0] = longitude[y];
                min[0] = longitude[x];
            }

            if (latitude[x] > latitude[y]) {
                max[1] = latitude[x];
                min[1] = latitude[y];
            } else {
                max[1] = latitude[y];
                min[1] = latitude[x];
            }

            for (unsigned m = first_modelling_node[xy];
                          m < first_modelling_node[xy + 1]; ++m) {
                if (modelling_node_longitude[m] > max[0]) {
                    max[0] = modelling_node_longitude[m]; 
                } else if (modelling_node_longitude[m] < min[0]) {
                    min[0] = modelling_node_longitude[m];
                }

                if (modelling_node_latitude[m] > max[1]) {
                    max[1] = modelling_node_latitude[m];
                } else if (modelling_node_latitude[m] < min[1]) {
                    min[1] = modelling_node_latitude[m];
                }
            }

            imin[xy][0] = min[0];
            imin[xy][1] = min[1];
            imax[xy][0] = max[0];
            imax[xy][1] = max[1];
        
            tree.Insert(min, max, xy);
        }
    }
}

GeoPositionToNodeRTree::RTreeQueryResult GeoPositionToNodeRTree::find_nearest_neighbor(float query_latitude, float query_longitude) {

	/* we expand the range around the latitude/longitude by this factor */ 
    float radius = 0.00005f;

	/* we store the envelope for each iteration until we have a hit */ 
	float search_rect_min[2];
	float search_rect_max[2];

	std::vector<unsigned> results;
	results.reserve(20);

	int nhits = 0;
	while (nhits == 0) {
		radius *= 10;

		search_rect_min[0] = query_longitude - radius;
		search_rect_min[1] = query_latitude - radius;
		search_rect_max[0] = query_longitude + radius;
		search_rect_max[1] = query_latitude + radius;

		nhits = tree.Search(search_rect_min, search_rect_max, MySearchCallback, (void *) &results);
	}

	/* imagine that we have found a result given a certain radius from our search position.
       the result is rectangle based, hence worst case it could be in the corners of the square
       enveloping the circle enveloping the radius. [o]
       The unit vector of the radius matching the inset [( )] will have a scaled radius of 1/sqrt(2) = 1 / (r / math.sqrt((r*r) + (r*r)))
       the optimal rect envelopes that radius.
       We can now guarantee that everything will be found, will be closest with respect of the rest of the set.
     */

	float optimal_rect_min[2];
	float optimal_rect_max[2];
	float optimal = radius / 1.41f;

	optimal_rect_min[0] = query_longitude - optimal;
	optimal_rect_min[1] = query_latitude - optimal;
	optimal_rect_max[0] = query_longitude + optimal;
	optimal_rect_max[1] = query_latitude + optimal;

	
	/* A second guarantee that we need is the knowledge of full coverage.
       If any envelope is fully contained within our optimal_rect we can be sure that any result in that set is closest.
       If no envelope is fully contained we must expand our search region.
       For optimisation purposes it might be more efficient to have bigger resultset, to guarantee that any of its results is contained. 
     */

	bool has_contained = false;
	float min_diff = 1000.0f;
	unsigned min_diff_id = 0;

	for (unsigned i : results) {
		if (optimal_rect_min[0] <= imin[i][0] && optimal_rect_min[1] <= imin[i][1] && optimal_rect_max[0] >= imax[i][0] && optimal_rect_max[1] >= imax[i][1]) {
			has_contained = true;
			break;
		} else {
			float overlap_x = fmax(0, fmin(optimal_rect_max[0], imax[i][0]) - fmax(optimal_rect_min[0], imin[i][0]));
			float overlap_y = fmax(0, fmin(optimal_rect_max[1], imax[i][1]) - fmax(optimal_rect_min[1], imin[i][1]));
			float surface_o = overlap_x * overlap_y;
			float surface_i = (imax[i][0] - imin[i][0]) * (imax[i][1] - imin[i][1]);
			float diff = surface_i - surface_o;

			if (diff < min_diff) {
				min_diff_id = i;
				min_diff = diff;
			}
		}
	}

	/* we need to search a bit broader to have contained results */
	if (!has_contained) {
		results.clear();

		float max_diff = 0.0f;
		if (imax[min_diff_id][0] > optimal_rect_max[0]) {
			float diff = imax[min_diff_id][0] - optimal_rect_max[0];
			if (diff > max_diff) max_diff = diff;
		}
		if (imax[min_diff_id][1] > optimal_rect_max[1]) {
			float diff = imax[min_diff_id][1] - optimal_rect_max[1];
			if (diff > max_diff) max_diff = diff;
		}
		if (imin[min_diff_id][0] < optimal_rect_min[0]) {
			float diff = optimal_rect_min[0] - imin[min_diff_id][0];
			if (diff > max_diff) max_diff = diff;
		}
		if (imin[min_diff_id][1] < optimal_rect_min[1]) {
			float diff = optimal_rect_min[1] - imin[min_diff_id][1];
			if (diff > max_diff) max_diff = diff;
		}

		search_rect_min[0] = optimal_rect_min[0] - max_diff;
		search_rect_min[1] = optimal_rect_min[1] - max_diff;
		search_rect_max[0] = optimal_rect_max[0] + max_diff;
		search_rect_max[1] = optimal_rect_max[1] + max_diff;

		tree.Search(search_rect_min, search_rect_max, MySearchCallback, (void *) &results);
		for (unsigned i : results) {
			if (search_rect_min[0] <= imin[i][0] && search_rect_min[1] <= imin[i][1] && search_rect_max[0] >= imax[i][0] && search_rect_max[1] >= imax[i][1]) {
				has_contained = true;
				break;
			}
		}
	}

	/* This part of the code is the true quality improvement for georeferencing.
       Given the results above we know for sure that there will exist a result that is closest to our search origin.
       Given that the results only have an envelope we will now iterate over the first and last point, and the modelling nodes.
       Each arc is evaluated on proximity to the search origin.
       Due to the fact that a way may be traversed in both directions, two arcs may be considered optimum.
	 */ 

	float best_traversed = INFINITY;
	float best_distance = INFINITY;
	float best_realdistance = INFINITY;
	unsigned best_id = invalid_id;
    unsigned best_m = invalid_id;
	float best_pp_lat;
	float best_pp_lon;

	for (unsigned xy : results) {
		unsigned x = tail[xy];
		unsigned y = head[xy];

		float prev_lat = latitude[x];
		float prev_lon = longitude[x];

		float traversed = 0.0f;

		double a_dist, b_dist, pp_dist, pp_lat, pp_lon;

		for (unsigned m = first_modelling_node[xy]; m < first_modelling_node[xy + 1]; ++m) {
			geo_dist_projected_to_arc(prev_lat, prev_lon, modelling_node_latitude[m], modelling_node_longitude[m], query_latitude, query_longitude, a_dist, b_dist, pp_dist, pp_lat, pp_lon);

			// float distance = traversed + (pp_dist * pp_dist);
			float distance = pp_dist;
			if (distance < best_distance) {
				best_realdistance = traversed + pp_dist;
                best_traversed = traversed;
				best_distance = distance;
				best_id = xy;
				best_pp_lat = pp_lat;
				best_pp_lon = pp_lon;
                best_m = m;
			} else if (distance == best_distance) {
                /* here we would prefer to snap to the edge in the direction the road, country specific */
                
                float delta_lat = modelling_node_latitude[m] - prev_lat;
                float delta_lon = modelling_node_longitude[m] - prev_lon;
                float delta_query_lat = query_latitude - prev_lat;
                float delta_query_lon = query_longitude - prev_lon;

                /* https://math.stackexchange.com/questions/274712/calculate-on-which-side-of-a-straight-line-is-a-given-point-located */
                float d = (delta_query_lon * delta_lat) - (delta_query_lat * delta_lon);
                if (d > 0.0f) {
                    best_id = xy;
                    best_m = m;
                } else if (d == 0.0f) {
                    /* consider that we are on the line, lets snap to the nearest distance
                    * TODO: test
                    */
                    float tmp_realdistance = traversed + pp_dist;
                    if (tmp_realdistance < best_realdistance) {
                        best_realdistance = tmp_realdistance;
                        best_traversed = traversed;
                        best_id = xy;
                        best_m = m;
                    }
                }
            }

			prev_lat = modelling_node_latitude[m];
			prev_lon = modelling_node_longitude[m];
			traversed += geo_dist(prev_lat, prev_lon, modelling_node_latitude[m], modelling_node_longitude[m]);
		}

		geo_dist_projected_to_arc(prev_lat, prev_lon, latitude[y], longitude[y], query_latitude, query_longitude, a_dist, b_dist, pp_dist, pp_lat, pp_lon);
		// float distance = traversed + (pp_dist * pp_dist);
        float distance = pp_dist;
        // std::cout << "pp_dist: " << pp_dist << std::endl;
		if (distance < best_distance) {
			best_realdistance = traversed + pp_dist;
			best_distance = distance;
            best_traversed = traversed;
			best_id = xy;
			best_pp_lat = pp_lat;
			best_pp_lon = pp_lon;
            best_m = first_modelling_node[xy + 1];
		} else if (distance == best_distance) {
            /* here we would prefer to snap to the edge in the direction the road, country specific */
            
            float delta_lat = latitude[y] - prev_lat;
            float delta_lon = longitude[y] - prev_lon;
            float delta_query_lat = query_latitude - prev_lat;
            float delta_query_lon = query_longitude - prev_lon;

            /* https://math.stackexchange.com/questions/274712/calculate-on-which-side-of-a-straight-line-is-a-given-point-located */
            float d = (delta_query_lon * delta_lat) - (delta_query_lat * delta_lon);
            if (d > 0.0f) {
                best_id = xy;
                best_m = first_modelling_node[xy + 1];
            } else if (d == 0.0f) {
                /* consider that we are on the line, lets snap to the nearest distance
                 * TODO: test
                 */
                float tmp_realdistance = traversed + pp_dist;
                if (tmp_realdistance < best_realdistance) {
                    best_realdistance = tmp_realdistance;
                    best_id = xy;
                    best_m = first_modelling_node[xy + 1];
                }
            }
        }
	}

    /*
    std::cout << "best_id: " << best_id << std::endl;
    std::cout << "tail: " << tail[best_id] << std::endl;
    std::cout << "head: " << head[best_id] << std::endl;

    std::cout << "LINESTRING(" << std::fixed << longitude[tail[best_id]] << " " << std::fixed << latitude[tail[best_id]] << ", " << std::fixed << longitude[head[best_id]] << " " << std::fixed << latitude[head[best_id]]  << ")" << std::endl;
    std::cout << "POINT(" << std::fixed << query_longitude << " " << std::fixed << query_latitude << ")" << std::endl;
    */

    RTreeQueryResult result = { invalid_id, invalid_id, INFINITY, INFINITY, INFINITY, INFINITY };
    if (best_id != invalid_id) {
        result.xy = best_id;
        result.m = best_m;
        result.distance_since_origin = best_traversed;
        result.distance_to_edge = best_distance;
        result.longitude = best_pp_lon;
        result.latitude = best_pp_lat;
    }

    return result;
}

} // RoutingKit
