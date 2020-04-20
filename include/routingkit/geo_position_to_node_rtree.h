#ifndef ROUTING_KIT_GEO_POSITION_TO_NODE_RTREE_H
#define ROUTING_KIT_GEO_POSITION_TO_NODE_RTREE_H

#include <routingkit/RTree.h>
#include <routingkit/geo_position_to_node.h>
#include <vector>
#include <array>

typedef RTree<unsigned, float, 2, float> MyTree;

namespace RoutingKit{

class GeoPositionToNodeRTree{
public:
    GeoPositionToNodeRTree(){};

    struct RTreeQueryResult{
        unsigned xy;
        unsigned m;
        float distance_since_origin;
        float distance_to_edge;
        float longitude;
        float latitude;
    };

    GeoPositionToNodeRTree(const std::vector<float>&latitude, const std::vector<float>&longitude, const std::vector<unsigned>&first_out, const std::vector<unsigned>&head, const std::vector<unsigned>&tail, const std::vector<unsigned>&first_modelling_node, const std::vector<float>&modelling_node_latitude, const std::vector<float>&modelling_node_longitude);

    RTreeQueryResult find_nearest_neighbor(float query_latitude, float query_longitude); // TODO: should be const, RTree doesn't do it
    std::vector<RTreeQueryResult> find_all(float query_latitude, float query_longitude); // TODO: should be const, RTree doesn't do it

// private:
    void expand_search_rect(float &query_latitude, float &query_longitude, float (&search_rect_min)[2], float (&search_rect_max)[2], float &radius, std::vector<unsigned> &results);
    void shrink_optimal_rect(float &query_latitude, float &query_longitude, float (&optimal_rect_min)[2], float (&optimal_rect_max)[2], float &radius);
    bool within_optimal_rect(float (&optimal_rect_min)[2], float (&optimal_rect_max)[2], std::vector<unsigned> &results, unsigned &min_diff_id);
    void expand_contained_rect(float (&optimal_rect_min)[2], float (&optimal_rect_max)[2], unsigned &min_diff_id, float (&search_rect_min)[2], float (&search_rect_max)[2]);
    bool validate_search_rect(float (&search_rect_min)[2], float (&search_rect_max)[2], std::vector<unsigned> &results);

    std::vector<unsigned> head;
    std::vector<unsigned> tail;
    std::vector<float> latitude;
    std::vector<float> longitude;
    std::vector<unsigned> first_modelling_node;
    std::vector<float> modelling_node_latitude;
    std::vector<float> modelling_node_longitude;
 
    std::vector<std::array<float, 2>> imin;
    std::vector<std::array<float, 2>> imax;

    MyTree tree;
};

} // RoutingKit

#endif
