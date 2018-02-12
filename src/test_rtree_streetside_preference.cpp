#include <iostream>
#include <routingkit/timer.h>
#include <routingkit/inverse_vector.h>
#include <routingkit/geo_position_to_node.h>
#include <routingkit/geo_position_to_node_rtree.h>

using namespace RoutingKit;
using namespace std;

int main(){
    /* TODO: implement the output in this program, that would also require a different result format */

    /* extremely simple usecase we have a two way road,
     * we prefer to snap to the edge in that matches the side of the road the query is on
     * (0)===(1)
     *     q
     */
    {
    std::vector<float> latitude;
    std::vector<float> longitude;
    std::vector<unsigned> head;
    std::vector<unsigned> tail;
   
    std::vector<unsigned> first_modelling_node;
    std::vector<float> modelling_node_latitude;
    std::vector<float> modelling_node_longitude;
 
    /* node 0 */
    latitude.push_back(0.0f);
    longitude.push_back(0.0f);
 
    /* node 1 */
    latitude.push_back(0.0f);
    longitude.push_back(0.002f);

    /* backward direction */
    tail.push_back(0);
    head.push_back(1);
    first_modelling_node.push_back(0);

    /* forward direction */
    tail.push_back(1);
    head.push_back(0);
    first_modelling_node.push_back(0);

    vector<unsigned> first_out = RoutingKit::invert_vector(tail, latitude.size());
    
    GeoPositionToNodeRTree map_geo_position_rtree(latitude, longitude, first_out, head, tail, first_modelling_node, modelling_node_latitude, modelling_node_longitude);

    map_geo_position_rtree.find_nearest_neighbor(-0.001f, 0.001f);

    std::cout << "same" << std::endl;

    map_geo_position_rtree.find_nearest_neighbor( 0.001f, 0.001f);
 }  

    std::cout << "new" << std::endl;
     {
    std::vector<float> latitude;
    std::vector<float> longitude;
    std::vector<unsigned> head;
    std::vector<unsigned> tail;
   
    std::vector<unsigned> first_modelling_node;
    std::vector<float> modelling_node_latitude;
    std::vector<float> modelling_node_longitude;
 
    /* node 0 */
    latitude.push_back(0.0f);
    longitude.push_back(0.0f);
 
    /* node 1 */
    latitude.push_back(0.002f);
    longitude.push_back(0.0f);

    /* backward direction */
    tail.push_back(0);
    head.push_back(1);
    first_modelling_node.push_back(0);

    /* forward direction */
    tail.push_back(1);
    head.push_back(0);
    first_modelling_node.push_back(0);

    vector<unsigned> first_out = RoutingKit::invert_vector(tail, latitude.size());
    
    GeoPositionToNodeRTree map_geo_position_rtree(latitude, longitude, first_out, head, tail, first_modelling_node, modelling_node_latitude, modelling_node_longitude);

    map_geo_position_rtree.find_nearest_neighbor(0.001f,-0.001f);

    std::cout << "same" << std::endl;

    map_geo_position_rtree.find_nearest_neighbor(0.001f, 0.001f);
 }   

}
