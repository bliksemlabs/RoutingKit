# travel-matrix
Service that provides matrices of shortest distince with matching fastest traveltimes between geo-locations. There are two components: a server and a client. Distance or traveltime from A to B can be different from those for B to A.

##Server
The server accepts json objects containing 2 lists: a list of departure geo-positions and a list of arrival geo-positions. It returns a json object with a matrix (list of lists) of distances or traveltimes optimized for shortest or fastest route.

The server is currently build using ZeroMQ push/pull technology and can be easily convered to router/dealer.

###POST data example

```python
{
    "departs": [[52.747, 4.3453],
                [52.3456, 4.7546],
                [52.5464, 4.5464]],
    "arrives": [[51.3453, 4.5564],
                [52.4564, 4.5465]]
}
```

Where:
* geo-positions are defined as (latitude, longitude).

###Response data example:

```python
{
    "matrix": [[17432, 12882],
               [10454, 33564],
               [21567, 56456]]
}
```
Where "type" and "optimal" are the same as in the request and the dimensions of this response matrix correspond to the post data above.

###Installation

Compile RoutingKit and compile the example by `make`.

###Run

./travel-matrix << path to osm.pbf >>
