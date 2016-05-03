// Exam number: Y0001392

#ifndef __TOPLEVEL_H_
#define __TOPLEVEL_H_
 
#include <stdio.h>
#include <stdlib.h>
#include <ap_int.h>
#include <hls_stream.h>

#define MAX_WORLD_SIZE 60
#define MAX_WALLS 20
#define MAX_WAYPOINTS 12

#define NOTHING 0
#define WAYPOINT 1
#define WALL 2

#define UNVISITED 0
#define OPEN 1
#define CLOSED 2

#define HORIZONTAL 0
#define VERTICAL 1

#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

//Typedefs
typedef ap_uint<32> uint32;
typedef ap_uint<2> uint2;
typedef ap_uint<1> uint1;
typedef ap_uint<6> uint6;
typedef ap_uint<14> uint14;
typedef unsigned char uint8;

typedef struct {
	uint6 x;
	uint6 y;
} point_t;

typedef struct {
	uint14 cost;
	uint2 status; // 0 - unvisited, 1 - open, 2 - closed
	uint2 type;
	uint2 parent_direction; // 0 - north, 1 - east, 2 - south, 3 - west
} node_t;

typedef struct {
	uint32 size_open; // Track number of open nodes
	node_t nodes[MAX_WORLD_SIZE][MAX_WORLD_SIZE]; // The world grid
} holder_t;

typedef point_t waypoint_t;

typedef struct {
	uint6 x;
	uint6 y;
	uint1 direction; // 0 horizontal; 1 vertical
	uint8 length;
} wall_t;

typedef struct {
	uint8 size;
	uint8 waypoints_size;
	waypoint_t waypoints[MAX_WAYPOINTS];
	uint8 walls_size;
	wall_t walls[MAX_WALLS];
} world_t;
 
//Prototypes
void toplevel(hls::stream<uint32> &input, hls::stream<uint32> &output);
 
#endif 
