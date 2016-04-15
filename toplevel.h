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

//Typedefs
typedef ap_uint<32> uint32;
typedef ap_int<32> int32;
typedef ap_uint<2> uint2;
typedef ap_uint<1> uint1;
typedef ap_uint<12> uint12;
typedef unsigned char uint8;

typedef struct __attribute__((packed)) {
	uint8 x;
	uint8 y;
} point_t;

typedef struct __attribute__((packed)) {
	uint12 cost;
	uint2 status; // 0 - unvisited, 1 - open, 2 - closed
	uint2 type;
} node_t;

typedef struct __attribute__((packed)) {
	uint12 size_open;
	node_t nodes[MAX_WORLD_SIZE][MAX_WORLD_SIZE];
} holder_t;

typedef point_t waypoint_t;

typedef struct {
	waypoint_t w;
	bool used;
	bool exists;
	uint8 order;
} search_waypoint_t;

typedef struct {
	uint8 index;
	uint12 cost;
} next_t;

typedef struct __attribute__((packed)) {
	uint8 x;
	uint8 y;
	uint1 direction; // 0 horizontal; 1 vertical
	uint8 length;
} wall_t;

typedef struct __attribute__((packed)) {
	uint8 size;
	uint8 waypoints_size;
	waypoint_t waypoints[MAX_WAYPOINTS];
	uint8 walls_size;
	wall_t walls[MAX_WALLS];
} world_t;
 
//Prototypes
void toplevel(hls::stream<uint32> &input, hls::stream<uint32> &output);
 
#endif 
