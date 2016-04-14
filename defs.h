#ifndef __DEFS_H_
#define __DEFS_H_

#include <ap_int.h>

#define MAX_SIZE 3600
#define MAX_WORLD_SIZE 60

//Typedefs
typedef ap_uint<32> uint32;
typedef ap_int<32> int32;
typedef ap_uint<2> uint2;
typedef unsigned char uint8;

typedef struct __attribute__((packed)) {
	uint8 x;
	uint8 y;
} point_t;

typedef struct __attribute__((packed))  {
	point_t point;
	int cost;
} node_t;

typedef struct __attribute__((packed)) {
	int size;
	bool exists[MAX_SIZE];
	node_t items[MAX_SIZE];
} list_t;

typedef struct __attribute__((packed)) {
	uint8 x;
	uint8 y;
} waypoint_t;

typedef struct __attribute__((packed)) {
	uint8 x;
	uint8 y;
	uint8 direction; // 0 horizontal; 1 vertical
	uint8 length;
} wall_t;

typedef struct __attribute__((packed)) {
	uint8 size;
	uint8 waypoints_size;
	waypoint_t waypoints[12];
	uint8 walls_size;
	wall_t walls[20];
} world_t;

#endif
