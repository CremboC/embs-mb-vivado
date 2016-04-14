#include "toplevel.h"

#define NOTHING 0
#define WAYPOINT 1
#define WALL 2

#define UNVISITED 0
#define OPEN 1
#define CLOSED 2

int manhattan(point_t p1, point_t p2) {
	return p2.x - p1.x + p2.y - p1.y;
}

world_t world;

// [x][y]
uint2 grid[MAX_WORLD_SIZE][MAX_WORLD_SIZE];

typedef struct __attribute__((packed)) {
	point_t point = {0, 0};
	uint12 cost = 0;
	uint2 status = UNVISITED; // 0 - unvisited, 1 - open, 2 - closed
	bool exist = false;
	uint2 type = NOTHING;
} node_t;

typedef struct __attribute__((packed)) {
	int size_closed;
	int size_open;
	node_t nodes[MAX_WORLD_SIZE][MAX_WORLD_SIZE];
} l_t;

l_t o;

bool is_empty(bool closed) {
	if (closed) return o.size_closed;
	else return o.size_open;
}

int in_list(node_t n, bool closed, point_t *p) {
	for (int x = 0; x < MAX_WORLD_SIZE; x++) {
		for (int y = 0; y < MAX_WORLD_SIZE; y++) {
			if (closed) {
				if (o.nodes[x][y].exist && o.nodes[x][y].status == CLOSED) {
					if (o.nodes[x][y].point.x == n.point.x && o.nodes[x][y].point.y == n.point.y) {
						p->x = x;
						p->y = y;
						return 1;
					}
				}
			} else {
				if (o.nodes[x][y].exist) {
					if (o.nodes[x][y].point.x == n.point.x && o.nodes[x][y].point.y == n.point.y) {
						p->x = x;
						p->y = y;
						return 1;
					}
				}
			}

		}
	}

	return -1;
}

void set_unvisited(uint8 x, uint8 y) {
	o.nodes[x][y].status = UNVISITED;
}

void set_closed(uint8 x, uint8 y) {
	o.nodes[x][y].status = CLOSED;
	o.size_closed++;
}

void set_open(uint8 x, uint8 y) {
	o.nodes[x][y].status = OPEN;
	o.size_open++;
}

void change_cost(uint8 x, uint8 y, uint12 new_cost) {
	o.nodes[x][y].cost = new_cost;
}

void add(uint8 x, uint8 y) {
	o.nodes[x][y].exist = true;
}

void remove(uint8 x, uint8 y) {
	o.nodes[x][y].exist = false;
}

void check_neighbor(uint8 x, uint8 y, node_t n) {
	// For the points that are north, south, east and west from n.point (and that are not walls)
	if (x < world.size && grid[x][y] != WALL) {
		// Create a node m with the neighbour point and cost n.cost + 1
		uint8 x = (uint8) (x);
		uint8 y = y;
		node_t m = {{x, y}, n.cost + 1};

		if (!o.nodes[x][y].status != CLOSED) { // If the neighbour point is not in a node in closed

			// If the point is in open but has a higher cost than m.cost
			if (o.nodes[x][y].status == OPEN && o.nodes[x][y].cost > m.cost) {
				change_cost(x, y, m.cost); // replace the cost with m.cost
			} else if (o.nodes[x][y].status != OPEN) { // If the point is not in open, add m to open
				o.nodes[x][y].status = OPEN;
			}
		}
	}
}

int a_star_2(point_t start, point_t end) {
	// Initialise closed to be the empty list.
	// ...

	// Initialise open to contain a node of start, with a cost of 0
	set_open(start.x, start.y);
	add(start.x, start.y);

	while (!is_empty(false)) { // Whilst there are nodes in the open list
		point_t min_heuristic_index;
		int min_heuristic = 100000000;

		// Find the node (n) in open with the lowest n.cost + manhattan(n.point, end)
		for (int x = 0; x < MAX_WORLD_SIZE; x++) {
			for (int y = 0; y < MAX_WORLD_SIZE; y++) {
				if (o.nodes[x][y].status == OPEN) {
					int heuristic = o.nodes[x][y].cost + manhattan(o.nodes[x][y].point, end);
					if (heuristic < min_heuristic) {
						min_heuristic_index = {x, y};
						min_heuristic = heuristic;
					}
				}
			}
		}

		// Remove n from open and add n to closed
		node_t n = o.nodes[min_heuristic_index.x][min_heuristic_index.y];
		set_closed(min_heuristic_index.x, min_heuristic_index.y);

		// If n.point is end then the search is complete. n.cost is the length of the path.
		if (n.point.x == end.x && n.point.y == end.y) {
//			printf("Search complete. Cost: %d\r\n", n.cost);
			return n.cost;
		}

		// For the points that are north, south, east and west from n.point (and that are not walls)
		check_neighbor(n.point.x + 1, n.point.y, n);
		check_neighbor(n.point.x - 1, n.point.y, n);
		check_neighbor(n.point.x, n.point.y + 1, n);
		check_neighbor(n.point.x, n.point.y - 1, n);
	}

	return -1;
}

//Top-level function
void toplevel(hls::stream<uint32> &input, hls::stream<uint32> &output) {
#pragma HLS RESOURCE variable=input core=AXI4Stream
#pragma HLS RESOURCE variable=output core=AXI4Stream
#pragma HLS INTERFACE ap_ctrl_none port=return

	world.size = input.read();
	world.walls_size = input.read();

	for (int i = 0; i < world.walls_size; i++) {
		wall_t wt;
		uint32 in = input.read();
		wt.x = (uint8) (in & 0xFF);
		wt.y = (uint8) ((in >> 8) & 0xFF);
		wt.direction = (uint8) ((in >> 16) & 0xFF);
		wt.length = (uint8) ((in >> 24) & 0xFF);

		world.walls[i] = wt;
		grid[wt.x][wt.y] = WALL;
	}

	world.waypoints_size = input.read();

	for (int i = 0; i < world.waypoints_size; i++) {
		waypoint_t wt;
		uint32 in = input.read();
		wt.x = (uint8) (in & 0xFF);
		wt.y = (uint8) ((in >> 8) & 0xFF);

		world.waypoints[i] = wt;
		grid[wt.x][wt.y] = WAYPOINT;
	}

	// create walls in grid2
	for (int i = 0; i < world.walls_size; i++) {
		switch (world.walls[i].direction) {
		case 0: // horizontal
			for (int j = 0; j < world.walls[i].length; j++) {
				if (world.walls[i].x + j < world.size) {
					grid[world.walls[i].x + j][world.walls[i].y] = WALL;
				}
			}
			break;
		case 1: // vertical
			for (int j = 0; j < world.walls[i].length; j++) {
				if (world.walls[i].y + j < world.size) {
					grid[world.walls[i].x][world.walls[i].y + j] = WALL;
				}
			}
			break;
		}
	}

	//	for (int x = 0; x < world.size; x++) {
	//		for (int y = 0; y < world.size; y++) {
	//			printf("%d ", (int) grid[y][x]);
	//		}
	//		printf("\r\n");
	//	}

	point_t start = {world.waypoints[0].x, world.waypoints[0].y};
	point_t end = {world.waypoints[1].x, world.waypoints[1].y};

	int cost = a_star_2(start, end);

	output.write(cost);

	return;
}

