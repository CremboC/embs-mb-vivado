#include "toplevel.h"

#define NOTHING 0
#define WAYPOINT 1
#define WALL 2

#define UNVISITED 0
#define OPEN 1
#define CLOSED 2

world_t world;
holder_t o;

int manhattan(point_t *p1, point_t *p2) {
	return p2->x - p1->x + p2->y - p1->y;
}

void set_unvisited(uint8 x, uint8 y) {
	o.nodes[x][y].status = UNVISITED;
}

void set_closed(uint8 x, uint8 y) {
	o.nodes[x][y].status = CLOSED;
	o.size_closed++;
	o.size_open--;
}

void set_open(uint8 x, uint8 y) {
	o.nodes[x][y].status = OPEN;
	o.size_open++;
	o.size_closed--;
}

void check_neighbor(uint8 x, uint8 y, node_t *n, bool check) {
	if (check && o.nodes[x][y].type != WALL) {
		unsigned short new_cost = n->cost + 1;

		switch (o.nodes[x][y].status) {
		case OPEN: // if open then potentialy change the cost with the new cost as we found a shorter route
			if (o.nodes[x][y].status == OPEN && o.nodes[x][y].cost > new_cost) {
				o.nodes[x][y].cost = new_cost; // replace the cost with m.cost
			}
			break;
		case UNVISITED: // if unvisited, mark it as open and set the cost to it
			set_open(x, y);
			o.nodes[x][y].cost = new_cost;
			break;
		}
	}
}

int a_star_2(point_t start, point_t end) {
	// Initialise closed to be the empty list.
	// ...

	// Initialise open to contain a node of start, with a cost of 0
	set_open(start.x, start.y);

	while (o.size_open > 0) { // Whilst there are nodes in the open list
		point_t min_heuristic_index;
		int min_heuristic = 100000000;

		// Find the node (n) in open with the lowest n.cost + manhattan(n.point, end)
		for (int x = 0; x < MAX_WORLD_SIZE; x++) {
			for (int y = 0; y < MAX_WORLD_SIZE; y++) {
				if (o.nodes[x][y].status == OPEN) {
					int heuristic = o.nodes[x][y].cost + manhattan(&o.nodes[x][y].point, &end);
					if (heuristic < min_heuristic) {
						min_heuristic_index.x = x;
						min_heuristic_index.y = y;
						min_heuristic = heuristic;
					}
				}
			}
		}

		// Remove n from open and add n to closed
		node_t n = o.nodes[min_heuristic_index.x][min_heuristic_index.y];
		set_closed(n.point.x, n.point.y);

		// If n.point is end then the search is complete. n.cost is the length of the path.
		if (n.point.x == end.x && n.point.y == end.y) {
//			printf("Search complete. Cost: %d\r\n", n.cost);
			return n.cost;
		}

		// For the points that are north, south, east and west from n.point (and that are not walls)
		check_neighbor(n.point.x + 1, n.point.y, &n, n.point.x + 1 < world.size);
		check_neighbor(n.point.x - 1, n.point.y, &n, n.point.x - 1 >= 0);
		check_neighbor(n.point.x, n.point.y + 1, &n, n.point.y + 1 < world.size);
		check_neighbor(n.point.x, n.point.y - 1, &n, n.point.y - 1 >= 0);
	}

	return -1;
}

//Top-level function
void toplevel(hls::stream<uint32> &input, hls::stream<uint32> &output) {
#pragma HLS RESOURCE variable=input core=AXI4Stream
#pragma HLS RESOURCE variable=output core=AXI4Stream
#pragma HLS INTERFACE ap_ctrl_none port=return

	for (int x = 0; x < MAX_WORLD_SIZE; x++) {
		for (int y = 0; y < MAX_WORLD_SIZE; y++) {
			o.nodes[x][y].cost = 0;
			o.nodes[x][y].point.x = x;
			o.nodes[x][y].point.y = y;
			o.nodes[x][y].status = UNVISITED;
			o.nodes[x][y].type = NOTHING;
		}
	}

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
		o.nodes[wt.x][wt.y].type = WALL;
	}

	world.waypoints_size = input.read();

	for (int i = 0; i < world.waypoints_size; i++) {
		waypoint_t wt;
		uint32 in = input.read();
		wt.x = (uint8) (in & 0xFF);
		wt.y = (uint8) ((in >> 8) & 0xFF);

		world.waypoints[i] = wt;
		o.nodes[wt.x][wt.y].type = WAYPOINT;
	}

	// create walls in grid2
	for (int i = 0; i < world.walls_size; i++) {
		switch (world.walls[i].direction) {
		case 0: // horizontal
			for (int j = 0; j < world.walls[i].length; j++) {
				if (world.walls[i].x + j < world.size) {
					o.nodes[world.walls[i].x + j][world.walls[i].y].type = WALL;
				}
			}
			break;
		case 1: // vertical
			for (int j = 0; j < world.walls[i].length; j++) {
				if (world.walls[i].y + j < world.size) {
					o.nodes[world.walls[i].x][world.walls[i].y + j].type = WALL;
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

