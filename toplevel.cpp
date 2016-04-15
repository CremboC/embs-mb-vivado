#include "toplevel.h"

world_t world;
holder_t o;

search_waypoint_t waypoints[MAX_WAYPOINTS] = {};

int manhattan(uint8 x1, uint8 y1, uint8 x2, uint8 y2) {
	return x2 - x1 + y2 - y1;
}

void set_closed(uint8 x, uint8 y) {
	o.nodes[x][y].status = CLOSED;
	o.size_open--;
}

void set_open(uint8 x, uint8 y) {
	o.nodes[x][y].status = OPEN;
	o.size_open++;
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

int a_star(waypoint_t start, waypoint_t end) {
	// Initialise closed to be the empty list.
	// ...

	// Initialise open to contain a node of start, with a cost of 0
	set_open(start.x, start.y);

	while (o.size_open > 0) { // Whilst there are nodes in the open list
		point_t min_h;
		int min_heuristic = 100000000;

		// Find the node (n) in open with the lowest n.cost + manhattan(n.point, end)
		for (int x = 0; x < MAX_WORLD_SIZE; x++) {
			for (int y = 0; y < MAX_WORLD_SIZE; y++) {
				if (o.nodes[x][y].status == OPEN) {
					int heuristic = o.nodes[x][y].cost + manhattan(x, y, end.x, end.y);
					if (heuristic < min_heuristic) {
						min_h.x = x;
						min_h.y = y;
						min_heuristic = heuristic;
					}
				}
			}
		}

		// Remove n from open and add n to closed
		node_t n = o.nodes[min_h.x][min_h.y];
		set_closed(min_h.x, min_h.y);

		// If n.point is end then the search is complete. n.cost is the length of the path.
		if (min_h.x == end.x && min_h.y == end.y) {
			return n.cost;
		}

		// For the points that are north, south, east and west from n.point (and that are not walls)
		check_neighbor(min_h.x + 1, min_h.y, &n, min_h.x + 1 < world.size);
		check_neighbor(min_h.x - 1, min_h.y, &n, min_h.x - 1 >= 0);
		check_neighbor(min_h.x, min_h.y + 1, &n, min_h.y + 1 < world.size);
		check_neighbor(min_h.x, min_h.y - 1, &n, min_h.y - 1 >= 0);
	}

	return -1;
}

void reset_world() {
	for (int x = 0; x < MAX_WORLD_SIZE; x++) {
		for (int y = 0; y < MAX_WORLD_SIZE; y++) {
			o.nodes[x][y].cost = 0;
			o.nodes[x][y].status = UNVISITED;
		}
	}
}

next_t find_next(search_waypoint_t start) {
	uint12 min_cost = 4095;
	uint8 min_index = -1;
	next_t n;

	for (int i = 0; i < world.waypoints_size; i++) {
		search_waypoint_t end = waypoints[i];

		if (end.exists && !end.used) {
			reset_world();
			uint12 cost = a_star(start.w, end.w);

			printf("%d\r\n", (int) cost);

			if (cost < min_cost) {
				min_cost = cost;
				min_index = i;
			}
		}
	}

	n.cost = min_cost;
	n.index = min_index;

	return n;
}

//Top-level function
void toplevel(hls::stream<uint32> &input, hls::stream<uint32> &output) {
#pragma HLS RESOURCE variable=input core=AXI4Stream
#pragma HLS RESOURCE variable=output core=AXI4Stream
#pragma HLS INTERFACE ap_ctrl_none port=return

	for (int x = 0; x < MAX_WORLD_SIZE; x++) {
		for (int y = 0; y < MAX_WORLD_SIZE; y++) {
			o.nodes[x][y].cost = 0;
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
		wt.direction = (uint1) ((in >> 16) & 0xFF);
		wt.length = (uint8) ((in >> 24) & 0xFF);

		world.walls[i] = wt;
		o.nodes[wt.x][wt.y].type = WALL;

		// create the walls at the same time
		for (int j = 0; j < wt.length; j++) {
			switch (wt.direction) {
			case HORIZONTAL:
					if (wt.x + j < world.size) {
						o.nodes[wt.x + j][wt.y].type = WALL;
					}
				break;
			case VERTICAL:
					if (wt.y + j < world.size) {
						o.nodes[wt.x][wt.y + j].type = WALL;
					}
				break;
			}
		}
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

	int total_cost = 0;
	int order = 0;
	for (int i = 0; i < world.waypoints_size; i++) {
		waypoints[i].w = world.waypoints[i];
		waypoints[i].used = false;
		waypoints[i].exists = true;
		waypoints[i].order = 0;
	}

	search_waypoint_t s = waypoints[0];
	waypoints[0].used = true;
	waypoints[0].order = order++;

	bool all_used = false;
	while (!all_used) {
		next_t n = find_next(s); // find next waypoint to visit
		total_cost += n.cost; // add the cost to get there
		waypoints[n.index].used = true; // and mark it as used so it's not visited again
		waypoints[n.index].order = order++;

		printf("Intermediate cost: %d\r\n", total_cost);

		// prepare for next iteration by setting the start waypoint as the one we just got to
		s = waypoints[n.index];

		// check if there are any unused waypoints
		all_used = true;
		for (int i = 1; i < world.waypoints_size; i++) {
			if (waypoints[i].exists && !waypoints[i].used) {
				all_used = false;
				break;
			}
		}
	}

	int last_waypoint;
	for (int i = 0; i < world.waypoints_size; i++) {
		if (waypoints[i].order == order - 1) {
			last_waypoint = i;
			break;
		}
	}

	// return to start/finish line
	reset_world();
	total_cost += a_star(waypoints[last_waypoint].w, waypoints[0].w);

	output.write(total_cost);

	return;
}

//	for (int x = 0; x < world.size; x++) {
//		for (int y = 0; y < world.size; y++) {
//			printf("%d ", (int) o.nodes[y][x].type);
//		}
//		printf("\r\n");
//	}

