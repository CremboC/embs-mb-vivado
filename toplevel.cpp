#include "toplevel.h"

world_t world;
holder_t o;

typedef struct {
	uint4 index;
	waypoint_t w;
	uint12 costs[MAX_WAYPOINTS];
} distance_t;

distance_t distances[MAX_WAYPOINTS];

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

void check_neighbor(uint8 x, uint8 y, uint8 parent_x, uint8 parent_y, node_t *n, bool check) {
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
			o.nodes[x][y].parent.x = parent_x;
			o.nodes[x][y].parent.y = parent_y;
			o.nodes[x][y].parent.exists = true;
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
	o.nodes[start.x][start.y].parent.exists = false;

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
						min_h.exists = true;
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
		check_neighbor(min_h.x + 1, min_h.y, min_h.x, min_h.y, &n, min_h.x + 1 < world.size);
		check_neighbor(min_h.x - 1, min_h.y, min_h.x, min_h.y, &n, min_h.x - 1 >= 0);
		check_neighbor(min_h.x, min_h.y + 1, min_h.x, min_h.y, &n, min_h.y + 1 < world.size);
		check_neighbor(min_h.x, min_h.y - 1, min_h.x, min_h.y, &n, min_h.y - 1 >= 0);
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

uint12 calculate_distance(uint4 *a) {
//	printf("Testing permutation: ");
//	for (unsigned int x = 0; x < world.waypoints_size - 1; x++)
//		printf("%d ", (int) (a[x]));
//	printf("\r\n");

    // start with distance from start point to the first point in the permutation
	uint12 total_cost = distances[0].costs[a[0]];
//	printf("Cost from %d;%d to %d;%d is %d \r\n", (int) distances[0].w.x,
//			(int) distances[0].w.y,
//			(int) distances[a[0]].w.x,
//			(int) distances[a[0]].w.y,
//			(int) distances[0].costs[a[0]]);

	for (uint4 i = 0; i < world.waypoints_size - 2; i++) {
		uint4 from_index = a[i];
		uint4 to_index;

		if (i + 1 > world.waypoints_size) break;

		to_index = a[i + 1];

//		printf("Cost from %d;%d to %d;%d is %d \r\n", (int) distances[from_index].w.x, (int) distances[from_index].w.y, (int) distances[to_index].w.x, (int) distances[to_index].w.y, (int) distances[from_index].costs[to_index]);
		total_cost += distances[from_index].costs[to_index];
	}

	// finally add the cost from the final point in the permutation to the start/finish
	total_cost += distances[a[world.waypoints_size - 2]].costs[0];

//	printf("Cost from %d;%d to %d;%d is %d \r\n", (int) distances[a[world.waypoints_size - 2]].w.x,
//			(int) distances[a[world.waypoints_size - 2]].w.y,
//			(int) distances[0].w.x,
//			(int) distances[0].w.y,
//			(int) distances[a[world.waypoints_size - 2]].costs[0]);
//
//	printf("cost: %d\r\n", (int) total_cost);

	return total_cost;
}

// from http://www.quickperm.org/01example.php
uint12 calculate_min_cost(uint4 *solution)
{
	static uint4 iterations = world.waypoints_size - 1;

	uint4 p[MAX_WAYPOINTS+1] = {};
	uint4 a[MAX_WAYPOINTS] = {};

	for (uint4 i = 0; i < iterations; i++) {
		a[i] = i + 1;
	}

	uint4 tmp, i, j; // Upper Index i; Lower Index j

	for (uint4 i = 0; i < iterations; i++) {  // initialise arrays; a[N] can be any type
		a[i] = i + 1;   // a[i] value is not revealed and can be arbitrary
    	p[i] = i;
	}

    // since the first "permutation" is just the array, say it is the minimum cost
	uint12 min_cost = calculate_distance(a);
	uint4 min_array[MAX_WAYPOINTS];
	for (int k = 0; k < MAX_WAYPOINTS; k++) {
		min_array[k] = a[k];
	}
//	printf("cost: %d\r\n", (int) min_cost);

	p[iterations] = iterations; // p[N] > 0 controls iteration and the index boundary for i

	i = 1;   // setup first swap points to be 1 and 0 respectively (i & j)
	while (i < iterations) {
    	p[i]--;             // decrease index "weight" for i by one
        j = i % 2 * p[i];   // IF i is odd then j = p[i] otherwise j = 0

        tmp = a[j];         // swap(a[j], a[i])
        a[j] = a[i];
        a[i] = tmp;

        // we have a new permutation at this point
        uint12 cost = calculate_distance(a);
        if (cost < min_cost) {
        	min_cost = cost;
        	for (int k = 0; k < MAX_WAYPOINTS; k++) {
        		min_array[k] = a[k];
        	}
        }

        i = 1;              // reset index i to 1 (assumed)
        while (!p[i]) {       // while (p[i] == 0)
        	p[i] = i;        // reset p[i] zero value
        	i++;             // set new index value for i (increase by one)
        } // while(!p[i])
	} // while(i < N)

//	printf("minimummin_cost cost: %d\r\n", (int) min_cost);

//	for (unsigned int x = 0; x < world.waypoints_size - 1; x++)
//		printf("%d ", (int) (min_array[x]));
//	printf("\r\n");

	for (int k = 0; k < MAX_WAYPOINTS; k++) {
		solution[k] = min_array[k];
	}
	return min_cost;
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

	for (uint8 i = 0; i < world.walls_size; i++) {
		wall_t wt;
		uint32 in = input.read();
		wt.x = (uint8) (in & 0xFF);
		wt.y = (uint8) ((in >> 8) & 0xFF);
		wt.direction = (uint1) ((in >> 16) & 0xFF);
		wt.length = (uint8) ((in >> 24) & 0xFF);

		world.walls[i] = wt;
		o.nodes[wt.x][wt.y].type = WALL;

		// create the walls at the same time
		for (uint4 j = 0; j < wt.length; j++) {
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

	for (uint4 i = 0; i < world.waypoints_size; i++) {
		waypoint_t wt;
		uint32 in = input.read();
		wt.x = (uint8) (in & 0xFF);
		wt.y = (uint8) ((in >> 8) & 0xFF);
		wt.exists = true;

		world.waypoints[i] = wt;
		o.nodes[wt.x][wt.y].type = WAYPOINT;
	}

	// first have your system work out the minimum distance between every pair of waypoints
	for (uint4 i = 0; i < world.waypoints_size; i++) {
		distance_t d;
		d.index = i;
		d.w = world.waypoints[i];

		for (int j = 0; j < world.waypoints_size; j++) {
			if (world.waypoints[j].exists) {
				reset_world();
				d.costs[j] = a_star(d.w, world.waypoints[j]);
			} else {
				d.costs[j] = 999;
			}
		}

		distances[i] = d;
	}

	uint4 path[MAX_WAYPOINTS];
	uint12 total_cost = calculate_min_cost(path);
	waypoint_t waypoint_path[MAX_WAYPOINTS + 1];

	waypoint_path[0] = world.waypoints[0];
	waypoint_path[world.waypoints_size] = world.waypoints[0];

	int i = 1;
	for (int k = 0; k < world.waypoints_size - 1; k++) {
		waypoint_path[i++] = world.waypoints[path[k]];
	}

	int current = 0;
	point_t absolute_path[1000];
	for (int i = 0; i <= world.waypoints_size; i++) {
		if (i + 1 > world.waypoints_size) break;

		waypoint_t from = waypoint_path[i];
		waypoint_t to = waypoint_path[i + 1];

		a_star(from, to);

		point_t parent = o.nodes[to.x][to.y].parent;
		absolute_path[current++].x = parent.x;
		absolute_path[current++].y = parent.y;

		bool found = false;
		while (!found) {
			if (parent.x != from.x && parent.y == from.y) {
				parent = o.nodes[parent.x][parent.y].parent;
				absolute_path[current++].x = parent.x;
				absolute_path[current++].y = parent.y;
			} else {
				found = true;
			}
		}

		// do A*
		// traverse from end to start by checking "parent"
		// reset world
		reset_world();
	}

	for (int i = 0; i <= total_cost; i++) {
		printf("(%d, %d)\r\n", (int) absolute_path[i].x, (int) absolute_path[i].y);
	}

	output.write(total_cost);

	return;
}

