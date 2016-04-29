#include "toplevel.h"

world_t world;
holder_t o;

// Cost matrix from the "current" waypoint to the indexed one.
uint14 distances[MAX_WAYPOINTS][MAX_WAYPOINTS];

/**
 * Calculate manhattan distance between two points. Makes sure the return value is positive.
 */
int manhattan(uint8 x1, uint8 y1, uint8 x2, uint8 y2) {
	int x = x1 > x2 ? x1 - x2 : x2 - x1;
	int y = y1 > y2 ? y1 - y2 : y2 - y1;
	return x + y;
}

/**
 * Helper function to close a node.
 * Also reduces the number of opened nodes so that our while loop works.
 */
void set_closed(uint8 x, uint8 y) {
	o.nodes[x][y].status = CLOSED;
	o.size_open--;
}

/**
 * Helper function to open a node.
 * Also increase the number of opened nodes so that the while loop works properly.
 */
void set_open(uint8 x, uint8 y) {
	o.nodes[x][y].status = OPEN;
	o.size_open++;
}

/**
 * Checks a node to potentially open it or decrease its cost if a shorter path has been found.
 * The x and y are the coordinates are the coordinates of the node to check.
 * The n is the parent node - it is required to get the previous cost
 * The check is used to make sure the node is not out of bounds for the world size.
 * The direction is the relative direction of the parent node. This is used to get the full path later on.
 */
void check_neighbor(uint8 x, uint8 y, node_t &n, bool check, uint2 direction) {
#pragma HLS PIPELINE
	if (check && o.nodes[x][y].type != WALL) {
		uint14 new_cost = n.cost + 1;

		switch (o.nodes[x][y].status) {
		case OPEN: // if open then potentially change the cost with the new cost as we found a shorter route
			if (o.nodes[x][y].cost > new_cost) {
				o.nodes[x][y].parent_direction = direction; // update the parent
				o.nodes[x][y].cost = new_cost; // replace the cost with m.cost
			}
			break;
		case UNVISITED: // if unvisited, mark it as open and set the cost to it
			set_open(x, y);
			o.nodes[x][y].parent_direction = direction;
			o.nodes[x][y].cost = new_cost;
			break;
		}
	}
}

/**
 * Do the whole a star between the passed start and end waypoints.
 * Returns the cost.
 */
int a_star(waypoint_t start, waypoint_t end) {
	// Initialise closed to be the empty list.
	// -- done implicitly because there are no closed nodes initially.

	// Initialise open to contain a node of start, with a cost of 0
	set_open(start.x, start.y);

	a_star_loop: while (o.size_open > 0) { // Whilst there are nodes in the open list
#pragma HLS LOOP_TRIPCOUNT min=1 max=3600
		point_t min_h;
		int min_heuristic = 100000000;

		// Find the node (n) in open with the lowest n.cost + manhattan(n.point, end)
		find_manhattan_x: for (int x = 0; x < world.size; x++) {
#pragma HLS LOOP_TRIPCOUNT min=10 max=60
			find_manhattan_y: for (int y = 0; y < world.size; y++) {
#pragma HLS LOOP_TRIPCOUNT min=10 max=60
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
		check_neighbor(min_h.x + 1, min_h.y, n, min_h.x + 1 < world.size, WEST);
		check_neighbor(min_h.x - 1, min_h.y, n, min_h.x - 1 >= 0, EAST);
		check_neighbor(min_h.x, min_h.y + 1, n, min_h.y + 1 < world.size, NORTH);
		check_neighbor(min_h.x, min_h.y - 1, n, min_h.y - 1 >= 0, SOUTH);
	}

	return -1;
}

/**
 * Helper to reset the world in between the A* runs.
 */
void reset_world() {
	reset_world_x: for (int x = 0; x < world.size; x++) {
#pragma HLS LOOP_TRIPCOUNT min=10 max=60
		reset_world_y: for (int y = 0; y < world.size; y++) {
#pragma HLS LOOP_TRIPCOUNT min=10 max=60
#pragma HLS LOOP_FLATTEN
			o.nodes[x][y].cost = 0;
			o.nodes[x][y].status = UNVISITED;
		}
	}
}

/**
 * Sum the distances between the waypoints in the given path.
 * This represents the length of the path.
 */
uint14 sum_distance(uint8 *path) {
    // start with distance from start point to the first point in the permutation
	uint14 total_cost = distances[0][path[0]];

	// loop through all the waypoints in the path
	// loop until waypoints_size - 2 because we also increment the index by one to get the
	// "to" point
	sum_waypoints: for (uint8 i = 0; i < world.waypoints_size - 2; i++) {
		uint8 from_index = path[i];
		uint8 to_index = path[i + 1];

		total_cost += distances[from_index][to_index];
	}

	// finally add the cost from the final point in the permutation to the start/finish
	total_cost += distances[path[world.waypoints_size - 2]][0];

	return total_cost;
}

// Uses the QuickPerm algorithm described in http://www.quickperm.org/01example.php with slight modifications.
// The solution parameter will be an array of indices that references waypoints in the world waypoints matrix.
// Returns the minimum cost.
uint14 find_min_cost(uint8 *solution) {
	// The N in the QuickPerm algorithm - specifies how many elements we are going to permute.
	// In this case, it's amount of waypoints minus one because the start/finish point is always the first one.
	int iterations = world.waypoints_size - 1;

	// Even though the following two arrays should be N + 1 and N (hence iterations)
	// Vivado doesn't support dynamic arrays hence we use the worst case instead.
	uint8 p[MAX_WAYPOINTS] = {};
	uint8 permutation[MAX_WAYPOINTS - 1] = {};

	int tmp, i, j; // Upper Index i; Lower Index j

	init_permutations: for (int i = 0; i < iterations; i++) {  // initialise arrays; a[N] can be any type
		permutation[i] = i + 1;   // a[i] value is not revealed and can be arbitrary
    	p[i] = i;
	}

    // Since the first "permutation" is just the array, say it is the minimum cost
	// and the best path.
	uint14 min_cost = sum_distance(permutation);
	uint8 min_array[MAX_WAYPOINTS - 1];

	copy_initial_solution: for (int k = 0; k < MAX_WAYPOINTS - 1; k++) {
		min_array[k] = permutation[k];
	}

	p[iterations] = iterations; // p[N] > 0 controls iteration and the index boundary for i

	i = 1;   // setup first swap points to be 1 and 0 respectively (i & j)
	make_permutations: while (i < iterations) {
    	p[i]--;             // decrease index "weight" for i by one

    	// can use logical and instead of modulus
    	j = !(i & 1) ? 0 : p[i]; // if i is odd then j = p[i] otherwise j = 0

        tmp = permutation[j];         // swap(a[j], a[i])
        permutation[j] = permutation[i];
        permutation[i] = tmp;

        // We have a new permutation at this point so we are going to test it
        // immediately by summing the distances between the waypoints in it.
        if (permutation[0] < permutation[iterations - 1]) {
            uint14 cost = sum_distance(permutation);
            if (cost < min_cost) { // Update the min cost and min path if it is better than before
            	min_cost = cost;
            	for (int k = 0; k < MAX_WAYPOINTS - 1; k++) {
            		min_array[k] = permutation[k];
            	}
            }
        }

        i = 1;              // reset index i to 1 (assumed)
        increase_odometer: while (!p[i]) {     // while (p[i] == 0)
        	p[i] = i;       // reset p[i] zero value
        	i++;            // set new index value for i (increase by one)
        }
	}

	// Finally update the solution array to our best answer.
	copy_solution: for (int k = 0; k < MAX_WAYPOINTS - 1; k++) {
		solution[k] = min_array[k];
	}
	return min_cost;
}

//Top-level function
void toplevel(hls::stream<uint32> &input, hls::stream<uint32> &output) {
#pragma HLS ARRAY_PARTITION variable=distances complete dim=1
#pragma HLS RESOURCE variable=input core=AXI4Stream
#pragma HLS RESOURCE variable=output core=AXI4Stream
#pragma HLS INTERFACE ap_ctrl_none port=return

	// Initialise the world.
	init_world_x: for (int x = 0; x < MAX_WORLD_SIZE; x++) {
		init_world_y: for (int y = 0; y < MAX_WORLD_SIZE; y++) {
#pragma HLS LOOP_FLATTEN
			o.nodes[x][y].cost = 0;
			o.nodes[x][y].status = UNVISITED;
			o.nodes[x][y].type = NOTHING;
		}
	}

	world.size = input.read(); // Read total world size.
	world.walls_size = input.read(); // Read number of walls in world.

	make_walls: for (int i = 0; i < world.walls_size; i++) {
#pragma HLS LOOP_TRIPCOUNT min=6 max=20
		wall_t wt;
		uint32 in = input.read(); // Read every wall as a single number for efficiency
		wt.x = (uint8) (in & 0xFF);
		wt.y = (uint8) ((in >> 8) & 0xFF);
		wt.direction = (uint1) ((in >> 16) & 0xFF);
		wt.length = (uint8) ((in >> 24) & 0xFF);

		world.walls[i] = wt;
		o.nodes[wt.x][wt.y].type = WALL;

		// Update the type of the nodes according to the wall.
		build_wall: for (int j = 0; j < wt.length; j++) {
#pragma HLS LOOP_TRIPCOUNT min=1 max=60
#pragma HLS PIPELINE
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

	world.waypoints_size = input.read(); // Read number of waypoints.

	make_waypoints: for (int i = 0; i < world.waypoints_size; i++) {
#pragma HLS LOOP_TRIPCOUNT min=4 max=12
		waypoint_t wt;
		uint32 in = input.read(); // Read waypoint as a single number for efficiency.
		wt.x = (uint8) (in & 0xFF);
		wt.y = (uint8) ((in >> 8) & 0xFF);

		world.waypoints[i] = wt;
		o.nodes[wt.x][wt.y].type = WAYPOINT;
	}

	// first have your system work out the minimum distance between every pair of waypoints
	distance_matrix_i: for (int i = 0; i < world.waypoints_size; i++) {
#pragma HLS LOOP_TRIPCOUNT min=4 max=12
		distance_matrix_j: for (int j = 0; j <= i; j++) {
#pragma HLS LOOP_TRIPCOUNT min=4 max=12
			reset_world();
			distances[i][j] = a_star(world.waypoints[i], world.waypoints[j]);
			distances[j][i] = distances[i][j]; // copy over to simplify coding later
		}
	}

	uint8 path[MAX_WAYPOINTS]; // This arrays stores the solution provided by checking all permutations
	uint8 full_path[MAX_WAYPOINTS + 1] = {0}; // Same as path, but starts with start/end point
	uint14 total_cost = find_min_cost(path); // Run QuickPerm to find most efficient permutation as well as its cost

	output.write(total_cost); // Output the length of the path

	// Copy over the waypoints from the calculated solution
	copy_path: for (int i = 1; i < world.waypoints_size; i++) {
#pragma HLS LOOP_TRIPCOUNT min=4 max=12
		full_path[i] = path[i - 1];
	}
	full_path[world.waypoints_size] = 0; // And append with start/end point

	// We now calculate the whole actual path to be able to draw it on the screen
	// The simplest way of doing it is doing A* again between waypoint in the solution
	// and tracking the parent.
	uint14 distance; // Temporary variable to store the calculate A* distance
	uint32 out; // Temporary store the value to output to the MicroBlaze software
	point_t parent;
	get_path: for (int i = 0; i <= world.waypoints_size - 1; i++) {
		waypoint_t from = world.waypoints[full_path[i]];
		waypoint_t to = world.waypoints[full_path[i + 1]];

		// Run A* so the parent nodes are set correctly.
		distance = a_star(from, to);

		// Depending on the parent direction, calculate its coordinates.
		switch (o.nodes[to.x][to.y].parent_direction) {
		case NORTH: // parent is north
			parent.x = to.x;
			parent.y = to.y - 1;
			break;
		case EAST: // parent is east
			parent.x = to.x + 1;
			parent.y = to.y;
			break;
		case SOUTH: // parent is south
			parent.x = to.x;
			parent.y = to.y + 1;
			break;
		case WEST: // parent is west
			parent.x = to.x - 1;
			parent.y = to.y;
			break;
		}

		out = parent.y;
		out = (out << 8) | parent.x;
		output.write(out); // Output the first path parent.

		// Now loop over all other coordinates in path.
		bool found = false;
		get_parents: for (;;) {
			// We have reached the goal, hence we can stop.
			if (parent.x == from.x && parent.y == from.y) break;

			// Depending on the parent direction, calculate its coordinates.
			switch (o.nodes[parent.x][parent.y].parent_direction) {
			case NORTH: // parent is north
				parent.y -= 1;
				break;
			case EAST: // parent is east
				parent.x += 1;
				break;
			case SOUTH: // parent is south
				parent.y += 1;
				break;
			case WEST: // parent is west
				parent.x -= 1;
				break;
			}

			out = parent.y;
			out = (out << 8) | parent.x;
			output.write(out); // Output the coordinates
		}

		// Prepare for next iteration by reseting the world so that A* is correct.
		reset_world();
	}

	out = world.waypoints[0].y;
	out = (out << 8) | world.waypoints[0].x;
	output.write(out); // Finally output the start/finish waypoint coordinates.

	return;
}

