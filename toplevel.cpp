#include "toplevel.h"

#define NOTHING 0
#define WAYPOINT 1
#define WALL 2

world_t world;

// [x][y]
uint2 grid[MAX_WORLD_SIZE][MAX_WORLD_SIZE];

int manhattan(point_t p1, point_t p2) {
	return p2.x - p1.x + p2.y - p1.y;
}

int a_star(point_t start, point_t end) {

	// Initialise closed to be the empty list.
	list_t closed;
	l_init(&closed);

	// Initialise open ...
	list_t open;
	l_init(&open);

	// ... to contain a node of start, with a cost of 0
	node_t start_node = {start, 0};
	l_add(&open, start_node);

	while (!l_is_empty(&open)) { // Whilst there are nodes in the open list
		int min_heuristic_index = -1;
		int min_heuristic = 100000000;

		// Find the node (n) in open with the lowest n.cost + manhattan(n.point, end)
		for (int i = 0; i < MAX_SIZE; i++) {
			if (open.exists[i]) {
				int heuristic = open.items[i].cost + manhattan(open.items[i].point, end);
				if (heuristic < min_heuristic) {
					min_heuristic_index = i;
					min_heuristic = heuristic;
				}
			}
		}

		node_t n = open.items[min_heuristic_index];
		l_remove(&open, min_heuristic_index); // Remove n from open
		l_add(&closed, n); // Add n to closed

		// If n.point is end then the search is complete. n.cost is the length of the path.
		if (n.point.x == end.x && n.point.y == end.y) {
//			printf("Search complete. Cost: %d\r\n", n.cost);
			return n.cost;
		}

		// For the points that are north, south, east and west from n.point (and that are not walls)
		if (n.point.x + 1 < world.size && grid[n.point.x + 1][n.point.y] != WALL) {
			// Create a node m with the neighbour point and cost n.cost + 1
			node_t m = {{n.point.x + 1, n.point.y}, n.cost + 1};

			if (l_in_list(&closed, m) == -1) { // If the neighbour point is not in a node in closed
				int index = l_in_list(&open, m);

				if (index > -1 && m.cost < open.items[index].cost) { // If the point is in open but has a higher cost than m.cost
					open.items[index].cost = m.cost; // replace the cost with m.cost
				} else if (index == -1) { // If the point is not in open, add m to open
					l_add(&open, m);
				}
			}
		}

		if (n.point.x - 1 >= 0 && grid[n.point.x - 1][n.point.y] != WALL) {
			// Create a node m with the neighbour point and cost n.cost + 1
			node_t m = {{n.point.x - 1, n.point.y}, n.cost + 1};

			if (l_in_list(&closed, m) == -1) { // If the neighbour point is not in a node in closed
				int index = l_in_list(&open, m);

				if (index > -1 && m.cost < open.items[index].cost) { // If the point is in open but has a higher cost than m.cost
					open.items[index].cost = m.cost; // replace the cost with m.cost
				} else if (index == -1) { // If the point is not in open, add m to open
					l_add(&open, m);
				}
			}
		}

		if (n.point.y + 1 < world.size && grid[n.point.x][n.point.y + 1] != WALL) {
			// Create a node m with the neighbour point and cost n.cost + 1
			node_t m = {{n.point.x, n.point.y + 1}, n.cost + 1};

			if (l_in_list(&closed, m) == -1) { // If the neighbour point is not in a node in closed
				int index = l_in_list(&open, m);

				if (index > -1 && m.cost < open.items[index].cost) { // If the point is in open but has a higher cost than m.cost
					open.items[index].cost = m.cost; // replace the cost with m.cost
				} else if (index == -1) { // If the point is not in open, add m to open
					l_add(&open, m);
				}
			}
		}


		if (n.point.y - 1 >= 0 && grid[n.point.x][n.point.y - 1] != WALL) {
			// Create a node m with the neighbour point and cost n.cost + 1
			node_t m = {{n.point.x, n.point.y - 1}, n.cost + 1};

			if (l_in_list(&closed, m) == -1) { // If the neighbour point is not in a node in closed
				int index = l_in_list(&open, m);

				if (index > -1 && m.cost < open.items[index].cost) { // If the point is in open but has a higher cost than m.cost
					open.items[index].cost = m.cost; // replace the cost with m.cost
				} else if (index == -1) { // If the point is not in open, add m to open
					l_add(&open, m);
				}
			}
		}
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

	int cost = a_star(start, end);

	output.write(cost);

	return;
}

