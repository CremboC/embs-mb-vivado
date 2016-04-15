#include "toplevel.h"

int main(void) {

	hls::stream<uint32> to_hw, from_hw;

	uint32 should_be = 20;

	to_hw.write(10); // size

	to_hw.write(3); // walls size

	// write walls
	to_hw.write(0x09000101);
	to_hw.write(0x05010203);
	to_hw.write(0x02000604);
//	to_hw.write(0x04010306);

	to_hw.write(2); // waypoints size

	// write waypoints -- y/x
//	to_hw.write(0x0202);
//	to_hw.write(0x0505);
//	to_hw.write(0x0807);
	to_hw.write(0x0000);
	to_hw.write(0x0405);

	toplevel(to_hw, from_hw);

	uint32 cost;
	from_hw.read(cost);

	printf("Got cost: %d\r\n", (int) cost);
	if (cost != should_be) {
		return 1;
	} else {
		return 0;
	}
}
