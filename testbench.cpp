#include "toplevel.h"

int main(void) {

	hls::stream<uint32> to_hw, from_hw;

	uint32 should_be = 226;

	to_hw.write(60); // size

	to_hw.write(20); // walls size

	// write walls -- length/direction/y/x
//	to_hw.write(0x09000101);
//	to_hw.write(0x05010203);
//	to_hw.write(0x02000604);
//	to_hw.write(0x04010306);

	            // 0F000638 0901111C 08001C2F 02013930 0500123A 01000B11 01002A11

	to_hw.write(0x08001C00); // 08001C00
	to_hw.write(0x06013523); // 06013523
	to_hw.write(0x13010E27); // 13010E27
	to_hw.write(0x12001635); // 12001635
	to_hw.write(0x13010F04); // 13010F04
	to_hw.write(0x0401302B); // 0401302B

	to_hw.write(0x04012820); // 04012820
	to_hw.write(0x0B00170C); // 0B00170C
	to_hw.write(0x0F012F11); // 0F012F11
	to_hw.write(0x0800352D); // 0800352D
	to_hw.write(0x09003714); // 09003714
	to_hw.write(0x0100333A); // 0100333A
	to_hw.write(0x04012A34); // 04012A34

	to_hw.write(0x0F000638);
	to_hw.write(0x0901111C);
	to_hw.write(0x08001C2F);

	to_hw.write(0x02013930);
	to_hw.write(0x0500123A);
	to_hw.write(0x01000B11);
	to_hw.write(0x01002A11);


	to_hw.write(12); // waypoints size

	// write waypoints -- y/x
	to_hw.write(0x2107);
	to_hw.write(0x3139);
	to_hw.write(0x0C2D);
	to_hw.write(0x2B31);
	to_hw.write(0x0115);
	to_hw.write(0x2803);

	to_hw.write(0x3630);
	to_hw.write(0x2B38);
	to_hw.write(0x2A39);
	to_hw.write(0x332D);

	to_hw.write(0x2D38);
	to_hw.write(0x353B);

	toplevel(to_hw, from_hw);

	uint32 cost;
	from_hw.read(cost);

	for (int i = 0; i < cost + 1; i++) {
		uint32 in;
		from_hw.read(in);

//		printf("%08x ", (int) in);

		uint8 x = (uint8) (in & 0xFF);
		uint8 y = (uint8) ((in >> 8) & 0xFF);

//		printf("(%d, %d) \r\n", (int) x, (int) y);
	}

	printf("Got cost: %d\r\n", (int) cost);
	if (cost != should_be) {
		return 1;
	} else {
		return 0;
	}
}
