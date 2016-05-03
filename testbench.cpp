// Exam number: Y0001392

#include "toplevel.h"

int main(void) {

	hls::stream<uint32> to_hw, from_hw;

	uint32 should_be = 226;
	int which = 2;

	switch (which) {
	case 0:
		should_be = 26;
		to_hw.write(10); // size

		to_hw.write(6); // walls size

		// write walls -- length/direction/y/x
		to_hw.write(0x01010800);
		to_hw.write(0x03010305);
		to_hw.write(0x02010409);
		to_hw.write(0x04000203);
		to_hw.write(0x04010504);
		to_hw.write(0x01010803);

		to_hw.write(4); // waypoints size

		to_hw.write(0x0002);
		to_hw.write(0x0207);
		to_hw.write(0x0302);
		to_hw.write(0x0506);
		break;
	case 1:
		should_be = 226;
		to_hw.write(60); // size

		to_hw.write(20); // walls size

		// write walls -- length/direction/y/x
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
		break;
	case 2:
		should_be = 276;
		to_hw.write(60); // size

		to_hw.write(20); // walls size

		// write walls -- length/direction/y/x
		to_hw.write(0x12011C2D);
		to_hw.write(0x0B00040E);
		to_hw.write(0x08011C3A);
		to_hw.write(0x10000325);
		to_hw.write(0x0D012A20);
		to_hw.write(0x0F012038);
		to_hw.write(0x01003B0E);
		to_hw.write(0x0E000239);
		to_hw.write(0x0B002E00);
		to_hw.write(0x10000009);
		to_hw.write(0x0D011437);
		to_hw.write(0x0B010C39);
		to_hw.write(0x0E011128);
		to_hw.write(0x0B000F38);
		to_hw.write(0x0C013126);
		to_hw.write(0x0B011F00);
		to_hw.write(0x0401243A);
		to_hw.write(0x04010805);
		to_hw.write(0x0B013A37);
		to_hw.write(0x01011F0C);

		to_hw.write(12); // waypoints size

		// write waypoints -- y/x
		to_hw.write(0x2A08);
		to_hw.write(0x3029);
		to_hw.write(0x1D2E);
		to_hw.write(0x233B);
		to_hw.write(0x1E13);
		to_hw.write(0x0639);
		to_hw.write(0x280E);
		to_hw.write(0x0535);
		to_hw.write(0x0101);
		to_hw.write(0x1E38);
		to_hw.write(0x3436);
		to_hw.write(0x3736);
		break;
	}

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
