/// © 2011. Ravi Teja Gudapati. All rights reserved.

#include "driv/uart0/uart0.h"
#include "driv/cpu/cpu.h"
#include "driv/delay/delay.h"
#include "driv/enet/enet.h"
#include "hw/sysmpu.h"
#include "twin.h"

static uint8_t buffer[1014];

bool_t sendBroadcastMsg() {
	uint32_t count = 0;
	uint32_t length = 86;

#if MY_TWIN == TWIN_B
	memcpy(&buffer[0], &broadcastAddr, 6U);
#elif MY_TWIN == TWIN_A
	memcpy(&buffer[0], &macAddrOth, 6U);
#else
#error ""
#endif

	// Source address: self
	memcpy(&buffer[6], &macAddr, 6U);
	buffer[12] = (length >> 8) & 0xFFU;
	buffer[13] = length & 0xFFU;

	for (count = 0; count < length; count++) {
		buffer[count + 14] = count;
	}

	return enet_sendFrame(buffer, length + 14);
}

int main(void) {
	cpu_init();

	hwSysMpu->CESR &= ~HwSysMpu_CESR_VLD_Mask;

	uart0_init(Uart_Baudrate_230400);
	delay_init();

	// Print start
	uart0_putc('s');

	if (!enet_init()) {
		uart0_putc('x');
	}

	while (1) {
		if(!sendBroadcastMsg()) {
			uart0_putc('f');
		} else {
			uart0_putc('t');
		}

		if(!enet_receiveFrame()) {
			uart0_putc('n');
		} else {
			uart0_putc('y');
		}
		delay_ms_monitor(1000);
	}
}
