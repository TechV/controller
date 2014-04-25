/*
 * servod.c Multiple Servo Driver for the RaspberryPi
 * Copyright (c) 2013 Richard Hirst <richardghirst@gmail.com>
 *
 * This program provides very similar functionality to servoblaster, except
 * that rather than implementing it as a kernel module, servod implements
 * the functionality as a usr space daemon.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/* TODO: Separate idle timeout handling from genuine set-to-zero requests */
/* TODO: Add ability to specify time frame over which an adjustment should be made */
/* TODO: Add servoctl utility to set and query servo positions, etc */
/* TODO: Add slow-start option */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <getopt.h>
#include <math.h>
#include "interface.h"

#define DMY	255	// Used to represent an invalid P1 pin, or unmapped servo

#define NUM_P1PINS	26
#define NUM_P5PINS	8

#define MAX_SERVOS	32	/* Only 21 really, but this lets you map servo IDs
				 * to P1 pins, if you want to
				 */
#define MAX_MEMORY_USAGE	(16*1024*1024)	/* Somewhat arbitrary limit of 16MB */

#define DEFAULT_CYCLE_TIME_US	20000
#define DEFAULT_STEP_TIME_US	10
#define DEFAULT_SERVO_MIN_US	500
#define DEFAULT_SERVO_MAX_US	2500

#define PAGE_SIZE		4096
#define PAGE_SHIFT		12

#define DMA_CHAN_SIZE		0x100
#define DMA_CHAN_MIN		0
#define DMA_CHAN_MAX		14
#define DMA_CHAN_DEFAULT	14

#define DMA_BASE		0x20007000
#define DMA_LEN			DMA_CHAN_SIZE * (DMA_CHAN_MAX+1)
#define PWM_BASE		0x2020C000
#define PWM_LEN			0x28
#define CLK_BASE	        0x20101000
#define CLK_LEN			0xA8
#define GPIO_BASE		0x20200000
#define GPIO_LEN		0x100
#define PCM_BASE		0x20203000
#define PCM_LEN			0x24

#define DMA_NO_WIDE_BURSTS	(1<<26)
#define DMA_WAIT_RESP		(1<<3)
#define DMA_D_DREQ		(1<<6)
#define DMA_PER_MAP(x)		((x)<<16)
#define DMA_END			(1<<1)
#define DMA_RESET		(1<<31)
#define DMA_INT			(1<<2)

#define DMA_CS			(0x00/4)
#define DMA_CONBLK_AD		(0x04/4)
#define DMA_DEBUG		(0x20/4)

#define GPIO_FSEL0		(0x00/4)
#define GPIO_SET0		(0x1c/4)
#define GPIO_CLR0		(0x28/4)
#define GPIO_LEV0		(0x34/4)
#define GPIO_PULLEN		(0x94/4)
#define GPIO_PULLCLK		(0x98/4)

#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1

#define PWM_CTL			(0x00/4)
#define PWM_DMAC		(0x08/4)
#define PWM_RNG1		(0x10/4)
#define PWM_FIFO		(0x18/4)

#define PWMCLK_CNTL		40
#define PWMCLK_DIV		41

#define PWMCTL_MODE1		(1<<1)
#define PWMCTL_PWEN1		(1<<0)
#define PWMCTL_CLRF		(1<<6)
#define PWMCTL_USEF1		(1<<5)

#define PWMDMAC_ENAB		(1<<31)
#define PWMDMAC_THRSHLD		((15<<8)|(15<<0))

#define PCM_CS_A		(0x00/4)
#define PCM_FIFO_A		(0x04/4)
#define PCM_MODE_A		(0x08/4)
#define PCM_RXC_A		(0x0c/4)
#define PCM_TXC_A		(0x10/4)
#define PCM_DREQ_A		(0x14/4)
#define PCM_INTEN_A		(0x18/4)
#define PCM_INT_STC_A		(0x1c/4)
#define PCM_GRAY		(0x20/4)

#define PCMCLK_CNTL		38
#define PCMCLK_DIV		39

#define DELAY_VIA_PWM		0
#define DELAY_VIA_PCM		1

#define ROUNDUP(val, blksz)	(((val)+((blksz)-1)) & ~(blksz-1))

typedef struct {
	uint32_t info, src, dst, length,
		 stride, next, pad[2];
} dma_cb_t;

typedef struct {
	uint32_t physaddr;
} page_map_t;

/* Define which P1 header pins to use by default.  These are the eight standard
 * GPIO pins (those coloured green in the diagram on this page:
 *    http://elinux.org/Rpi_Low-level_peripherals
 *
 * Which P1 header pins are actually used can be overridden via command line
 * parameter '--p1pins=...'.
 */

static const int board_rev = 2;
static char *default_p1_pins = "7,11,12,13,15,16,18,22";
static char *default_p5_pins = "";

static uint8_t rev1_p1pin2gpio_map[] = {
	DMY,	// P1-1   3v3
	DMY,	// P1-2   5v
	0,	// P1-3   GPIO 0 (SDA)
	DMY,	// P1-4   5v
	1,	// P1-5   GPIO 1 (SCL)
	DMY,	// P1-6   Ground
	4,	// P1-7   GPIO 4 (GPCLK0)
	14,	// P1-8   GPIO 14 (TXD)
	DMY,	// P1-9   Ground
	15,	// P1-10  GPIO 15 (RXD)
	17,	// P1-11  GPIO 17
	18,	// P1-12  GPIO 18 (PCM_CLK)
	21,	// P1-13  GPIO 21
	DMY,	// P1-14  Ground
	22,	// P1-15  GPIO 22
	23,	// P1-16  GPIO 23
	DMY,	// P1-17  3v3
	24,	// P1-18  GPIO 24
	10,	// P1-19  GPIO 10 (MOSI)
	DMY,	// P1-20  Ground
	9,	// P1-21  GPIO 9 (MISO)
	25,	// P1-22  GPIO 25
	11,	// P1-23  GPIO 11 (SCLK)
	8,	// P1-24  GPIO 8 (CE0)
	DMY,	// P1-25  Ground
	7,	// P1-26  GPIO 7 (CE1)
};

static uint8_t rev1_p5pin2gpio_map[] = {
	DMY,	// (P5-1 on rev 2 boards)
	DMY,	// (P5-2 on rev 2 boards)
	DMY,	// (P5-3 on rev 2 boards)
	DMY,	// (P5-4 on rev 2 boards)
	DMY,	// (P5-5 on rev 2 boards)
	DMY,	// (P5-6 on rev 2 boards)
	DMY,	// (P5-7 on rev 2 boards)
	DMY,	// (P5-8 on rev 2 boards)
};

static uint8_t rev2_p1pin2gpio_map[] = {
	DMY,	// P1-1   3v3
	DMY,	// P1-2   5v
	2,	// P1-3   GPIO 2 (SDA)
	DMY,	// P1-4   5v
	3,	// P1-5   GPIO 3 (SCL)
	DMY,	// P1-6   Ground
	4,	// P1-7   GPIO 4 (GPCLK0)
	14,	// P1-8   GPIO 14 (TXD)
	DMY,	// P1-9   Ground
	15,	// P1-10  GPIO 15 (RXD)
	17,	// P1-11  GPIO 17
	18,	// P1-12  GPIO 18 (PCM_CLK)
	27,	// P1-13  GPIO 27
	DMY,	// P1-14  Ground
	22,	// P1-15  GPIO 22
	23,	// P1-16  GPIO 23
	DMY,	// P1-17  3v3
	24,	// P1-18  GPIO 24
	10,	// P1-19  GPIO 10 (MOSI)
	DMY,	// P1-20  Ground
	9,	// P1-21  GPIO 9 (MISO)
	25,	// P1-22  GPIO 25
	11,	// P1-23  GPIO 11 (SCLK)
	8,	// P1-24  GPIO 8 (CE0)
	DMY,	// P1-25  Ground
	7,	// P1-26  GPIO 7 (CE1)
};

static uint8_t rev2_p5pin2gpio_map[] = {
	DMY,	// P5-1   5v0
	DMY,	// P5-2   3v3
	28,	// P5-3   GPIO 28 (I2C0_SDA)
	29,	// P5-4   GPIO 29 (I2C0_SCL)
	30,	// P5-5   GPIO 30
	31,	// P5-6   GPIO 31
	DMY,	// P5-7   Ground
	DMY,	// P5-8   Ground
};

// cycle_time_us is the pulse cycle time per servo, in microseconds.
// Typically it should be 20ms, or 20000us.

// step_time_us is the pulse width increment granularity, again in microseconds.
// Setting step_time_us too low will likely cause problems as the DMA controller
// will use too much memory bandwidth.  10us is a good value, though you
// might be ok setting it as low as 2us.

static int cycle_time_us;
static int step_time_us;

static uint8_t servo2gpio[MAX_SERVOS];
static uint8_t p1pin2servo[NUM_P1PINS+1];
static uint8_t p5pin2servo[NUM_P5PINS+1];
static int servostart[MAX_SERVOS];
static int servowidth[MAX_SERVOS];
static int num_servos;
static uint32_t gpiomode[MAX_SERVOS];
static int restore_gpio_modes;

page_map_t *page_map;

static uint8_t *virtbase;
static uint8_t *virtcached;

static volatile uint32_t *pwm_reg;
static volatile uint32_t *pcm_reg;
static volatile uint32_t *clk_reg;
static volatile uint32_t *dma_reg;
static volatile uint32_t *gpio_reg;

static int delay_hw = DELAY_VIA_PWM;

static struct timeval *servo_kill_time;

static int dma_chan;
static int invert = 0;
static int servo_min_ticks;
static int servo_max_ticks;
static int num_samples;
static int num_cbs;
static int num_pages;
static uint32_t *turnoff_mask;
static uint32_t *turnon_mask;
static dma_cb_t *cb_base;

static void set_servo(int servo, int width);
static void set_servo_idle(int servo);
static void gpio_set_mode(uint32_t gpio, uint32_t mode);
static char *gpio2pinname(uint8_t gpio);

static void
udelay(int us)
{
	struct timespec ts = { 0, us * 1000 };

	nanosleep(&ts, NULL);
}

static void cleanup() {
		int i;

	if (dma_reg && virtbase) {
		for (i = 0; i < MAX_SERVOS; i++) {
			if (servo2gpio[i] != DMY)
				set_servo(i, 0);
		}
		udelay(cycle_time_us);
		dma_reg[DMA_CS] = DMA_RESET;
		udelay(10);
	}
	if (restore_gpio_modes) {
		for (i = 0; i < MAX_SERVOS; i++) {
			if (servo2gpio[i] != DMY)
				gpio_set_mode(servo2gpio[i], gpiomode[i]);
		}
	}
}

static void
terminate(int dummy)
{
	cleanup();
	exit(1);
}

static void
fatal(char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	cleanup();
}


static uint32_t gpio_get_mode(uint32_t gpio)
{
	uint32_t fsel = gpio_reg[GPIO_FSEL0 + gpio/10];

	return (fsel >> ((gpio % 10) * 3)) & 7;
}

static void
gpio_set_mode(uint32_t gpio, uint32_t mode)
{
	uint32_t fsel = gpio_reg[GPIO_FSEL0 + gpio/10];

	fsel &= ~(7 << ((gpio % 10) * 3));
	fsel |= mode << ((gpio % 10) * 3);
	gpio_reg[GPIO_FSEL0 + gpio/10] = fsel;
}

static void
gpio_set(int gpio, int level)
{
	if (level)
		gpio_reg[GPIO_SET0] = 1 << gpio;
	else
		gpio_reg[GPIO_CLR0] = 1 << gpio;
}

static uint32_t
mem_virt_to_phys(void *virt)
{
	uint32_t offset = (uint8_t *)virt - virtbase;

	return page_map[offset >> PAGE_SHIFT].physaddr + (offset % PAGE_SIZE);
}

static void *
map_peripheral(uint32_t base, uint32_t len)
{
	int fd = open("/dev/mem", O_RDWR | O_SYNC);
	void * vaddr = (void*)-1;

	if (fd < 0) {
		fatal("servod: Failed to open /dev/mem: %m\n");
	}
	vaddr = mmap(NULL, len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, base);
	if (vaddr == MAP_FAILED) {
		fatal("servod: Failed to map peripheral at 0x%08x: %m\n", base);
	}
	close(fd);

	return vaddr;
}

static void
set_servo_idle(int servo)
{
	/* Just remove the 'turn-on' action and allow the 'turn-off' action at
	 * the end of the current pulse to turn it off.  Special case if
	 * current width is 100%; in that case there will be no 'turn-off'
	 * action, so we will need to force the output off here.  We must not
	 * force the output in other cases, because that might lead to
	 * truncated pulses which would make a servo change position.
	 */
	turnon_mask[servo] = 0;
	if (servowidth[servo] == num_samples)
		gpio_set(servo2gpio[servo], invert ? 1 : 0);
}

/* Carefully add or remove bits from the turnoff_mask such that regardless
 * of where the DMA controller is in its cycle, and whether we are increasing
 * or decreasing the pulse width, the generated pulse will only ever be the
 * old width or the new width.  If we don't take such care then there could be
 * a cycle with some pulse width between the two requested ones.  That doesn't
 * really matter for servos, but when driving LEDs some odd intensity for one
 * cycle can be noticeable.  It may be that the servo output has been turned
 * off via the inactivity timer, which is handled by always setting the turnon
 * mask appropriately at the end of this function.
 */
static void
set_servo(int servo, int width) /* width in number of steps */
{
	volatile uint32_t *dp;
	int i;
	uint32_t mask = 1 << servo2gpio[servo];


	if (width > servowidth[servo]) {
		dp = turnoff_mask + servostart[servo] + width;
		if (dp >= turnoff_mask + num_samples)
			dp -= num_samples;

		for (i = width; i > servowidth[servo]; i--) {
			dp--;
			if (dp < turnoff_mask)
				dp = turnoff_mask + num_samples - 1;
			//printf("%5d, clearing at %p\n", dp - ctl->turnoff, dp);
			*dp &= ~mask;
		}
	} else if (width < servowidth[servo]) {
		dp = turnoff_mask + servostart[servo] + width;
		if (dp >= turnoff_mask + num_samples)
			dp -= num_samples;

		for (i = width; i < servowidth[servo]; i++) {
			//printf("%5d, setting at %p\n", dp - ctl->turnoff, dp);
			*dp++ |= mask;
			if (dp >= turnoff_mask + num_samples)
				dp = turnoff_mask;
		}
	}
	servowidth[servo] = width;
	if (width == 0) {
		turnon_mask[servo] = 0;
	} else {
		turnon_mask[servo] = mask;
	}
}

static int
make_pagemap(void)
{
	int i, fd, memfd, pid;
	char pagemap_fn[64];

	page_map = (page_map_t *)malloc(num_pages * sizeof(*page_map));
	if (page_map == 0) {
		fatal("servod: Failed to malloc page_map: %m\n");
		return -1;
	}
	memfd = open("/dev/mem", O_RDWR | O_SYNC);
	if (memfd < 0) {
		fatal("servod: Failed to open /dev/mem: %m\n");
		return -1;
	}
	pid = getpid();
	sprintf(pagemap_fn, "/proc/%d/pagemap", pid);
	fd = open(pagemap_fn, O_RDONLY);
	if (fd < 0) {
		fatal("servod: Failed to open %s: %m\n", pagemap_fn);
		return -1;
	}
	if (lseek(fd, (uint32_t)(size_t)virtcached >> 9, SEEK_SET) !=
						(uint32_t)(size_t)virtcached >> 9) {
		fatal("servod: Failed to seek on %s: %m\n", pagemap_fn);
		return -1;
	}
	for (i = 0; i < num_pages; i++) {
		uint64_t pfn;
		if (read(fd, &pfn, sizeof(pfn)) != sizeof(pfn)) {
			fatal("servod: Failed to read %s: %m\n", pagemap_fn);
			return -1;
		}
		if (((pfn >> 55) & 0x1bf) != 0x10c) {
			fatal("servod: Page %d not present (pfn 0x%016llx)\n", i, pfn);
			return -1;
		}
		page_map[i].physaddr = (uint32_t)pfn << PAGE_SHIFT | 0x40000000;
		if (mmap(virtbase + i * PAGE_SIZE, PAGE_SIZE, PROT_READ|PROT_WRITE,
			MAP_SHARED|MAP_FIXED|MAP_LOCKED|MAP_NORESERVE,
			memfd, (uint32_t)pfn << PAGE_SHIFT | 0x40000000) !=
				virtbase + i * PAGE_SIZE) {
			fatal("Failed to create uncached map of page %d at %p\n",
				i,  virtbase + i * PAGE_SIZE);
				return -1;
		}
	}
	close(fd);
	close(memfd);
	memset(virtbase, 0, num_pages * PAGE_SIZE);
	return 0;
}

static void
setup_sighandlers(void)
{
	int i;

	// Catch all signals possible - it is vital we kill the DMA engine
	// on process exit!
	for (i = 0; i < 64; i++) {
		struct sigaction sa;

		memset(&sa, 0, sizeof(sa));
		sa.sa_handler = terminate;
		sigaction(i, &sa, NULL);
	}
}

static void
init_ctrl_data(void)
{
	dma_cb_t *cbp = cb_base;
	uint32_t phys_fifo_addr, cbinfo;
	uint32_t phys_gpclr0;
	uint32_t phys_gpset0;
	int servo, i, numservos = 0, curstart = 0;
	uint32_t maskall = 0;

	if (invert) {
		phys_gpclr0 = 0x7e200000 + 0x1c;
		phys_gpset0 = 0x7e200000 + 0x28;
	} else {
		phys_gpclr0 = 0x7e200000 + 0x28;
		phys_gpset0 = 0x7e200000 + 0x1c;
	}

	if (delay_hw == DELAY_VIA_PWM) {
		phys_fifo_addr = (PWM_BASE | 0x7e000000) + 0x18;
		cbinfo = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ | DMA_PER_MAP(5);
	} else {
		phys_fifo_addr = (PCM_BASE | 0x7e000000) + 0x04;
		cbinfo = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ | DMA_PER_MAP(2);
	}

	memset(turnon_mask, 0, MAX_SERVOS * sizeof(*turnon_mask));

	for (servo = 0 ; servo < MAX_SERVOS; servo++) {
		servowidth[servo] = 0;
		if (servo2gpio[servo] != DMY) {
			numservos++;
			maskall |= 1 << servo2gpio[servo];
		}
	}

	for (i = 0; i < num_samples; i++)
		turnoff_mask[i] = maskall;

	for (servo = 0; servo < MAX_SERVOS; servo++) {
		if (servo2gpio[servo] != DMY) {
			servostart[servo] = curstart;
			curstart += num_samples / num_servos;
		}
	}

	servo = 0;
	while (servo < MAX_SERVOS && servo2gpio[servo] == DMY)
		servo++;

	for (i = 0; i < num_samples; i++) {
		cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
		cbp->src = mem_virt_to_phys(turnoff_mask + i);
		cbp->dst = phys_gpclr0;
		cbp->length = 4;
		cbp->stride = 0;
		cbp->next = mem_virt_to_phys(cbp + 1);
		cbp++;
		if (i == servostart[servo]) {
			cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
			cbp->src = mem_virt_to_phys(turnon_mask + servo);
			cbp->dst = phys_gpset0;
			cbp->length = 4;
			cbp->stride = 0;
			cbp->next = mem_virt_to_phys(cbp + 1);
			cbp++;
			servo++;
			while (servo < MAX_SERVOS && servo2gpio[servo] == DMY)
				servo++;
		}
		// Delay
		cbp->info = cbinfo;
		cbp->src = mem_virt_to_phys(turnoff_mask);	// Any data will do
		cbp->dst = phys_fifo_addr;
		cbp->length = 4;
		cbp->stride = 0;
		cbp->next = mem_virt_to_phys(cbp + 1);
		cbp++;
	}
	cbp--;
	cbp->next = mem_virt_to_phys(cb_base);
}

static void
init_hardware(void)
{
	if (delay_hw == DELAY_VIA_PWM) {
		// Initialise PWM
		pwm_reg[PWM_CTL] = 0;
		udelay(10);
		clk_reg[PWMCLK_CNTL] = 0x5A000006;		// Source=PLLD (500MHz)
		udelay(100);
		clk_reg[PWMCLK_DIV] = 0x5A000000 | (500<<12);	// set pwm div to 500, giving 1MHz
		udelay(100);
		clk_reg[PWMCLK_CNTL] = 0x5A000016;		// Source=PLLD and enable
		udelay(100);
		pwm_reg[PWM_RNG1] = step_time_us;
		udelay(10);
		pwm_reg[PWM_DMAC] = PWMDMAC_ENAB | PWMDMAC_THRSHLD;
		udelay(10);
		pwm_reg[PWM_CTL] = PWMCTL_CLRF;
		udelay(10);
		pwm_reg[PWM_CTL] = PWMCTL_USEF1 | PWMCTL_PWEN1;
		udelay(10);
	} else {
		// Initialise PCM
		pcm_reg[PCM_CS_A] = 1;				// Disable Rx+Tx, Enable PCM block
		udelay(100);
		clk_reg[PCMCLK_CNTL] = 0x5A000006;		// Source=PLLD (500MHz)
		udelay(100);
		clk_reg[PCMCLK_DIV] = 0x5A000000 | (500<<12);	// Set pcm div to 500, giving 1MHz
		udelay(100);
		clk_reg[PCMCLK_CNTL] = 0x5A000016;		// Source=PLLD and enable
		udelay(100);
		pcm_reg[PCM_TXC_A] = 0<<31 | 1<<30 | 0<<20 | 0<<16; // 1 channel, 8 bits
		udelay(100);
		pcm_reg[PCM_MODE_A] = (step_time_us - 1) << 10;
		udelay(100);
		pcm_reg[PCM_CS_A] |= 1<<4 | 1<<3;		// Clear FIFOs
		udelay(100);
		pcm_reg[PCM_DREQ_A] = 64<<24 | 64<<8;		// DMA Req when one slot is free?
		udelay(100);
		pcm_reg[PCM_CS_A] |= 1<<9;			// Enable DMA
		udelay(100);
	}

	// Initialise the DMA
	dma_reg[DMA_CS] = DMA_RESET;
	udelay(10);
	dma_reg[DMA_CS] = DMA_INT | DMA_END;
	dma_reg[DMA_CONBLK_AD] = mem_virt_to_phys(cb_base);
	dma_reg[DMA_DEBUG] = 7; // clear debug error flags
	dma_reg[DMA_CS] = 0x10880001;	// go, mid priority, wait for outstanding writes

	if (delay_hw == DELAY_VIA_PCM) {
		pcm_reg[PCM_CS_A] |= 1<<2;			// Enable Tx
	}
}

static int
parse_pin_lists(int p1first, char *p1pins, char*p5pins)
{
	char *name, *pins;
	int i, mapcnt;
	uint8_t *map, *pNpin2servo;
	int lst, servo = 0;
	FILE *fp;

	memset(servo2gpio, DMY, sizeof(servo2gpio));
	memset(p1pin2servo, DMY, sizeof(p1pin2servo));
	memset(p5pin2servo, DMY, sizeof(p5pin2servo));
	for (lst = 0; lst < 2; lst++) {
		if (lst == 0 && p1first) {
			name = "P1";
			pins = p1pins;
			if (board_rev == 1) {
				map = rev1_p1pin2gpio_map;
				mapcnt = sizeof(rev1_p1pin2gpio_map);
			} else {
				map = rev2_p1pin2gpio_map;
				mapcnt = sizeof(rev2_p1pin2gpio_map);
			}
			pNpin2servo = p1pin2servo;
		} else {
			name = "P5";
			pins = p5pins;
			if (board_rev == 1) {
				map = rev1_p5pin2gpio_map;
				mapcnt = sizeof(rev1_p5pin2gpio_map);
			} else {
				map = rev2_p5pin2gpio_map;
				mapcnt = sizeof(rev2_p5pin2gpio_map);
			}
			pNpin2servo = p5pin2servo;
		}
		while (*pins) {
			char *end;
			long pin = strtol(pins, &end, 0);

			if (*end && (end == pins || *end != ',')) {
				fatal("Invalid character '%c' in %s pin list\n", *end, name);
				return -1;
			}
			if (pin < 0 || pin > mapcnt) {
				fatal("Invalid pin number %d in %s pin list\n", pin, name);
				return -1;
			}
			if (servo == MAX_SERVOS) {
				fatal("Too many servos specified\n");
				return -1;
			}
			if (pin == 0) {
				servo++;
			} else {
				if (map[pin-1] == DMY) {
					fatal("Pin %d on header %s cannot be used for a servo output\n", pin, name);
					return -1;
				}
				pNpin2servo[pin] = servo;
				servo2gpio[servo++] = map[pin-1];
				num_servos++;
			}
			pins = end;
			if (*pins == ',')
				pins++;
		}
	}
	return 0;
}

int sc_update(int servo, int width) {
	//if in us -> width /= step_time_us;
	//otherwise in steps
	set_servo(servo, width);
}

int sc_close() {

}

int sc_open()
{
	int i;
	char *p1pins = default_p1_pins;
	char *p5pins = default_p5_pins;
	int p1first = 1, hadp1 = 0, hadp5 = 0;
	char *servo_min_arg = NULL;
	char *servo_max_arg = NULL;
	char *idle_timeout_arg = NULL;
	char *cycle_time_arg = NULL;
	char *step_time_arg = NULL;
	char *dma_chan_arg = NULL;
	char *p;

	setvbuf(stdout, NULL, _IOLBF, 0);

if (parse_pin_lists(p1first, p1pins, p5pins)<0)
	return -1;

dma_chan = DMA_CHAN_DEFAULT;
cycle_time_us = DEFAULT_CYCLE_TIME_US;
step_time_us = DEFAULT_STEP_TIME_US;
servo_min_ticks = DEFAULT_SERVO_MIN_US / step_time_us;
servo_max_ticks = DEFAULT_SERVO_MAX_US / step_time_us;

	num_samples = cycle_time_us / step_time_us;
	num_cbs =     num_samples * 2 + MAX_SERVOS;
	num_pages =   (num_cbs * sizeof(dma_cb_t) + num_samples * 4 +
				MAX_SERVOS * 4 + PAGE_SIZE - 1) >> PAGE_SHIFT;

	if (num_pages > MAX_MEMORY_USAGE / PAGE_SIZE) {
		fatal("Using too much memory; reduce cycle-time or increase step-size\n");
		return -1;
	}

	if (servo_max_ticks > num_samples) {
		fatal("max value is larger than cycle time\n");
		return -1;
	}
	if (servo_min_ticks >= servo_max_ticks) {
		fatal("min value is >= max value\n");
		return -1;
	}

	printf("Using hardware:                %s\n", delay_hw == DELAY_VIA_PWM ? "PWM" : "PCM");
	printf("Using DMA channel:         %7d\n", dma_chan);
	printf("Number of servos:          %7d\n", num_servos);
	printf("Servo cycle time:          %7dus\n", cycle_time_us);
	printf("Pulse increment step size: %7dus\n", step_time_us);
	printf("Minimum width value:       %7d (%dus)\n", servo_min_ticks,
						servo_min_ticks * step_time_us);
	printf("Maximum width value:       %7d (%dus)\n", servo_max_ticks,
						servo_max_ticks * step_time_us);
	printf("Output levels:            %s\n", invert ? "Inverted" : "  Normal");
	for (i = 0; i < MAX_SERVOS; i++) {
		if (servo2gpio[i] == DMY)
			continue;
		printf("    %2d on %-5s          GPIO-%d\n", i, gpio2pinname(servo2gpio[i]), servo2gpio[i]);
	}
	printf("\n");

	setup_sighandlers();

	if ((dma_reg = (uint32_t*)map_peripheral(DMA_BASE, DMA_LEN)) == (void *) -1) {
		return -1;
	}
	dma_reg += dma_chan * DMA_CHAN_SIZE / sizeof(uint32_t);
	if ((pwm_reg = (uint32_t*)map_peripheral(PWM_BASE, PWM_LEN)) == (void *) -1) {
		return -1;
	}
	if ((pcm_reg = (uint32_t*)map_peripheral(PCM_BASE, PCM_LEN)) == (void *) -1) {
		return -1;
	}
	if ((clk_reg = (uint32_t*)map_peripheral(CLK_BASE, CLK_LEN)) == (void *) -1) {
		return -1;
	}
	if ((gpio_reg = (uint32_t*)map_peripheral(GPIO_BASE, GPIO_LEN)) == (void *) -1) {
		return -1;
	}

	/*
	 * Map the pages to our virtual address space; this reserves them and
	 * locks them in memory.  However, these are L1 & L2 non-coherent
	 * cached pages and we want coherent access to them so the DMA
	 * controller sees our changes immediately.  To get that, we create a
	 * second mapping of the same size and immediately free it.  This gives
	 * us an address in our virtual address space where we can map in a
	 * coherent view of the physical pages that were allocated by the first
	 * mmap(). This coherent mapping happens in make_pagemap().  All
	 * accesses to our memory that is shared with the DMA controller are
	 * via this second coherent mapping.  The memset() below forces the
	 * pages to be allocated.
	 */
	virtcached = (uint8_t*)mmap(NULL, num_pages * PAGE_SIZE, PROT_READ|PROT_WRITE,
			MAP_SHARED|MAP_ANONYMOUS|MAP_NORESERVE|MAP_LOCKED,
			-1, 0);
	if (virtcached == MAP_FAILED) {
		fatal("servod: Failed to mmap for cached pages: %m\n");
		return -1;
	}
	if ((unsigned long)virtcached & (PAGE_SIZE-1)) {
		fatal("servod: Virtual address is not page aligned\n");
		return -1;
	}
	
	memset(virtcached, 0, num_pages * PAGE_SIZE);

	virtbase = (uint8_t*)mmap(NULL, num_pages * PAGE_SIZE, PROT_READ|PROT_WRITE,
			MAP_SHARED|MAP_ANONYMOUS|MAP_NORESERVE|MAP_LOCKED,
			-1, 0);
	if (virtbase == MAP_FAILED) {
		fatal("servod: Failed to mmap uncached pages: %m\n");
		return -1;
	}
	if ((unsigned long)virtbase & (PAGE_SIZE-1)) {
		fatal("servod: Virtual address is not page aligned\n");
		return -1;
	}
	munmap(virtbase, num_pages * PAGE_SIZE);

	if (make_pagemap()<0) {
		return -1;
	}

	/*
	 * Now the memory is all mapped, we can set up the pointers to the
	 * bit masks used to turn outputs on and off, and to the DMA control
	 * blocks.  The control blocks must be 32 byte aligned (so round up
	 * to multiple of 8, as we're then multiplying by 4).
	 */
	turnoff_mask = (uint32_t *)virtbase;
	turnon_mask = (uint32_t *)(virtbase + num_samples * sizeof(uint32_t));
	cb_base = (dma_cb_t *)(virtbase +
		ROUNDUP(num_samples + MAX_SERVOS, 8) * sizeof(uint32_t));

	for (i = 0; i < MAX_SERVOS; i++) {
		if (servo2gpio[i] == DMY)
			continue;
		gpiomode[i] = gpio_get_mode(servo2gpio[i]);
		gpio_set(servo2gpio[i], invert ? 1 : 0);
		gpio_set_mode(servo2gpio[i], GPIO_MODE_OUT);
	}
	restore_gpio_modes = 1;

	init_ctrl_data();
	init_hardware();

	return 0;
}
