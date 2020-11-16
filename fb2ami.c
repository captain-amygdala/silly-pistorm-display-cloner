//
//  How to access GPIO registers from C-code on the Raspberry-Pi
//  Example program
//  15-January-2012
//  Dom and Gert
//  Revised: 15-Feb-2013


// Access from ARM Running Linux

//#define BCM2708_PERI_BASE        0x20000000
#define BCM2708_PERI_BASE       0x3F000000
#define BCM2708_PERI_SIZE       0x01000000
#define GPIO_BASE               (BCM2708_PERI_BASE + 0x200000)  /* GPIO controller */
#define GPCLK_BASE              (BCM2708_PERI_BASE + 0x101000)
#define GPIO_ADDR                0x200000                       /* GPIO controller */
#define GPCLK_ADDR               0x101000
#define CLK_PASSWD      0x5a000000
#define CLK_GP0_CTL     0x070
#define CLK_GP0_DIV     0x074

#define SA0 5
#define SA1 3
#define SA2 2

#define W16              GPIO_CLR = 1 << SA0; GPIO_CLR = 1 << SA1; GPIO_CLR = 1 << SA2;
#define R16              GPIO_SET = 1 << SA0; GPIO_CLR = 1 << SA1; GPIO_CLR = 1 << SA2;
#define W8               GPIO_CLR = 1 << SA0; GPIO_SET = 1 << SA1; GPIO_CLR = 1 << SA2;
#define R8               GPIO_SET = 1 << SA0; GPIO_SET = 1 << SA1; GPIO_CLR = 1 << SA2;
#define STATUSREGADDR    GPIO_CLR = 1 << SA0; GPIO_CLR = 1 << SA1; GPIO_SET = 1 << SA2;



#define VPOSR   0xDFF004
#define INTREQR 0xDFF01E
#define DSKPTH  0xDFF020
#define DSKLEN  0xDFF024
#define LISAID  0xDFF07C
#define COP1LCH 0xDFF080
#define COP1LCL 0xDFF082
#define COPJMP1 0xDFF088
#define DIWSTRT 0xDFF08E
#define DIWSTOP 0xDFF090
#define DDFSTRT 0xDFF092
#define DDFSTOP 0xDFF094
#define DMACON  0xDFF096
#define INTENA  0xDFF09A
#define INTREQ  0xDFF09C
#define ADKCON  0xDFF09E
#define BPLCON0 0xDFF100
#define BPLCON1 0xDFF102
#define BPLCON2 0xDFF104
#define BPL1MOD 0xDFF108
#define BPL2MOD 0xDFF10A
#define COLOR0  0xDFF180
#define COLOR1  0xDFF182
#define COLOR2  0xDFF184
#define COLOR3  0xDFF186
#define COLOR4  0xDFF188
#define COLOR5  0xDFF18A
#define COLOR6  0xDFF18C
#define COLOR7  0xDFF18E
#define COLOR8  0xDFF190
#define COLOR9  0xDFF192
#define COLOR10  0xDFF194
#define COLOR11  0xDFF196
#define COLOR12  0xDFF198
#define COLOR13  0xDFF19A
#define COLOR14  0xDFF19C
#define COLOR15  0xDFF19E






#define BPL1PTH 0xdff0e0
#define BPL1PTL 0xdff0e2
#define BPL2PTH 0xdff0e4
#define BPL2PTL 0xdff0e6
#define BPL3PTH 0xdff0e8
#define BPL3PTL 0xdff0eA
#define BPL4PTH 0xdff0eC
#define BPL4PTL 0xdff0eE
#define BPL5PTH 0xdff0F0
#define BPL5PTL 0xdff0F2
#define BPL6PTH 0xdff0F4
#define BPL6PTL 0xdff0F6

#define COPBASE 0x0
#define BP1BASE 0x1000
#define BP2BASE BP1BASE + 0x28
#define BP3BASE BP2BASE + 0x28
#define BP4BASE BP3BASE + 0x28
#define BP5BASE BP4BASE + 0x28
#define BP6BASE BP5BASE + 0x28

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <sched.h>
//#include "planar.h"
#include <stdio.h>
#include <syslog.h>
//#include "propane.h"
//#include <sys/fcntl.h>
//#include <sys/ioctl.h>
//#include <linux/fb.h>
//#include <sys/mman.h>

#include <bcm_host.h>




#define PAGE_SIZE (4 * 1024)
#define BLOCK_SIZE (4 * 1024)

#define GPIOSET(no, ishigh)           \
	do {                                  \
		if (ishigh)                   \
			set |= (1 << (no));   \
		else                          \
			reset |= (1 << (no)); \
	} while (0)


int mem_fd;
int mem_fd_gpclk;
void *gpio_map;
void *gpclk_map;

volatile unsigned int val;

// I/O access
volatile unsigned int *gpio;
volatile unsigned int *gpclk;
volatile unsigned int gpfsel0;
volatile unsigned int gpfsel1;
volatile unsigned int gpfsel2;

volatile unsigned int gpfsel0_o;
volatile unsigned int gpfsel1_o;
volatile unsigned int gpfsel2_o;

volatile unsigned int gpfsel0_W16;
volatile unsigned int gpfsel1_W16;
volatile unsigned int gpfsel2_W16;

volatile unsigned int gpfsel0_W8;
volatile unsigned int gpfsel1_W8;
volatile unsigned int gpfsel2_W8;

volatile unsigned int gpfsel0_R16;
volatile unsigned int gpfsel1_R16;
volatile unsigned int gpfsel2_R16;

volatile unsigned int gpfsel0_R8;
volatile unsigned int gpfsel1_R8;
volatile unsigned int gpfsel2_R8;






// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio + ((g) / 10)) &= ~(7 << (((g) % 10) * 3))
#define OUT_GPIO(g) *(gpio + ((g) / 10)) |= (1 << (((g) % 10) * 3))
#define SET_GPIO_ALT(g, a) *(gpio + (((g) / 10))) |= (((a) <= 3 ? (a) + 4 : (a) == 4 ? 3 : 2) << (((g) % 10) * 3))

#define GPIO_SET *(gpio + 7)                    // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio + 10)                   // clears bits which are 1 ignores bits which are 0

#define GET_GPIO(g) (*(gpio + 13) & (1 << g))   // 0 if LOW, (1<<g) if HIGH

#define GPIO_PULL *(gpio + 37)                  // Pull up/pull down
#define GPIO_PULLCLK0 *(gpio + 38)              // Pull up/pull down clock

void setup_io();

uint32_t read8(uint32_t address);
void write8(uint32_t address, uint32_t data);

uint32_t read16(uint32_t address);
void write16(uint32_t address, uint32_t data);

void write32(uint32_t address, uint32_t data);
uint32_t read32(uint32_t address);

uint16_t read_reg(void);
void write_reg(unsigned int value);

#ifndef NULL
#define NULL  (void *) 0
#endif

/*
 * Set the data bus width.
 */
typedef unsigned long datum;

/*
 * Function prototypes.
 */
datum   memTestDataBus(uint32_t address);
datum * memTestAddressBus(uint32_t * baseAddress, unsigned long nBytes);
datum * memTestDevice(volatile datum * baseAddress, unsigned long nBytes);


//char *viewbuf = (char *)malloc( 320 * 240 * 8 );

unsigned char fbdata[614400];
unsigned char fbdata2[153600];
unsigned char planar[153600];
unsigned char eight[153600];

uint8_t RGB565TORGB323(uint16_t COLOR)
{


    uint8_t B =  (((COLOR) & 0x001F) << 3);
    uint8_t G  = (((COLOR) & 0x07E0) >> 3);
    uint8_t R  = (((COLOR) & 0xF800) >> 8);



    uint8_t k = 0;
    k =  (((R)&0xE0) | ((G>>3)&0x18) | ((B>>5)&0x7));



    return k;
}


void fb2grey(void *dest, uint16_t *src, uint16_t x, uint16_t y,uint16_t shift)
{
    int i=0;
    for(i=0; i<x*y; i++)
    {
	int16_t R = ((src[i] & 0xF800)>>11);
	int16_t G =((src[i] & 0x07E0)>>5);
 	int16_t B =(src[i] & 0x001F);
	uint8_t grey = R * 0.3 + G * 0.58 + B * 0.11;
  	*(uint8_t *)(dest+i) = grey>>shift;
    }
}


static void c2p(unsigned char *source, unsigned char *dest, unsigned int width, unsigned int height, unsigned int planes)
{
    unsigned int alignedwidth, x, y, p, bpr, bpl;
    //alignedwidth = (width + 15) & ~15;
    //bpr = alignedwidth / 8;
    bpr = width / 8;
    bpl = bpr * planes;

    for(y = 0; y < height; y++)
    {
	for(x = 0; x < width; x++)
	{
	    unsigned int mask   = 0x80 >> (x & 7);
	    unsigned int offset = x / 8;
	    unsigned char chunkypix = source[x];

	    for(p = 0; p < planes; p++)
	    {
		if (chunkypix & (1 << p))
	    	    dest[p * bpr + offset] |= mask;
		else
		    dest[p * bpr + offset] &= ~mask;
	    }
	}

    	source += width;
    	dest += bpl;
    }
}



int process() {
    DISPMANX_DISPLAY_HANDLE_T display;
    DISPMANX_MODEINFO_T display_info;
    DISPMANX_RESOURCE_HANDLE_T screen_resource;
    VC_IMAGE_TRANSFORM_T transform;
    uint32_t image_prt;
    VC_RECT_T rect1;
    int ret;

    bcm_host_init();

    display = vc_dispmanx_display_open(0);
    if (!display) {
        syslog(LOG_ERR, "Unable to open primary display");
        return -1;
    }
    ret = vc_dispmanx_display_get_info(display, &display_info);
    if (ret) {
        syslog(LOG_ERR, "Unable to get primary display information");
        return -1;
    }
    syslog(LOG_INFO, "Primary display is %d x %d", display_info.width, display_info.height);

    screen_resource = vc_dispmanx_resource_create(VC_IMAGE_RGB565   , 320, 240, &image_prt);
    if (!screen_resource) {
        syslog(LOG_ERR, "Unable to create screen buffer");
        vc_dispmanx_display_close(display);
        return -1;
    }



    vc_dispmanx_rect_set(&rect1, 0, 0, 320, 240);

    while (1) {

        ret = vc_dispmanx_snapshot(display, screen_resource, 0);
        vc_dispmanx_resource_read_data(screen_resource, &rect1, fbdata, 320 * 16 / 8);
        //usleep(25 * 1000);

	fb2grey(eight,(uint16_t *)fbdata, 320,240,2);
	c2p(eight,planar,320,240,4);

	for (uint32_t i = 0; i < (0x2580 * 4); i += 2)
	write16(BP1BASE + i,(planar[i+1]) | planar[i] <<8);

    }

   ret = vc_dispmanx_resource_delete(screen_resource);
    vc_dispmanx_display_close(display);
}




int main(int argc, char **argv)
{
	int g, rep;


	const struct sched_param priority = { 99 };

	sched_setscheduler(0, SCHED_FIFO, &priority);
	mlockall(MCL_CURRENT);


	setup_io();

	*(gpclk + (CLK_GP0_CTL / 4)) = CLK_PASSWD | (1 << 5);
	usleep(10);
	while ((*(gpclk + (CLK_GP0_CTL / 4))) & (1 << 7));
	usleep(100);
	*(gpclk + (CLK_GP0_DIV / 4)) = CLK_PASSWD | (6 << 12);          //divider
	usleep(10);
	*(gpclk + (CLK_GP0_CTL / 4)) = CLK_PASSWD | 5 | (1 << 4);       //pll? 6=plld, 5=pllc
	usleep(10);
	while (((*(gpclk + (CLK_GP0_CTL / 4))) & (1 << 7)) == 0);
	usleep(100);

	SET_GPIO_ALT(4, 0); //gpclk0

	//set SA to output
	INP_GPIO(2);
	OUT_GPIO(2);
	INP_GPIO(3);
	OUT_GPIO(3);
	INP_GPIO(5);
	OUT_GPIO(5);

	//set gpio0 (aux0) and gpio1 (aux1) to input
	INP_GPIO(0);
	INP_GPIO(1);

	// Set GPIO pins 6,7 and 8-23 to output
	for (g = 6; g <= 23; g++) {
		INP_GPIO(g);
		OUT_GPIO(g);
	}
	gpfsel0_o = *(gpio);            //store gpio ddr
	gpfsel1_o = *(gpio + 1);        //store gpio ddr
	gpfsel2_o = *(gpio + 2);        //store gpio ddr

	// Set GPIO pins 8-23 to input
	for (g = 8; g <= 23; g++)
		INP_GPIO(g);
	gpfsel0 = *(gpio);      //store gpio ddr
	gpfsel1 = *(gpio + 1);  //store gpio ddr
	gpfsel2 = *(gpio + 2);  //store gpio ddr

	GPIO_CLR = 1 << 2;
	GPIO_CLR = 1 << 3;
	GPIO_SET = 1 << 5;

	GPIO_SET = 1 << 6;
	GPIO_SET = 1 << 7;

	//reset smi statemachine

	write_reg(0x01);
	usleep(100);
	write_reg(0x00);
	usleep(100);

//copper test
	write8(0xbfe201, 0x0101);       //CIA OVL
	write8(0xbfe001, 0x0000);       //CIA OVL LOW

	write16(DMACON, 0x0000);        //dma stop
	write16(BPLCON0, 0x4000);       //lores 1 bitplane
	write16(BPLCON1, 0x0);
	write16(BPLCON2, 0x0);
	write16(BPL1MOD, 40*3);
	write16(BPL2MOD, 40*3);


//bitplane and window

#define WIDTH 320
#define HEIGHT 240
#define XSTRT 129
#define XSTOP XSTRT+WIDTH
#define YSTRT 44
#define YSTOP YSTRT+HEIGHT
#define HSTRT 129
#define RES	8 // 8 = lores, 4 = hires

write16(DDFSTRT, (HSTRT/2-RES) );         //starthor
write16(DDFSTOP, (HSTRT/2-RES)+(8*((WIDTH/16)-1)));         //stop hor
write16(DIWSTRT, XSTRT+(YSTRT*256));       //start window
write16(DIWSTOP, (XSTOP-256)+(YSTOP-256)*256);       //stop window




	write16(COLOR0, 0x0000);        //color0
	write16(COLOR1, 0x1111);        //color1
        write16(COLOR2, 0x2222);        //color1
        write16(COLOR3, 0x3333);        //color1
        write16(COLOR4, 0x4444);        //color1
        write16(COLOR5, 0x5555);        //color1
        write16(COLOR6, 0x6666);        //color1
        write16(COLOR7, 0x7777);        //color1
        write16(COLOR8, 0x8888);        //color0
        write16(COLOR9, 0x9999);        //color1
        write16(COLOR10, 0xAAAA);        //color1
        write16(COLOR11, 0xBBBB);        //color1
        write16(COLOR12, 0xCCCC);        //color1
        write16(COLOR13, 0xDDDD);        //color1
        write16(COLOR14, 0xEEEE);        //color1
        write16(COLOR15, 0xffff);        //color1





//load copperlist into chipmem
	uint32_t addr = COPBASE; //0x2000 looks like a fine place for it...

	write16(addr, BPL1PTH); addr += 2; write16(addr, 0x0000); addr += 2;    //bitplane pointer
	write16(addr, BPL1PTL); addr += 2; write16(addr, BP1BASE); addr += 2;    //bitplane pointer
        write16(addr, BPL2PTH); addr += 2; write16(addr, 0x0000); addr += 2;    //bitplane pointer
        write16(addr, BPL2PTL); addr += 2; write16(addr, BP2BASE); addr += 2;    //bitplane pointer
        write16(addr, BPL3PTH); addr += 2; write16(addr, 0x0000); addr += 2;    //bitplane pointer
        write16(addr, BPL3PTL); addr += 2; write16(addr, BP3BASE); addr += 2;    //bitplane pointer
        write16(addr, BPL4PTH); addr += 2; write16(addr, 0x0000); addr += 2;    //bitplane pointer
        write16(addr, BPL4PTL); addr += 2; write16(addr, BP4BASE); addr += 2;    //bitplane pointer
/*
       	write16(addr, BPL5PTH); addr += 2; write16(addr, 0x0000); addr += 2;    //bitplane pointer
        write16(addr, BPL5PTL); addr += 2; write16(addr, BP5BASE); addr += 2;    //bitplane pointer
       	write16(addr, BPL6PTH); addr += 2; write16(addr, 0x0000); addr += 2;    //bitplane pointer
        write16(addr, BPL6PTL); addr += 2; write16(addr, BP6BASE); addr += 2;    //bitplane pointer
*/
/*
	write16(addr, 0x9001); addr += 2; write16(addr, 0xFFFE); addr += 2;     //wait for line 144
	write16(addr, 0x0180); addr += 2; write16(addr, 0x0F00); addr += 2;     //move red color to $DFF180
	write16(addr, 0xA001); addr += 2; write16(addr, 0xFFFE); addr += 2;     //wait for line 160
	write16(addr, 0x0180); addr += 2; write16(addr, 0x0FFF); addr += 2;     //move white color to $DFF180
	write16(addr, 0xA401); addr += 2; write16(addr, 0xFFFE); addr += 2;     //wait for line 164
	write16(addr, 0x0180); addr += 2; write16(addr, 0x000F); addr += 2;     //move blue color to $DFF180
	write16(addr, 0xAA01); addr += 2; write16(addr, 0xFFFE); addr += 2;     //wait for line 170
	write16(addr, 0x0180); addr += 2; write16(addr, 0x0FFF); addr += 2;     //move white color to $DFF180
	write16(addr, 0xAE01); addr += 2; write16(addr, 0xFFFE); addr += 2;     //wait for line 174
	write16(addr, 0x0180); addr += 2; write16(addr, 0x0F00); addr += 2;     //move red color to $DFF180
	write16(addr, 0xBE01); addr += 2; write16(addr, 0xFFFE); addr += 2;     //wait for line 190
	write16(addr, 0x0180); addr += 2; write16(addr, 0x0000); addr += 2;     //move black color to $DFF180
*/
	write16(addr, 0xFFFF); addr += 2; write16(addr, 0xFFFE); addr += 2;     //end of copper list

	write32(COP1LCH, COPBASE);
	write16(COPJMP1, COPBASE);       //start copper
	write16(DMACON, 0x8390);        //dma go go go


//	 for (uint16_t i = 0; i < 0x2580; i += 2)
//        write16(BP1BASE + i,(gImage_propane[i+1]) | gImage_propane[i] <<8);

	process();

	return 0;
} // main

void write32(uint32_t address, uint32_t data)
{
	write16(address, data >> 16);
	write16(address + 2, data);
}

uint32_t read32(uint32_t address)
{
	uint16_t a = read16(address);
	uint16_t b = read16(address + 2);

	return (a << 16) | b;
}


void write16(uint32_t address, uint32_t data)
{
	//int val;

	asm volatile ("dmb" ::: "memory");
	W16
	asm volatile ("nop" ::);
	asm volatile ("nop" ::);
	asm volatile ("nop" ::);
	//write phase
	*(gpio) = gpfsel0_o;
	*(gpio + 1) = gpfsel1_o;
	*(gpio + 2) = gpfsel2_o;

	*(gpio + 7) = (address & 0x0000ffff) << 8;
	*(gpio + 10) = (~address & 0x0000ffff) << 8;
	GPIO_CLR = 1 << 7;
	GPIO_CLR = 1 << 7; //delay
	GPIO_SET = 1 << 7;

	*(gpio + 7) = (address >> 16) << 8;
	*(gpio + 10) = (~address >> 16) << 8;
	GPIO_CLR = 1 << 7;
	GPIO_CLR = 1 << 7; //delay
	GPIO_SET = 1 << 7;

	//write phase
	*(gpio + 7) = (data & 0x0000ffff) << 8;
	*(gpio + 10) = (~data & 0x0000ffff) << 8;
	GPIO_CLR = 1 << 7;
	GPIO_CLR = 1 << 7; //delay
        GPIO_SET = 1 << 7;

	*(gpio) = gpfsel0;
	*(gpio + 1) = gpfsel1;
	*(gpio + 2) = gpfsel2;
	while ((GET_GPIO(0)));

	asm volatile ("dmb" ::: "memory");
}

void write8(uint32_t address, uint32_t data)
{
	if ((address & 1) == 0)
	    data = data << 8; //EVEN, A0=0,UDS
	else data = data ; //ODD , A0=1,LDS

        asm volatile ("dmb" ::: "memory");
        W8
        asm volatile ("nop" ::);
        asm volatile ("nop" ::);
        asm volatile ("nop" ::);
        //write phase
        *(gpio) = gpfsel0_o;
        *(gpio + 1) = gpfsel1_o;
        *(gpio + 2) = gpfsel2_o;

        *(gpio + 7) = (address & 0x0000ffff) << 8;
        *(gpio + 10) = (~address & 0x0000ffff) << 8;
        GPIO_CLR = 1 << 7;
        GPIO_CLR = 1 << 7; //delay
        GPIO_SET = 1 << 7;

        *(gpio + 7) = (address >> 16) << 8;
        *(gpio + 10) = (~address >> 16) << 8;
        GPIO_CLR = 1 << 7;
        GPIO_CLR = 1 << 7; //delay
        GPIO_SET = 1 << 7;

        //write phase
        *(gpio + 7) = (data & 0x0000ffff) << 8;
        *(gpio + 10) = (~data & 0x0000ffff) << 8;
        GPIO_CLR = 1 << 7;
        GPIO_CLR = 1 << 7; //delay
        GPIO_SET = 1 << 7;

        *(gpio) = gpfsel0;
        *(gpio + 1) = gpfsel1;
        *(gpio + 2) = gpfsel2;
        while ((GET_GPIO(0)));

        asm volatile ("dmb" ::: "memory");
}



uint32_t read16(uint32_t address)
{
	volatile int val;
	asm volatile ("dmb" ::: "memory");
	R16
	asm volatile ("nop" ::);
	asm volatile ("nop" ::);
	asm volatile ("nop" ::);
	//write phase
	*(gpio) = gpfsel0_o;
	*(gpio + 1) = gpfsel1_o;
	*(gpio + 2) = gpfsel2_o;

	val = address;// & 0x0000FFFF;
	*(gpio + 7) = (val & 0xffff) << 8;
	*(gpio + 10) = (~val & 0xffff) << 8;

	GPIO_CLR = 1 << 7;
	GPIO_CLR = 1 << 7; //delay
	GPIO_SET = 1 << 7;

	val = address >> 16;
	*(gpio + 7) = (val & 0xffff) << 8;
	*(gpio + 10) = (~val & 0xffff) << 8;
	GPIO_CLR = 1 << 7;
	GPIO_CLR = 1 << 7; //delay
	GPIO_SET = 1 << 7;

	//read phase

	*(gpio) = gpfsel0;
	*(gpio + 1) = gpfsel1;
	*(gpio + 2) = gpfsel2;

	GPIO_CLR = 1 << 6;
	while (!(GET_GPIO(0)));
	GPIO_CLR = 1 << 6;
        GPIO_CLR = 1 << 6;
	val = *(gpio + 13);
	GPIO_SET = 1 << 6;

	asm volatile ("dmb" ::: "memory");
	return (val >>8)&0xffff;
}

uint32_t read8(uint32_t address)
{
	int val;

	asm volatile ("dmb" ::: "memory");
	R8
	asm volatile ("nop" ::);
	asm volatile ("nop" ::);
	asm volatile ("nop" ::);
	//write phase
	*(gpio) = gpfsel0_o;
	*(gpio + 1) = gpfsel1_o;
	*(gpio + 2) = gpfsel2_o;

	val = address;// & 0x0000FFFF;
	*(gpio + 7) = (val & 0xffff) << 8;
	*(gpio + 10) = (~val & 0xffff) << 8;

	GPIO_CLR = 1 << 7;
	GPIO_CLR = 1 << 7; //delay
	GPIO_SET = 1 << 7;

	val = address >> 16;
	*(gpio + 7) = (val & 0xffff) << 8;
	*(gpio + 10) = (~val & 0xffff) << 8;
	GPIO_CLR = 1 << 7;
	GPIO_CLR = 1 << 7; //delay
	GPIO_SET = 1 << 7;

	//read phase

	*(gpio) = gpfsel0;
	*(gpio + 1) = gpfsel1;
	*(gpio + 2) = gpfsel2;

	GPIO_CLR = 1 << 6;
	while (!(GET_GPIO(0)));
	//GPIO_CLR = 1 << 6;
        //GPIO_CLR = 1 << 6;
	val = *(gpio + 13);
	GPIO_SET = 1 << 6;

	asm volatile ("dmb" ::: "memory");
	return (val >>8)&0xffff;
	//return (val & 0xffff) >> 8;
}



void write_reg(unsigned int value)
{
	asm volatile ("dmb" ::: "memory");
	STATUSREGADDR
	asm volatile ("nop" ::);
	asm volatile ("nop" ::);
	asm volatile ("nop" ::);

	*(gpio) = gpfsel0_o;
	*(gpio + 1) = gpfsel1_o;
	*(gpio + 2) = gpfsel2_o;
	*(gpio + 7) = (value & 0xffff) << 8;
	*(gpio + 10) = (~value & 0xffff) << 8;
	GPIO_CLR = 1 << 7;
	GPIO_CLR = 1 << 7; //delay
	GPIO_SET = 1 << 7;
	//Bus HIGH-Z
	*(gpio) = gpfsel0;
	*(gpio + 1) = gpfsel1;
	*(gpio + 2) = gpfsel2;
	asm volatile ("dmb" ::: "memory");
}


uint16_t read_reg(void)
{
	uint32_t val;

	asm volatile ("dmb" ::: "memory");
	STATUSREGADDR
	asm volatile ("nop" ::);
	asm volatile ("nop" ::);
	asm volatile ("nop" ::);
	//Bus HIGH-Z
	*(gpio) = gpfsel0;
	*(gpio + 1) = gpfsel1;
	*(gpio + 2) = gpfsel2;

	GPIO_CLR = 1 << 6;
	GPIO_CLR = 1 << 6;      //delay
	GPIO_CLR = 1 << 6;      //delay
	GPIO_CLR = 1 << 6;      //delay
	val = *(gpio + 13);
	GPIO_SET = 1 << 6;
	asm volatile ("dmb" ::: "memory");

	return (uint16_t)(val >> 8);
}


//
// Set up a memory regions to access GPIO
//
void setup_io()
{
	/* open /dev/mem */
	if ((mem_fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
		printf("can't open /dev/mem \n");
		exit(-1);
	}

	/* mmap GPIO */
	gpio_map = mmap(
		NULL,                   //Any adddress in our space will do
		BCM2708_PERI_SIZE,      //Map length
		PROT_READ | PROT_WRITE, // Enable reading & writting to mapped memory
		MAP_SHARED,             //Shared with other processes
		mem_fd,                 //File to map
		BCM2708_PERI_BASE       //Offset to GPIO peripheral
		);

	close(mem_fd); //No need to keep mem_fd open after mmap

	if (gpio_map == MAP_FAILED) {
		printf("gpio mmap error %d\n", (int)gpio_map);//errno also set!
		exit(-1);
	}

	gpio = ((volatile unsigned *)gpio_map) + GPIO_ADDR / 4;
	gpclk = ((volatile unsigned *)gpio_map) + GPCLK_ADDR / 4;
} // setup_io

/**********************************************************************
 *
 * Function:    memTestDataBus()
 *
 * Description: Test the data bus wiring in a memory region by
 *              performing a walking 1's test at a fixed address
 *              within that region.  The address (and hence the
 *              memory region) is selected by the caller.
 *
 * Notes:       
 *
 * Returns:     0 if the test succeeds.  
 *              A non-zero result is the first pattern that failed.
 *
 **********************************************************************/
datum
memTestDataBus(uint32_t address)
{
    uint16_t pattern;
    volatile uint16_t read;		

    /*
     * Perform a walking 1's test at the given address.
     */
    for (pattern = 1; pattern != 0; pattern <<= 1)
    {
        /*
         * Write the test pattern.
         */
        // *address = pattern;
	write16(address, pattern);
        /*
         * Read it back (immediately is okay for this test).
         */
        //if (*address != pattern)
	read = read16(address); 
	if ( read != pattern)
        {
            printf("address=%#x\n",address);
	    printf("read=%#x\n",read16(address));
            printf("pattern=%#x\n",pattern);
            return (pattern);
        }
    }

    return (0);

}   /* memTestDataBus() */


//uint32_t read16(uint32_t address);
//void write16(uint32_t address, uint32_t data);



/**********************************************************************
 *
 * Function:    memTestAddressBus()
 *
 * Description: Test the address bus wiring in a memory region by
 *              performing a walking 1's test on the relevant bits
 *              of the address and checking for aliasing. This test
 *              will find single-bit address failures such as stuck
 *              -high, stuck-low, and shorted pins.  The base address
 *              and size of the region are selected by the caller.
 *
 * Notes:       For best results, the selected base address should
 *              have enough LSB 0's to guarantee single address bit
 *              changes.  For example, to test a 64-Kbyte region, 
 *              select a base address on a 64-Kbyte boundary.  Also, 
 *              select the region size as a power-of-two--if at all 
 *              possible.
 *
 * Returns:     NULL if the test succeeds.  
 *              A non-zero result is the first address at which an
 *              aliasing problem was uncovered.  By examining the
 *              contents of memory, it may be possible to gather
 *              additional information about the problem.
 *
 **********************************************************************/
datum * 
memTestAddressBus(uint32_t * baseAddress, unsigned long nBytes)
{
    unsigned long addressMask = (nBytes/sizeof(datum) - 1);
    unsigned long offset;
    unsigned long testOffset;

    datum pattern     = (datum) 0xAAAAAAAA;
    datum antipattern = (datum) 0x55555555;


    /*
     * Write the default pattern at each of the power-of-two offsets.
     */
    for (offset = 1; (offset & addressMask) != 0; offset <<= 1)
    {
        //baseAddress[offset] = pattern;
	//write16(baseAddress[offset], pattern);
    }

    /* 
     * Check for address bits stuck high.
     */
    testOffset = 0;
    baseAddress[testOffset] = antipattern;

    for (offset = 1; (offset & addressMask) != 0; offset <<= 1)
    {
        if (baseAddress[offset] != pattern)
        {
            return ((datum *) &baseAddress[offset]);
        }
    }

    baseAddress[testOffset] = pattern;

    /*
     * Check for address bits stuck low or shorted.
     */
    for (testOffset = 1; (testOffset & addressMask) != 0; testOffset <<= 1)
    {
        baseAddress[testOffset] = antipattern;

		if (baseAddress[0] != pattern)
		{
			return ((datum *) &baseAddress[testOffset]);
		}

        for (offset = 1; (offset & addressMask) != 0; offset <<= 1)
        {
            if ((baseAddress[offset] != pattern) && (offset != testOffset))
            {
                return ((datum *) &baseAddress[testOffset]);
            }
        }

        baseAddress[testOffset] = pattern;
    }

    return (NULL);

}   /* memTestAddressBus() */


/**********************************************************************
 *
 * Function:    memTestDevice()
 *
 * Description: Test the integrity of a physical memory device by
 *              performing an increment/decrement test over the
 *              entire region.  In the process every storage bit 
 *              in the device is tested as a zero and a one.  The
 *              base address and the size of the region are
 *              selected by the caller.
 *
 * Notes:       
 *
 * Returns:     NULL if the test succeeds.
 *
 *              A non-zero result is the first address at which an
 *              incorrect value was read back.  By examining the
 *              contents of memory, it may be possible to gather
 *              additional information about the problem.
 *
 **********************************************************************/
datum * 
memTestDevice(volatile datum * baseAddress, unsigned long nBytes)	
{
    unsigned long offset;
    unsigned long nWords = nBytes / sizeof(datum);

    datum pattern;
    datum antipattern;


    /*
     * Fill memory with a known pattern.
     */
    for (pattern = 1, offset = 0; offset < nWords; pattern++, offset++)
    {
        baseAddress[offset] = pattern;
    }

    /*
     * Check each location and invert it for the second pass.
     */
    for (pattern = 1, offset = 0; offset < nWords; pattern++, offset++)
    {
        if (baseAddress[offset] != pattern)
        {
            return ((datum *) &baseAddress[offset]);
        }

        antipattern = ~pattern;
        baseAddress[offset] = antipattern;
    }

    /*
     * Check each location for the inverted pattern and zero it.
     */
    for (pattern = 1, offset = 0; offset < nWords; pattern++, offset++)
    {
        antipattern = ~pattern;
        if (baseAddress[offset] != antipattern)
        {
            return ((datum *) &baseAddress[offset]);
        }
    }

    return (NULL);

}   /* memTestDevice() */
