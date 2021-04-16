//
//  Silly Display Cloner
//  Shows the PI Framebuffer on AMIGA OCS video
//

#include "ps_protocol.h"
#include <bcm_host.h>
#include <fcntl.h>
#include <sched.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <syslog.h>
#include <unistd.h>

// Some important Amiga Chipset Registers
#define VPOSR 0xDFF004
#define INTREQR 0xDFF01E
#define DSKPTH 0xDFF020
#define DSKLEN 0xDFF024
#define LISAID 0xDFF07C
#define COP1LCH 0xDFF080
#define COP1LCL 0xDFF082
#define COPJMP1 0xDFF088
#define DIWSTRT 0xDFF08E
#define DIWSTOP 0xDFF090
#define DDFSTRT 0xDFF092
#define DDFSTOP 0xDFF094
#define DMACON 0xDFF096
#define INTENA 0xDFF09A
#define INTREQ 0xDFF09C
#define ADKCON 0xDFF09E
#define BPLCON0 0xDFF100
#define BPLCON1 0xDFF102
#define BPLCON2 0xDFF104
#define BPL1MOD 0xDFF108
#define BPL2MOD 0xDFF10A
#define COLOR0 0xDFF180
#define COLOR1 0xDFF182
#define COLOR2 0xDFF184
#define COLOR3 0xDFF186
#define COLOR4 0xDFF188
#define COLOR5 0xDFF18A
#define COLOR6 0xDFF18C
#define COLOR7 0xDFF18E
#define COLOR8 0xDFF190
#define COLOR9 0xDFF192
#define COLOR10 0xDFF194
#define COLOR11 0xDFF196
#define COLOR12 0xDFF198
#define COLOR13 0xDFF19A
#define COLOR14 0xDFF19C
#define COLOR15 0xDFF19E

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

// volatile unsigned int val;
unsigned char fbdata[614400];
unsigned char fbdata2[153600];
unsigned char planar[153600];
unsigned char eight[153600];

uint8_t RGB565TORGB323(uint16_t COLOR) {

  uint8_t B = (((COLOR)&0x001F) << 3);
  uint8_t G = (((COLOR)&0x07E0) >> 3);
  uint8_t R = (((COLOR)&0xF800) >> 8);

  uint8_t k = 0;
  k = (((R)&0xE0) | ((G >> 3) & 0x18) | ((B >> 5) & 0x7));

  return k;
}

void fb2grey(void *dest, uint16_t *src, uint16_t x, uint16_t y,
             uint16_t shift) {
  int i = 0;
  for (i = 0; i < x * y; i++) {
    int16_t R = ((src[i] & 0xF800) >> 11);
    int16_t G = ((src[i] & 0x07E0) >> 5);
    int16_t B = (src[i] & 0x001F);
    uint8_t grey = R * 0.3 + G * 0.58 + B * 0.11;
    *(uint8_t *)(dest + i) = grey >> shift;
  }
}

static void c2p(unsigned char *source, unsigned char *dest, unsigned int width,
                unsigned int height, unsigned int planes) {
  unsigned int alignedwidth, x, y, p, bpr, bpl;
  // alignedwidth = (width + 15) & ~15;
  // bpr = alignedwidth / 8;
  bpr = width / 8;
  bpl = bpr * planes;

  for (y = 0; y < height; y++) {
    for (x = 0; x < width; x++) {
      unsigned int mask = 0x80 >> (x & 7);
      unsigned int offset = x / 8;
      unsigned char chunkypix = source[x];

      for (p = 0; p < planes; p++) {
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
  syslog(LOG_INFO, "Primary display is %d x %d", display_info.width,
         display_info.height);

  screen_resource =
      vc_dispmanx_resource_create(VC_IMAGE_RGB565, 320, 240, &image_prt);
  if (!screen_resource) {
    syslog(LOG_ERR, "Unable to create screen buffer");
    vc_dispmanx_display_close(display);
    return -1;
  }

  vc_dispmanx_rect_set(&rect1, 0, 0, 320, 240);

  while (1) {

    ret = vc_dispmanx_snapshot(display, screen_resource, 0);
    vc_dispmanx_resource_read_data(screen_resource, &rect1, fbdata,
                                   320 * 16 / 8);
    // usleep(25 * 1000);

    fb2grey(eight, (uint16_t *)fbdata, 320, 240, 2);
    c2p(eight, planar, 320, 240, 4);

    for (uint32_t i = 0; i < (0x2580 * 4); i += 2)
      write16(BP1BASE + i, (planar[i + 1]) | planar[i] << 8);
  }

  ret = vc_dispmanx_resource_delete(screen_resource);
  vc_dispmanx_display_close(display);
}

int main(int argc, char **argv) {

  /*
          const struct sched_param priority = { 99 };

          sched_setscheduler(0, SCHED_FIFO, &priority);
          mlockall(MCL_CURRENT);
  */

  ps_setup_protocol();
  ps_reset_state_machine();
  ps_pulse_reset();

  usleep(100000);

  // copper test
  write8(0xbfe201, 0x0101); // CIA OVL
  write8(0xbfe001, 0x0000); // CIA OVL LOW

  write16(DMACON, 0x0000);  // dma stop
  write16(BPLCON0, 0x4000); // lores 1 bitplane
  write16(BPLCON1, 0x0);
  write16(BPLCON2, 0x0);
  write16(BPL1MOD, 40 * 3);
  write16(BPL2MOD, 40 * 3);

  // bitplane and window

#define WIDTH 320
#define HEIGHT 240
#define XSTRT 129
#define XSTOP XSTRT + WIDTH
#define YSTRT 44
#define YSTOP YSTRT + HEIGHT
#define HSTRT 129
#define RES 8 // 8 = lores, 4 = hires

  write16(DDFSTRT, (HSTRT / 2 - RES));                            // starthor
  write16(DDFSTOP, (HSTRT / 2 - RES) + (8 * ((WIDTH / 16) - 1))); // stop hor
  write16(DIWSTRT, XSTRT + (YSTRT * 256));               // start window
  write16(DIWSTOP, (XSTOP - 256) + (YSTOP - 256) * 256); // stop window

  write16(COLOR0, 0x0000);  // color0
  write16(COLOR1, 0x1111);  // color1
  write16(COLOR2, 0x2222);  // color1
  write16(COLOR3, 0x3333);  // color1
  write16(COLOR4, 0x4444);  // color1
  write16(COLOR5, 0x5555);  // color1
  write16(COLOR6, 0x6666);  // color1
  write16(COLOR7, 0x7777);  // color1
  write16(COLOR8, 0x8888);  // color0
  write16(COLOR9, 0x9999);  // color1
  write16(COLOR10, 0xAAAA); // color1
  write16(COLOR11, 0xBBBB); // color1
  write16(COLOR12, 0xCCCC); // color1
  write16(COLOR13, 0xDDDD); // color1
  write16(COLOR14, 0xEEEE); // color1
  write16(COLOR15, 0xffff); // color1

  // load copperlist into chipmem
  uint32_t addr = COPBASE; // 0x2000 looks like a fine place for it...

  write16(addr, BPL1PTH);
  addr += 2;
  write16(addr, 0x0000);
  addr += 2; // bitplane pointer
  write16(addr, BPL1PTL);
  addr += 2;
  write16(addr, BP1BASE);
  addr += 2; // bitplane pointer
  write16(addr, BPL2PTH);
  addr += 2;
  write16(addr, 0x0000);
  addr += 2; // bitplane pointer
  write16(addr, BPL2PTL);
  addr += 2;
  write16(addr, BP2BASE);
  addr += 2; // bitplane pointer
  write16(addr, BPL3PTH);
  addr += 2;
  write16(addr, 0x0000);
  addr += 2; // bitplane pointer
  write16(addr, BPL3PTL);
  addr += 2;
  write16(addr, BP3BASE);
  addr += 2; // bitplane pointer
  write16(addr, BPL4PTH);
  addr += 2;
  write16(addr, 0x0000);
  addr += 2; // bitplane pointer
  write16(addr, BPL4PTL);
  addr += 2;
  write16(addr, BP4BASE);
  addr += 2; // bitplane pointer

  write16(addr, 0xFFFF);
  addr += 2;
  write16(addr, 0xFFFE);
  addr += 2; // end of copper list

  write32(COP1LCH, COPBASE);
  write16(COPJMP1, COPBASE); // start copper
  write16(DMACON, 0x8390);   // dma go go go


  process();

  return 0;
}
