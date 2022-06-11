/********************************************************************
* Description:  dma_paced.h
*               DMA based GPIO interface
*
* Author: Brian Moyer
* License: GPL Version 2
s********************************************************************/

// https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
#define MEM_FLAG_DIRECT (1 << 2)
#define MEM_FLAG_COHERENT (2 << 2)
#define MEM_FLAG_L1_NONALLOCATING (MEM_FLAG_DIRECT | MEM_FLAG_COHERENT)

#define TICK_CNT 20
#define CB_CNT (TICK_CNT * 2)

#define PLLD_MHZ 250
#define CLK_DIVI 5
#define PWM_PERMICRO PLLD_MHZ/CLK_DIVI
#define CLK_MICROS 0.5

typedef struct DMACtrlReg
{
    uint32_t cs;      // DMA Channel Control and Status register
    uint32_t cb_addr; // DMA Channel Control Block Address
} DMACtrlReg;

typedef struct DMAControlBlock
{
    uint32_t tx_info;    // Transfer information
    uint32_t src;        // Source (bus) address
    uint32_t dest;       // Destination (bus) address
    uint32_t tx_len;     // Transfer length (in bytes)
    uint32_t stride;     // 2D stride
    uint32_t next_cb;    // Next DMAControlBlock (bus) address
    uint32_t padding[2]; // 2-word padding
} DMAControlBlock;

typedef struct DMAMemHandle
{
    void *virtual_addr; // Virutal base address of the page
    uint32_t bus_addr;  // Bus adress of the page, this is not a pointer because it does not point to valid virtual address
    uint32_t mb_handle; // Used by mailbox property interface
    uint32_t size;
} DMAMemHandle;

typedef struct CLKCtrlReg
{
    // See https://elinux.org/BCM2835_registers#CM
    uint32_t ctrl;
    uint32_t div;
} CLKCtrlReg;

typedef struct PWMCtrlReg
{
    uint32_t ctrl;     // 0x0, Control
    uint32_t status;   // 0x4, Status
    uint32_t dma_cfg;  // 0x8, DMA configuration
    uint32_t padding1; // 0xC, 4-byte padding
    uint32_t range1;   // 0x10, Channel 1 range
    uint32_t data1;    // 0x14, Channel 1 data
    uint32_t fifo_in;  // 0x18, FIFO input
    uint32_t padding2; // 0x1C, 4-byte padding again
    uint32_t range2;   // 0x20, Channel 2 range
    uint32_t data2;    // 0x24, Channel 2 data
} PWMCtrlReg;

int mailbox_fd = -1;
DMAMemHandle *dma_cbs;
DMAMemHandle *dma_ticks;
volatile DMACtrlReg *dma_reg;
volatile PWMCtrlReg *pwm_reg;
volatile CLKCtrlReg *clk_reg;

void dma_alloc_buffers(void);
void dma_init_cbs(void);
void init_hw_clk(void);
void init_pwm(void);
void dma_start(void);
void dma_end(void);
void *map_peripheral(uint32_t addr, uint32_t size);
static inline uint32_t *ith_tick_virt_addr(int i);
