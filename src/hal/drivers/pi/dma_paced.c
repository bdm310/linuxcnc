/********************************************************************
* Description:  dma_paced.c
*               DMA based GPIO interface
*
* Author: Brian Moyer
* License: GPL Version 2
s********************************************************************/

DMAMemHandle *dma_malloc(unsigned int size)
{
    if (mailbox_fd < 0)
    {
        mailbox_fd = mbox_open();
        assert(mailbox_fd >= 0);
    }

    // Make `size` a multiple of PAGE_SIZE
    size = ((size + PAGE_SIZE - 1) / PAGE_SIZE) * PAGE_SIZE;

    DMAMemHandle *mem = (DMAMemHandle *)malloc(sizeof(DMAMemHandle));
    // Documentation: https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
    mem->mb_handle = mem_alloc(mailbox_fd, size, PAGE_SIZE, MEM_FLAG_L1_NONALLOCATING);
    mem->bus_addr = mem_lock(mailbox_fd, mem->mb_handle);
    mem->virtual_addr = mapmem(BUS_TO_PHYS(mem->bus_addr), size);
    mem->size = size;

    assert(mem->bus_addr != 0);

    fprintf(stderr, "MBox alloc: %d bytes, bus: %08X, virt: %08X\n", mem->size, mem->bus_addr, (uint32_t)mem->virtual_addr);

    return mem;
}

void dma_free(DMAMemHandle *mem)
{
    if (mem->virtual_addr == NULL)
        return;

    unmapmem(mem->virtual_addr, mem->size);
    mem_unlock(mailbox_fd, mem->mb_handle);
    mem_free(mailbox_fd, mem->mb_handle);
    mem->virtual_addr = NULL;
}

void *map_peripheral(uint32_t addr, uint32_t size)
{
    int mem_fd;
    
    seteuid(0);
    setfsuid(geteuid());
    // Check mem(4) about /dev/mem
    mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    setfsuid(getuid());
    
    if (mem_fd < 0)
    {
        perror("Failed to open /dev/mem: ");
        exit(-1);
    }

    uint32_t *result = (uint32_t *)mmap(
        NULL,
        size,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        mem_fd,
        PERI_PHYS_BASE + addr);

    close(mem_fd);

    if (result == MAP_FAILED)
    {
        perror("mmap error: ");
        exit(-1);
    }
    return result;
}

void dma_alloc_buffers()
{
    dma_cbs = dma_malloc(CB_CNT * sizeof(DMAControlBlock));
    dma_ticks = dma_malloc(TICK_CNT * sizeof(uint32_t));
}

static inline DMAControlBlock *ith_cb_virt_addr(int i) { return (DMAControlBlock *)dma_cbs->virtual_addr + i; }

static inline uint32_t ith_cb_bus_addr(int i) { return dma_cbs->bus_addr + i * sizeof(DMAControlBlock); }

static inline uint32_t *ith_tick_virt_addr(int i) { return (uint32_t *)dma_ticks->virtual_addr + i; }

static inline uint32_t ith_tick_bus_addr(int i) { return dma_ticks->bus_addr + i * sizeof(uint32_t); }

void dma_init_cbs()
{
    DMAControlBlock *cb;
    for (int i = 0; i < TICK_CNT; i++)
    {
        // Tick block
        cb = ith_cb_virt_addr(2 * i);
        cb->tx_info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
        cb->src = PERI_BUS_BASE + SYST_BASE + SYST_CLO;
        cb->dest = ith_tick_bus_addr(i);
        cb->tx_len = 4;
        cb->next_cb = ith_cb_bus_addr(2 * i + 1);

        // Delay block
        cb = ith_cb_virt_addr(2 * i + 1);
        cb->tx_info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_DEST_DREQ | DMA_PERIPHERAL_MAPPING(5);
        cb->src = ith_cb_bus_addr(0); // Dummy data
        cb->dest = PERI_BUS_BASE + PWM_BASE + PWM_FIFO;
        cb->tx_len = 4;
        cb->next_cb = ith_cb_bus_addr((2 * i + 2) % CB_CNT);
    }
    fprintf(stderr, "Init: %d cbs, %d ticks\n", CB_CNT, TICK_CNT);
}

void init_hw_clk()
{
    // See Chanpter 6.3, BCM2835 ARM peripherals for controlling the hardware clock
    // Also check https://elinux.org/BCM2835_registers#CM for the register mapping

    // kill the clock if busy
    if (clk_reg->ctrl & CLK_CTL_BUSY)
    {
        do
        {
            clk_reg->ctrl = BCM_PASSWD | CLK_CTL_KILL;
        } while (clk_reg->ctrl & CLK_CTL_BUSY);
    }

    // Set clock source to plld
    clk_reg->ctrl = BCM_PASSWD | CLK_CTL_SRC(CLK_CTL_SRC_PLLD);
    usleep(10);

    // The original clock speed is 500MHZ, we divide it by 5 to get a 100MHZ clock
    clk_reg->div = BCM_PASSWD | CLK_DIV_DIVI(CLK_DIVI);
    usleep(10);

    // Enable the clock
    clk_reg->ctrl |= (BCM_PASSWD | CLK_CTL_ENAB);
}

void init_pwm()
{
    // reset PWM
    pwm_reg->ctrl = 0;
    usleep(10);
    pwm_reg->status = -1;
    usleep(10);

    /*
     * set number of bits to transmit
     * e.g, if CLK_MICROS is 5, since we have set the frequency of the
     * hardware clock to 100 MHZ, then the time taken for `100 * CLK_MICROS` bits
     * is (500 / 100) = 5 us, this is how we control the DMA sampling rate
     */
    pwm_reg->range1 = PWM_PERMICRO * CLK_MICROS;

    // enable PWM DMA, raise panic and dreq thresholds to 15
    pwm_reg->dma_cfg = PWM_DMAC_ENAB | PWM_DMAC_PANIC(15) | PWM_DMAC_DREQ(15);
    usleep(10);

    // clear PWM fifo
    pwm_reg->ctrl = PWM_CTL_CLRF1;
    usleep(10);

    // enable PWM channel 1 and use fifo
    pwm_reg->ctrl = PWM_CTL_USEF1 | PWM_CTL_MODE1 | PWM_CTL_PWEN1;
}

void dma_start()
{
    // Reset the DMA channel
    dma_reg->cs = DMA_CHANNEL_ABORT;
    dma_reg->cs = 0;
    dma_reg->cs = DMA_CHANNEL_RESET;
    dma_reg->cb_addr = 0;

    dma_reg->cs = DMA_INTERRUPT_STATUS | DMA_END_FLAG;

    // Make cb_addr point to the first DMA control block and enable DMA transfer
    dma_reg->cb_addr = ith_cb_bus_addr(0);
    dma_reg->cs = DMA_PRIORITY(8) | DMA_PANIC_PRIORITY(8) | DMA_DISDEBUG;
    dma_reg->cs |= DMA_WAIT_ON_WRITES | DMA_ACTIVE;
}

void dma_end()
{
    // Shutdown DMA channel, otherwise it won't stop after program exits
    dma_reg->cs |= DMA_CHANNEL_ABORT;
    usleep(100);
    dma_reg->cs &= ~DMA_ACTIVE;
    dma_reg->cs |= DMA_CHANNEL_RESET;
    usleep(100);

    // Release the memory used by DMA, otherwise the memory will be leaked after program exits
    dma_free(dma_ticks);
    dma_free(dma_cbs);

    free(dma_ticks);
    free(dma_cbs);
}
