/* -*- Mode: C; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/*
 * VMSocket PCI device.
 *
 * Author:
 *      Giuseppe Coviello <cjg@cruxppc.org>
 *
 * Based On: ivshmem.c
 *
 * This code is licensed under the GNU GPL v2.
 */

#include <sys/mman.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <poll.h>
#include "hw.h"
#include "pci.h"
#include "pc.h"
#include "qemu_socket.h"
#include "qemu-thread.h"
#include "memory.h"

#define PCI_COMMAND_IOACCESS                0x0001
#define PCI_COMMAND_MEMACCESS               0x0002
#define PCI_COMMAND_BUSMASTER               0x0004

#undef DEBUG_VMSOCKET
//#define DEBUG_VMSOCKET
#ifdef DEBUG_VMSOCKET
#define VMSOCKET_DPRINTF(fmt, args...)                  \
    do {printf("VMSOCKET: " fmt, ##args); } while (0)
#else
#define VMSOCKET_DPRINTF(fmt, args...)
#endif

/* Registers */
/* Read Only */
#define VMSOCKET_STATUS_L_REG       0x0
#define VMSOCKET_READ_END_L_REG     0x80
#define VMSOCKET_INTR_GET_L_REG     0xA0
/* Write Only */
#define VMSOCKET_CONNECT_W_REG      0x20
#define VMSOCKET_CLOSE_W_REG        0x30
#define VMSOCKET_WRITE_COMMIT_L_REG 0x40
#define VMSOCKET_READ_BEGIN_L_REG   0x60
#define VMSOCKET_INTR_SET_L_REG     0xC0
/* Buffer Size */
#define INBUF_SIZE 1024 * 1024 * 2
#define OUTBUF_SIZE 1024 * 1024 * 2
#define BUF_SIZE INBUF_SIZE + OUTBUF_SIZE
typedef struct VMSocketState {
    
    uint32_t regs_addr;
    MemoryRegion *regs;

    void *inbuffer;
    uint32_t inbuffer_offset;
    uint32_t inbuffer_size;
    MemoryRegion *in_mem_region;

    void *outbuffer;
    uint32_t outbuffer_offset;
    uint32_t outbuffer_size;
    MemoryRegion *out_mem_region;

    int fd;
    int status;
    int interrupt;

    pthread_mutex_t mutex;
    int count;
    int readed;

    PCIDevice *pci_dev;
} VMSocketState;

typedef struct PCI_VMSocketState {
    PCIDevice dev;
    VMSocketState vmsocket_state;
} PCI_VMSocketState;

char *vmsocket_device = NULL;
static GThreadPool *threadPool;

void vmsocket_init(const char * optarg) {
    //chardev = strdup(strchr(optarg, ':') + 1);
}

static void vmsocket_write(VMSocketState *s, uint32_t count)
{
    s->status = write(s->fd, s->outbuffer, count);
    fsync(s->fd);
#ifdef DEBUG_VMSOCKET
    perror("write");
#endif
    VMSOCKET_DPRINTF("Write request: count: %u status: %d\n", count,
                     s->status);
}

static void vmsocket_read_begin(gpointer arg, gpointer user_data )
{
    VMSocketState *s = (VMSocketState *) arg;
    pthread_mutex_lock(&s->mutex);
    s->readed = read(s->fd, s->inbuffer, s->count);
    VMSOCKET_DPRINTF("Read request: %u readed: %u\n", (unsigned) s->count,
                     (unsigned) s->readed);
    s->interrupt = 1;
    qemu_set_irq(s->pci_dev->irq[0], 1);
    //return NULL;
}

static void vmsocket_regs_writew(void *opaque, target_phys_addr_t addr,
                                 uint32_t val) {
    VMSocketState * s = opaque;
    //VMSOCKET_DPRINTF("sock file:%s\n",vmsocket_device);
    switch(addr & 0xFF) {
    case VMSOCKET_CONNECT_W_REG:
        s->fd = unix_connect(vmsocket_device);
        s->status = s->fd;
        VMSOCKET_DPRINTF("Connect. Status: %d.\n", s->status);
        break;
    case VMSOCKET_CLOSE_W_REG:
        s->status = close(s->fd);
        VMSOCKET_DPRINTF("Close Status: %d.\n", s->status);
        break;
    default:
        VMSOCKET_DPRINTF("writing word to invalid register 0x%x.",
                         (uint32_t) addr & 0xFF);
    }
}

static void vmsocket_regs_writel(void *opaque, target_phys_addr_t addr,
                                 uint32_t val) {
    VMSocketState * s = opaque;

    switch(addr & 0xFF) {
    case VMSOCKET_WRITE_COMMIT_L_REG:
        VMSOCKET_DPRINTF("WriteCommit: count: %u.\n", val);
        vmsocket_write(s, val);
        break;
    case VMSOCKET_READ_BEGIN_L_REG:
        pthread_mutex_lock(&s->mutex);
        s->interrupt = 0;
        s->readed = 0;
        s->count = val;
        VMSOCKET_DPRINTF("Read request: %u\n", (unsigned) s->count);
        g_thread_pool_push(threadPool, s, NULL);
        pthread_mutex_unlock(&s->mutex);//FIXME:not portable
        break;
    case VMSOCKET_INTR_SET_L_REG:
        s->interrupt = val;
        qemu_set_irq(s->pci_dev->irq[0], s->interrupt);
        break;
    default:
        VMSOCKET_DPRINTF("writing long to invalid register 0x%x.",
                         (uint32_t) addr & 0xFF);
    }
}

static uint32_t vmsocket_regs_readl(void *opaque, target_phys_addr_t addr)
{
    VMSocketState * s = opaque;
    switch (addr & 0xFF) {
    case VMSOCKET_STATUS_L_REG:
        return s->status;
    case VMSOCKET_READ_END_L_REG:
        s->interrupt = 0;
        qemu_set_irq(s->pci_dev->irq[0], 0);
        pthread_mutex_unlock(&s->mutex);//FIXME:not portable
        return s->readed;
    case VMSOCKET_INTR_GET_L_REG:
        return s->interrupt;
    }
    VMSOCKET_DPRINTF("reading long from invalid register 0x%x.\n",
                     (uint32_t) addr & 0xFF);
    return 0;
}

static const MemoryRegionOps regs_ops = {
    .old_mmio = {
        .write = {NULL, vmsocket_regs_writew, vmsocket_regs_writel},
        .read = { NULL, NULL, vmsocket_regs_readl },
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};


static int pci_vmsocket_initfn(PCIDevice *dev) 
{
    g_thread_init(NULL);
    threadPool = g_thread_pool_new(vmsocket_read_begin,
                        NULL,
                        -1,
                        FALSE,
                        NULL);

    PCI_VMSocketState *d = DO_UPCAST(PCI_VMSocketState, dev, dev);
    VMSocketState *s = &d->vmsocket_state;
    uint8_t *pci_conf = d->dev.config;

    s->pci_dev = &d->dev;
    
    VMSOCKET_DPRINTF("Enabled!\n");
    VMSOCKET_DPRINTF("s: %p, s->pci_dev: %p\n", s, s->pci_dev);    
    pci_conf[PCI_HEADER_TYPE] = PCI_HEADER_TYPE_NORMAL; // header_type
    pci_conf[PCI_INTERRUPT_PIN] = 1; // we are going to support interrupts

    s->regs = g_malloc(sizeof(MemoryRegion));
    memory_region_init_io(s->regs, &regs_ops, s, "vmsocket.regs", 0x100);
    

    /* I/O Buffers */
    s->inbuffer_size = INBUF_SIZE; /* FIXME: make it configurable */
    s->in_mem_region = g_malloc(sizeof(MemoryRegion));
    memory_region_init_ram(s->in_mem_region, "vmsocket.in", s->inbuffer_size);
    s->inbuffer = memory_region_get_ram_ptr(s->in_mem_region);
    vmstate_register_ram_global(s->in_mem_region);

    s->outbuffer_size = OUTBUF_SIZE; /* FIXME: make it configurable */
    s->out_mem_region = g_malloc(sizeof(MemoryRegion));    
    memory_region_init_ram(s->out_mem_region, "vmsocket.out", s->outbuffer_size);
    s->outbuffer = memory_region_get_ram_ptr(s->out_mem_region);
    vmstate_register_ram_global(s->out_mem_region);
    

    /* Regions */

    pci_register_bar(&d->dev, 0,  PCI_BASE_ADDRESS_SPACE_MEMORY,
                     s->regs);
    pci_register_bar(&d->dev, 1, PCI_BASE_ADDRESS_MEM_PREFETCH,
                     s->in_mem_region);
    pci_register_bar(&d->dev, 2, PCI_BASE_ADDRESS_MEM_PREFETCH,
                     s->out_mem_region);

    s->interrupt = 0;
    qemu_set_irq(s->pci_dev->irq[0], 0);

    s->count = 0;
    s->readed = 0;
    pthread_mutex_init(&s->mutex, NULL);
    return 0;
}

int vmsocket_get_buffer_size(void)
{
    return BUF_SIZE; // FIXME: make it configurable
}
DeviceState *pci_vmsocket_init(PCIBus *bus)
{
    return &pci_create_simple(bus, -1, "vmsocket")->qdev;
}

static Property vmsocket_pci_properties[] = {
    DEFINE_PROP_UINT32("vmsocket_in", PCI_VMSocketState, vmsocket_state.inbuffer_size, 2),
    DEFINE_PROP_UINT32("vmsocket_out", PCI_VMSocketState, vmsocket_state.outbuffer_size, 2),
    DEFINE_PROP_END_OF_LIST(),
};
static void pci_vmsocket_exit(PCIDevice *dev)
{
     PCI_VMSocketState *d = DO_UPCAST(PCI_VMSocketState, dev, dev);
     VMSocketState *s = &d->vmsocket_state;
     memory_region_destroy(s->regs);
     memory_region_destroy(s->in_mem_region);
     memory_region_destroy(s->out_mem_region);
     VMSOCKET_DPRINTF("exit");
}
static void vmsocket_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    /* PCI config */
    k->no_hotplug = 1;
    k->init = pci_vmsocket_initfn;
    k->exit = pci_vmsocket_exit;
    k->vendor_id = PCI_VENDOR_ID_REDHAT_QUMRANET;
    k->device_id = 0x6662;
    k->class_id = PCI_CLASS_OTHERS;
    //dc->vmsd = &vmstate_vga_pci;
    dc->props = vmsocket_pci_properties;
    dc->desc = "vmsocket";
    
}

static TypeInfo vmsocket_info = {
    .name          = "vmsocket",
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(PCI_VMSocketState),
    .class_init    = vmsocket_class_init,
};

static void vmsocket_register_types(void)
{
    type_register_static(&vmsocket_info);
}

type_init(vmsocket_register_types)
