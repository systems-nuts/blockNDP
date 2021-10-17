/*
 * QEMU NVM Express controller with another QEMU as a backend
 */

/* Original version of the QEMU NVM Express Controller
 * 
 * Copyright (c) 2012, Intel Corporation
 *
 * Written by Keith Busch <keith.busch@intel.com>
 *
 * This code is licensed under the GNU GPL v2 or later.
 */
/* This version of the QEMU NVM Express Controller
 * 
 * Copyright (c) 2017, Antonio Barbalace
 * 
 * This code is a prototype, therefore not all the possible functionalities and
 * commands are implemented. Additionally, some of the code must be refactored
 * and move out from this source -- such as the common code with the SSD specific
 * NVMe counterpart.
 */

/**
 * Reference Specs: http://www.nvmexpress.org, 1.2, 1.1, 1.0e
 *
 *  http://www.nvmexpress.org/resources/
 */
/**
 * Usage: add options:
 *  -chardev socket,path=<file_nvme>,nowait,id=<nvme_id> 
 *  -chardev socket,path=<file_dma>,nowait,id=<dma_id>
 *  -device nvme_ctrl,serial=<serial>,cmb_size_mb=<size>,chardev=<nvme_id>,dmadev=<dma_id> 
 *
 * Both chardev sockets are client sockets because the SSD is the server socket.
 * Note cmb_size_mb denotes size of CMB in MB. CMB is assumed to be at
 * offset 0 in BAR2 and supports only WDS, RDS and SQS for now.
 */

#include "qemu/osdep.h"
#include "hw/block/block.h"
#include "hw/hw.h"
#include "hw/pci/msix.h"
#include "hw/pci/pci.h"
#include "sysemu/sysemu.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "sysemu/block-backend.h"

#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/event_notifier.h"
#include "qom/object_interfaces.h"
#include "chardev/char-fe.h"

#include "nvme.h"

//#define DEBUGME
#ifdef DEBUGME
#define QEMU_LOG_MASK(severity,fmt, ...) \
do { qemu_log_mask(severity, fmt, ## __VA_ARGS__); } while (0)
#else
#define QEMU_LOG_MASK(severity, fmt, ...) \
do {} while (0)
#endif
//do { qemu_log_mask(severity, "nvme_ctrl: " fmt , ## __VA_ARGS__); } while (0) 
    
/* need to redefine some data structure here */
#define USE_SERVER_DRIVER

typedef struct NvmeCtrlRequest {
    struct NvmeCtrlSQueue       *sq;
    BlockAIOCB              *aiocb;
    uint64_t                prp2;
    uint16_t                status;
    long                    written;
    bool                    has_sg;
    NvmeCqe                 cqe;
    BlockAcctCookie         acct;
    QEMUSGList              qsg;
    QEMUIOVector            iov;
    QTAILQ_ENTRY(NvmeCtrlRequest)entry;
} NvmeCtrlRequest;

typedef struct NvmeCtrlSQueue {
    struct NvmeCtrlState *ctrl;
    uint16_t    sqid;
    uint16_t    cqid;
    uint32_t    head;
    uint32_t    tail;
    uint32_t    size;
    uint64_t    dma_addr;
    QEMUTimer   *timer;
    NvmeCtrlRequest *io_req;
    QTAILQ_HEAD(ctrl_sq_req_list, NvmeCtrlRequest) req_list;
    QTAILQ_HEAD(ctrl_out_req_list, NvmeCtrlRequest) out_req_list;
    QTAILQ_ENTRY(NvmeCtrlSQueue) entry;
} NvmeCtrlSQueue;

typedef struct NvmeCtrlCQueue {
    struct NvmeCtrlState *ctrl;
    uint8_t     phase;
    uint16_t    cqid;
    uint16_t    irq_enabled;
    uint32_t    head;
    uint32_t    tail;
    uint32_t    vector;
    uint32_t    size;
    uint64_t    dma_addr;
    QEMUTimer   *timer;
    QTAILQ_HEAD(ctrl_sq_list, NvmeCtrlSQueue) sq_list;
    QTAILQ_HEAD(ctrl_cq_req_list, NvmeCtrlRequest) req_list;
} NvmeCtrlCQueue;


// for DMA data structures
#define DMA_FLAG_TO_HOST  (0x1 << 3)
#define DMA_FLAG_TO_SSD   (0x1 << 4)
#define DMA_FLAG_DIRECT   (0x1 << 5)
#define DMA_FLAG_AUTO     (0x1 << 6)

#define DMA_DATA_SIZE       4096

/* packets do not include source but only destination */
/* packets do not include source but only destination */
typedef struct DMAPacket {
    uint64_t addr; // this is addr or prp1
    uint64_t prp2; // this is prp2 when it exists
    uint32_t offset4KB; // this is only used by the host
    uint32_t len;
    uint32_t sequence;
    uint32_t idx;
    uint32_t size; // this is only for DMA data stransfers
    uint16_t slot;
    uint16_t flags;  //includes packet directions
    char     data[]; //here goes the data
} DMAPacket;


#define DMA_PACKET_SIZE     (sizeof(DMAPacket) + DMA_DATA_SIZE)



#undef TYPE_NVME
#define TYPE_NVME_CTRL "nvme_ctrl"
#undef NVME
#define NVME_CTRL(obj) \
        OBJECT_CHECK(NvmeCtrlState, (obj), TYPE_NVME_CTRL)

typedef struct NvmeCtrlState {
    PCIDevice    parent_obj;
    MemoryRegion iomem;
    MemoryRegion ctrl_mem;
    NvmeBar      bar;
    //BlockConf    conf;

    CharBackend  server_chr;
    CharBackend  dma_chr;

    uint32_t    page_size;
    uint16_t    page_bits;
    uint16_t    max_prp_ents;
    uint16_t    cqe_size;
    uint16_t    sqe_size;
    uint32_t    reg_size;
    uint32_t    num_namespaces;
    uint32_t    num_queues;
    uint32_t    max_q_ents;
    uint64_t    ns_size;
    uint32_t    cmb_size_mb;
    uint32_t    cmbsz;
    uint32_t    cmbloc;
    uint8_t     *cmbuf;

    uint32_t    can_receiveDMA;
    DMAPacket   *bufferDMA;
    uint32_t    dma_pkt_idx;

    char            *serial;
    NvmeNamespace   *namespaces;
    NvmeCtrlSQueue      **sq;
    NvmeCtrlCQueue      **cq;
    NvmeCtrlSQueue      admin_sq;
    NvmeCtrlCQueue      admin_cq;
    NvmeIdCtrl      id_ctrl;
} NvmeCtrlState;

/*
 * Communication protocol between server and client
 * on the chr_fe we are sending 64 byte packets (submission queue size) from nvme_ctrl -> nvme_dev
 * and completion queue size messages on the other side
 * to distinguish between submission queues commands and emulated device commands
 * bytes 15-8 (uint64_t size) of the 64 byte to:
 *  - save the type of command
 *  - save the queue id
 */

enum NvmeCtrlPrivCommands {
    NVME_PRIV_CMD = 0xA5,
    NVME_SPEC_CMD = 0x96,
};

enum NvmeCtrlPrivCmd {
    NVME_PRIV_CMD_LINK_UP = 0xA6,
    NVME_PRIV_CMD_LINK_DOWN,
    NVME_PRIV_CMD_ENABLE,
    NVME_PRIV_CMD_DISABLE,

    NVME_PRIV_CMD_DMA_DONE,
    NVME_PRIV_CMD_DMA_ERROR,
    NVME_PRIV_CMD_DMA_REQ,
    NVME_PRIV_CMD_DDMA_DONE,
    NVME_PRIV_CMD_DDMA_ERROR,
    NVME_PRIV_CMD_DDMA_REQ,
};

typedef struct NvmeCmd_res1 {
    uint8_t cmd; //command type: internal, admin queue, submission queue
    uint8_t priv; //private commands
    uint16_t sqid; //submission queue id;
    uint32_t slot; //this is for auto DMA completions
} NvmeCmd_res1;


typedef struct NvmeCqe_rsvd {
    uint8_t cmd; //command type: internal, admin queue, IO queue
    uint8_t priv; //private command content
    uint16_t rsvd; //future use
} NvmeCqe_rsvd;


typedef struct NvmeCqe_RXDMA {
    uint8_t slot;
    uint8_t offset;
} NvmeCqe_RXDMA;


static void nvme_process_sq(void *opaque);

static void nvme_addr_read(NvmeCtrlState *n, hwaddr addr, void *buf, int size)
{
    if (n->cmbsz && addr >= n->ctrl_mem.addr &&
                addr < (n->ctrl_mem.addr + int128_get64(n->ctrl_mem.size))) {
        memcpy(buf, (void *)&n->cmbuf[addr - n->ctrl_mem.addr], size);
    } else {
        pci_dma_read(&n->parent_obj, addr, buf, size);
    }
}

static int nvme_check_sqid(NvmeCtrlState *n, uint16_t sqid)
{
    return sqid < n->num_queues && n->sq[sqid] != NULL ? 0 : -1;
}

static int nvme_check_cqid(NvmeCtrlState *n, uint16_t cqid)
{
    return cqid < n->num_queues && n->cq[cqid] != NULL ? 0 : -1;
}

static void nvme_inc_cq_tail(NvmeCtrlCQueue *cq)
{
    cq->tail++;
    if (cq->tail >= cq->size) {
        cq->tail = 0;
        cq->phase = !cq->phase;
    }
}

static void nvme_inc_sq_head(NvmeCtrlSQueue *sq)
{
    sq->head = (sq->head + 1) % sq->size;
}

static uint8_t nvme_cq_full(NvmeCtrlCQueue *cq)
{
    return (cq->tail + 1) % cq->size == cq->head;
}

static uint8_t nvme_sq_empty(NvmeCtrlSQueue *sq)
{
    return sq->head == sq->tail;
}

static void nvme_isr_notify(NvmeCtrlState *n, NvmeCtrlCQueue *cq)
{
    if (cq->irq_enabled) {
        if (msix_enabled(&(n->parent_obj))) {
            msix_notify(&(n->parent_obj), cq->vector);
        } else {
            pci_irq_pulse(&n->parent_obj);
        }
    }
}

static uint16_t nvme_map_prp(QEMUSGList *qsg, QEMUIOVector *iov, uint64_t prp1,
                             uint64_t prp2, uint32_t len, NvmeCtrlState *n)
{
    hwaddr trans_len = n->page_size - (prp1 % n->page_size);
    trans_len = MIN(len, trans_len);
    int num_prps = (len >> n->page_bits) + 1;

    if (!prp1) {
        return NVME_INVALID_FIELD | NVME_DNR;
// if requested prp1 address belongs to ctrl_mem USE qemu_iovec
    } else if (n->cmbsz &&
               prp1 >= n->ctrl_mem.addr &&
               prp1 < n->ctrl_mem.addr + int128_get64(n->ctrl_mem.size)) {
        qsg->nsg = 0;
        qemu_iovec_init(iov, num_prps);
        qemu_iovec_add(iov, (void *)&n->cmbuf[prp1 - n->ctrl_mem.addr], trans_len);
// othersiwe use pci_dma_sglist
    } else {
        pci_dma_sglist_init(qsg, &n->parent_obj, num_prps);
        qemu_sglist_add(qsg, prp1, trans_len);
    }

    len -= trans_len;
    if (len) {
        if (!prp2) {
            goto unmap;
        }
        if (len > n->page_size) {
            uint64_t prp_list[n->max_prp_ents];
            uint32_t nents, prp_trans;
            int i = 0;

            nents = (len + n->page_size - 1) >> n->page_bits;
            prp_trans = MIN(n->max_prp_ents, nents) * sizeof(uint64_t);
            nvme_addr_read(n, prp2, (void *)prp_list, prp_trans);
            while (len != 0) {
                uint64_t prp_ent = le64_to_cpu(prp_list[i]);

                if (i == n->max_prp_ents - 1 && len > n->page_size) {
                    if (!prp_ent || prp_ent & (n->page_size - 1)) {
                        goto unmap;
                    }

                    i = 0;
                    nents = (len + n->page_size - 1) >> n->page_bits;
                    prp_trans = MIN(n->max_prp_ents, nents) * sizeof(uint64_t);
                    nvme_addr_read(n, prp_ent, (void *)prp_list,
                        prp_trans);
                    prp_ent = le64_to_cpu(prp_list[i]);
                }

                if (!prp_ent || prp_ent & (n->page_size - 1)) {
                    goto unmap;
                }

                trans_len = MIN(len, n->page_size);
                if (qsg->nsg){
                    qemu_sglist_add(qsg, prp_ent, trans_len);
                } else {
                    qemu_iovec_add(iov, (void *)&n->cmbuf[prp_ent - n->ctrl_mem.addr], trans_len);
                }
                len -= trans_len;
                i++;
            }
        } else {
            if (prp2 & (n->page_size - 1)) {
                goto unmap;
            }
            if (qsg->nsg) {
                qemu_sglist_add(qsg, prp2, len);
            } else {
                qemu_iovec_add(iov, (void *)&n->cmbuf[prp2 - n->ctrl_mem.addr], trans_len);
            }
        }
    }
    return NVME_SUCCESS;

 unmap:
    qemu_sglist_destroy(qsg);
    return NVME_INVALID_FIELD | NVME_DNR;
}

#ifndef USE_SERVER_DRIVER
static uint16_t nvme_dma_read_prp(NvmeCtrlState *n, uint8_t *ptr, uint32_t len,
    uint64_t prp1, uint64_t prp2)
{
    QEMUSGList qsg;
    QEMUIOVector iov;
    uint16_t status = NVME_SUCCESS;

    if (nvme_map_prp(&qsg, &iov, prp1, prp2, len, n)) {
        return NVME_INVALID_FIELD | NVME_DNR;
    }
    if (qsg.nsg > 0) {
        if (dma_buf_read(ptr, len, &qsg)) {
            status = NVME_INVALID_FIELD | NVME_DNR;
        }
        qemu_sglist_destroy(&qsg);
    } else {
        if (qemu_iovec_to_buf(&iov, 0, ptr, len) != len) {
            status = NVME_INVALID_FIELD | NVME_DNR;
        }
        qemu_iovec_destroy(&iov);
    }
    return status;
}
#endif

static uint16_t cqe_addr_hash (uint64_t value) {
#if 0    
    return ((value >> 16) & 0xFFFF);
#else
    unsigned long umask, _umask, res, _res;
    unsigned short _ret = 0;
    int i;

    umask = 0xAAAAAAAAAAAAAAAA; // the other mask is 0x55.. == ~umask
    res = ((value & umask)>>1) ^ (value & ~umask); // value was 64 bit now is 32 bit
    _umask = 0x1111111111111111; // the other mask is 0x44.. == (~umask ^ _umask)
    _res = ((res & (~umask ^ _umask))>>2) ^ (res & _umask);
    for (i=0; i<16; i++)
        _ret |= ((_res >> (i*4)) << i);
    
    return _ret;
#endif
}

static int nvme_ctrl_can_receive(void *opaque)
{
    return sizeof(NvmeCqe);
}

static uint64_t dma_buf_rw_full(char *ptr, int32_t len,
                                QEMUSGList *sg, int32_t offset,
                                DMADirection dir);
static void nvme_enqueue_req_completion(NvmeCtrlCQueue *cq, NvmeCtrlRequest *req);
static void nvme_ctrl_receive(void *opaque, const uint8_t *buf, int size)
{
    NvmeCtrlState *n = opaque;
    NvmeCqe nvme_compl;
    memcpy((char *) &nvme_compl, buf, (size > sizeof(NvmeCqe)) ? sizeof(NvmeCqe) : size);

    QEMU_LOG_MASK(LOG_TRACE,"receive %d [[%d]]\n", size, (unsigned int)sizeof(NvmeCqe));

    switch ( ((NvmeCqe_rsvd *) &nvme_compl.rsvd)->cmd ) {
        case NVME_PRIV_CMD: {
            switch ( ((NvmeCqe_rsvd *) &nvme_compl.rsvd)->priv ) {
                case NVME_PRIV_CMD_DMA_REQ: {
                    uint8_t slot = ((NvmeCqe_RXDMA *) &((NvmeCqe_rsvd *) &nvme_compl.rsvd)->rsvd)->slot;
                    uint8_t offset = ((NvmeCqe_RXDMA *) &((NvmeCqe_rsvd *) &nvme_compl.rsvd)->rsvd)->offset;
                    uint32_t devAddr = nvme_compl.result;
                    uint64_t prp1 = *((uint64_t *) &nvme_compl.sq_head), prp2 =~0;
                    int data_size = -1;
                    int i =0;

                    NvmeCtrlRequest *req, *next;
                    NvmeCtrlSQueue *sq;
                                
                    while ( ( sq = n->sq[i++] ) ) { // assume that the entries are zeroed! :-)
                        //try to find the corresponding request, using prp1 
                        QTAILQ_FOREACH_SAFE(req, &sq->out_req_list, entry, next) {
                            if ( (uint64_t)req->aiocb == prp1 ) {
                                //found we can extract the prp2 (if existent)
                                prp2 = req->prp2;
                            
                                QEMU_LOG_MASK(LOG_TRACE,
                                    "SQ%d(%d): (DMA RX) pairing with prp1:0x%lx prp2:0x%lx\n",
                                    i, sq->sqid, (unsigned long)req->aiocb, prp2);
                                // TODO we can try to search for duplicates here to make sure everything is correct
                                goto req_found; 
                            } 
                        }
                    }
                    if (prp2 == ~0) {
                        error_report("ERROR cannot find element with prp1:0x%lx in any queue CRITICAL error\n", prp1);
                        return;
                    }
req_found:
                    // the following is experimental, it is maybe the case that we need to move the information about the max num of pages available also on this side
                    data_size= (offset + 1) << 12; // TODO instead of 12 need to put a variable 

                    // allocate the memory for the DMApacket
                    DMAPacket * pkt = g_malloc0(sizeof(DMAPacket) + DMA_DATA_SIZE);
                    pkt->addr = (uint64_t)devAddr;
                    pkt->prp2 = prp2;
                    pkt->offset4KB = offset;
                    pkt->len = (1 << 12);
                    pkt->sequence = 0;
                    pkt->idx = n->dma_pkt_idx++;
                    pkt->size = 1;
                    pkt->slot = slot;
                    pkt->flags = DMA_FLAG_AUTO | DMA_FLAG_TO_SSD;

                    //now that the prp2 has been found and this is also a valid request we can check the scatter-gather list
                    QEMUSGList qsg;
                    QEMUIOVector iov;
        


                    //if prp2 exists it is necessary to go search for a prp list
                    if (prp2) {
                        // create a scatter gather list from prp1 and prp2 first
                        uint16_t _ret;

                        if ( (_ret = nvme_map_prp(&qsg, &iov, prp1, prp2, data_size, n)) != NVME_SUCCESS) { 
                            error_report("ERROR DMA RX cannot map the PRPs prp1:0x%lx prp2:0x%lx ret:%d\n",
                                    prp1, prp2, (unsigned int)_ret);
                            return;
                        }
                        if ( qsg.nsg == 0 ) {
                            error_report("ERROR DMA RX not able to get an SGList -- iovec case not handled [prp1:0x%lx prp2:0x%lx]\n",
                                    prp1, prp2);
                            return;
                        }  
                        QEMU_LOG_MASK(LOG_TRACE,
                                      "DMA RX (on host) ready to dma_buf_rw_full [prp1:0x%lx prp2:0x%lx] slot:%d offset:%d \n",
                                      prp1, prp2, slot, offset);
 
                        // read the data in the buffer                                        
                        dma_buf_rw_full(&(pkt->data[0]), DMA_DATA_SIZE, &qsg, (offset *4096), DMA_DIRECTION_TO_DEVICE);
                    }
                    else {
                        cpu_physical_memory_read( prp1 + (offset *4096), &(pkt->data[0]), DMA_DATA_SIZE);
                    }

                    // then write this to the SSD
                    qemu_chr_fe_write_all(&n->dma_chr, (const uint8_t * )pkt, (sizeof(DMAPacket) + DMA_DATA_SIZE) );        // release all resources
                    g_free (pkt); 
                    
                    QEMU_LOG_MASK(LOG_TRACE,
                                      "DMA RX (on host) SENDING DMA PACKET to SSD [prp1:0x%lx prp2:0x%lx] slot:%d offset:%d \n",
                                      prp1, prp2, slot, offset);
 
                    break;
                }
                default:
                    warn_report("WARN private command 0x%x not supported jet\n",
                        ((NvmeCqe_rsvd *) &nvme_compl.rsvd)->priv );
            }
            break;
        }
        case NVME_SPEC_CMD: {
             QEMU_LOG_MASK(LOG_TRACE,
                "result:0x%x sq_head:%d sq_id:%d cid:%d status:0x%x\n",
                nvme_compl.result, (unsigned int)nvme_compl.sq_head, (unsigned int) nvme_compl.sq_id,
                (unsigned int) nvme_compl.cid, (unsigned int)nvme_compl.status);

//#ifdef USE_SERVER_DRIVER
            NvmeCtrlRequest *req, *next;
            NvmeCtrlCQueue *cq;
            NvmeCtrlSQueue *sq = n->sq[nvme_compl.sq_id]; //let's get the right queue based on the id

            QEMU_LOG_MASK(LOG_TRACE,
                          "found sq_id %d for cid %d\n", sq->sqid, nvme_compl.cid);

            //let's try to find the corresponding request, using CID and the hash
            QTAILQ_FOREACH_SAFE(req, &sq->out_req_list, entry, next) {
                if ( (req->cqe.cid == nvme_compl.cid) &&
                    cqe_addr_hash((uint64_t)req->aiocb) == ((NvmeCqe_rsvd *) &nvme_compl.rsvd)->rsvd) {

                    cq = n->cq[req->sq->cqid];
                    //found we can enqueue a completion
                    QEMU_LOG_MASK(LOG_TRACE,
                        "cq->phase:%d pairing with 0x%lx hash 0x%hx (sqid %d cqid %d)\n",
                        (int)cq->phase, (unsigned long)req->aiocb, cqe_addr_hash((unsigned long)req->aiocb),
                                  sq->sqid, cq->cqid);

                    req->status = nvme_compl.status;
                    req->cqe.result = le64_to_cpu(nvme_compl.result);

                    // this is just enqueuing the completion, nvme_post_cqes() is publishing them
                    nvme_enqueue_req_completion(cq, req);
                    return;
                } else
                    QEMU_LOG_MASK(LOG_TRACE,
                          "FOREACH found sq_id %d current cid %d\n", sq->sqid, req->cqe.cid);
            }
            // here is the not found condition
            error_report("ERROR cannot pair completion from the server empty %d\n",
                          (int) QTAILQ_EMPTY(&sq->out_req_list) );
//#endif

            break;
        }
        default:
                 warn_report("command not supported jet\n");
    }

    return;
}

static void nvme_ctrl_event(void *opaque, int event)
{
    //NVMe_DEVState *s = opaque;
    //TODO implement event

     QEMU_LOG_MASK(LOG_TRACE,"event %x\n", event);
/*    if (event == CHR_EVENT_BREAK)
        serial_receive_break(s);*/
}

// DMA packets ************************************************************************

static int nvme_ctrl_can_receiveDMA(void *opaque)
{
    NvmeCtrlState *n = opaque;

    return n->can_receiveDMA;
}

/*
 * This code should be moved to dma.* or dma-helpers.*
 * For now it is here to speed up development
 */
//static uint64_t dma_buf_rw_full(uint8_t *ptr, int32_t len,
static uint64_t dma_buf_rw_full(char *ptr, int32_t len,
                                QEMUSGList *sg, int32_t offset,
                                DMADirection dir)
{
    uint64_t resid;
    int sg_cur_index;

    resid = sg->size; // total scatter gather size
    if (offset > resid)
        //there is an error
        return len;

    sg_cur_index = 0;
    len = MIN(len, (resid - offset));

    while (len > 0) { // until we have space in the buffer or in the scatter-gather list we copy
        ScatterGatherEntry entry = sg->sg[sg_cur_index++];
        int32_t xfer = MIN(len, entry.len);
        unsigned long sg_base_addr = entry.base;
        if ( (xfer - offset) <= 0) {
            offset -= xfer;
            continue;
        }
        else if ( !offset ) {
            xfer -= offset;
            sg_base_addr += offset;
            offset = 0;
        }

        dma_memory_rw(sg->as, sg_base_addr, ptr, xfer, dir);
        ptr += xfer;
        len -= xfer;
        resid -= xfer;
    }

    return resid;
}


static void nvme_ctrl_receiveDMA(void *opaque, const uint8_t *buf, int size)
{
    NvmeCtrlState *n = opaque;
    int size_to_copy = (size > n->can_receiveDMA) ? n->can_receiveDMA : size;

    // buffering first (in case the message is not completely transferred
    memcpy( ((char *) n->bufferDMA) + (DMA_PACKET_SIZE - n->can_receiveDMA), buf, size_to_copy);
    n->can_receiveDMA -= size_to_copy;
    if (n->can_receiveDMA)
        // more data need to be buffered, cannot process the message, just return
        return;

    QEMU_LOG_MASK(LOG_TRACE,"DMA receive %d [[%d]] %s addr:0x%lx (0x%lx) prp2:%lx off:%x len:%d seq:%d idx:%d flags:0x%x slot:%d\n",
                  size, (int)DMA_PACKET_SIZE, (n->bufferDMA->flags & DMA_FLAG_DIRECT) ? "DIRECT" : "AUTO",
                  n->bufferDMA->addr, le64_to_cpu(n->bufferDMA->addr), n->bufferDMA->prp2, n->bufferDMA->offset4KB,
                  n->bufferDMA->len, n->bufferDMA->sequence, n->bufferDMA->idx, n->bufferDMA->flags, (int)n->bufferDMA->slot);

    if ( (n->bufferDMA->flags & DMA_FLAG_TO_SSD) ) { 
        error_report("ERROR received a DMA packet for SSD but this is HOST code\n");
        goto exit_error;
    }
    
    if ( (n->bufferDMA->flags & DMA_FLAG_DIRECT) ) {
        // just write to the cpu memory the buffer, we have all the infos to do that
        uint64_t offset = (n->bufferDMA->sequence * DMA_DATA_SIZE); // this is also wroooong TODO 
        uint64_t size = ((n->bufferDMA->len - offset) < DMA_DATA_SIZE) ?
                        (n->bufferDMA->len - offset) : DMA_DATA_SIZE; // occhio maybe this is the problem TODO                        
                        
        /*qemu_log_mask(LOG_TRACE, "DMA receive hwaddr 0x%lx addr 0x%lx size %ld\n",
                      ((unsigned long)le64_to_cpu(n->bufferDMA->addr)) + offset,
                                    (unsigned long)&(n->bufferDMA->data[0]), size);*/
        cpu_physical_memory_write( ((unsigned long)le64_to_cpu(n->bufferDMA->addr)) + offset,
                                    &(n->bufferDMA->data[0]), size);
        // Alternative we can try the following code
        // before we are trying to make also direct writings synch
/*        QEMUSGList qsg;
        QEMUIOVector iov;
        int _ret;
        if ( (_ret = nvme_map_prp(&qsg, &iov, n->bufferDMA->addr, 0, 4096, n)) != NVME_SUCCESS) {
            // TODO here write
        }
        else {
            qemu_log_mask("again un-succesfull\n");
        }*/

        //if this is the last packet of an idx send a message of completion
        if ( n->bufferDMA->len <= (offset + DMA_DATA_SIZE) ) {
            NvmeCmd done_command;
            memset(&done_command, 0, sizeof(NvmeCmd));
            ((NvmeCmd_res1 *) &done_command.res1)->cmd = NVME_PRIV_CMD;
            ((NvmeCmd_res1 *) &done_command.res1)->priv = NVME_PRIV_CMD_DDMA_DONE;
            ((NvmeCmd_res1 *) &done_command.res1)->slot = -1;

            qemu_chr_fe_write_all(&n->server_chr, (const uint8_t * )&done_command,sizeof(NvmeCmd));
            
            qemu_log_mask(LOG_TRACE,"triggering DMA DIRECT completion for slot %d (len:%d off:%d DMA:%d)\n",
                          n->bufferDMA->slot,
                          (int)n->bufferDMA->len, (int)offset, (int) DMA_PACKET_SIZE );
        }
    }
    else if ( (n->bufferDMA->flags & DMA_FLAG_AUTO) ) {
        QEMUSGList qsg;
        QEMUIOVector iov;
        
        NvmeNamespace * ns = &n->namespaces[0]; // TODO, this obviously works only for one namespace! 
        const uint8_t lba_index = NVME_ID_NS_FLBAS_INDEX(ns->id_ns.flbas);
        const uint8_t data_shift = ns->id_ns.lbaf[lba_index].ds;
        //uint64_t data_size = n->bufferDMA->size << data_shift;
        uint64_t data_size = n->bufferDMA->size << 12; /// TODO 12 is correct but it must be fetched from the device state as well as integrating this in the front end for the host NVMe controller

        uint64_t prp1 = le64_to_cpu(n->bufferDMA->addr);
        uint64_t prp2 = le64_to_cpu(n->bufferDMA->prp2);
        uint64_t offset;

        //if prp2 exists it is necessary to go search for a prp list
        if (prp2) {
            // create a scatter gather list from prp1 and prp2 first
            uint16_t _ret;

            if ( (_ret = nvme_map_prp(&qsg, &iov, prp1, prp2, data_size, n)) != NVME_SUCCESS) { 
                error_report("ERROR DMA cannot map the PRPs prp1:0x%lx prp2:0x%lx ret:%x data_shift:%d data_size:%ld page_size:%d page_bits:%d\n",
                    prp1, prp2, (unsigned int)_ret, (int)data_shift, data_size, 
                              n->page_size, n->page_bits);
                return;
            }
            if ( qsg.nsg == 0 ) {
                error_report("ERROR DMA not able to get an SGList -- iovec case not handled [prp1:0x%lx prp2:0x%lx data_shift:%d data_size:%ld]\n",
                    prp1, prp2, (int) data_shift, data_size);
                return;
            }

            // this is very similar to the above but takes into consideration also the above
            offset = (n->bufferDMA->sequence * DMA_DATA_SIZE);
            uint64_t size = ((n->bufferDMA->len - offset) < DMA_DATA_SIZE) ?
                            (n->bufferDMA->len - offset) : DMA_DATA_SIZE;
            dma_buf_rw_full(&(n->bufferDMA->data[0]), size,
                            &qsg, ((4096 * n->bufferDMA->offset4KB) + offset),
                            DMA_DIRECTION_FROM_DEVICE);
        }
        // if prp2 is 0 there is no need to go search for prp list and construct the scatter gather list
        // this is similar to direct case but requires to use the 4KB offsets
        else {
            // just write to the cpu memory the buffer, we have all the infos to do that
            offset = (n->bufferDMA->sequence * DMA_DATA_SIZE);
            uint64_t size = ((n->bufferDMA->len - offset) < DMA_DATA_SIZE) ?
                        (n->bufferDMA->len - offset) : DMA_DATA_SIZE;

            cpu_physical_memory_write( prp1 + ((4096 * n->bufferDMA->offset4KB) + offset),
                                    &(n->bufferDMA->data[0]), size);
        }

/*        
        //if this is the last packet of an idx send a message of completion
        if ( (!prp2 && ((n->bufferDMA->len - offset) < DMA_PACKET_SIZE)) ||   //case prp2 == 0
             (prp2 &&                                                         //case prp2 != 0
                ((n->bufferDMA->len - offset) < DMA_PACKET_SIZE) &&
                (!qsg.size && (qsg.size == (n->bufferDMA->offset4KB +1)* 4096)) ) ) { */

// NOTE we are always sending a completion, because this is incrementing the DMA Tx counters, on the other side it is decided if the transfer is completely dones
        if ( n->bufferDMA->len <= (offset + DMA_DATA_SIZE) ) {
            NvmeCmd done_command;
            memset(&done_command, 0, sizeof(NvmeCmd));
            ((NvmeCmd_res1 *) &done_command.res1)->cmd = NVME_PRIV_CMD;
            ((NvmeCmd_res1 *) &done_command.res1)->priv = NVME_PRIV_CMD_DMA_DONE;
            ((NvmeCmd_res1 *) &done_command.res1)->slot = n->bufferDMA->slot;

            qemu_chr_fe_write_all(&n->server_chr, (const uint8_t * )&done_command,sizeof(NvmeCmd));

            QEMU_LOG_MASK(LOG_TRACE,"triggering DMA completion for slot %d [prp1:0x%lx prp2:0x%lx] (len:%d off:%d DMA:%d - data_shift:%d data_size:%d)\n",
                          n->bufferDMA->slot, prp1, prp2,
                          (int)n->bufferDMA->len, (int)offset, (int) DMA_PACKET_SIZE, (int)data_shift, (int)data_size );
        }
        
    }

exit_error:
    n->can_receiveDMA = DMA_PACKET_SIZE;
    return;
}

static void nvme_ctrl_eventDMA(void *opaque, int event)
{
    //NVMe_DEVState *s = opaque;
    //TODO implement event    

     QEMU_LOG_MASK(LOG_TRACE,"DMA event %x\n", event);
/*    if (event == CHR_EVENT_BREAK)
        serial_receive_break(s);*/
}








static void nvme_post_cqes(void *opaque)
{
    NvmeCtrlCQueue *cq = opaque;
    NvmeCtrlState *n = cq->ctrl;
    NvmeCtrlRequest *req, *next;

    QTAILQ_FOREACH_SAFE(req, &cq->req_list, entry, next) {
        NvmeCtrlSQueue *sq;
        hwaddr addr;

        if (nvme_cq_full(cq)) {
            break;
        }

        QTAILQ_REMOVE(&cq->req_list, req, entry);
        sq = req->sq;
        req->cqe.status = cpu_to_le16((req->status << 1) | cq->phase);
        req->cqe.sq_id = cpu_to_le16(sq->sqid);
        req->cqe.sq_head = cpu_to_le16(sq->head);
        //req->cqe.cid = cpu_to_le16(req->cid);

        if (req->status != 0)
            error_report("ERROR status %x, tail:%d size:%d sq_id:%d cid:%d phase:%d\n",
                          req->status, cq->tail, n->cqe_size, sq->sqid, req->cqe.cid, cq->phase);
        
        addr = cq->dma_addr + cq->tail * n->cqe_size;
        nvme_inc_cq_tail(cq); // increment with return (not update the hardware)
        pci_dma_write(&n->parent_obj, addr, (void *)&req->cqe,
            sizeof(req->cqe));
        QTAILQ_INSERT_TAIL(&sq->req_list, req, entry); // TODO why is this correct?
    }
    nvme_isr_notify(n, cq);
}

static void nvme_enqueue_req_completion(NvmeCtrlCQueue *cq, NvmeCtrlRequest *req)
{
    assert(cq->cqid == req->sq->cqid);
    QTAILQ_REMOVE(&req->sq->out_req_list, req, entry);
    QTAILQ_INSERT_TAIL(&cq->req_list, req, entry);
    timer_mod(cq->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 500);
}

#ifndef USE_SERVER_DRIVER
static void nvme_rw_cb(void *opaque, int ret)
{
    NvmeCtrlRequest *req = opaque;
    NvmeCtrlSQueue *sq = req->sq;
    NvmeCtrlState *n = sq->ctrl;
    NvmeCtrlCQueue *cq = n->cq[sq->cqid];

    if (!ret) {
        block_acct_done(blk_get_stats(n->conf.blk), &req->acct);
        req->status = NVME_SUCCESS;
    } else {
        block_acct_failed(blk_get_stats(n->conf.blk), &req->acct);
        req->status = NVME_INTERNAL_DEV_ERROR;
    }

    if (req->has_sg) {
        qemu_sglist_destroy(&req->qsg);
    }
    nvme_enqueue_req_completion(cq, req);
}

static uint16_t nvme_flush(NvmeCtrlState *n, NvmeNamespace *ns, NvmeCmd *cmd,
    NvmeCtrlRequest *req)
{
    req->has_sg = false;
    block_acct_start(blk_get_stats(n->conf.blk), &req->acct, 0,
         BLOCK_ACCT_FLUSH);
    req->aiocb = blk_aio_flush(n->conf.blk, nvme_rw_cb, req);

    return NVME_NO_COMPLETE;
}

static uint16_t nvme_write_zeros(NvmeCtrlState *n, NvmeNamespace *ns, NvmeCmd *cmd,
    NvmeCtrlRequest *req)
{
    NvmeRwCmd *rw = (NvmeRwCmd *)cmd;
    const uint8_t lba_index = NVME_ID_NS_FLBAS_INDEX(ns->id_ns.flbas);
    const uint8_t data_shift = ns->id_ns.lbaf[lba_index].ds;
    uint64_t slba = le64_to_cpu(rw->slba);
    uint32_t nlb  = le16_to_cpu(rw->nlb) + 1;
    uint64_t aio_slba = slba << (data_shift - BDRV_SECTOR_BITS);
    uint32_t aio_nlb = nlb << (data_shift - BDRV_SECTOR_BITS);

    if (slba + nlb > ns->id_ns.nsze) {
        return NVME_LBA_RANGE | NVME_DNR;
    }

    req->has_sg = false;
    block_acct_start(blk_get_stats(n->conf.blk), &req->acct, 0,
                     BLOCK_ACCT_WRITE);
    req->aiocb = blk_aio_pwrite_zeroes(n->conf.blk, aio_slba, aio_nlb,
                                        BDRV_REQ_MAY_UNMAP, nvme_rw_cb, req);
    return NVME_NO_COMPLETE;
}

static uint16_t nvme_rw(NvmeCtrlState *n, NvmeNamespace *ns, NvmeCmd *cmd,
    NvmeCtrlRequest *req)
{
    NvmeRwCmd *rw = (NvmeRwCmd *)cmd;
    uint32_t nlb  = le32_to_cpu(rw->nlb) + 1;
    uint64_t slba = le64_to_cpu(rw->slba);
    uint64_t prp1 = le64_to_cpu(rw->prp1);
    uint64_t prp2 = le64_to_cpu(rw->prp2);

    uint8_t lba_index  = NVME_ID_NS_FLBAS_INDEX(ns->id_ns.flbas);
    uint8_t data_shift = ns->id_ns.lbaf[lba_index].ds;
    uint64_t data_size = (uint64_t)nlb << data_shift;
    uint64_t data_offset = slba << data_shift;
    int is_write = rw->opcode == NVME_CMD_WRITE ? 1 : 0;
    enum BlockAcctType acct = is_write ? BLOCK_ACCT_WRITE : BLOCK_ACCT_READ;

    if ((slba + nlb) > ns->id_ns.nsze) {
        block_acct_invalid(blk_get_stats(n->conf.blk), acct);
        return NVME_LBA_RANGE | NVME_DNR;
    }

    if (nvme_map_prp(&req->qsg, &req->iov, prp1, prp2, data_size, n)) {
        block_acct_invalid(blk_get_stats(n->conf.blk), acct);
        return NVME_INVALID_FIELD | NVME_DNR;
    }

    dma_acct_start(n->conf.blk, &req->acct, &req->qsg, acct);
    if (req->qsg.nsg > 0) {
        req->has_sg = true;
        req->aiocb = is_write ?
            dma_blk_write(n->conf.blk, &req->qsg, data_offset, BDRV_SECTOR_SIZE,
                          nvme_rw_cb, req) :
            dma_blk_read(n->conf.blk, &req->qsg, data_offset, BDRV_SECTOR_SIZE,
                         nvme_rw_cb, req);
    } else {
        req->has_sg = false;
        req->aiocb = is_write ?
            blk_aio_pwritev(n->conf.blk, data_offset, &req->iov, 0, nvme_rw_cb,
                            req) :
            blk_aio_preadv(n->conf.blk, data_offset, &req->iov, 0, nvme_rw_cb,
                           req);
    }

    return NVME_NO_COMPLETE;
}

static uint16_t nvme_io_cmd(NvmeCtrlState *n, NvmeCmd *cmd, NvmeCtrlRequest *req)
{
    NvmeNamespace *ns;
    uint32_t nsid = le32_to_cpu(cmd->nsid);

    if (nsid == 0 || nsid > n->num_namespaces) {
        return NVME_INVALID_NSID | NVME_DNR;
    }

    ns = &n->namespaces[nsid - 1];
    switch (cmd->opcode) {
    case NVME_CMD_FLUSH:
        return nvme_flush(n, ns, cmd, req);
    case NVME_CMD_WRITE_ZEROS:
        return nvme_write_zeros(n, ns, cmd, req); // TODO NOT SUPPORTED BY THE OpenSSD board
    case NVME_CMD_WRITE:
    case NVME_CMD_READ:
        return nvme_rw(n, ns, cmd, req);
    default:
        return NVME_INVALID_OPCODE | NVME_DNR;
    }
}
#endif

static void nvme_free_sq(NvmeCtrlSQueue *sq, NvmeCtrlState *n)
{
    n->sq[sq->sqid] = NULL;
    timer_del(sq->timer);
    timer_free(sq->timer);
    g_free(sq->io_req);
    if (sq->sqid) {
        g_free(sq);
    }
}

static uint16_t nvme_del_sq(NvmeCtrlState *n, NvmeCmd *cmd)
{
    NvmeDeleteQ *c = (NvmeDeleteQ *)cmd;
    NvmeCtrlRequest *req, *next;
    NvmeCtrlSQueue *sq;
    NvmeCtrlCQueue *cq;
    uint16_t qid = le16_to_cpu(c->qid);

    if (!qid || nvme_check_sqid(n, qid)) {
        return NVME_INVALID_QID | NVME_DNR;
    }

    sq = n->sq[qid];
    while (!QTAILQ_EMPTY(&sq->out_req_list)) {
        req = QTAILQ_FIRST(&sq->out_req_list);
#ifndef USE_SERVER_DRIVER
        assert(req->aiocb);
        blk_aio_cancel(req->aiocb);
#endif
    }
    if (!nvme_check_cqid(n, sq->cqid)) {
        cq = n->cq[sq->cqid];
        QTAILQ_REMOVE(&cq->sq_list, sq, entry);

        nvme_post_cqes(cq);
        QTAILQ_FOREACH_SAFE(req, &cq->req_list, entry, next) {
            if (req->sq == sq) {
                QTAILQ_REMOVE(&cq->req_list, req, entry);
                QTAILQ_INSERT_TAIL(&sq->req_list, req, entry);
            }
        }
    }

    nvme_free_sq(sq, n);
    return NVME_SUCCESS;
}

static void nvme_init_sq(NvmeCtrlSQueue *sq, NvmeCtrlState *n, uint64_t dma_addr,
    uint16_t sqid, uint16_t cqid, uint16_t size)
{
    int i;
    NvmeCtrlCQueue *cq;

    sq->ctrl = n;
    sq->dma_addr = dma_addr;
    sq->sqid = sqid;
    sq->size = size;
    sq->cqid = cqid;
    sq->head = sq->tail = 0;
    sq->io_req = g_new(NvmeCtrlRequest, sq->size);

    QTAILQ_INIT(&sq->req_list);
    QTAILQ_INIT(&sq->out_req_list);
    for (i = 0; i < sq->size; i++) {
        sq->io_req[i].sq = sq;
        QTAILQ_INSERT_TAIL(&(sq->req_list), &sq->io_req[i], entry);
    }
    sq->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, nvme_process_sq, sq);

    assert(n->cq[cqid]);
    cq = n->cq[cqid];
    QTAILQ_INSERT_TAIL(&(cq->sq_list), sq, entry);
    n->sq[sqid] = sq;
}

static uint16_t nvme_create_sq(NvmeCtrlState *n, NvmeCmd *cmd)
{
    NvmeCtrlSQueue *sq;
    NvmeCreateSq *c = (NvmeCreateSq *)cmd;

    uint16_t cqid = le16_to_cpu(c->cqid);
    uint16_t sqid = le16_to_cpu(c->sqid);
    uint16_t qsize = le16_to_cpu(c->qsize);
    uint16_t qflags = le16_to_cpu(c->sq_flags);
    uint64_t prp1 = le64_to_cpu(c->prp1);

    if (!cqid || nvme_check_cqid(n, cqid)) {
        return NVME_INVALID_CQID | NVME_DNR;
    }
    if (!sqid || !nvme_check_sqid(n, sqid)) {
        return NVME_INVALID_QID | NVME_DNR;
    }
    if (!qsize || qsize > NVME_CAP_MQES(n->bar.cap)) {
        return NVME_MAX_QSIZE_EXCEEDED | NVME_DNR;
    }
    if (!prp1 || prp1 & (n->page_size - 1)) {
        return NVME_INVALID_FIELD | NVME_DNR;
    }
    if (!(NVME_SQ_FLAGS_PC(qflags))) {
        return NVME_INVALID_FIELD | NVME_DNR;
    }
    sq = g_malloc0(sizeof(*sq));
    nvme_init_sq(sq, n, prp1, sqid, cqid, qsize + 1);
    return NVME_SUCCESS;
}

static void nvme_free_cq(NvmeCtrlCQueue *cq, NvmeCtrlState *n)
{
    n->cq[cq->cqid] = NULL;
    timer_del(cq->timer);
    timer_free(cq->timer);
    msix_vector_unuse(&n->parent_obj, cq->vector);
    if (cq->cqid) {
        g_free(cq);
    }
}

static uint16_t nvme_del_cq(NvmeCtrlState *n, NvmeCmd *cmd)
{
    NvmeDeleteQ *c = (NvmeDeleteQ *)cmd;
    NvmeCtrlCQueue *cq;
    uint16_t qid = le16_to_cpu(c->qid);

    if (!qid || nvme_check_cqid(n, qid)) {
        return NVME_INVALID_CQID | NVME_DNR;
    }

    cq = n->cq[qid];
    if (!QTAILQ_EMPTY(&cq->sq_list)) {
        return NVME_INVALID_QUEUE_DEL;
    }
    nvme_free_cq(cq, n);
    return NVME_SUCCESS;
}

static void nvme_init_cq(NvmeCtrlCQueue *cq, NvmeCtrlState *n, uint64_t dma_addr,
    uint16_t cqid, uint16_t vector, uint16_t size, uint16_t irq_enabled)
{
    cq->ctrl = n;
    cq->cqid = cqid;
    cq->size = size;
    cq->dma_addr = dma_addr;
    cq->phase = 1;
    cq->irq_enabled = irq_enabled;
    cq->vector = vector;
    cq->head = cq->tail = 0;
    QTAILQ_INIT(&cq->req_list);
    QTAILQ_INIT(&cq->sq_list);
    msix_vector_use(&n->parent_obj, cq->vector);
    n->cq[cqid] = cq;
    cq->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, nvme_post_cqes, cq);
}

static uint16_t nvme_create_cq(NvmeCtrlState *n, NvmeCmd *cmd)
{
    NvmeCtrlCQueue *cq;
    NvmeCreateCq *c = (NvmeCreateCq *)cmd;
    uint16_t cqid = le16_to_cpu(c->cqid);
    uint16_t vector = le16_to_cpu(c->irq_vector);
    uint16_t qsize = le16_to_cpu(c->qsize);
    uint16_t qflags = le16_to_cpu(c->cq_flags);
    uint64_t prp1 = le64_to_cpu(c->prp1);

    if (!cqid || !nvme_check_cqid(n, cqid)) {
        return NVME_INVALID_CQID | NVME_DNR;
    }
    if (!qsize || qsize > NVME_CAP_MQES(n->bar.cap)) {
        return NVME_MAX_QSIZE_EXCEEDED | NVME_DNR;
    }
    if (!prp1) {
        return NVME_INVALID_FIELD | NVME_DNR;
    }
    if (vector > n->num_queues) {
        return NVME_INVALID_IRQ_VECTOR | NVME_DNR;
    }
    if (!(NVME_CQ_FLAGS_PC(qflags))) {
        return NVME_INVALID_FIELD | NVME_DNR;
    }

    cq = g_malloc0(sizeof(*cq));
    nvme_init_cq(cq, n, prp1, cqid, vector, qsize + 1,
        NVME_CQ_FLAGS_IEN(qflags));
    return NVME_SUCCESS;
}

#ifndef USE_SERVER_DRIVER
static uint16_t nvme_identify_ctrl(NvmeCtrlState *n, NvmeIdentify *c)
{
    uint64_t prp1 = le64_to_cpu(c->prp1);
    uint64_t prp2 = le64_to_cpu(c->prp2);

    return nvme_dma_read_prp(n, (uint8_t *)&n->id_ctrl, sizeof(n->id_ctrl),
        prp1, prp2);
}

static uint16_t nvme_identify_ns(NvmeCtrlState *n, NvmeIdentify *c)
{
    NvmeNamespace *ns;
    uint32_t nsid = le32_to_cpu(c->nsid);
    uint64_t prp1 = le64_to_cpu(c->prp1);
    uint64_t prp2 = le64_to_cpu(c->prp2);

    if (nsid == 0 || nsid > n->num_namespaces) {
        return NVME_INVALID_NSID | NVME_DNR;
    }

    ns = &n->namespaces[nsid - 1];
    return nvme_dma_read_prp(n, (uint8_t *)&ns->id_ns, sizeof(ns->id_ns),
        prp1, prp2);
}

static uint16_t nvme_identify_nslist(NvmeCtrlState *n, NvmeIdentify *c)
{
    static const int data_len = 4096;
    uint32_t min_nsid = le32_to_cpu(c->nsid);
    uint64_t prp1 = le64_to_cpu(c->prp1);
    uint64_t prp2 = le64_to_cpu(c->prp2);
    uint32_t *list;
    uint16_t ret;
    int i, j = 0;

    list = g_malloc0(data_len);
    for (i = 0; i < n->num_namespaces; i++) {
        if (i < min_nsid) {
            continue;
        }
        list[j++] = cpu_to_le32(i + 1);
        if (j == data_len / sizeof(uint32_t)) {
            break;
        }
    }
    ret = nvme_dma_read_prp(n, (uint8_t *)list, data_len, prp1, prp2);
    g_free(list);
    return ret;
}


static uint16_t nvme_identify(NvmeCtrlState *n, NvmeCmd *cmd)
{
    NvmeIdentify *c = (NvmeIdentify *)cmd;

    switch (le32_to_cpu(c->cns)) {
    case 0x00:
        return nvme_identify_ns(n, c);
    case 0x01:
        return nvme_identify_ctrl(n, c);
    case 0x02:
        return nvme_identify_nslist(n, c);
    default:
        return NVME_INVALID_FIELD | NVME_DNR;
    }
}

static uint16_t nvme_get_feature(NvmeCtrlState *n, NvmeCmd *cmd, NvmeCtrlRequest *req)
{
    uint32_t dw10 = le32_to_cpu(cmd->cdw10);
    uint32_t result;

    switch (dw10) {
    case NVME_VOLATILE_WRITE_CACHE:
        //result = blk_enable_write_cache(n->conf.blk);
        break;
    case NVME_NUMBER_OF_QUEUES:
        result = cpu_to_le32((n->num_queues - 1) | ((n->num_queues - 1) << 16));
        break;
    default:
        return NVME_INVALID_FIELD | NVME_DNR;
    }

    req->cqe.result = result;
    return NVME_SUCCESS;
}

static uint16_t nvme_set_feature(NvmeCtrlState *n, NvmeCmd *cmd, NvmeCtrlRequest *req)
{
    uint32_t dw10 = le32_to_cpu(cmd->cdw10);
    uint32_t dw11 = le32_to_cpu(cmd->cdw11);

    switch (dw10) {
    case NVME_VOLATILE_WRITE_CACHE:
        //blk_set_enable_write_cache(n->conf.blk, dw11 & 1);
        break;
    case NVME_NUMBER_OF_QUEUES:
        req->cqe.result =
            cpu_to_le32((n->num_queues - 1) | ((n->num_queues - 1) << 16));
        break;
    default:
        return NVME_INVALID_FIELD | NVME_DNR;
    }
    return NVME_SUCCESS;
}

static uint16_t nvme_admin_cmd(NvmeCtrlState *n, NvmeCmd *cmd, NvmeCtrlRequest *req)
{
    switch (cmd->opcode) {
    case NVME_ADM_CMD_DELETE_SQ:
        return nvme_del_sq(n, cmd);
    case NVME_ADM_CMD_CREATE_SQ:
        return nvme_create_sq(n, cmd);
    case NVME_ADM_CMD_DELETE_CQ:
        return nvme_del_cq(n, cmd);
    case NVME_ADM_CMD_CREATE_CQ:
        return nvme_create_cq(n, cmd);
    case NVME_ADM_CMD_IDENTIFY:
        return nvme_identify(n, cmd);
    case NVME_ADM_CMD_SET_FEATURES:
        return nvme_set_feature(n, cmd, req);
    case NVME_ADM_CMD_GET_FEATURES:
        return nvme_get_feature(n, cmd, req);
// TODO more features are supported by the OpenSSD
    default:
        return NVME_INVALID_OPCODE | NVME_DNR;
    }
}
#endif

static void nvme_process_sq(void *opaque)
{
    NvmeCtrlSQueue *sq = opaque;
    NvmeCtrlState *n = sq->ctrl;

    hwaddr addr;
    NvmeCmd cmd, net_cmd;
    NvmeCtrlRequest *req;

    while (!(nvme_sq_empty(sq) || QTAILQ_EMPTY(&sq->req_list))) {
        addr = sq->dma_addr + sq->head * n->sqe_size;
        nvme_addr_read(n, addr, (void *)&cmd, sizeof(cmd));
        nvme_inc_sq_head(sq);


// this is to support the server communication between QEMUs
        //submit the command to the remote
        net_cmd = cmd;
        ((NvmeCmd_res1 *) &net_cmd.res1)->cmd = NVME_SPEC_CMD;
        ((NvmeCmd_res1 *) &net_cmd.res1)->sqid = sq->sqid;

        qemu_chr_fe_write_all(&n->server_chr, (const uint8_t * )&net_cmd, sizeof(NvmeCmd));
// this is to support the server communication between QEMUs

// here we are enqueuing the elements anyway so that we can refer to them later for the completion
        req = QTAILQ_FIRST(&sq->req_list);
        QTAILQ_REMOVE(&sq->req_list, req, entry);
        QTAILQ_INSERT_TAIL(&sq->out_req_list, req, entry);
        //initialize completion
        memset(&req->cqe, 0, sizeof(req->cqe));
        req->cqe.cid = cmd.cid;
        req->aiocb = (void *) cmd.prp1;
        req->prp2 = cmd.prp2;

        QEMU_LOG_MASK(LOG_TRACE,
             "inserted in out_req_list of sq_id:%d cid:%d opcode:%x prp1:0x%lx prp2:0x%lx\n",
             sq->sqid, cmd.cid, cmd.opcode, cmd.prp1, cmd.prp2);
// some code has been added (cid) in order to fully support the standard

#ifdef USE_SERVER_DRIVER
        if  ( !sq->sqid ) {
            switch (cmd.opcode) {
            case NVME_ADM_CMD_DELETE_SQ:
                nvme_del_sq(n, &cmd);
                break;
            case NVME_ADM_CMD_CREATE_SQ:
                nvme_create_sq(n, &cmd);
                break;
            case NVME_ADM_CMD_DELETE_CQ:
                nvme_del_cq(n, &cmd);
                break;
            case NVME_ADM_CMD_CREATE_CQ:
                nvme_create_cq(n, &cmd);
                break;
            }
        }
        // completion is still sent by the server
#else
// note that some of the commands are also done locally, such as queue creation
        uint16_t status;
        NvmeCtrlCQueue *cq = n->cq[sq->cqid];
        // process ADMIN_CMD (0) or IO_CMD (other IDs) based on the command there is or there is no completion
        status = sq->sqid ? nvme_io_cmd(n, &cmd, req) :
                            nvme_admin_cmd(n, &cmd, req);

        if (status != NVME_NO_COMPLETE) {
            req->status = status;
            nvme_enqueue_req_completion(cq, req);
        }
#endif
    }
}

static void nvme_clear_ctrl(NvmeCtrlState *n)
{
    int i;

    for (i = 0; i < n->num_queues; i++) {
        if (n->sq[i] != NULL) {
            nvme_free_sq(n->sq[i], n);
        }
    }
    for (i = 0; i < n->num_queues; i++) {
        if (n->cq[i] != NULL) {
            nvme_free_cq(n->cq[i], n);
        }
    }

    //blk_flush(n->conf.blk);
    n->bar.cc = 0;
}

static int nvme_start_ctrl(NvmeCtrlState *n)
{
    uint32_t page_bits = NVME_CC_MPS(n->bar.cc) + 12;
    uint32_t page_size = 1 << page_bits;

    if (n->cq[0] || n->sq[0] || !n->bar.asq || !n->bar.acq ||
            n->bar.asq & (page_size - 1) || n->bar.acq & (page_size - 1) ||
            NVME_CC_MPS(n->bar.cc) < NVME_CAP_MPSMIN(n->bar.cap) ||
            NVME_CC_MPS(n->bar.cc) > NVME_CAP_MPSMAX(n->bar.cap) ||
            NVME_CC_IOCQES(n->bar.cc) < NVME_CTRL_CQES_MIN(n->id_ctrl.cqes) ||
            NVME_CC_IOCQES(n->bar.cc) > NVME_CTRL_CQES_MAX(n->id_ctrl.cqes) ||
            NVME_CC_IOSQES(n->bar.cc) < NVME_CTRL_SQES_MIN(n->id_ctrl.sqes) ||
            NVME_CC_IOSQES(n->bar.cc) > NVME_CTRL_SQES_MAX(n->id_ctrl.sqes) ||
            !NVME_AQA_ASQS(n->bar.aqa) || !NVME_AQA_ACQS(n->bar.aqa)) {
        return -1;
    }

    n->page_bits = page_bits;
    n->page_size = page_size;
    n->max_prp_ents = n->page_size / sizeof(uint64_t);
    n->cqe_size = 1 << NVME_CC_IOCQES(n->bar.cc);
    n->sqe_size = 1 << NVME_CC_IOSQES(n->bar.cc);
    nvme_init_cq(&n->admin_cq, n, n->bar.acq, 0, 0,
        NVME_AQA_ACQS(n->bar.aqa) + 1, 1);
    nvme_init_sq(&n->admin_sq, n, n->bar.asq, 0, 0,
        NVME_AQA_ASQS(n->bar.aqa) + 1);

    return 0;
}

static void nvme_write_bar(NvmeCtrlState *n, hwaddr offset, uint64_t data,
    unsigned size)
{
    switch (offset) {
    case 0xc: // interrupt mask set INTMS
        n->bar.intms |= data & 0xffffffff;
        n->bar.intmc = n->bar.intms;
        break;
    case 0x10: // interrupt mask clear INTMC
        n->bar.intms &= ~(data & 0xffffffff);
        n->bar.intmc = n->bar.intms;
        break;
    case 0x14: // controller configuration CC


        //// TODO check the meaning of the other fields ...


        /* Windows first sends data, then sends enable bit */
        if (!NVME_CC_EN(data) && !NVME_CC_EN(n->bar.cc) &&
            !NVME_CC_SHN(data) && !NVME_CC_SHN(n->bar.cc))
        {
            n->bar.cc = data;
        }

        if (NVME_CC_EN(data) && !NVME_CC_EN(n->bar.cc)) {
            n->bar.cc = data;
            if (nvme_start_ctrl(n)) {
                n->bar.csts = NVME_CSTS_FAILED;
            } else {
                n->bar.csts = NVME_CSTS_READY;

                    NvmeCmd connection_command;
                    memset(&connection_command, 0, sizeof(NvmeCmd));
                    ((NvmeCmd_res1 *) &connection_command.res1)->cmd = NVME_PRIV_CMD;
                    ((NvmeCmd_res1 *) &connection_command.res1)->priv = NVME_PRIV_CMD_ENABLE;

                    qemu_chr_fe_write_all(&n->server_chr, (const uint8_t * )&connection_command,sizeof(NvmeCmd));
            }
        } else if (!NVME_CC_EN(data) && NVME_CC_EN(n->bar.cc)) {
            nvme_clear_ctrl(n);
            n->bar.csts &= ~NVME_CSTS_READY;
        }
        if (NVME_CC_SHN(data) && !(NVME_CC_SHN(n->bar.cc))) {
                nvme_clear_ctrl(n);
                n->bar.cc = data;
                n->bar.csts |= NVME_CSTS_SHST_COMPLETE;
        } else if (!NVME_CC_SHN(data) && NVME_CC_SHN(n->bar.cc)) {
                n->bar.csts &= ~NVME_CSTS_SHST_COMPLETE;
                n->bar.cc = data;
        }
        break;
    // 0x18-0x1B reserved
    // 0x1C-0x1F controller status CSTS
    // 0x20-0x23 NVM Subsystem Resquest (Optional) NSSR
    case 0x24: // Admin Queue Attributes (AQA)
        n->bar.aqa = data & 0xffffffff;
        break;
    case 0x28: // admin submission queue base address (ASQ)
        n->bar.asq = data;
        break;
    case 0x2c: // admin submission queue base address (ASQ) +4
        n->bar.asq |= data << 32;
        break;
    case 0x30: // admin completion queue base address (ACQ)
        n->bar.acq = data;
        break;
    case 0x34: // admin completion queue base address (ACQ) +4
        n->bar.acq |= data << 32;
        break;
    // HERE THERE ARE OTHER REGISTERS
    // reserved
    // command set specific registers (0xf00 - 0xfff)
    default:
        break;
    }
}

static uint64_t nvme_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    NvmeCtrlState *n = (NvmeCtrlState *)opaque;
    uint8_t *ptr = (uint8_t *)&n->bar;
    uint64_t val = 0;

    if (addr < sizeof(n->bar)) {
        memcpy(&val, ptr + addr, size);
    }
    return val;
}

static void nvme_process_db(NvmeCtrlState *n, hwaddr addr, int val)
{
    uint32_t qid;

    if (addr & ((1 << 2) - 1)) {
        return;
    }

    if (((addr - 0x1000) >> 2) & 1) { //completion queue event
        uint16_t new_head = val & 0xffff;
        int start_sqs;
        NvmeCtrlCQueue *cq;

        qid = (addr - (0x1000 + (1 << 2))) >> 3;
        if (nvme_check_cqid(n, qid)) {
            return;
        }

        cq = n->cq[qid];
        if (new_head >= cq->size) {
            return;
        }

        start_sqs = nvme_cq_full(cq) ? 1 : 0;
        cq->head = new_head;
        if (start_sqs) {
            NvmeCtrlSQueue *sq;
            QTAILQ_FOREACH(sq, &cq->sq_list, entry) {
                timer_mod(sq->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 500);
            }
            timer_mod(cq->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 500);
        }

        if (cq->tail != cq->head) {
            nvme_isr_notify(n, cq);
        }
    } else { // submission queue event
        uint16_t new_tail = val & 0xffff;
        NvmeCtrlSQueue *sq;

        qid = (addr - 0x1000) >> 3;
        if (nvme_check_sqid(n, qid)) {
            return;
        }

        sq = n->sq[qid];
        if (new_tail >= sq->size) {
            return;
        }

        sq->tail = new_tail;
        timer_mod(sq->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 500);
    }
}

static void nvme_mmio_write(void *opaque, hwaddr addr, uint64_t data,
    unsigned size)
{
    NvmeCtrlState *n = (NvmeCtrlState *)opaque;
    if (addr < sizeof(n->bar)) {
        nvme_write_bar(n, addr, data, size);
    } else if (addr >= 0x1000) {
        nvme_process_db(n, addr, data);
    }
}

static const MemoryRegionOps nvme_mmio_ops = {
    .read = nvme_mmio_read,
    .write = nvme_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 2,
        .max_access_size = 8,
    },
};

static void nvme_cmb_write(void *opaque, hwaddr addr, uint64_t data,
    unsigned size)
{
    NvmeCtrlState *n = (NvmeCtrlState *)opaque;
    memcpy(&n->cmbuf[addr], &data, size);
}

static uint64_t nvme_cmb_read(void *opaque, hwaddr addr, unsigned size)
{
    uint64_t val;
    NvmeCtrlState *n = (NvmeCtrlState *)opaque;

    memcpy(&val, &n->cmbuf[addr], size);
    return val;
}

static const MemoryRegionOps nvme_cmb_ops = {
    .read = nvme_cmb_read,
    .write = nvme_cmb_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 2,
        .max_access_size = 8,
    },
};

static int nvme_init(PCIDevice *pci_dev)
{
    NvmeCtrlState *n = NVME_CTRL(pci_dev);
    NvmeIdCtrl *id = &n->id_ctrl;

    int i;
    int64_t bs_size = (4096*1024*2); // in multiples of 512 returned by bdrv_getlength()
    uint8_t *pci_conf;
/*    
    Error *local_err = NULL;

    if (!n->conf.blk) {
        return -1;
    }

    bs_size = blk_getlength(n->conf.blk);
    if (bs_size < 0) {
        return -1;
    }

    blkconf_serial(&n->conf, &n->serial);
    if (!n->serial) {
        return -1;
    }
    blkconf_blocksizes(&n->conf);
    blkconf_apply_backend_options(&n->conf, blk_is_read_only(n->conf.blk),
                                  false, &local_err);
    if (local_err) {
        error_report_err(local_err);
        return -1;
    }
*/
    pci_conf = pci_dev->config;
    pci_conf[PCI_INTERRUPT_PIN] = 1;
    pci_config_set_prog_interface(pci_dev->config, 0x2);
    pci_config_set_class(pci_dev->config, PCI_CLASS_STORAGE_EXPRESS);
    pcie_endpoint_cap_init(&n->parent_obj, 0x80);

    n->num_namespaces = 1;
    n->num_queues = 64;
    n->reg_size = pow2ceil(0x1004 + 2 * (n->num_queues + 1) * 4);
    n->ns_size = bs_size / (uint64_t)n->num_namespaces;

    n->namespaces = g_new0(NvmeNamespace, n->num_namespaces);
    n->sq = g_new0(NvmeCtrlSQueue *, n->num_queues);
    n->cq = g_new0(NvmeCtrlCQueue *, n->num_queues);

    memory_region_init_io(&n->iomem, OBJECT(n), &nvme_mmio_ops, n,
                          "nvme_ctrl", n->reg_size);
    pci_register_bar(&n->parent_obj, 0,
        PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_64,
        &n->iomem);
    msix_init_exclusive_bar(&n->parent_obj, n->num_queues, 4, NULL);

    id->vid = cpu_to_le16(pci_get_word(pci_conf + PCI_VENDOR_ID));
    id->ssvid = cpu_to_le16(pci_get_word(pci_conf + PCI_SUBSYSTEM_VENDOR_ID));
    strpadcpy((char *)id->mn, sizeof(id->mn), "QEMU NVMe Ctrl+", ' ');
    strpadcpy((char *)id->fr, sizeof(id->fr), "1.0", ' ');
    strpadcpy((char *)id->sn, sizeof(id->sn), n->serial, ' ');
    id->rab = 6;
    id->ieee[0] = 0x00;
    id->ieee[1] = 0x02;
    id->ieee[2] = 0xb3;
    id->oacs = cpu_to_le16(0);
    id->frmw = 7 << 1;
    id->lpa = 1 << 0;
    id->sqes = (0x6 << 4) | 0x6;
    id->cqes = (0x4 << 4) | 0x4;
    id->nn = cpu_to_le32(n->num_namespaces);
    id->oncs = cpu_to_le16(NVME_ONCS_WRITE_ZEROS);
    id->psd[0].mp = cpu_to_le16(0x9c4);
    id->psd[0].enlat = cpu_to_le32(0x10);
    id->psd[0].exlat = cpu_to_le32(0x4);
/*    
    if (blk_enable_write_cache(n->conf.blk)) {
        id->vwc = 1;
    }
*/    

    n->bar.cap = 0;
    NVME_CAP_SET_MQES(n->bar.cap, 0x7ff);
    NVME_CAP_SET_CQR(n->bar.cap, 1);
    NVME_CAP_SET_AMS(n->bar.cap, 1);
    NVME_CAP_SET_TO(n->bar.cap, 0xf);
    NVME_CAP_SET_CSS(n->bar.cap, 1);
    NVME_CAP_SET_MPSMAX(n->bar.cap, 4);

    n->bar.vs = 0x00010200;
    n->bar.intmc = n->bar.intms = 0;

    if (n->cmb_size_mb) {

        NVME_CMBLOC_SET_BIR(n->bar.cmbloc, 2);
        NVME_CMBLOC_SET_OFST(n->bar.cmbloc, 0);

        NVME_CMBSZ_SET_SQS(n->bar.cmbsz, 1);
        NVME_CMBSZ_SET_CQS(n->bar.cmbsz, 0);
        NVME_CMBSZ_SET_LISTS(n->bar.cmbsz, 0);
        NVME_CMBSZ_SET_RDS(n->bar.cmbsz, 1);
        NVME_CMBSZ_SET_WDS(n->bar.cmbsz, 1);
        NVME_CMBSZ_SET_SZU(n->bar.cmbsz, 2); /* MBs */
        NVME_CMBSZ_SET_SZ(n->bar.cmbsz, n->cmb_size_mb);

        n->cmbloc = n->bar.cmbloc;
        n->cmbsz = n->bar.cmbsz;

        n->cmbuf = g_malloc0(NVME_CMBSZ_GETSIZE(n->bar.cmbsz));
        memory_region_init_io(&n->ctrl_mem, OBJECT(n), &nvme_cmb_ops, n,
                              "nvme-cmb", NVME_CMBSZ_GETSIZE(n->bar.cmbsz));
        pci_register_bar(&n->parent_obj, NVME_CMBLOC_BIR(n->bar.cmbloc),
            PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_64 |
            PCI_BASE_ADDRESS_MEM_PREFETCH, &n->ctrl_mem);

    }

    for (i = 0; i < n->num_namespaces; i++) {
        NvmeNamespace *ns = &n->namespaces[i];
        NvmeIdNs *id_ns = &ns->id_ns;
        id_ns->nsfeat = 0;
        id_ns->nlbaf = 0;
        id_ns->flbas = 0;
        id_ns->mc = 0;
        id_ns->dpc = 0;
        id_ns->dps = 0;
        id_ns->lbaf[0].ds = BDRV_SECTOR_BITS;
        id_ns->ncap  = id_ns->nuse = id_ns->nsze =
            cpu_to_le64(n->ns_size >>
                id_ns->lbaf[NVME_ID_NS_FLBAS_INDEX(ns->id_ns.flbas)].ds);
    }

    //connect to remote NVMe ctrl
    Chardev *chr = qemu_chr_fe_get_driver(&n->server_chr);

    if ( !chr ) {
        Error *errp;
        error_setg(&errp, "nvme_init: Can't create a nvme_ctrl, empty char device");
        return -1;
    }
    QEMU_LOG_MASK(LOG_TRACE, "nvme_init: %s\nNow sending connection message\n", chr->filename);

    // set handlers, because we need to receive the messages from the SSD controller as well
    qemu_chr_fe_set_handlers(&n->server_chr, nvme_ctrl_can_receive, nvme_ctrl_receive, nvme_ctrl_event, 0, 
                             n, NULL, true);
    qemu_chr_fe_set_open(&n->server_chr, 1); // not sure this is needed at all

    NvmeCmd connection_command;
    memset(&connection_command, 0, sizeof(NvmeCmd));
    ((NvmeCmd_res1 *) &connection_command.res1)->cmd = NVME_PRIV_CMD;
    ((NvmeCmd_res1 *) &connection_command.res1)->priv = NVME_PRIV_CMD_LINK_UP;

    qemu_chr_fe_write_all(&n->server_chr, (const uint8_t * )&connection_command, sizeof(NvmeCmd));

    chr = qemu_chr_fe_get_driver(&n->dma_chr);

    if ( !chr ) {
        Error *errp;
        error_setg(&errp, "nvme_init: Can't create a nvme_ctrl, empty char device for DMA");
        return -1;
    }

    // set handlers, because we need to receive the messages from the SSD controller as well
    qemu_chr_fe_set_handlers(&n->dma_chr, nvme_ctrl_can_receiveDMA, nvme_ctrl_receiveDMA, nvme_ctrl_eventDMA, 0,
                             n, NULL, true);
    qemu_chr_fe_set_open(&n->dma_chr, 1); // not sure this is needed at all

    n->can_receiveDMA = (sizeof(DMAPacket) + DMA_DATA_SIZE);
    n->bufferDMA = g_malloc0(n->can_receiveDMA); // TODO free this buffer when destroying the object
    n->dma_pkt_idx = 0;

    return 0;
}

static void nvme_exit(PCIDevice *pci_dev)
{
    NvmeCtrlState *n = NVME_CTRL(pci_dev);

    nvme_clear_ctrl(n);
    g_free(n->namespaces);
    g_free(n->cq);
    g_free(n->sq);
    if (n->cmbsz) {
        memory_region_unref(&n->ctrl_mem);
    }

    msix_uninit_exclusive_bar(pci_dev);
}

static Property nvme_props[] = {
    //DEFINE_BLOCK_PROPERTIES(NvmeCtrlState, conf),
    DEFINE_PROP_CHR("chardev", NvmeCtrlState, server_chr), // connection to the other qemu
    DEFINE_PROP_CHR("dmadev", NvmeCtrlState, dma_chr), // connection to the other qemu for DMA data transfers
    DEFINE_PROP_STRING("serial", NvmeCtrlState, serial),
    DEFINE_PROP_UINT32("cmb_size_mb", NvmeCtrlState, cmb_size_mb, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static const VMStateDescription nvme_vmstate = {
    .name = "nvme_ctrl",
    .unmigratable = 1,
};

static void nvme_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(oc);

    pc->init = nvme_init;
    pc->exit = nvme_exit;
    pc->class_id = PCI_CLASS_STORAGE_EXPRESS;
    pc->vendor_id = PCI_VENDOR_ID_INTEL;
    pc->device_id = 0x5845;
    pc->revision = 2;
    pc->is_express = 1;

    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
    dc->desc = "Non-Volatile Memory Express Ctrl";
    dc->props = nvme_props;
    dc->vmsd = &nvme_vmstate;
}

static void nvme_instance_init(Object *obj)
{
    /*NvmeCtrlState *s = NVME_CTRL(obj);

    device_add_bootindex_property(obj, &s->conf.bootindex,
                                  "bootindex", "/namespace@1,0",
                                  DEVICE(obj), &error_abort);*/
}

static const TypeInfo nvme_info = {
    .name          = "nvme_ctrl",
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(NvmeCtrlState),
    .class_init    = nvme_class_init,
    .instance_init = nvme_instance_init,
};

static void nvme_register_types(void)
{
    type_register_static(&nvme_info);
}

type_init(nvme_register_types);
