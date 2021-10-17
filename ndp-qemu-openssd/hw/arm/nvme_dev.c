/*
 * Copyright (c) 2017, Antonio Barbalace
 *
 * Most of this file originates from the host_dll source code of Cosmos+
 * The code is prototype quality, some features are not implemented
 */

/**
 * Usage: add options:
 *      -chardev socket,path=<file_nvme>,server,nowait,id=<nvme_id> 
 *      -chardev socket,path=<file_dma>,server,nowait,id=<dma_id> 
 *      -device nvme_dev,phys=<phys_address>,chardev=<nvme_id>,dmadev=<dma_id>
 *
 */

/*#include "hw/sysbus.h"
#include "hw/devices.h"
#include "hw/flash.h"
#include "sysemu/blockdev.h"
*/

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/event_notifier.h"
#include "qom/object_interfaces.h"
#include "chardev/char-fe.h"

#include "hw/qdev.h"
#include "hw/block/block.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
//#include "hw/block/flash.h"
//#include "qapi/error.h"
//#include "qapi/visitor.h"
#include "sysemu/sysemu.h"
#include "sysemu/block-backend.h"
#include "sysemu/dma.h"

//the following contains the main data structures from the Cosmos FPGA NVMe implementation (includes offsets)
#include "host_lld.h"
    
//#define DEBUGME
#ifdef DEBUGME
#define QEMU_LOG_MASK(severity,fmt, ...) \
do { qemu_log_mask(severity, fmt, ## __VA_ARGS__); } while (0)
#else
#define QEMU_LOG_MASK(severity, fmt, ...) \
do {} while (0)
#endif
//do { qemu_log_mask(severity, "nvme_dev: " fmt , ## __VA_ARGS__); } while (0) 
    

/* TODO add these in another file */
// submission queue *******************************************************************
typedef struct NvmeCmd {
    uint8_t     opcode;
    uint8_t     fuse;
    uint16_t    cid;
    uint32_t    nsid;
    uint64_t    res1;
    uint64_t    mptr;
    uint64_t    prp1;
    uint64_t    prp2;
    uint32_t    cdw10;
    uint32_t    cdw11;
    uint32_t    cdw12;
    uint32_t    cdw13;
    uint32_t    cdw14;
    uint32_t    cdw15;
} NvmeCmd;

typedef struct NvmeRwCmd {
    uint8_t     opcode;
    uint8_t     flags;
    uint16_t    cid;
    uint32_t    nsid;
    uint64_t    rsvd2;
    uint64_t    mptr;
    uint64_t    prp1;
    uint64_t    prp2;
    uint64_t    slba;
    uint16_t    nlb;
    uint16_t    control;
    uint32_t    dsmgmt;
    uint32_t    reftag;
    uint16_t    apptag;
    uint16_t    appmask;
} NvmeRwCmd;

enum NvmeCtrlPrivCommands {
    NVME_PRIV_CMD = 0xA5,
    NVME_SPEC_CMD = 0x96, //command from specifications (admin or IO is based on the sqid)
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
    uint8_t cmd; //command type: internal, admin queue, IO queue
    uint8_t priv; //private commands
    uint16_t sqid; //submission queue id;
    uint32_t slot;
} NvmeCmd_res1;

// completion queue *******************************************************************
typedef struct NvmeCqe {
    uint32_t    result;
    uint32_t    rsvd;
    uint16_t    sq_head;
    uint16_t    sq_id;
    uint16_t    cid;
    uint16_t    status;
} NvmeCqe;

typedef struct NvmeCqe_rsvd {
    uint8_t cmd; //command type: internal, admin queue, IO queue
    uint8_t priv; //private command content
    uint16_t rsvd; //future use
} NvmeCqe_rsvd;

typedef struct NvmeCqe_RXDMA {
    uint8_t slot;
    uint8_t offset;
} NvmeCqe_RXDMA;
    
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

// DMA packets ************************************************************************
#define DMA_FLAG_TO_HOST  (0x1 << 3)
#define DMA_FLAG_TO_SSD   (0x1 << 4)
#define DMA_FLAG_DIRECT   (0x1 << 5)
#define DMA_FLAG_AUTO     (0x1 << 6)

#define DMA_DATA_SIZE       4096

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

//=============================================================================
// INTERRUPTS (the following are platform specific)
//=============================================================================

#define DEV_IRQ_OFFSET 32
#define DEV_IRQ_ASSERT_INTR 61    
    
//=============================================================================
// COMMANDS
//=============================================================================

#define V2FCommand_NOP 0
#define V2FCommand_Reset 1
#define V2FCommand_SetFeatures 6
#define V2FCommand_GetFeatures 46
#define V2FCommand_ReadPageTrigger 13
#define V2FCommand_ReadPageTransfer 18
#define V2FCommand_ProgramPage 28
#define V2FCommand_BlockErase 37
#define V2FCommand_StatusCheck 41
#define V2FCommand_ReadPageTransferRaw 55


struct _NVMe_DEVState;


typedef enum devRequest_state {
    REQ_INIT =0,               // during initialization
    REQ_NOT_IN_SRAM =1,        // only in linked list not in SRAM
    REQ_WAITING_PUBLISH =2,    // in linked list and in SRAM but never published in the nvme_cmd_fifo_reg
    REQ_WAITING_READ =3,       // in linked list, in SRAM, and currently in the nvme_cmd_fifo_reg
    REQ_WAITING_COMPL =4,      // in linked list, in SRAM, and already read by the software from the nvme_cmd_fifo_reg
} devRequest_state;
const char * devRequest_state_char[] = {"init", "not_in_sram", "waiting_publish", "waiting_read", "waiting_compl"};

//cmd queue data structure arriving from the host, they can be either ADM or IO (NVM)
//all commands are going to be pushed on the SRAM in FIFO order
typedef struct nvme_devRequest {
    struct _NVMe_DEVState * state;

    NvmeCmd         cmd;
    uint16_t        status; //controller SRAM status for this command
    uint16_t        slot;   //SRAM slot
    uint32_t        written; //in pages
    uint32_t        data_size; //in pages
    uint32_t        qid;
    QTAILQ_ENTRY(nvme_devRequest) entry;
} nvme_devRequest;

#define BITS_PER_UINT32 ( (sizeof(uint32_t) * 8) )
#define MAX_CMD_SRAM_SLOTS 128
#define MAX_CMD_SRAM_UINT32 ( (MAX_CMD_SRAM_SLOTS / BITS_PER_UINT32) )

//=============================================================================
// STATE
//=============================================================================

#define TYPE_NVME_DEV "nvme_dev"

typedef struct _NVMe_DEVState {
    /*< private >*/
    SysBusDevice parent;

    /*< public >*/
    MemoryRegion mmio;
    
    /*< public >*/
//    BlockAcctStats acct; //NOTE that a block already includes stats
    CharBackend server_chr; /* the SSD is the server */
    uint8_t server_state; // TODO before anything check if the connection is alive

    CharBackend dma_chr; /* the SSD's DMA server */
    uint8_t dma_state;  // TODO before anything check if the connection is alive
    
    /*
    qemu_irq irq;
    DeviceState *flash;
    */
    
    // it can generate an IRQ to the CPU NVMe_ctrl->CPU
    qemu_irq irq;
    SysBusDevice * sbd;

// START TODO    
    uint8_t  manf_id, chip_id;
    uint64_t physical_address;

    //nvme_devRequest * io_req; //can be optimized by creating a free list
    QTAILQ_HEAD(req_list, nvme_devRequest) req_list; // requests waiting for completion
    uint32_t cmd_sram_head, cmd_sram_tail; //same empty and full philosophy of NVMe queues
    uint32_t cmd_sram_bitmask[MAX_CMD_SRAM_UINT32]; // this is the bitmask to enable out of order slots releases
    
    //DMA support
    uint32_t    can_receiveDMA;
    DMAPacket   *bufferDMA;
    uint32_t dma_pkt_idx; // current packet id for dma reconstruction

// END TODO

    /* HW registers */
    uint32_t dev_irq_mask;
    uint32_t dev_irq_reg;
    uint32_t pcie_status_reg;
    uint32_t pcie_func_reg;
      
    uint32_t nvme_status_reg;
    uint32_t nvme_cmd_fifo_reg;
    uint32_t nvme_admin_queue_set_reg;
    
    uint32_t nvme_cpl_fifo_reg[3];
    uint32_t nvme_io_sq_set_reg[8][2]; // TODO this is a vector 
    uint32_t nvme_io_cq_set_reg[8][2]; // TODO this is a vector
    
    uint32_t host_dma_fifo_cnt_reg;
    uint32_t host_dma_cmd_fifo_reg[4];
    
    uint32_t nvme_cmd_sram[MAX_CMD_SRAM_SLOTS][16]; 
} NVMe_DEVState;



///////////////////////////////////////////////////////////////////////////////
// CMD_SRAM and CMD_fifo handling runtimes
///////////////////////////////////////////////////////////////////////////////
static inline int cmd_sram_empty (NVMe_DEVState * s) {
    return (s->cmd_sram_head == s->cmd_sram_tail);
}
static inline int cmd_sram_full (NVMe_DEVState * s) {
    return ((s->cmd_sram_tail +1) % MAX_CMD_SRAM_SLOTS == s->cmd_sram_head);
}
/* returns a valid slot id (>= 0) if success
 * -1 if error
 */
static inline int cmd_sram_enqueue (NVMe_DEVState * s, nvme_devRequest * req)
{ 
    int slot = -1;
 
    // TODO take a lock on the sram
    if ( !(cmd_sram_full(s)) ) {
        slot = s->cmd_sram_tail;
    
        *(NvmeCmd *)&s->nvme_cmd_sram[slot] = req->cmd; 
        s->cmd_sram_bitmask[(slot / BITS_PER_UINT32)] |= (unsigned int) (0x1 << (slot % BITS_PER_UINT32));
        s->cmd_sram_tail = (slot +1) % MAX_CMD_SRAM_SLOTS; // increment pointer
    }
    // TODO release a lock on the sram
    
    return slot;
}
/*    // TODO differently from the case above here it is not sure that req->slot that is dequeued is the one that 
 *    is staying == to the head therefore the following code must be changed so that when you release you first release the bit in the bitmask and when it comes to update the head pointer you only act IIF head e lo stesso di quello che tu irlasci, additionallym quando rilasci devi controllare se ci sono altri bit a uno, in quel caso, fai il release di tutti
 * 
 * // returns the dequeued object <<< ? not sure
 * returns the number of continuous released slots
 */
//static inline NvmeCmd * cmd_sram_dequeue (NVMe_DEVState * s, int slot)
static inline int cmd_sram_dequeue (NVMe_DEVState * s, int slot)
{ 
    //NvmeCmd temp;
    int slot_idx = slot / BITS_PER_UINT32;
    unsigned int slot_bit = 0x1 << (slot % BITS_PER_UINT32);
    int i = 0;
    
    //TODO take a lock on the sram
    if ( s->cmd_sram_bitmask[slot_idx] & slot_bit ) {
        // element exists
        s->cmd_sram_bitmask[slot_idx] &= ~slot_bit; //remove from the bitmask
        //temp = *((NvmeCmd *) &s->nvme_cmd_sram[slot]);
        memset(&(s->nvme_cmd_sram[slot]), 0, 64); // put some MACRO INSTEAD of 64 TODO
        //advance the head as much as possible
        while ( !( s->cmd_sram_bitmask[s->cmd_sram_head / BITS_PER_UINT32] 
                    & (0x1 << (s->cmd_sram_head % BITS_PER_UINT32)) ) &&
                !cmd_sram_empty(s) ) { 
            s->cmd_sram_head = (s->cmd_sram_head +1) % MAX_CMD_SRAM_SLOTS;
            i++;
        }
    }
    // TODO release a lock on the sram
    
    QEMU_LOG_MASK(LOG_TRACE, "%s: head:%d tail:%d slot bitmask:0x%x:0x%x:0x%x:0x%x .idx:%d .bit:%x \n",
           __func__, s->cmd_sram_head, s->cmd_sram_tail, 
           s->cmd_sram_bitmask[0], s->cmd_sram_bitmask[1], s->cmd_sram_bitmask[2], s->cmd_sram_bitmask[3],
           slot_idx, slot_bit);

    return i; // cannot return the temp because it is allocated on the stack -- if needed we can change this later
}

#define ANY_VALUE (int)(~0)
static inline nvme_devRequest * cmd_fifo_find (NVMe_DEVState * s, int slot, int status)
{
    nvme_devRequest *p = 0, *q = 0;
    
    QTAILQ_FOREACH_SAFE(p, &(s->req_list), entry, q) {
        if ( (status == ANY_VALUE) && p->slot == slot )
            return p;
        if ( (slot == ANY_VALUE) && p->status == status )
            return p;
        if ( (status == p->status) && (slot == p->slot) )
            return p;
    }
    
    return 0;
}

///////////////////////////////////////////////////////////////////////////////
// generic nvme runtimes
///////////////////////////////////////////////////////////////////////////////
#ifdef DEBUGME
static void nvme_dev_dump_cmd(struct NvmeCmd * cmd)
{
    if (cmd->res1) {
        qemu_log_mask(LOG_TRACE,
            "SQ opc:0x%x fuse:%d psdt:%d cid:%d nsid:0x%x mptr:0x%lx prp1:0x%lx prp2:0x%lx cdw10:0x%x res1 .cmd:%s .priv:0x%x .sqid:%d .slot:%d\n",
            (unsigned int) cmd->opcode, (unsigned int) cmd->fuse & 0x3,
            (unsigned int) (cmd->fuse >> 6) & 0x3,  (unsigned int) cmd->cid,
            cmd->nsid, cmd->mptr, cmd->prp1, cmd->prp2, cmd->cdw10,
            ( ((NvmeCmd_res1 *) &(cmd->res1))->cmd == NVME_PRIV_CMD ) ? "PRIV" : "SPEC",
            ((NvmeCmd_res1 *) &(cmd->res1))->priv, ((NvmeCmd_res1 *) &(cmd->res1))->sqid, (int)((NvmeCmd_res1 *) &(cmd->res1))->slot);
        // for NVM commands dump the request
        if ( (((NvmeCmd_res1 *) &(cmd->res1))->cmd != NVME_PRIV_CMD) && // not a driver specific command
             (((NvmeCmd_res1 *) &(cmd->res1))->sqid != 0) )             // not from admin queue
            qemu_log_mask(LOG_TRACE,
                "NVM %s cmd 0x%x slba:0x%lx lr:%d fua:%d prinfo:0x%x nlb:%d\n",
                ( ((NvmeRwCmd*) cmd)->opcode == 0x2) ? "READ" : ( ((NvmeRwCmd*) cmd)->opcode == 0x1) ? "WRITE" : "?",
                ( (NvmeRwCmd*) cmd)->opcode, ((NvmeRwCmd*) cmd)->slba, 
                ( ((NvmeRwCmd*) cmd)->control & 0x8000 >> 31), ( ((NvmeRwCmd*) cmd)->control & 0x4000 >> 30),
                ( ((NvmeRwCmd*) cmd)->control & 0xc000), ((NvmeRwCmd*) cmd)->nlb );
    }
    else
        qemu_log_mask(LOG_TRACE,
            "SQ opc:0x%x fuse:%d psdt:%d cid:%d nsid:0x%x mptr:0x%lx prp1:0x%lx prp2:0x%lx\n",
            (unsigned int) cmd->opcode, (unsigned int) cmd->fuse & 0x3,
            (unsigned int) (cmd->fuse >> 6) & 0x3,  (unsigned int) cmd->cid,
            cmd->nsid, cmd->mptr, cmd->prp1, cmd->prp2);
}
static void nvme_dev_dump_cqe(struct NvmeCqe * cqe)
{
    if (cqe->rsvd)
        qemu_log_mask(LOG_TRACE,
            "CQ result:0x%x sq_head:%d sq_id:%d cid:%d status:0x%x rsvd .cmd:%s .priv:0x%x .rsvd:0x%x\n",
            cqe->result, (unsigned int)cqe->sq_head, (unsigned int) cqe->sq_id, 
            (unsigned int) cqe->cid, (unsigned int)cqe->status, 
            ( ((NvmeCqe_rsvd *) &cqe->rsvd)->cmd == NVME_PRIV_CMD ) ? "PRIV" : "SPEC",
            (unsigned int) ((NvmeCqe_rsvd *) &cqe->rsvd)->priv, ((NvmeCqe_rsvd *) &cqe->rsvd)->rsvd);
    else
        qemu_log_mask(LOG_TRACE,
            "CQ result:0x%x sq_head:%d sq_id:%d cid:%d status:0x%x\n",
            cqe->result, (unsigned int)cqe->sq_head, (unsigned int) cqe->sq_id, 
            (unsigned int) cqe->cid, (unsigned int)cqe->status);
}
#endif

///////////////////////////////////////////////////////////////////////////////
// QEMU handling
///////////////////////////////////////////////////////////////////////////////

#define NVMe_DEV(obj) \
    OBJECT_CHECK(NVMe_DEVState, obj, TYPE_NVME_DEV)

// on this connection we only receive 64 bytes or the size of an NvmeCmd
static int nvme_dev_can_receive(void *opaque)
{
    return sizeof(NvmeCmd); //64
}

static void nvme_dev_receive(void *opaque, const uint8_t *buf, int size)
{   
    NVMe_DEVState *s = opaque; 
    NvmeCmd nvme_cmd;
    memcpy((char *) &nvme_cmd, buf, size > 64 ? 64 : size);

#ifdef DEBUGME    
    nvme_dev_dump_cmd(&nvme_cmd); //dump for debugging in the log
#endif    

    switch ( ((NvmeCmd_res1 *) &nvme_cmd.res1)->cmd ) {
        case NVME_PRIV_CMD: {
            NvmeCmd_res1 ncr1 = *((NvmeCmd_res1 *) &nvme_cmd.res1);
            switch (ncr1.priv) {
                case NVME_PRIV_CMD_LINK_UP:
                    ((DEV_IRQ_REG *) &s->dev_irq_reg)->pcieLink = 1;
                    ((PCIE_STATUS_REG *) &s->pcie_status_reg)->pcieLinkUp = 1;
                    
                    if ( ((DEV_IRQ_REG *)&s->dev_irq_mask)->pcieLink )
                        qemu_set_irq(s->irq, 1);
                    return;
                case NVME_PRIV_CMD_LINK_DOWN:
                    break;
                case NVME_PRIV_CMD_ENABLE:
                    ((DEV_IRQ_REG *) &s->dev_irq_reg)->nvmeCcEn = 1;
                    ((NVME_STATUS_REG *) &s->nvme_status_reg)->ccEn = 1;
                    
                    if ( ((DEV_IRQ_REG *) &s->dev_irq_mask)->nvmeCcEn )
                        qemu_set_irq(s->irq, 1);
                    return;
                case NVME_PRIV_CMD_DISABLE:
                    break;                   
                case NVME_PRIV_CMD_DMA_DONE: {
                    //increment the DMA counter 
                    ((HOST_DMA_FIFO_CNT_REG *) &(s->host_dma_fifo_cnt_reg))->autoDmaTx++;
                        
                    //now clean the slot (copied from the code in nvme_dev_mem_write() )
                    //TODO refactoring

/*****************************************************************************/
                //remove the one that is signaled by the software
                int cid =-1, sqId =-1, slot =-1;
                unsigned long prp1 =~0;
                
                slot = ((NvmeCmd_res1 *) &nvme_cmd.res1)->slot;
                QEMU_LOG_MASK(LOG_TRACE, "COMPLETION AUTO TX for slot %d\n", slot);
                
                nvme_devRequest * req = cmd_fifo_find(s, slot, ANY_VALUE);
                if ( !req ) {
                    error_report("ERROR RECV cannot find slot %d in cmd_fifo. restart is needed\n", slot);
                    break;
                }
                if ( req->status != REQ_WAITING_COMPL )
                    error_report("ERROR RECV the cmd_fifo status must be %d but it is %d\n", 
                           REQ_WAITING_COMPL, req->status);
                
                prp1 = ((NvmeCmd *)&s->nvme_cmd_sram[slot])->prp1;
                cid = ((NvmeCmd *)&s->nvme_cmd_sram[slot])->cid;
                sqId = ((NvmeCmd_res1 *) &(((NvmeCmd *)&s->nvme_cmd_sram[slot])->res1) )->sqid;
                if ( (cid != req->cmd.cid) || (sqId != req->qid) )
                    error_report("ERROR CMD_SRAM info is different from CMD_FIFO info cid 0x%x 0x%x sqId 0x%x 0x%x\n",
                            cid, req->cmd.cid, sqId, req->qid);
                    
                if (req->qid != 0) { // is an IO command
                    req->written++; 
                    
                    if (req->written < req->data_size) {// did not finish to write all the data to host
                        error_report("ERROR MORE DATA TO TRANSFER TX slot:%d qid:%d written:%d data_size:%d\n",
                                    req->slot, req->qid, req->written, req->data_size);
                        break;
                    }
                }
                
                // handling the completion of the command
                cmd_sram_dequeue(s, slot); // remove from CMD_SRAM
                QTAILQ_REMOVE(&(s->req_list), req, entry); // remove from CMD_FIFO
                g_free(req); // delete allocated space

                // enqueue the next one in SRAM
                //search the next in the LIST and enqeue it
                req = cmd_fifo_find(s, ANY_VALUE, REQ_NOT_IN_SRAM);
                if (req) {
                    int _slot;
                    if ( ( _slot = cmd_sram_enqueue(s,req)) >= 0 ) {
                        req->slot = _slot;
                        req->status = REQ_WAITING_PUBLISH;
                    }
// TODO we should consider to write the following in a function/macro                
                    if ( (req->status == REQ_WAITING_PUBLISH) && !(s->nvme_cmd_fifo_reg) ) {
                        ((NVME_CMD_FIFO_REG *) &s->nvme_cmd_fifo_reg)->qID = req->qid;
                        ((NVME_CMD_FIFO_REG *) &s->nvme_cmd_fifo_reg)->cmdSlotTag = req->slot;
                        ((NVME_CMD_FIFO_REG *) &s->nvme_cmd_fifo_reg)->cmdSeqNum = req->cmd.cid; // ? this is not used in the firmware code!!!
                        ((NVME_CMD_FIFO_REG *) &s->nvme_cmd_fifo_reg)->cmdValid = 1;
                
                        req->status = REQ_WAITING_READ;
                    }
                }
                
                //now construct the completion and send it
                NvmeCqe completion;
                completion.result = 0; //success
                completion.sq_head = 0; // TODO ?
                completion.sq_id = sqId;
                completion.cid = cid;
                completion.status = 0;
                //construct the completion -- private fields 
                ((NvmeCqe_rsvd *) &completion.rsvd)->cmd = NVME_SPEC_CMD;
                if (prp1 != ~0)
                    ((NvmeCqe_rsvd *) &completion.rsvd)->rsvd = cqe_addr_hash(prp1);

#ifdef DEBUGME                
                nvme_dev_dump_cqe(&completion);
#endif                
                qemu_chr_fe_write_all(&s->server_chr, (const uint8_t * )&completion, sizeof(NvmeCqe));
/*****************************************************************************/
                    
                    break;
                }
                case NVME_PRIV_CMD_DDMA_ERROR:
                    break;         
                case NVME_PRIV_CMD_DDMA_DONE: {
                    //increment the DMA counter 
                    ((HOST_DMA_FIFO_CNT_REG *) &(s->host_dma_fifo_cnt_reg))->directDmaTx++;
                    break;
                }
                case NVME_PRIV_CMD_DMA_ERROR:
                    break;                                 
                    
                default:
                    warn_report("WARN command not supported yet PRIV\n");
                    break;
            }
            break;
        }
        case NVME_SPEC_CMD: { // any command must be moved to the special SRAM area (no interrupt is generated)
            /* DESIGN
             * Let's queue these packets in this module, we cannot just put them in the SRAM 
             * because there may be no space, thus a list to enqueue the items is required
             */
            int _slot;
            nvme_devRequest * req = g_malloc0(sizeof(nvme_devRequest));
            memcpy(&(req->cmd), &nvme_cmd, sizeof(NvmeCmd));
            req->state = s;
            req->qid = ((NvmeCmd_res1 *) &req->cmd.res1)->sqid; //req->cmd.res1 = 0; //TODO maybe this should be removed
            req->status = REQ_INIT; req->slot =-1;
            req->written = 0; req->data_size = 0;
            QTAILQ_INSERT_TAIL(&(s->req_list), req, entry);
            
            if ( ( _slot = cmd_sram_enqueue(s, req)) >= 0) {
                req->slot = _slot;
                req->status = REQ_WAITING_PUBLISH;
            }
            else 
                req->status = REQ_NOT_IN_SRAM;
            
            if ( (req->status == REQ_WAITING_PUBLISH) && !(s->nvme_cmd_fifo_reg) ) {
                ((NVME_CMD_FIFO_REG *) &s->nvme_cmd_fifo_reg)->qID = req->qid;
                ((NVME_CMD_FIFO_REG *) &s->nvme_cmd_fifo_reg)->cmdSlotTag = req->slot;
                ((NVME_CMD_FIFO_REG *) &s->nvme_cmd_fifo_reg)->cmdSeqNum = req->cmd.cid; // ? this is not used in the firmware code!!!
                ((NVME_CMD_FIFO_REG *) &s->nvme_cmd_fifo_reg)->cmdValid = 1;
                
                req->status = REQ_WAITING_READ;
            }
            QEMU_LOG_MASK(LOG_TRACE, "submission command qid %d slot %d command %x status %s\n", 
                    (int)req->qid, (int)req->slot, (unsigned int)req->cmd.opcode, devRequest_state_char[req->status] );
            break;
        }
        default:
            warn_report("WARN command not supported jet\n");
    }
    
    return;
}

static void nvme_dev_event(void *opaque, int event)
{
    //NVMe_DEVState *s = opaque;
    
    QEMU_LOG_MASK(LOG_TRACE, "event %x\n", event);
/*    if (event == CHR_EVENT_BREAK)
        serial_receive_break(s);*/
}    


/////////////////////////////////////////////// DMA ///////////////////////////////////////////////
static int nvme_dev_can_receiveDMA(void *opaque)
{
    NVMe_DEVState *s = opaque;

    return s->can_receiveDMA;
}

static void nvme_dev_receiveDMA(void *opaque, const uint8_t *buf, int size)
{   
    //here the same code that we find in nvme_ctrl.c for the host ... because they do exactly the same
    NVMe_DEVState *s = opaque; 
    int size_to_copy = (size > s->can_receiveDMA) ? s->can_receiveDMA : size;
    
    // buffering first (in case the message is not completely transferred
    memcpy( ((char *) s->bufferDMA) + (DMA_PACKET_SIZE - s->can_receiveDMA), buf, size_to_copy);
    s->can_receiveDMA -= size_to_copy;
    if (s->can_receiveDMA)
        // more data need to be buffered, cannot process the message, just return
        return;

    QEMU_LOG_MASK(LOG_TRACE,"DMA receive %d [[%d]] %s addr:0x%lx (0x%lx) prp2:%lx off:%x len:%d seq:%d idx:%d flags:0x%x slot:%d\n",
                  size, (int)DMA_PACKET_SIZE, (s->bufferDMA->flags & DMA_FLAG_DIRECT) ? "DIRECT" : "AUTO",
                  s->bufferDMA->addr, le64_to_cpu(s->bufferDMA->addr), s->bufferDMA->prp2, s->bufferDMA->offset4KB,
                  s->bufferDMA->len, s->bufferDMA->sequence, s->bufferDMA->idx, s->bufferDMA->flags, (int)s->bufferDMA->slot);

    if ( (s->bufferDMA->flags & DMA_FLAG_TO_HOST) ) { 
        error_report("ERROR received a DMA packet for HOST but this is SSD code\n");
        goto exit_error;
    }    
       
    if ( (s->bufferDMA->flags & DMA_FLAG_DIRECT) ) {
        error_report("ERROR DMA RX DIRECT CURRENTLY NOT IMPLEMENTED\n");
    }
    else if ( (s->bufferDMA->flags & DMA_FLAG_AUTO) ) {
        // here we are identifying the request first using slot_id
        uint32_t slot = s->bufferDMA->slot;
        //uint64_t offset = (s->bufferDMA->offset4KB * DMA_DATA_SIZE); // this is only used in principle on the host side
        uint32_t devAddr = (uint32_t)s->bufferDMA->addr; // len is data len
        uint32_t size = 4096;
        
        QEMU_LOG_MASK(LOG_TRACE, "COMPLETION AUTO RX for slot %d\n", slot);
        
        nvme_devRequest * req = cmd_fifo_find(s, slot, ANY_VALUE);
        if ( !req ) {
            QEMU_LOG_MASK(LOG_TRACE, "ERROR CPL_FIFO_REG, cannot find slot %d in cmd_fifo. restart is needed\n", slot);
            goto exit_error;
        }
                
        if ( req->status != REQ_WAITING_COMPL )
            QEMU_LOG_MASK(LOG_TRACE, "ERROR the RX cmd_fifo status must be %d but it is %d\n", 
                    REQ_WAITING_COMPL, req->status);
        if ( req->cmd.prp2 != s->bufferDMA->prp2 )
            QEMU_LOG_MASK(LOG_TRACE, "ERROR the RX cmd_fifo prp2 must be 0x%lx but it is 0x%lx\n", 
                    req->cmd.prp2, s->bufferDMA->prp2);
                
        cpu_physical_memory_write( devAddr, &(s->bufferDMA->data[0]), size); //assumes that the offset is calculated by the FW (this is what the current FW is doing
        
        // now it is possible to increment the counter for the statistics
        ((HOST_DMA_FIFO_CNT_REG *) &(s->host_dma_fifo_cnt_reg))->autoDmaRx += 1;
        QEMU_LOG_MASK(LOG_TRACE, "AUTO TYPE RX done with receive FIFO CNT REG %x slot %d (written:%d data_size:%d)\n",
                s->host_dma_fifo_cnt_reg, slot, req->written, req->data_size);        
        
        //now handling the possible completion and substitution in SRAM
        req->written++; // we are doing this here even if it is not "written" but "transferred" 
        if (req->written < req->data_size) {
            QEMU_LOG_MASK(LOG_TRACE,
                "MORE DATA TO TRANSFER RX slot:%d qid:%d written:%d data_size:%d\n",
                req->slot, req->qid, req->written, req->data_size);
            goto exit_error;
        }

// TODO refactoring: the following need be to clean up: this is used in other part in this code, put in a function
        int cid = req->cmd.cid, sqId = req->qid;
        unsigned long prp1 = req->cmd.prp1;

        // here we have handle completion and removing it from RAM
        cmd_sram_dequeue(s, slot); // remove from CMD_SRAM
        QTAILQ_REMOVE(&(s->req_list), req, entry); // remove from CMD_FIFO
        g_free(req); // delete allocated space

        // enqueue the next one in SRAM
        //search the next in the LIST and enqeue it
        req = cmd_fifo_find(s, ANY_VALUE, REQ_NOT_IN_SRAM);
        if (req) {
            int _slot;
            if ( ( _slot = cmd_sram_enqueue(s,req)) >= 0 ) {
                req->slot = _slot;
                req->status = REQ_WAITING_PUBLISH;
            }

            if ( (req->status == REQ_WAITING_PUBLISH) && !(s->nvme_cmd_fifo_reg) ) {
                ((NVME_CMD_FIFO_REG *) &s->nvme_cmd_fifo_reg)->qID = req->qid;
                ((NVME_CMD_FIFO_REG *) &s->nvme_cmd_fifo_reg)->cmdSlotTag = req->slot;
                ((NVME_CMD_FIFO_REG *) &s->nvme_cmd_fifo_reg)->cmdSeqNum = req->cmd.cid; // ? this is not used in the firmware code!!!
                ((NVME_CMD_FIFO_REG *) &s->nvme_cmd_fifo_reg)->cmdValid = 1;
                
                req->status = REQ_WAITING_READ;
            }
        }
                
        //now construct the completion and send it
        NvmeCqe completion;
        completion.result = 0; //success
        completion.sq_head = 0; // TODO ?
        completion.sq_id = sqId;
        completion.cid = cid;
        completion.status = 0;
        //construct the completion -- private fields 
        ((NvmeCqe_rsvd *) &completion.rsvd)->cmd = NVME_SPEC_CMD;
        if (prp1 != ~0)
            ((NvmeCqe_rsvd *) &completion.rsvd)->rsvd = cqe_addr_hash(prp1);
        
#ifdef DEBUGME        
        nvme_dev_dump_cqe(&completion);
#endif        
        qemu_chr_fe_write_all(&s->server_chr, (const uint8_t * )&completion, sizeof(NvmeCqe));
/*****************************************************************************/
    }
exit_error:    
    s->can_receiveDMA = DMA_PACKET_SIZE;
    return;
}

static void nvme_dev_eventDMA(void *opaque, int event)
{
    //NVMe_DEVState *s = opaque;
    
    QEMU_LOG_MASK(LOG_TRACE,"DMA event %x\n", event);
}

////////////////////////////////////////////////////////////////////////////////////////////
// basic functions
////////////////////////////////////////////////////////////////////////////////////////////

/*
 * this function returns the result of the read
 */    
static uint64_t
nvme_dev_mem_read(void *opaque, hwaddr addr, unsigned size)
{
    uint32_t ret = 0;
    NVMe_DEVState *s = NVMe_DEV(opaque); 

    /* TODO the following can be avoided */
    if ( (addr >= NVME_CMD_SRAM_ADDR) && (addr < NVME_DEV_SIZE) ) {
        uint32_t *ptr = &(s->nvme_cmd_sram[0][0]);
        ret = ptr[(addr - NVME_CMD_SRAM_ADDR)/sizeof(uint32_t)];
        return ret;
    }    
    
    /* SQ set registers array */
    if ( (addr >= NVME_IO_SQ_SET_REG_ADDR) && (addr < NVME_IO_SQ_SET_REG_ADDR_END) ) {
        uint32_t *ptr = &(s->nvme_io_sq_set_reg[0][0]);
        ret = ptr[(addr - NVME_IO_SQ_SET_REG_ADDR)/sizeof(uint32_t)];
        return ret;
    }

    /* CQ set registers array */
    if ( (addr >= NVME_IO_CQ_SET_REG_ADDR) && (addr < NVME_IO_CQ_SET_REG_ADDR_END) ) {
        uint32_t *ptr = &(s->nvme_io_cq_set_reg[0][0]);
        ret = ptr[(addr - NVME_IO_CQ_SET_REG_ADDR)/sizeof(uint32_t)];
        return ret;
    }

    switch (addr) {
        case DEV_IRQ_MASK_REG_ADDR: 
            //TODO this is a write instruction only, cannot be handled here, no action
            break;
        case DEV_IRQ_CLEAR_REG_ADDR:
            //TODO this is a write instruction only, cannot be handled here, no action
            break;
        case DEV_IRQ_STATUS_REG_ADDR:
            ret = s->dev_irq_reg;
            break;
        case PCIE_STATUS_REG_ADDR:
            ret = s->pcie_status_reg;
            break;
        case PCIE_FUNC_REG_ADDR:
            ret = s->pcie_func_reg;
            break;
        case NVME_STATUS_REG_ADDR:
            ret = s->nvme_status_reg;
            break;
        case NVME_CMD_FIFO_REG_ADDR: {
            nvme_devRequest * req;
            int slot = ((NVME_CMD_FIFO_REG *) &s->nvme_cmd_fifo_reg)->cmdSlotTag;
            int qid = ((NVME_CMD_FIFO_REG *) &s->nvme_cmd_fifo_reg)->qID;
            
            // save the value to be returned
            ret = s->nvme_cmd_fifo_reg;
            if (!ret) // in case the register is clear just return
                break;
            
            // find the one currently published in CMD_FIFO, change its state to REQ_WAITING_COMPL
            req = cmd_fifo_find(s, slot, REQ_WAITING_READ);
            if ( !req || 
                 (req->qid !=  qid) ) {
                    error_report("ERROR CMD_FIFO_REG, cannot find slot %d in cmd_fifo. restart is needed\n", slot);
                    break;
            }
            // we have the element, we can now clear the register and change the status of the CMD_FIFO entry
            s->nvme_cmd_fifo_reg = 0; // needed for the check later
            req->status = REQ_WAITING_COMPL;
            
            // find another cmd to be published 
            // Note that considering we are not removing anything from the SRAM
            // we are not trying to add any new entry on the CMD_SRAM
            req = cmd_fifo_find(s, ANY_VALUE, REQ_WAITING_PUBLISH);           
            if ( req && !(s->nvme_cmd_fifo_reg) ) {
                ((NVME_CMD_FIFO_REG *) &s->nvme_cmd_fifo_reg)->qID = req->qid;
                ((NVME_CMD_FIFO_REG *) &s->nvme_cmd_fifo_reg)->cmdSlotTag = req->slot;
                ((NVME_CMD_FIFO_REG *) &s->nvme_cmd_fifo_reg)->cmdSeqNum = req->cmd.cid; // ? this is not used in the firmware code!!!
                ((NVME_CMD_FIFO_REG *) &s->nvme_cmd_fifo_reg)->cmdValid = 1;
                
                req->status = REQ_WAITING_READ;
            }
            
            break;
        }
        case NVME_ADMIN_QUEUE_SET_REG_ADDR:
            ret = s->nvme_admin_queue_set_reg;
            break;
            
        case NVME_CPL_FIFO_REG_ADDR0:
            ret = s->nvme_cpl_fifo_reg[0];
            break;
        case NVME_CPL_FIFO_REG_ADDR1:
            ret = s->nvme_cpl_fifo_reg[1];
            break;
        case NVME_CPL_FIFO_REG_ADDR2:
            ret = s->nvme_cpl_fifo_reg[2];
            break;

        case HOST_DMA_FIFO_CNT_REG_ADDR:
            ret = s->host_dma_fifo_cnt_reg;
            /*qemu_log_mask(LOG_TRACE, "s->host_dma_fifo_cnt_reg 0x%x 0x%x\n",
                          (unsigned int)ret, s->host_dma_fifo_cnt_reg); */
            break;
            
        case HOST_DMA_CMD_FIFO_REG_ADDR0:
            ret = s->host_dma_cmd_fifo_reg[0];
            break;
        case HOST_DMA_CMD_FIFO_REG_ADDR1:
            ret = s->host_dma_cmd_fifo_reg[1];
            break;
        case HOST_DMA_CMD_FIFO_REG_ADDR2:
            ret = s->host_dma_cmd_fifo_reg[2];
            break;
        case HOST_DMA_CMD_FIFO_REG_ADDR3:
            ret = s->host_dma_cmd_fifo_reg[3];
            break;
 
        default:
            warn_report("nvme_dev_mem_read: undefined memory address@hidden %x\n", (unsigned int)addr);
        break;
    }

    return ret;        
}    

/*
 * this function just writes
 */    
static void
nvme_dev_mem_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    NVMe_DEVState *s = NVMe_DEV(opaque);
    
    /* TODO the following can be avoided */
    if ( (addr >= NVME_CMD_SRAM_ADDR) && (addr < NVME_DEV_SIZE) ) {
        uint32_t *ptr = &(s->nvme_cmd_sram[0][0]);
        ptr[(addr - NVME_CMD_SRAM_ADDR)/sizeof(uint32_t)] = (uint32_t) val;
        return;
    }
    
    /* SQ set registers array */
    if ( (addr >= NVME_IO_SQ_SET_REG_ADDR) && (addr < NVME_IO_SQ_SET_REG_ADDR_END) ) {
        uint32_t *ptr = &(s->nvme_io_sq_set_reg[0][0]);
        ptr[(addr - NVME_IO_SQ_SET_REG_ADDR)/sizeof(uint32_t)] = (uint32_t) val;
        if ((addr & 4)) {
            // TODO trigger command
        /*	nvmeReg.valid = valid;
	nvmeReg.cqVector = cqVector;
	nvmeReg.sqSize = qSzie;
	nvmeReg.pcieBaseAddrL = pcieBaseAddrL;
	nvmeReg.pcieBaseAddrH = pcieBaseAddrH;
*/  
        }
        return;
    }

    /* CQ set registers array */
    if ( (addr >= NVME_IO_CQ_SET_REG_ADDR) && (addr < NVME_IO_CQ_SET_REG_ADDR_END) ) {
        uint32_t *ptr = &(s->nvme_io_cq_set_reg[0][0]);
        ptr[(addr - NVME_IO_SQ_SET_REG_ADDR)/sizeof(uint32_t)] = (uint32_t) val;
        if ((addr & 4)) {
            // TODO trigger command
        /*	nvmeReg.valid = valid;
	nvmeReg.cqVector = cqVector;
	nvmeReg.sqSize = qSzie;
	nvmeReg.pcieBaseAddrL = pcieBaseAddrL;
	nvmeReg.pcieBaseAddrH = pcieBaseAddrH;
*/  
        }
        return;
    }

    switch (addr) {
        case DEV_IRQ_MASK_REG_ADDR: 
            s->dev_irq_mask |= (uint32_t) val;
            QEMU_LOG_MASK(LOG_TRACE, "nvme_dev_mem_write IRQ_MASK val 0x%x reg 0x%x\n", (unsigned int)val, s->dev_irq_reg);
            break;
        case DEV_IRQ_CLEAR_REG_ADDR:
            s->dev_irq_reg &= (uint32_t) ~val;
            QEMU_LOG_MASK(LOG_TRACE, "nvme_dev_mem_write IRQ_CLEAR val 0x%x reg 0x%x\n", (unsigned int)val, s->dev_irq_reg);
            
            if ( !(s->dev_irq_reg) ) //if there are no other interrupts pending clear the irq line
                qemu_set_irq(s->irq, 0);
            break;
        case DEV_IRQ_STATUS_REG_ADDR:
            //TODO this is a read instruction only, cannot be handled here, no action
            break;
        case PCIE_STATUS_REG_ADDR:
            s->pcie_status_reg = (uint32_t) val;
            break;
        case PCIE_FUNC_REG_ADDR:
            s->pcie_func_reg = (uint32_t) val;
            break;
        case NVME_STATUS_REG_ADDR: {
            //uint32_t tmp = 0; // TODO
/*	nvmeReg.cstsRdy = rdy;
	nvmeReg.cstsShst = shst;
        */  
            //tmp = s->nvme_status_reg;
            s->nvme_status_reg = (uint32_t) val;
            //if (tmp != val) ACT, otherwise maybe just a trigger? TODO TODO DECISION: do the action anyway
                
            break;
        }
        case NVME_CMD_FIFO_REG_ADDR:
            s->nvme_cmd_fifo_reg = (uint32_t) val;
            break;
        case NVME_ADMIN_QUEUE_SET_REG_ADDR:
/*	nvmeReg.sqValid = sqValid;
	nvmeReg.cqValid = cqValid;
	nvmeReg.cqIrqEn = cqIrqEn;
        */
            s->nvme_admin_queue_set_reg = (uint32_t) val;
            break;

/////////////////////////////////////////////////////////////////////////////////////
// completion command
            
        case NVME_CPL_FIFO_REG_ADDR0:
            s->nvme_cpl_fifo_reg[0] = (uint32_t) val;
            break;
        case NVME_CPL_FIFO_REG_ADDR1:
            s->nvme_cpl_fifo_reg[1] = (uint32_t) val;
            break;
        case NVME_CPL_FIFO_REG_ADDR2: {
            int cid =-1, sqId =-1, slot =-1;
            uint64_t prp1 = ~0;
            NVME_CPL_FIFO_REG cpl_fifo_reg;
            
            s->nvme_cpl_fifo_reg[2] = (uint32_t) val;
            cpl_fifo_reg = *(NVME_CPL_FIFO_REG *) &s->nvme_cpl_fifo_reg;
            slot = cpl_fifo_reg.cmdSlotTag;

            QEMU_LOG_MASK(LOG_TRACE, "COMPLETION for slot %d\n", slot);
            
            // in the follofing IF the software only specify the slot, the rest is available in the nvme_cmd_sram entry
            if (cpl_fifo_reg.cplType == AUTO_CPL_TYPE ||
                cpl_fifo_reg.cplType == CMD_SLOT_RELEASE_TYPE) {
                // just remove the entry from the SDRAM but also from the list of elements
                
                //remove the one that is signaled by the software
                nvme_devRequest * req = cmd_fifo_find(s, slot, ANY_VALUE);
                if ( !req ) {
                    error_report("ERROR CPL_FIFO_REG, cannot find slot %d in cmd_fifo. restart is needed\n", slot);
                    break;
                }
                
                if ( req->status != REQ_WAITING_COMPL )
                    error_report("ERROR the cmd_fifo status must be %d but it is %d\n", 
                           REQ_WAITING_COMPL, req->status);
                
                prp1 = ((NvmeCmd *)&s->nvme_cmd_sram[slot])->prp1;                    
                cid = ((NvmeCmd *)&s->nvme_cmd_sram[slot])->cid;
                sqId = ((NvmeCmd_res1 *) &(((NvmeCmd *)&s->nvme_cmd_sram[slot])->res1) )->sqid;
                if ( (cid != req->cmd.cid) || (sqId != req->qid) )
                    error_report("ERROR CMD_SRAM info is different from CMD_FIFO info cid 0x%x 0x%x sqId 0x%x 0x%x\n",
                            cid, req->cmd.cid, sqId, req->qid);
                
                cmd_sram_dequeue(s, slot); // remove from CMD_SRAM
                QTAILQ_REMOVE(&(s->req_list), req, entry); // remove from CMD_FIFO
                g_free(req); // delete allocated space

                // enqueue the next one in SRAM
                //search the next in the LIST and enqeue it
                req = cmd_fifo_find(s, ANY_VALUE, REQ_NOT_IN_SRAM);
                if (req) {
                    int _slot;
                    if ( ( _slot = cmd_sram_enqueue(s,req)) >= 0 ) {
                        req->slot = _slot;
                        req->status = REQ_WAITING_PUBLISH;
                    }
// TODO we should consider to write the following in a function/macro                
                    if ( (req->status == REQ_WAITING_PUBLISH) && !(s->nvme_cmd_fifo_reg) ) {
                        ((NVME_CMD_FIFO_REG *) &s->nvme_cmd_fifo_reg)->qID = req->qid;
                        ((NVME_CMD_FIFO_REG *) &s->nvme_cmd_fifo_reg)->cmdSlotTag = req->slot;
                        ((NVME_CMD_FIFO_REG *) &s->nvme_cmd_fifo_reg)->cmdSeqNum = req->cmd.cid; // ? this is not used in the firmware code!!!
                        ((NVME_CMD_FIFO_REG *) &s->nvme_cmd_fifo_reg)->cmdValid = 1;
                
                        req->status = REQ_WAITING_READ;
                    }
                }
            }
            
            // here send a completion based on the data submitted or recovered
            if (cpl_fifo_reg.cplType == AUTO_CPL_TYPE ||
                cpl_fifo_reg.cplType == ONLY_CPL_TYPE) {
                NvmeCqe completion;
                
                // just send a completion
                if (cid == -1)
                    cid = cpl_fifo_reg.cid;
                if (sqId == -1)
                    sqId = cpl_fifo_reg.sqId;
                
                //construct the completion -- standard fields
                completion.result = cpl_fifo_reg.specific;
                completion.sq_head = 0; // TODO
                completion.sq_id = sqId;
                completion.cid = cid;
                completion.status = cpl_fifo_reg.statusFieldWord;
                //construct the completion -- private fields 
                ((NvmeCqe_rsvd *) &completion.rsvd)->cmd = NVME_SPEC_CMD;
                if (prp1 != ~0)
                    ((NvmeCqe_rsvd *) &completion.rsvd)->rsvd = cqe_addr_hash(prp1);

#ifdef DEBUGME                
                nvme_dev_dump_cqe(&completion);
#endif                
                qemu_chr_fe_write_all(&s->server_chr, (const uint8_t * )&completion, sizeof(NvmeCqe));
            }

            break;
        }
        
////////////////////////////////////////////////////////////////////////////////////////////////
// DMA FIFO handling

        case HOST_DMA_FIFO_CNT_REG_ADDR:
            //TODO this is used only in read -> no action
            //s->host_dma_fifo_cnt_reg[0] = (uint32_t) val;
            break;
            
        case HOST_DMA_CMD_FIFO_REG_ADDR0:
            s->host_dma_cmd_fifo_reg[0] = (uint32_t) val;
            break;
        case HOST_DMA_CMD_FIFO_REG_ADDR1:
            s->host_dma_cmd_fifo_reg[1] = (uint32_t) val;
            break;
        case HOST_DMA_CMD_FIFO_REG_ADDR2:
            s->host_dma_cmd_fifo_reg[2] = (uint32_t) val;
            break;
        case HOST_DMA_CMD_FIFO_REG_ADDR3: {
            s->host_dma_cmd_fifo_reg[3] = (uint32_t) val;
            
            uint32_t packets =0;
            HOST_DMA_CMD_FIFO_REG cmd_fifo = *(HOST_DMA_CMD_FIFO_REG *) &(s->host_dma_cmd_fifo_reg[0]);
            DMAPacket * pkt = g_malloc0(sizeof(DMAPacket) + DMA_DATA_SIZE);
            int i;
            
            if (cmd_fifo.dmaType == HOST_DMA_DIRECT_TYPE) {
                pkt->len = cmd_fifo.dmaLen;
                pkt->flags = DMA_FLAG_DIRECT;
                pkt->sequence = 0;
                
                switch (cmd_fifo.dmaDirection) {
                    case HOST_DMA_RX_DIRECTION: {
                        pkt->flags |= DMA_FLAG_TO_SSD;
                        pkt->addr = (uint64_t)cmd_fifo.devAddr;
                        g_free(pkt);
                        
                        
                        // this must be sent to the HOST driver has a cq packet if possible
                        // already implemented for AUTO transfers

                        
//    NVME_PRIV_CMD_DDMA_REQ, TODO TODO TODO

                        
                        warn_report("WARN DIRECT TYPE RX direction is not implemented yet\n");
                        break;
                    }
                    case HOST_DMA_TX_DIRECTION: {                       
                        pkt->flags |= DMA_FLAG_TO_HOST;
                        pkt->addr = (uint64_t) cmd_fifo.pcieAddrH << 32 | (uint64_t) cmd_fifo.pcieAddrL ;
                        
                        // this must be sent to the HOST has a DMA packet with data included
                        
                        // get new packet id from the state
                        pkt->idx = s->dma_pkt_idx++;
                        // how many packets need to be sent
                        packets =  cmd_fifo.dmaLen / DMA_DATA_SIZE + (cmd_fifo.dmaLen % DMA_DATA_SIZE) ? 1 : 0;
                        // send the packets in a loop
                        for (i=0; i < packets; i++) {
                            cpu_physical_memory_read(cmd_fifo.devAddr + (i*DMA_DATA_SIZE), //hwaddr
                                                     pkt->data, //qemu addr
                                                     (i+1 == packets) ? cmd_fifo.dmaLen : DMA_DATA_SIZE); //buffer, len
                            qemu_chr_fe_write_all(&s->dma_chr, 
                                                  (const uint8_t * )pkt, (sizeof(DMAPacket) + DMA_DATA_SIZE) );
                            pkt->sequence++;
                        }
                        // free resources
                        g_free(pkt);
                        QEMU_LOG_MASK(LOG_TRACE, "DIRECT TYPE TX done with trasmission len %d DmaTx %x packets %d\n",
                                (unsigned int)cmd_fifo.dmaLen, s->host_dma_fifo_cnt_reg, packets);
                        
                        break;
                    }
                }
            }
            // the following exploits the fact that the PRP is in the host memory and it will not be removed until we are done processing this
            else if (cmd_fifo.dmaType == HOST_DMA_AUTO_TYPE) {
                nvme_devRequest * req;
                uint8_t data_shift = 0; uint64_t data_size =0;
                
                // need to find the associated CMD in the SRAM before proceeding
                req = cmd_fifo_find(s, cmd_fifo.cmdSlotTag, ANY_VALUE);
                if ( !req ) {
                    error_report("ERROR AUTO TYPE cannot continue processing slot %d not found in cmd_fifo\n", 
                            cmd_fifo.cmdSlotTag);
                    return;
                }
                if ( req->status == REQ_INIT || req->status == REQ_NOT_IN_SRAM || 
                req->status == REQ_WAITING_PUBLISH || req->status == REQ_WAITING_READ ) {
                    warn_report("WARN AUTO TYPE nvme_devRequest status is wrong (%d)\n", 
                            req->status);
                    //return;
                }

                if (req->qid) { // this is IO command, here the infos about the command
                    NvmeRwCmd *rw = (NvmeRwCmd *)&(req->cmd);
                    uint32_t nlb  = le32_to_cpu(rw->nlb) + 1;
                    //uint64_t slba = le64_to_cpu(rw->slba);

                    //uint8_t lba_index  = NVME_ID_NS_FLBAS_INDEX(ns->id_ns.flbas);
                    data_shift = 12; //ns->id_ns.lbaf[lba_index].ds;
                    data_size = (uint64_t)nlb << data_shift;
                    //uint64_t data_offset = slba << data_shift;
                    pkt->size = (uint32_t)(data_size >> data_shift);
                }
                
                pkt->len = cmd_fifo.dmaLen = 4096; // use a macro, this corresponds to SECTOR_SIZE_FTL -- for the single transfer
                pkt->sequence = 0;
                pkt->slot = cmd_fifo.cmdSlotTag;
                pkt->flags = DMA_FLAG_AUTO;
                
                switch (cmd_fifo.dmaDirection) {
                    case HOST_DMA_RX_DIRECTION: {
                        //pkt->flags |= DMA_FLAG_TO_SSD; // this code must be moved
                        g_free(pkt);
                        
                        //req->written++; // moved this to the completion
                        if (!req->data_size)
                            req->data_size = (int)(data_size >> data_shift);
                        
                        // this must be sent to the HOST driver has a cq packet
                        //now construct the completion and send it
                        NvmeCqe completion;
                        ((NvmeCqe_rsvd *) &completion.rsvd)->cmd = NVME_PRIV_CMD;
                        ((NvmeCqe_rsvd *) &completion.rsvd)->priv = NVME_PRIV_CMD_DMA_REQ;
                        ((NvmeCqe_RXDMA *) &((NvmeCqe_rsvd *) &completion.rsvd)->rsvd)->slot = cmd_fifo.cmdSlotTag;
                        ((NvmeCqe_RXDMA *) &((NvmeCqe_rsvd *) &completion.rsvd)->rsvd)->offset = cmd_fifo.cmd4KBOffset;
                        
                        completion.result = cmd_fifo.devAddr; //we put here the SSD address (TODO we should create a linked list for this)
                        *((uint64_t *) &completion.sq_head) = req->cmd.prp1; // we put here prp1
                
                        qemu_chr_fe_write_all(&s->server_chr, (const uint8_t * )&completion, sizeof(NvmeCqe));
                        QEMU_LOG_MASK(LOG_TRACE,
                            "AUTO TYPE RX direction request sent slot:%d devAddr:0x%x offset:%d prp1:0x%lx (IO pages %d)\n",
                                cmd_fifo.cmdSlotTag, cmd_fifo.devAddr, cmd_fifo.cmd4KBOffset, req->cmd.prp1,
                                (int)(data_size >> data_shift) );                                  
                        break;
                    }
                    case HOST_DMA_TX_DIRECTION: {                       
                        pkt->flags |= DMA_FLAG_TO_HOST;
                        pkt->addr = req->cmd.prp1; // THIS IS THE PRP address in physical host memory
                        pkt->prp2 = req->cmd.prp2;
                        pkt->offset4KB = cmd_fifo.cmd4KBOffset;
                        
                        // this must be sent to the HOST has a DMA packet with data included
                                                
                        // get new packet id from the state
                        pkt->idx = s->dma_pkt_idx++;
                        // how many packets need to be sent
                        packets =  cmd_fifo.dmaLen / DMA_DATA_SIZE + (cmd_fifo.dmaLen % DMA_DATA_SIZE) ? 1 : 0;
                        // send the packets in a loop
                        for (i=0; i < packets; i++) {
                            cpu_physical_memory_read(cmd_fifo.devAddr + (i*DMA_DATA_SIZE), //hwaddr
                                                     pkt->data, DMA_DATA_SIZE); //buffer, len
                            qemu_chr_fe_write_all(&s->dma_chr, 
                                                  (const uint8_t * )pkt, (sizeof(DMAPacket) + DMA_DATA_SIZE) );
                            pkt->sequence++;
                        }
                        // free resources
                        g_free(pkt);
                        
                        //((HOST_DMA_FIFO_CNT_REG *) &(s->host_dma_fifo_cnt_reg))->autoDmaTx++; // this can only be done we receive the completion
                        
                        //req->written++; // moved this to the completion
                        if (!req->data_size)
                            req->data_size = (int)(data_size >> data_shift);
                        
                        QEMU_LOG_MASK(LOG_TRACE,
                            "AUTO TYPE TX done with trasmission len %d (pakets:%d) DmaTx %x (IO pages %d)\n",
                                (unsigned int)cmd_fifo.dmaLen, packets, s->host_dma_fifo_cnt_reg, 
                                (int)(data_size >> data_shift) );  
                        break;
                    }
                }
            }
            else {
                warn_report("unknown DMA type\n");
                break;
            }            

// TODO finish this by using a thread as well

/*	hostDmaReg.devAddr = devAddr;
	hostDmaReg.pcieAddrL = pcieAddrL;
	hostDmaReg.pcieAddrH = pcieAddrH;
	
	hostDmaReg.dword[3] = 0;
	hostDmaReg.dmaType = HOST_DMA_DIRECT_TYPE;
	hostDmaReg.dmaDirection = HOST_DMA_TX_DIRECTION;
	hostDmaReg.dmaLen = len;
        
        
        hostDmaReg.devAddr = devAddr;
	hostDmaReg.pcieAddrH = pcieAddrH;
	hostDmaReg.pcieAddrL = pcieAddrL;

	hostDmaReg.dword[3] = 0;
	hostDmaReg.dmaType = HOST_DMA_DIRECT_TYPE;
	hostDmaReg.dmaDirection = HOST_DMA_RX_DIRECTION;
	hostDmaReg.dmaLen = len;
        */

            break;
        }
        default:
            warn_report("nvme_dev_mem_read: undefined memory address@hidden %x\n", (unsigned int)addr);
        break;
    }
}
    
    
static const MemoryRegionOps mmio_ops = {
    .read  = nvme_dev_mem_read,
    .write = nvme_dev_mem_write,
    .endianness = DEVICE_LITTLE_ENDIAN, //DEVICE_NATIVE_ENDIAN
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};
    
static void nvme_dev_reset(DeviceState *ds)
{
    NVMe_DEVState *s = NVMe_DEV(SYS_BUS_DEVICE(ds));
/*    Error *local_errp = NULL;

    s->flash = DEVICE(object_property_get_link(OBJECT(s),
                                               "flash",
                                               &local_errp));
    if (local_errp) {
        fprintf(stderr, "ftnandc021: Unable to get flash link\n");
        abort();
    }
    */

    s->dma_pkt_idx = 0;

    s->dev_irq_mask = 0;
    s->dev_irq_reg = 0;
    s->pcie_status_reg = 0;
    s->pcie_func_reg = 0;
      
    s->nvme_status_reg = 0;
    s->nvme_cmd_fifo_reg = 0;
    s->nvme_admin_queue_set_reg = 0;
    
    s->nvme_cpl_fifo_reg[0] = 0; s->nvme_cpl_fifo_reg[1] = 0; s->nvme_cpl_fifo_reg[2] = 0;
    memset(&(s->nvme_io_sq_set_reg), 0, sizeof(s->nvme_io_sq_set_reg));
    memset(&(s->nvme_io_cq_set_reg), 0, sizeof(s->nvme_io_cq_set_reg));
    
    s->host_dma_fifo_cnt_reg = 0;
    s->host_dma_cmd_fifo_reg[0] = 0; s->host_dma_cmd_fifo_reg[1] = 0; 
    s->host_dma_cmd_fifo_reg[1] = 0; s->host_dma_cmd_fifo_reg[3] = 0;
    
    memset(&(s->nvme_cmd_sram), 0, sizeof(s->nvme_cmd_sram));
    
    /* We can assume our GPIO outputs have been wired up now */
    qemu_set_irq(s->irq, 0);

    QTAILQ_INIT(&s->req_list);
    s->cmd_sram_head = s->cmd_sram_tail = 0;
}

static void nvme_dev_realize(DeviceState *dev, Error **errp)
{
    //Error *local_err =NULL;
    NVMe_DEVState *s = NVMe_DEV(dev);
    Chardev *chr = qemu_chr_fe_get_driver(&s->server_chr);
    Chardev *dma_chr = qemu_chr_fe_get_driver(&s->dma_chr);

    if ( !chr ) {
        error_setg(errp, "nvme_dev_realize: Can't create nvme_dev ctrl, empty char device");
        return;
    }
    if ( !dma_chr ) {
        error_setg(errp, "nvme_dev_realize: Can't create nvme_dev ctrl, empty char device for DMA");
        return;
    }
    
    sysbus_init_mmio(s->sbd, &s->mmio);
    sysbus_mmio_map(s->sbd, 0, s->physical_address);   

        qemu_log_mask(LOG_TRACE,
                "nvme_dev_realize: mmio mapped\n");
    
    /*
    sysbus_init_irq(sbd, &s->irq); // from cadence_uart --calls-> qdev_init_gpio_out_named()
    
    qdev_init_gpio_in(&sbd->qdev, ftnandc021_handle_ack, 1); // from ftnandc021
    qdev_init_gpio_out(&sbd->qdev, &s->req, 1);              // from ftnandc021
    
    Depending on the fact that we know which interrupt to use or not the code changes,
    for current purposes it is not necessary to create and wire a gpio with a irq
    beacuse the qemu_irq has been already created by the cpu/platform code.
    More work is needed to make this more generic
    */
    // need to create my irq here first
    sysbus_init_irq(s->sbd, &s->irq);
    
    // now try to fetch CPU object to attach the IRQ
    BusState * default_bus = sysbus_get_default();
    DeviceState * cpu_dev = qdev_find_recursive(default_bus, "a9mpcore_priv");
    if ( !cpu_dev ) {
        error_setg(errp, "nvme_dev_realize: cannot find cpu object %p %p", default_bus, cpu_dev);
        return;
    }
    // this is copied by xilinx_zynq.c initialization -- basically connect the cpu interrupt with the NVMe ctrl interrupt
    qemu_irq cpu_irq = qdev_get_gpio_in(cpu_dev, (DEV_IRQ_ASSERT_INTR - DEV_IRQ_OFFSET));
    sysbus_connect_irq(s->sbd, 0, cpu_irq);
    
    /*
     * build the servers so that clients can connects
     */
    // chr frontend handlers for the nvme server
    qemu_chr_fe_set_handlers(&s->server_chr, nvme_dev_can_receive, nvme_dev_receive, nvme_dev_event, 0,
                             s, NULL, true);
    qemu_chr_fe_set_open(&s->server_chr, 1); // not sure this is necessary
    // chr frontend handlers for the DMA
    qemu_chr_fe_set_handlers(&s->dma_chr, nvme_dev_can_receiveDMA, nvme_dev_receiveDMA, nvme_dev_eventDMA, 0,
                             s, NULL, true);
    qemu_chr_fe_set_open(&s->dma_chr, 1); // not sure this is necessary
    
    qemu_log_mask(LOG_TRACE,
                "nvme_dev_realize: s %lx phys 0x%lx notifier %s DMA %s\n",
                (unsigned long) s, (unsigned long)s->physical_address, chr->filename, dma_chr->filename);
    
    s->can_receiveDMA = (sizeof(DMAPacket) + DMA_DATA_SIZE);
    s->bufferDMA = g_malloc0(s->can_receiveDMA); // TODO free this buffer when destroying the object
    s->dma_pkt_idx = 0;
}

static const VMStateDescription vmstate_nvme_dev = {
    .name = TYPE_NVME_DEV,
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(dev_irq_mask, NVMe_DEVState),
        VMSTATE_UINT32(dev_irq_reg, NVMe_DEVState),
        VMSTATE_UINT32(pcie_status_reg, NVMe_DEVState),
        VMSTATE_UINT32(pcie_func_reg, NVMe_DEVState),
        VMSTATE_UINT32(nvme_status_reg, NVMe_DEVState),
        VMSTATE_UINT32(nvme_cmd_fifo_reg, NVMe_DEVState),
        VMSTATE_UINT32(nvme_admin_queue_set_reg, NVMe_DEVState),
        VMSTATE_UINT32(host_dma_fifo_cnt_reg, NVMe_DEVState),
    
        VMSTATE_UINT32_ARRAY(nvme_cpl_fifo_reg, NVMe_DEVState, 3),
        VMSTATE_UINT32_2DARRAY(nvme_io_sq_set_reg, NVMe_DEVState, 8, 2),
        VMSTATE_UINT32_2DARRAY(nvme_io_cq_set_reg, NVMe_DEVState, 8, 2),
        VMSTATE_UINT32_ARRAY(host_dma_cmd_fifo_reg, NVMe_DEVState, 4),
        VMSTATE_UINT32_2DARRAY(nvme_cmd_sram, NVMe_DEVState, 128, 16),
        VMSTATE_END_OF_LIST()
    }
};    

static void nvme_dev_instance_init(Object *obj)
{
    NVMe_DEVState *s = NVMe_DEV(obj);
/* THIS IS TO LINK WITH THE FLASH DEVICES -- WE DON'T USE THIS FOR NOW
    object_property_add_link(obj,
                             "flash",
                             TYPE_DEVICE,
                             (Object **) &s->flash,
                             NULL);
                             */

    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    s->sbd =sbd;
    
    memory_region_init_io(&s->mmio,
                          obj,
                          &mmio_ops,
                          s,
                          TYPE_NVME_DEV,
                          NVME_DEV_SIZE);
}
    
/* TODO finish this */
static Property nvme_dev_properties[] = {
    DEFINE_PROP_CHR("chardev", NVMe_DEVState, server_chr),
    DEFINE_PROP_CHR("dmadev", NVMe_DEVState, dma_chr),
    DEFINE_PROP_UINT64("phys", NVMe_DEVState, physical_address, NVME_HOST_IP_ADDR),
    DEFINE_PROP_END_OF_LIST(),
};

static void nvme_dev_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd    = &vmstate_nvme_dev;
    dc->reset   = nvme_dev_reset;
    dc->realize = nvme_dev_realize;
    dc->props = nvme_dev_properties;
    dc->hotpluggable = true;
    dc->user_creatable = true;
    //dc->no_user = 1;
}

static const TypeInfo nvme_dev_info = {
    .name          = TYPE_NVME_DEV,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NVMe_DEVState),
    .instance_init = nvme_dev_instance_init,
    .class_init    = nvme_dev_class_init,
};

static void nvme_dev_register_types(void)
{
    type_register_static(&nvme_dev_info);
}

type_init(nvme_dev_register_types)
