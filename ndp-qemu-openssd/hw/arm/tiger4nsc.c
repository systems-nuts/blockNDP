/*
 * Antonio Barbalace, 2017
 *
 * Most of this file originates from the fmc_driver source code of Cosmos+
 * Note that the file has been tested only with two ways and respective geometry
 * every other configuration is not guaranteed to work.
 */

/**
 * Usage: add options:
 *      -drive file=<file_rawFlashDrive>,if=none,format=raw,id=<rawFlashDrive_id>
 *      -device tiger4nsc,drive=<rawFlashDrive_id>,ways=<ways_num>,phys=<phys_addr>
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

/* configurable options */

#define	WAY_NUM					8

/* fixed options */

// NOTE: there is a device controller per channel -- thus, this is not a configuration option
/*
#define	CHANNEL_NUM				2
#define	MAX_CHANNEL_NUM	                        8
*/

#define	MAX_WAY_NUM				8
//#define	DIE_NUM					(CHANNEL_NUM * WAY_NUM)

#define V2Fmmio_SIZE 0x10000
#define V2Fmmio_ADDR 0x43c00000

//ECC error information
#define ERROR_INFO_NUM 11

#define	SECTOR_SIZE_FTL 4096	//4KB
//#define	PAGE_SIZE       16384	//16KB
#define	PAGE_SIZE       (16384/2)
#define SPARE_SIZE      256	//last 8 bytes are CRC bytes

#define	SLC_MODE				1
#define	MLC_MODE				2
#define	BIT_PER_FLASH_CELL		SLC_MODE //select SLC_MODE or MLC_MODE

#define	PAGE_NUM_PER_BLOCK		(128 * BIT_PER_FLASH_CELL)
#define	PAGE_NUM_PER_SLC_BLOCK	128
#define	PAGE_NUM_PER_MLC_BLOCK	256

#define	BLOCK_NUM_PER_LUN		512 //DRAM size doesn't enough for page mapping when MLC mode uses all blocks. If you want to use all blocks, map cache function should be implemented.
#define	MAX_BLOCK_NUM_PER_LUN	4096
#define LUN_NUM_PER_DIE			2
#define	MAX_LUN_NUM_PER_DIE		2
#define	BLOCK_SIZE_MB			((PAGE_SIZE * PAGE_NUM_PER_BLOCK) / (1024 * 1024))

//LUN
#define LUN_0_BASE_ADDR	0x00000000
#define LUN_1_BASE_ADDR	0x00200000

static inline unsigned int rowAddress_get_linear(unsigned int phy_RowAddress) {
    unsigned int rowAddress = (phy_RowAddress < LUN_1_BASE_ADDR) ? 0 : ((128 / BIT_PER_FLASH_CELL) * BLOCK_NUM_PER_LUN);
    if (rowAddress)
        phy_RowAddress -= LUN_1_BASE_ADDR;

// in the hope that everything will be configurable in the future    
    if (BIT_PER_FLASH_CELL == SLC_MODE) {
        unsigned int tempPage; //, _tempPage;
        unsigned int tempBlock;
        
        if ((phy_RowAddress & (2* PAGE_NUM_PER_BLOCK -1)) == (2* PAGE_NUM_PER_BLOCK -1)) {// this supports LLSCommand_ReadRawPage, LLSCommand_ReadLsbPage, LLSCommand_WriteLsbPage
            tempPage = (PAGE_NUM_PER_BLOCK -1);
            tempBlock = phy_RowAddress - (2* PAGE_NUM_PER_BLOCK -1);
            
/*            _tempPage = ((phy_RowAddress % 2) ? ((phy_RowAddress +1) >> 1) % PAGE_NUM_PER_BLOCK : 0);
            qemu_log_mask(LOG_TRACE,
                "get_linear: addr %x tmpPage %x tmpBlock %x OR %x %x (%x)\n", 
                phy_RowAddress, tempPage, tempBlock,
                _tempPage, phy_RowAddress - ((_tempPage == 0)? 0 : ((_tempPage * 2) -1)), phy_RowAddress - ((_tempPage * 2) -1) ); */
        }
        else {
            tempPage = (phy_RowAddress % 2) ? 
                                ((phy_RowAddress +1) >> 1) % PAGE_NUM_PER_BLOCK : 0;
            tempBlock = phy_RowAddress - ((tempPage == 0)? 0 : ((tempPage * 2) -1));
        }
        
        tempBlock = tempBlock /2;
        rowAddress += tempPage + tempBlock;
        
    }
    else if (BIT_PER_FLASH_CELL == MLC_MODE) {
        rowAddress += phy_RowAddress;
    }
        
    return rowAddress;
}

typedef struct
{
	unsigned int cmdSelect;    //addr 0
	unsigned int rowAddress;   //addr 4
	unsigned int userData;     //addr 8
	unsigned int dataAddress;  //addr 12
        
	unsigned int spareAddress; //addr 16 (0x10)
	unsigned int errorCountAddress; //addr 20
	unsigned int completionAddress; //addr 24
	unsigned int waySelection; //addr 28
        
	unsigned int channelBusy;  //addr 32 (0x20)
	unsigned int readyBusy;    //addr 36
} V2FMCRegisters;

//offsets
#define OFFSET_cmdSelect 0
#define OFFSET_rowAddress 4 
#define OFFSET_userData 8
#define OFFSET_dataAddress 12
#define OFFSET_spareAddress 16 
#define OFFSET_errorCountAddress 20
#define OFFSET_completionAddress 24
#define OFFSET_waySelection 28

#define OFFSET_channelBusy 32
//there is a ready bit per way
#define OFFSET_readyBusy 36

//commands
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

struct _Tiger4nscState;

typedef struct tiger4nscRequest {
    struct _Tiger4nscState * state;
    BlockAcctCookie cookie;

    BlockAIOCB*     aiocb;
    uint32_t        cmdSelect;
    uint32_t        rowAddress;
    uint32_t        waySelection;
    uint32_t        completionAddress;
    uint32_t        errorCountAddress;   

    void*           erased_block;
    
    uint32_t        has_qsg;
    QEMUSGList      qsg;
    QEMUIOVector    iov;
    QTAILQ_ENTRY(tiger4nscRequest) entry;
} tiger4nscRequest;


#define TYPE_TIGER4NSC "tiger4nsc"

typedef struct _Tiger4nscState {
    /*< private >*/
    SysBusDevice parent;

    /*< private >*/
    BlockConf conf;
    
    /*< public >*/
    MemoryRegion mmio;
    
    /*< public >*/
//    BlockAcctStats acct; //NOTE that a block already includes stats
    
    /*
    qemu_irq irq;
    DeviceState *flash;

    // DMA hardware handshake
    qemu_irq req;
    */
    SysBusDevice * sbd;

// START TODO    
    uint8_t  manf_id, chip_id;
    uint64_t physical_address;

    
    //Rem that channels is not an option as one controller is one channel
    uint8_t ways;
    uint64_t spare_size;
    
    uint64_t page_size;
    uint64_t page_num_per_block;
    
    uint64_t lun_num_per_die;
    
    uint64_t block_size;
    uint64_t block_num_per_lun;
    uint64_t block_num_per_die;
    uint64_t block_num_per_channel;
    uint64_t total_size;
    uint64_t  disk_size;
    
    //tiger4nscRequest * io_req; //can be optimized by creating a free list
    QTAILQ_HEAD(req_list, tiger4nscRequest) req_list; // requests waiting for completion

// END TODO

    /* HW register caches */
    uint32_t cmdSelect;
    uint32_t rowAddress;
    uint32_t userData;
    uint32_t dataAddress;
    uint32_t spareAddress;
    uint32_t errorCountAddress;
    uint32_t completionAddress;
    uint32_t waySelection;
    uint32_t channelBusy;
    uint32_t readyBusy;    
} Tiger4nscState;

#define TIGER4NSC(obj) \
    OBJECT_CHECK(Tiger4nscState, obj, TYPE_TIGER4NSC)
    
//request status
#define RS_RUNNING	0
#define RS_DONE		1
#define RS_FAIL		2
#define RS_WARNING	3    

static void tiger4nsc_completion(void *opaque, int ret)    
{
    tiger4nscRequest *req = opaque;
    Tiger4nscState *s = req->state;
    
    //Note that this is writing up to 11bytes of ECC of some other kind of error check struct errorInfoArray
    uint32_t error[ERROR_INFO_NUM] = {0xFF000000, ~0x00}; // 0xffffffff means no error for the second entry
    uint32_t completion = 0x1; // status completed
    completion |= 0x60 << 1; // done
    
    if (ret <0) { // case of failure
        completion |= 0x03 << 1; // failure
        block_acct_failed(blk_get_stats(s->conf.blk), &req->cookie); //statistics
    }
    else { // case of everything went well
        block_acct_done(blk_get_stats(s->conf.blk), &req->cookie); //statistics
    }
    
    QTAILQ_REMOVE(&(s->req_list), req, entry);
    
    switch (req->cmdSelect) {
        case V2FCommand_ReadPageTransferRaw:
        case V2FCommand_ReadPageTransfer:
        case V2FCommand_ProgramPage: {
            cpu_physical_memory_write(req->completionAddress, &completion, sizeof(completion));
            
            if ( (req->cmdSelect == V2FCommand_ReadPageTransfer) ||
                 (req->cmdSelect == V2FCommand_ProgramPage) )
                if ( req->errorCountAddress != 0 &&
                    req->errorCountAddress != ~0 )
                        cpu_physical_memory_write(req->errorCountAddress, error, sizeof(error));
    
            if (req->has_qsg)
                qemu_sglist_destroy(&(req->qsg));
            
            qemu_log_mask(LOG_TRACE,
                "tiger4nsc_completion: %lx Read/Program ret %d comp @ %lx (val 0x%x) erCnt @ %lx (val 0x%x:0x%x)\n", 
                s->physical_address, ret, (long unsigned )req->completionAddress, completion, 
                                          (long unsigned)req->errorCountAddress, error[0], error[1]);
            break;
        }
        case V2FCommand_BlockErase: {
            if (!(req->has_qsg))
                qemu_iovec_destroy(&(req->iov));
            if (req->erased_block)
                g_free(req->erased_block);

            // TODO how to handle the error ??? because we cannot write anywhere with the erase command ...
            
            qemu_log_mask(LOG_TRACE,
                "tiger4nsc_completion: %lx Erase ret %d comp @ %lx erCnt @ %lx way %d @ 0x%x req@%lx\n", 
                s->physical_address, ret, (long unsigned )req->completionAddress, (long unsigned)req->errorCountAddress, (unsigned) req->waySelection, (unsigned) req->rowAddress, (unsigned long)req);
            break;
        }
        default:
            qemu_log_mask(LOG_TRACE,
                "tiger4nsc_completion: %lx ILLEGAL COMMAND %d\n", 
                s->physical_address, req->cmdSelect);
    }
    
    g_free(req);
}    
    
static void tiger4nsc_command(Tiger4nscState *s, uint32_t cmd)
{
    int file_align = 1;

    switch (cmd) {
        /* StatusCheck command
         * completionAddress register contains the physical address on which to write the status
         * the completion status is an integer
         * 
         * NOT CLEAR: what does this mean when there is no active command?
         */
        case V2FCommand_StatusCheck: {
            tiger4nscRequest *p = 0, *q = 0;
            uint32_t completion =0;
            int stats =0, found =0;
            
            //completion |= 0x03 << 1; // failure -- there cannot be any failure here
           
           //the algorithm is the following: I can find anything on the list with the same completion address I either don't do anything or write just 0x1
            //I cannot find anything on the list, in that case I write completion done
            
            QTAILQ_FOREACH_SAFE(p, &(s->req_list), entry, q) {
                if ( (p->waySelection == s->waySelection) &&
                     (p->cmdSelect == V2FCommand_BlockErase) ) {
                            completion = 0x1; // status completed
                            cpu_physical_memory_write(s->completionAddress, &completion, sizeof(completion));
                            found++;
                    break;
                }
                
                if ( (p->completionAddress == s->completionAddress) &&
                     (p->waySelection == s->waySelection) ) {
                            found++;
                    break;
                }
                stats++;
            }
            
            if (found == 0) {
                    completion = 0x1; // status completed
                    completion |= 0x60 << 1; // done
                    cpu_physical_memory_write(s->completionAddress, &completion, sizeof(completion));
            }                
            
            qemu_log_mask(LOG_TRACE,
                "tiger4nsc_command: %lx StatusCheck rowAddr %x (%x) userData %x dataAddr %x spareAddr %x waySel %x completion %x (0x%x) stats %d\n", 
                s->physical_address, s->rowAddress, (int)rowAddress_get_linear(s->rowAddress),
                s->userData, s->dataAddress, s->spareAddress, s->waySelection, s->completionAddress, completion, stats);

            break;
        }
        /* ReadPageTransfer command
         * OUTPUTs
         * completionAddress reguster contains the physical address on which to write the status
         * the completion status is an integer (unit32_t)
         * the errorInfo is also an integer (uint32_t)
         * NEED TO WRITE THE PAGE IN dataAddress PAGE_SIZE 16384 //16KB
         * NEED TO WRITE THE PAGE IN spareDataAddress SPARE_SIZE 256
         * INPUTs
         * way
         * rowAddress
         * pageDataBuffer -> dataAddress
         * spareDataBuffer -> spareDataAddress
         */        
        case V2FCommand_ReadPageTransferRaw:
        case V2FCommand_ReadPageTransfer: {
            tiger4nscRequest * req = g_malloc0(sizeof(tiger4nscRequest));
            long file_offset = 0;
            //enum BlockAcctType acct = is_write ? BLOCK_ACCT_WRITE : BLOCK_ACCT_READ;

            memset(req, 0, sizeof(tiger4nscRequest));
            req->state = s;
            req->cmdSelect = cmd; //supports ReadPageTransfer and ReadPageTransferRaw
            req->completionAddress = s->completionAddress;
            req->errorCountAddress = (cmd == V2FCommand_ReadPageTransfer) ? s->errorCountAddress : ~0;
            req->waySelection = s->waySelection;
            QTAILQ_INSERT_TAIL(&(s->req_list), req, entry); //enqueue in the list of completions
            
            qemu_sglist_init(&(req->qsg), DEVICE(s), 2, &address_space_memory); // rem to destroy this TODO TODO TODO            
            if (cmd == V2FCommand_ReadPageTransfer) {
                qemu_sglist_add(&(req->qsg), s->dataAddress, PAGE_SIZE); //data
                qemu_sglist_add(&(req->qsg), s->spareAddress, SPARE_SIZE); //spare
            }
            else {
               qemu_sglist_add(&(req->qsg), s->dataAddress, (PAGE_SIZE + SPARE_SIZE) ); //data + spare
            }
            req->has_qsg =1;
            dma_acct_start(s->conf.blk, &(req->cookie), &(req->qsg), BLOCK_ACCT_READ);
            
            file_offset = (rowAddress_get_linear(s->rowAddress) * (PAGE_SIZE + SPARE_SIZE)) + 
                        (s->waySelection * (PAGE_SIZE + SPARE_SIZE) * s->page_num_per_block * s->block_num_per_die); // in one channel dies = ways
            req->aiocb = dma_blk_read(s->conf.blk, &(req->qsg), 
                         file_offset, file_align, 
                         tiger4nsc_completion, req); //opaque is a void pointer
            
            //qemu_log_mask(LOG_GUEST_ERROR,
            qemu_log_mask(LOG_TRACE,
                "tiger4nsc_command: %lx %s rowAddr %x (%x) userData %x dataAddr %x spareAddr %x waySel %x/%x  completion %x (file_offset:0x%lx)\n", 
                s->physical_address, 
                (cmd == V2FCommand_ReadPageTransfer) ? "ReadPageTransfer" : (cmd == V2FCommand_ReadPageTransferRaw) ? "ReadPageTransferRaw" : "ReadPageTransferXXX",
                s->rowAddress, (int)rowAddress_get_linear(s->rowAddress),
                s->userData, s->dataAddress, s->spareAddress, s->waySelection, s->ways, s->completionAddress,
                file_offset);           
            
            break;
        }
        case V2FCommand_ProgramPage:{
            tiger4nscRequest * req = g_malloc0(sizeof(tiger4nscRequest));
            long file_offset = 0; 

            memset(req, 0, sizeof(tiger4nscRequest));
            req->state = s;
            req->cmdSelect = V2FCommand_ProgramPage;
            req->completionAddress = s->completionAddress;
            req->errorCountAddress = s->errorCountAddress;
            req->waySelection = s->waySelection;
            QTAILQ_INSERT_TAIL(&(s->req_list), req, entry); //enqueue in the list of completions
            
            qemu_sglist_init(&(req->qsg), DEVICE(s), 2, &address_space_memory); // rem to destroy this TODO TODO TODO            
            qemu_sglist_add(&(req->qsg), s->dataAddress, PAGE_SIZE); //data
            qemu_sglist_add(&(req->qsg), s->spareAddress, SPARE_SIZE); //spare
            req->has_qsg =1;
            dma_acct_start(s->conf.blk, &(req->cookie), &(req->qsg), BLOCK_ACCT_WRITE);
            
            file_offset = (rowAddress_get_linear(s->rowAddress) * (PAGE_SIZE + SPARE_SIZE)) + 
                        (s->waySelection * (PAGE_SIZE + SPARE_SIZE) * s->page_num_per_block * s->block_num_per_die); // in one channel dies = ways
            req->aiocb = dma_blk_write(s->conf.blk, &(req->qsg), 
                         file_offset, file_align, 
                         tiger4nsc_completion, req); //opaque is a void pointer          
            
            //qemu_log_mask(LOG_GUEST_ERROR,
            qemu_log_mask(LOG_TRACE,
                "tiger4nsc_command: %lx ProgramPage rowAddr %x (%x) userData %x dataAddr %x spareAddr %x waySel %x/%x completion %x (file_offset:0x%lx)\n", 
                s->physical_address, s->rowAddress, (int)rowAddress_get_linear(s->rowAddress),
                s->userData, s->dataAddress, s->spareAddress, s->waySelection, s->ways, s->completionAddress, file_offset);           
            
            break;
        }   
        
        /*
         * there is no dataAddress or spareAddress passed to this command, no completion either ...
         * ONLY 
         *  -- way
         *  -- rowAddress
         */
        case V2FCommand_BlockErase:{
            tiger4nscRequest * req = g_malloc0(sizeof(tiger4nscRequest));
            
            void * erased_block = g_malloc0(PAGE_SIZE + SPARE_SIZE); // move this as a global or per instance variable
            long file_offset = 0; size_t size = 0; //, _size=0;
            int i;
            
            if (erased_block)
                memset(erased_block, (~0), (PAGE_SIZE + SPARE_SIZE)); // TODO ERASE THIS IN THE COMPLETION

            req->erased_block = erased_block;

            memset(req, 0, sizeof(tiger4nscRequest));
            req->state = s;
            req->cmdSelect = V2FCommand_BlockErase;
            req->completionAddress = ~0;
            req->errorCountAddress = ~0;
            req->waySelection = s->waySelection;
            QTAILQ_INSERT_TAIL(&(s->req_list), req, entry); //enqueue in the list of completions

            //CREATE AN IOVEC list this is because we only know host address -- again, for performance this can be created once at init and being used at runtime multiple times 
            qemu_iovec_init(&(req->iov), PAGE_NUM_PER_BLOCK);
            for (i=0; i< PAGE_NUM_PER_BLOCK; i++)
                qemu_iovec_add(&(req->iov), erased_block, (PAGE_SIZE + SPARE_SIZE));
            // for the same reason accounting cannot be done with the dma_* APIs
            block_acct_start(blk_get_stats(s->conf.blk), &(req->cookie),
                           size = iov_size((req->iov.iov), PAGE_NUM_PER_BLOCK), BLOCK_ACCT_WRITE);
                            //_size = req->iov.size;
            
            //NOTE this way we are forcing to have continuous pages for each block (thus, two different ways cannot be interleaved
            file_offset = (rowAddress_get_linear(s->rowAddress) * (PAGE_SIZE + SPARE_SIZE)) + 
                        (s->waySelection * (PAGE_SIZE + SPARE_SIZE) * s->page_num_per_block * s->block_num_per_die); // in one channel dies = ways
            req->rowAddress = file_offset;
            
            req->aiocb = blk_aio_pwritev(s->conf.blk, file_offset, &(req->iov), 0, tiger4nsc_completion, req);           
            
            qemu_log_mask(LOG_TRACE,
                "tiger4nsc_command: %lx BlockErase rowAddr %x (%x) userData %x??? dataAddr %x??? spareAddr %x??? waySel %x completion %x??? (size: %ld) req@%lx (file_offset:0x%lx)\n", 
                s->physical_address, s->rowAddress, (int)rowAddress_get_linear(s->rowAddress),
                s->userData, s->dataAddress, s->spareAddress, s->waySelection, s->completionAddress, (unsigned long) size, (unsigned long) req, file_offset);
            break;
        }
        /*case V2FCommand_NOP: {
            //qemu_log_mask(LOG_GUEST_ERROR,
            qemu_log_mask(LOG_TRACE,
                "tiger4nsc_command: %lx NOP rowAddr %x (%x) userData %x dataAddr %x spareAddr %x waySel %x ERROR\n", 
                s->physical_address, s->rowAddress, (int)rowAddress_get_linear(s->rowAddress),
                s->userData, s->dataAddress, s->spareAddress, s->waySelection);           
            break;
        }
        case V2FCommand_Reset: {
            //qemu_log_mask(LOG_GUEST_ERROR,
            qemu_log_mask(LOG_TRACE,
                "tiger4nsc_command: %lx Reset rowAddr %x (%x) userData %x dataAddr %x spareAddr %x waySel %x ERROR\n", 
                s->physical_address, s->rowAddress, (int)rowAddress_get_linear(s->rowAddress),
                s->userData, s->dataAddress, s->spareAddress, s->waySelection);           
            break;
        }
        case V2FCommand_SetFeatures: {
            //qemu_log_mask(LOG_GUEST_ERROR,
            qemu_log_mask(LOG_TRACE,
                "tiger4nsc_command: %lx SetFeatures rowAddr %x (%x) userData %x dataAddr %x spareAddr %x waySel %x ERROR\n", 
                s->physical_address, s->rowAddress, (int)rowAddress_get_linear(s->rowAddress),
                s->userData, s->dataAddress, s->spareAddress, s->waySelection);           
            break;
        }
        case V2FCommand_GetFeatures: {
            //qemu_log_mask(LOG_GUEST_ERROR,
            qemu_log_mask(LOG_TRACE,
                "tiger4nsc_command: %lx GetFeatures rowAddr %x (%x) userData %x dataAddr %x spareAddr %x waySel %x ERROR\n", 
                s->physical_address, s->rowAddress, (int)rowAddress_get_linear(s->rowAddress),
                s->userData, s->dataAddress, s->spareAddress, s->waySelection);           
            break;
        }
        case V2FCommand_ReadPageTrigger: {
            //qemu_log_mask(LOG_GUEST_ERROR,
            qemu_log_mask(LOG_TRACE,
                "tiger4nsc_command: %lx ReadPageTrigger rowAddr %x (%x) userData %x dataAddr %x spareAddr %x waySel %x ERROR\n", 
                s->physical_address, s->rowAddress, (int)rowAddress_get_linear(s->rowAddress),
                s->userData, s->dataAddress, s->spareAddress, s->waySelection);           
            break;
        }*/
        case V2FCommand_NOP:
        case V2FCommand_Reset:
        case V2FCommand_SetFeatures:
        case V2FCommand_GetFeatures:
        case V2FCommand_ReadPageTrigger:
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
            //qemu_log_mask(LOG_TRACE,
                "tiger4nsc_command: undefined command %x ERROR\n", (unsigned int)cmd);    
        break;
    }
}
    
/*
 * this function returns the result of the read
 */    
static uint64_t
tiger4nsc_mem_read(void *opaque, hwaddr addr, unsigned size)
{
    uint32_t ret = 0;
    Tiger4nscState *s = TIGER4NSC(opaque); 

    switch (addr) {
        case OFFSET_cmdSelect: 
            ret = s->cmdSelect;
            break;                                    
        case OFFSET_rowAddress:
            ret = s->rowAddress;
            break;                                    
        case OFFSET_userData:
            ret = s->userData;
            break;                                    
        case OFFSET_dataAddress:
            ret = s->dataAddress;
            break;                                    
        case OFFSET_spareAddress:
            ret = s->spareAddress;
            break;                                    
        case OFFSET_errorCountAddress:
            ret = s->errorCountAddress;
            break;                                    
        case OFFSET_completionAddress:
            ret = s->completionAddress;
            break;                                    
        case OFFSET_waySelection:
            ret = s->waySelection;
            break;                        
        case OFFSET_channelBusy:
            ret = s->channelBusy;
            break;                        
        case OFFSET_readyBusy:
            ret = s->readyBusy;
            break;            
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
            //qemu_log_mask(LOG_TRACE,
                //"tiger4nsc_mem_read: undefined memory address@hidden %" HWADDR_PRId "\n", addr);
                "tiger4nsc_mem_read: undefined memory address@hidden %x\n", (unsigned int)addr);
        break;
    }

    return ret;        
}    

/*
 * this function just writes
 */    
static void
tiger4nsc_mem_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    //uint32_t i;
    Tiger4nscState *s = TIGER4NSC(opaque);

    switch (addr) {
        case OFFSET_cmdSelect: 
            s->cmdSelect = (uint32_t)val;
            tiger4nsc_command(s, val);
            break;
        case OFFSET_rowAddress:
            s->rowAddress = (uint32_t)val;
            break;
        case OFFSET_userData:
            s->userData = (uint32_t)val;
            break;
        case OFFSET_dataAddress:
            s->dataAddress = (uint32_t)val;
            break;
        case OFFSET_spareAddress:
            s->spareAddress = (uint32_t)val;
            break;
        case OFFSET_errorCountAddress:
            s->errorCountAddress = (uint32_t)val;
            break;
        case OFFSET_completionAddress:
            s->completionAddress = (uint32_t)val;
            break;
        case OFFSET_waySelection:
            s->waySelection = (uint32_t)val;
            break;
//these two in principle cannot be written            
        case OFFSET_channelBusy:
            //s->rowAddress = (uint32_t)val);
            break;
        case OFFSET_readyBusy:
            //s->readyBusy = (uint32_t)val;
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
            //qemu_log_mask(LOG_TRACE,
                //"tiger4nsc_mem_read: undefined memory address@hidden %" HWADDR_PRId "\n", addr);
                "tiger4nsc_mem_read: undefined memory address@hidden %x\n", (unsigned int)addr);
        break;
    }
}
    
    
static const MemoryRegionOps mmio_ops = {
    .read  = tiger4nsc_mem_read,
    .write = tiger4nsc_mem_write,
    .endianness = DEVICE_LITTLE_ENDIAN, //DEVICE_NATIVE_ENDIAN
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};
    
static void tiger4nsc_reset(DeviceState *ds)
{
    Tiger4nscState *s = TIGER4NSC(SYS_BUS_DEVICE(ds));
/*    Error *local_errp = NULL;

    s->flash = DEVICE(object_property_get_link(OBJECT(s),
                                               "flash",
                                               &local_errp));
    if (local_errp) {
        fprintf(stderr, "ftnandc021: Unable to get flash link\n");
        abort();
    }
    */
// not sure that those number are valid for every device
    s->spare_size = SPARE_SIZE; //in bytes

    s->page_size = PAGE_SIZE; //in bytes
    s->page_num_per_block = 128 * BIT_PER_FLASH_CELL;
    //s->page_num_per_lun = (s->page_num_per_block * s->block_num_per_lun);
    
    s->block_size = (s->page_size * s->page_num_per_block);
    s->block_num_per_lun = (BLOCK_NUM_PER_LUN / BIT_PER_FLASH_CELL);

    s->lun_num_per_die = LUN_NUM_PER_DIE;
    
    s->block_num_per_die = (s->block_num_per_lun * s->lun_num_per_die);
    s->block_num_per_channel = (s->block_num_per_die * s->ways);
    
    s->total_size = (s->block_num_per_channel * s->block_size);
    
    s->disk_size = (s->block_num_per_channel * 
                    ((s->page_size + s->spare_size) * s->page_num_per_block) ); 

    qemu_log_mask(LOG_TRACE,
                "tiger4nsc_reset: object at %lx size: %ld MB (%ld MB)\n",
                (unsigned long)(void*)ds, (s->total_size >> 20), (s->disk_size >> 20) );   
    qemu_log_mask(LOG_TRACE,
                "tiger4nsc_reset: page_num_per_block: %ld block_num_per_lun:%ld block_num_per_die:%ld block_num_per_channel:%ld \n",
                s->page_num_per_block, s->block_num_per_lun, s->block_num_per_die, s->block_num_per_channel);   

    s->cmdSelect = 0;
    s->rowAddress = 0;
    s->userData = 0;
    s->dataAddress = 0;
    s->spareAddress = 0;
    s->errorCountAddress = 0;
    s->completionAddress = 0;
    s->waySelection = 0;
    s->channelBusy = 0; //one controller is one channel
    s->readyBusy = ~0; //one controller has multiple ways
    
    /* We can assume our GPIO outputs have been wired up now */
//    qemu_set_irq(s->req, 0);
    QTAILQ_INIT(&s->req_list);
}

static void tiger4nsc_realize(DeviceState *dev, Error **errp)
{
    Error *local_err =NULL;
    Tiger4nscState *s = TIGER4NSC(dev);

    sysbus_init_mmio(s->sbd, &s->mmio);
//    sysbus_init_irq(sbd, &s->irq);
    sysbus_mmio_map(s->sbd, 0, s->physical_address);   

    /*
    qdev_init_gpio_in(&sbd->qdev, ftnandc021_handle_ack, 1);
    qdev_init_gpio_out(&sbd->qdev, &s->req, 1);
    */
    
    qemu_log_mask(LOG_TRACE,
                "tiger4nsc_realize: s %lx phys 0x%lx ways %d\n",
                (unsigned long) s, (unsigned long)s->physical_address, (unsigned int) s->ways );   
    
    blkconf_apply_backend_options(&s->conf, blk_is_read_only(s->conf.blk),
                                  false, &local_err);
    if (local_err) {
        error_report_err(local_err);
    }

}

static const VMStateDescription vmstate_tiger4nsc = {
    .name = TYPE_TIGER4NSC,
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(cmdSelect, Tiger4nscState),
        VMSTATE_UINT32(rowAddress, Tiger4nscState),
        VMSTATE_UINT32(userData, Tiger4nscState),
        VMSTATE_UINT32(dataAddress, Tiger4nscState),
        VMSTATE_UINT32(spareAddress, Tiger4nscState),
        VMSTATE_UINT32(errorCountAddress, Tiger4nscState),
        VMSTATE_UINT32(completionAddress, Tiger4nscState),
        VMSTATE_UINT32(waySelection, Tiger4nscState),
        VMSTATE_UINT32(channelBusy, Tiger4nscState),
        VMSTATE_UINT32(readyBusy, Tiger4nscState),
        VMSTATE_END_OF_LIST()
    }
};    

static void tiger4nsc_instance_init(Object *obj)
{
    Tiger4nscState *s = TIGER4NSC(obj);
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
                          TYPE_TIGER4NSC,
                          V2Fmmio_SIZE);
}
    
static Property tiger4nsc_properties[] = {
    DEFINE_BLOCK_PROPERTIES(Tiger4nscState, conf),        
    DEFINE_PROP_UINT8("ways", Tiger4nscState, ways, WAY_NUM),
    DEFINE_PROP_UINT64("phys", Tiger4nscState, physical_address, V2Fmmio_ADDR),
/*    DEFINE_PROP_STRING("serial", NvmeCtrl, serial),
    DEFINE_PROP_DRIVE("drive", Tiger4nscState, blk), */
    DEFINE_PROP_END_OF_LIST(),
};

static void tiger4nsc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd    = &vmstate_tiger4nsc;
    dc->reset   = tiger4nsc_reset;
    dc->realize = tiger4nsc_realize;
    dc->props = tiger4nsc_properties;
    dc->hotpluggable = true;
    dc->user_creatable = true;
    //dc->no_user = 1;
}

static const TypeInfo tiger4nsc_info = {
    .name          = TYPE_TIGER4NSC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Tiger4nscState),
    .instance_init = tiger4nsc_instance_init,
    .class_init    = tiger4nsc_class_init,
};

static void tiger4nsc_register_types(void)
{
    type_register_static(&tiger4nsc_info);
}

type_init(tiger4nsc_register_types)
