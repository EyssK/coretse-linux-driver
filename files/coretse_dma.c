

#include <linux/netdevice.h>
#include "altera_utils.h"
#include "altera_tse.h"

#include "core_tse.h"
#include "coretse_dma.h"
#include "coretse_dmahw.h"
#include "portage.h"

extern int debug;
static void coretsedma_set_rx(struct altera_tse_private *priv, short index);

static void tx_desc_ring_init(struct altera_tse_private *priv)
{
    int inc;    
    struct coretse_desc* tx_dma_desc = priv->tx_dma_desc;
    dma_addr_t next_tx_phys = priv->txdescphys;
    
    for(inc = 0; inc < TSE_TX_RING_SIZE; ++inc) {
        tx_dma_desc[inc].pkt_start_addr = 0u;
        tx_dma_desc[inc].pkt_size = DMA_DESC_EMPTY_FLAG_MASK;
 
        if((TSE_TX_RING_SIZE - 1) == inc) {
            tx_dma_desc[inc].next_desriptor = priv->txdescphys & 0xFFFFFFFC;
        }
        else {
            tx_dma_desc[inc].next_desriptor = next_tx_phys & 0xFFFFFFFC;
        }
 
        tx_dma_desc[inc].index = inc;
        next_tx_phys += sizeof(struct coretse_desc);
    }
}

static void rx_desc_ring_init(struct altera_tse_private *priv)
{
    int inc;
    struct coretse_desc* rx_dma_desc = priv->rx_dma_desc;
    dma_addr_t next_rx_phys = priv->rxdescphys;

    for(inc = 0; inc < TSE_RX_RING_SIZE; ++inc) {
        rx_dma_desc[inc].pkt_start_addr = 0u;
        rx_dma_desc[inc].pkt_size = 0u;
 
        if((TSE_RX_RING_SIZE - 1) == inc) {
            rx_dma_desc[inc].next_desriptor = priv->rxdescphys & 0xFFFFFFFC;
        }
        else {
            rx_dma_desc[inc].next_desriptor = next_rx_phys & 0xFFFFFFFC;
        }
 
        rx_dma_desc[inc].index = inc;
        next_rx_phys += sizeof(struct coretse_desc);
    }
}

int coretsedma_initialize(struct altera_tse_private *priv)
{
    struct coretse_dma_instance *dma_inst;
    dev_dbg(priv->device, "%s, priv->rx_dma_desc=0x%p priv->rxdescmem=0x%x\n",__func__,priv->rx_dma_desc,priv->rxdescmem );
    
    INIT_LIST_HEAD(&priv->txlisthd);
    INIT_LIST_HEAD(&priv->rxlisthd);
    
    priv->rxdescphys = (dma_addr_t) 0;
    priv->txdescphys = (dma_addr_t) 0;
    
    priv->tx_dma_desc = kmalloc(priv->txdescmem, GFP_KERNEL);
    priv->rx_dma_desc = kmalloc(priv->rxdescmem, GFP_KERNEL);
    
    priv->rxdescphys = dma_map_single(priv->device,
                                      (void __force *)priv->rx_dma_desc,
                                      priv->rxdescmem, DMA_BIDIRECTIONAL);
    
    if (dma_mapping_error(priv->device, priv->rxdescphys)) {
        coretsedma_uninitialize(priv);
        netdev_err(priv->dev, "error mapping rx descriptor memory\n");
        return -EINVAL;
    }
    
    priv->txdescphys = dma_map_single(priv->device,
                                      (void __force *)priv->tx_dma_desc,
                                      priv->txdescmem, DMA_TO_DEVICE);
    
    if (dma_mapping_error(priv->device, priv->txdescphys)) {
        coretsedma_uninitialize(priv);
        netdev_err(priv->dev, "error mapping tx descriptor memory\n");
        return -EINVAL;
    }   
    
    /* Initialize internal descriptors related variables. */
    priv->coretse_dma_instance = kmalloc(sizeof(struct coretse_dma_instance), GFP_KERNEL);
    dma_inst = priv->coretse_dma_instance;
    
    /* Initialize descriptor memory, sync memory to cache */
    tx_desc_ring_init(priv);
    rx_desc_ring_init(priv);
    dma_sync_single_for_device(priv->device, priv->txdescphys,
				   priv->txdescmem, DMA_TO_DEVICE);
	dma_sync_single_for_device(priv->device, priv->rxdescphys,
				   priv->rxdescmem, DMA_TO_DEVICE);
    
    /* Initialize Tx descriptors related variables. */
    dma_inst->first_tx_index = INVALID_INDEX;
    dma_inst->last_tx_index = INVALID_INDEX;
    dma_inst->next_tx_index = 0;
    dma_inst->nb_available_tx_desc = TSE_TX_RING_SIZE;
    
    /* Initialize Rx descriptors related variables. */
    dma_inst->nb_available_rx_desc = dma_rx_num;
    dma_inst->next_free_rx_desc_index = 0;
    dma_inst->first_rx_desc_index = 0;
    
    dev_dbg(priv->device,"coretsedma_initialize priv->txdescphys 0x%llx\n",priv->txdescphys);
    dev_dbg(priv->device,"coretsedma_initialize priv->tx_dma_desc %p\n", priv->tx_dma_desc);
    
    return 0;
}

void coretsedma_uninitialize(struct altera_tse_private *priv)
{
    dev_dbg(priv->device, "coretsedma_uninitialize\n");
    
    if (priv->rxdescphys)
        dma_unmap_single(priv->device, priv->rxdescphys,
                         priv->rxdescmem, DMA_BIDIRECTIONAL);
        
    if (priv->txdescphys)
        dma_unmap_single(priv->device, priv->txdescphys,
                         priv->txdescmem, DMA_TO_DEVICE);
        
    kfree(priv->coretse_dma_instance);
    kfree(priv->tx_dma_desc);
    kfree(priv->rx_dma_desc);
}

void coretse_dma_reset(struct altera_tse_private *priv)
{
    // no need to reset yet
}

inline void coretse_dma_enable_txirq(struct altera_tse_private *priv)
{
    struct tmp {
        struct altera_tse_mac __iomem *base_addr;
    } _tmp = {priv->mac_dev}, *this_tse = &_tmp;

    HAL_set_32bit_reg_field(this_tse->base_addr, DMAINTRMASK_TXPKT_SENT, 0x1);    
}

inline void coretse_dma_disable_txirq(struct altera_tse_private *priv)
{
    struct tmp {
        struct altera_tse_mac __iomem *base_addr;
    } _tmp = {priv->mac_dev}, *this_tse = &_tmp;

    HAL_set_32bit_reg_field(this_tse->base_addr, DMAINTRMASK_TXPKT_SENT, 0x0);
}

inline void coretse_dma_enable_rxirq(struct altera_tse_private *priv)
{
    struct tmp {
        struct altera_tse_mac __iomem *base_addr;
    } _tmp = {priv->mac_dev}, *this_tse = &_tmp;

    HAL_set_32bit_reg_field(this_tse->base_addr, DMAINTRMASK_RXPKT_RCVD, 0x1);    
}

inline void coretse_dma_disable_rxirq(struct altera_tse_private *priv)
{
    struct tmp {
        struct altera_tse_mac __iomem *base_addr;
    } _tmp = {priv->mac_dev}, *this_tse = &_tmp;

    HAL_set_32bit_reg_field(this_tse->base_addr, DMAINTRMASK_RXPKT_RCVD, 0x0);
}

void coretsedma_clear_rxirq(struct altera_tse_private *priv)
{
    struct tmp {
        struct altera_tse_mac __iomem *base_addr;
    } _tmp = {priv->mac_dev}, *this_tse = &_tmp;

    HAL_set_32bit_reg_field(this_tse->base_addr, DMARXSTATUS_RXPKT_RCVD, 0x1);
}

void coretsedma_clear_txirq(struct altera_tse_private *priv)
{
    struct tmp {
        struct altera_tse_mac __iomem *base_addr;
    } _tmp = {priv->mac_dev}, *this_tse = &_tmp;

    HAL_set_32bit_reg_field(this_tse->base_addr, DMATXSTATUS_TXPKT_SENT, 0x1);
}

u32 coretsedma_tx_completions(struct altera_tse_private *priv)
{
    struct coretse_dma_instance *dma_inst;
    short index;
    u32 dma_irq;

    dma_inst = priv->coretse_dma_instance;
    index = dma_inst->first_tx_index;

    spin_lock(&priv->rxdma_irq_lock);
    dma_irq = priv->dma_irq_tx;
    priv->dma_irq_tx = 0;
    spin_unlock(&priv->rxdma_irq_lock);
    if (dma_irq & DMATXSTATUS_TXPKT_SENT_MASK) {
        ++dma_inst->nb_available_tx_desc;

        // Handling Tx one by one
        /* all pending tx packets sent. */
        dma_inst->first_tx_index = INVALID_INDEX;
        dma_inst->last_tx_index = INVALID_INDEX;
    }
    return 1;
}

int TSE_send_pkt(struct altera_tse_private *priv, struct tse_buffer *buffer)
{
    struct tmp {
        struct altera_tse_mac __iomem *base_addr;
    } _tmp = {priv->mac_dev}, *this_tse = &_tmp;
    u8 status = 0;
    
    struct coretse_dma_instance *dma_inst = priv->coretse_dma_instance;
    struct coretse_desc __iomem * tx_descs = priv->tx_dma_desc;
    struct coretse_desc __iomem * p_next_tx_desc;
    dma_addr_t next_tx_phys;
    
    if(INVALID_INDEX == dma_inst->first_tx_index) {
        dma_inst->first_tx_index = dma_inst->next_tx_index;
    }
    
    dma_inst->last_tx_index = dma_inst->next_tx_index;
    
    p_next_tx_desc = &(tx_descs[dma_inst->next_tx_index]);
    p_next_tx_desc->pkt_start_addr = buffer->dma_addr;
    p_next_tx_desc->pkt_size = buffer->len;
    p_next_tx_desc->next_desriptor = 0;
    p_next_tx_desc->index = 0;
    //p_next_tx_desc->caller_info = 0;

    next_tx_phys = priv->txdescphys + dma_inst->next_tx_index*sizeof(struct coretse_desc);
    next_tx_phys &= 0xFFFFFFFF;

//    netdev_dbg(priv->dev, "p_next_tx_desc=0x%p tx_descs=\t0x%p\n", p_next_tx_desc, tx_descs);
//    netdev_dbg(priv->dev, "dma_inst->next_tx_index=\t%d\n", dma_inst->next_tx_index);
//    netdev_dbg(priv->dev, "send dma buffer=\t0x%llx\n", next_tx_phys);
//    netdev_dbg(priv->dev, "pkt add dma buffer=\t0x%x\n", p_next_tx_desc->pkt_start_addr);

    dma_sync_single_for_device(priv->device, priv->txdescphys,
                               priv->txdescmem, DMA_TO_DEVICE);
    
    /*
        *         If TX is found disabled, this might be because this is the first
        *         time packet is being sent or the DMA completed previous transfer and
        *         stopped transmission by itself caused by TX underrun or bus error.
        *         This function neglects the errors and tries to send the current packet.
        */
    if(0 == HAL_get_32bit_reg_field(this_tse->base_addr, DMATXCTRL_TX_EN)) {
        HAL_set_32bit_reg(this_tse->base_addr, DMATXDESC, next_tx_phys);
    }
    
    /*
        *         Enable DMA transmit anyway to cover the case where Tx completed after
        *         the read of DMA_TX_CTRL.
        */
    HAL_set_32bit_reg_field(this_tse->base_addr, DMATXCTRL_TX_EN, 0x01);
    
    /*
    *         Point the next_tx_desc to next free descriptor in the ring. Wrap around
    *         in case next descriptor is pointing to last in the ring
    */
    dma_inst->next_tx_index = (dma_inst->next_tx_index + 1) % TSE_TX_RING_SIZE;
    
    /*
        *         Packet scheduled for transmission successfully only when there is no
        *         TX bus error
        */
    if(0 == HAL_get_32bit_reg_field(this_tse->base_addr, DMARXSTATUS_BUSERR)) {
        status = 1;
    }
    
    return status;
}

/* status is returned on upper 16 bits,
 * length is returned in lower 16 bits
 */
u32 coretsedma_rx_status(struct altera_tse_private *priv) {
    struct tmp {
        struct altera_tse_mac __iomem *base_addr;
    } _tmp = {priv->mac_dev}, *this_tse = &_tmp;
    struct coretse_dma_instance *dma_inst = priv->coretse_dma_instance;
    struct coretse_desc __iomem *rx_descs = priv->rx_dma_desc;
    struct coretse_desc __iomem *p_next_rx_desc;
    u32 dma_irq;
    unsigned int rxstatus = 0;

    spin_lock(&priv->rxdma_irq_lock);
    dma_irq = priv->dma_irq_rx;
    priv->dma_irq_rx = 0;
    spin_unlock(&priv->rxdma_irq_lock);
    // netdev_dbg(priv->dev, "%s: dma_irq=0x%x\n", __func__, dma_irq);
    if (dma_irq & DMARXSTATUS_RXPKT_RCVD_MASK) {
        // packet received
        netdev_dbg(priv->dev, "%s: Packet received\n",__func__);

        dma_sync_single_for_cpu(priv->device,
                                priv->rxdescphys,
                                priv->rxdescmem,
                                DMA_FROM_DEVICE);
        p_next_rx_desc = &rx_descs[dma_inst->first_rx_desc_index];
        rxstatus = (p_next_rx_desc->pkt_size & DMA_DESC_PKT_SIZE_MASK) - 4u;

        /* update first_rx_desc_index */
        ++dma_inst->nb_available_rx_desc;
        dma_inst->first_rx_desc_index = (dma_inst->first_rx_desc_index + 1) % TSE_RX_RING_SIZE;
        coretsedma_set_rx(priv,dma_inst->first_rx_desc_index);
    }

    return rxstatus;
}

/*
 *  Add the buffer to the next free rx descriptor.
 */
void coretsedma_add_rx_desc(struct altera_tse_private *priv, struct tse_buffer *rxbuffer)
{
    struct tmp {
        struct altera_tse_mac __iomem *base_addr;
    } _tmp = {priv->mac_dev}, *this_tse = &_tmp;
    struct coretse_dma_instance *dma_inst = priv->coretse_dma_instance;
    struct coretse_desc __iomem * rx_descs = priv->rx_dma_desc;
    short next_rx_desc_index;

    if(dma_inst->nb_available_rx_desc > 0)
    {
        --dma_inst->nb_available_rx_desc;
        next_rx_desc_index = dma_inst->next_free_rx_desc_index;
        rx_descs[next_rx_desc_index].pkt_start_addr = rxbuffer->dma_addr & 0xFFFFFFFF;
        /* Set own flag of this descriptor to indicate ready for RX */
        rx_descs[next_rx_desc_index].pkt_size = DMA_DESC_EMPTY_FLAG_MASK;
        dma_sync_single_for_device(priv->device, priv->rxdescphys, priv->rxdescmem, DMA_TO_DEVICE);

//      netdev_dbg(priv->dev, "%s: Desc=0x%llx Buffer=0x%llx\n",__func__,priv->rxdescphys + next_rx_desc_index*sizeof(struct coretse_desc), rxbuffer->dma_addr);
//      netdev_dbg(priv->dev, "%s: PktSize=0x%x Next=0x%x\n",__func__, rx_descs[next_rx_desc_index].pkt_size, rx_descs[next_rx_desc_index].next_desriptor);
	
        /*
         *     If the RX is found disabled, then it might be because this is the
         *     first time a packet is scheduled for reception or the RX ENABLE is
         *     made zero by RX overflow or RX bus error. In either case, this
         *     function tries to schedule the current packet for reception.
         */
        if(0 == HAL_get_32bit_reg_field(this_tse->base_addr, DMARXCTRL_RX_EN))
        {
            /* Make current descriptor to point to the packet requested */
            dma_inst->first_rx_desc_index = next_rx_desc_index;
            coretsedma_set_rx(priv, next_rx_desc_index);
        }

        /* update next_free_rx_desc_index */
        dma_inst->next_free_rx_desc_index = (dma_inst->next_free_rx_desc_index+1)%TSE_RX_RING_SIZE;
    }
    else {
        netdev_err(priv->dev,"%s: no descriptor available! memory leak\n", __func__);
    }
}

void coretsedma_start_rxdma(struct altera_tse_private *priv)
{
    struct coretse_dma_instance *dma_inst = priv->coretse_dma_instance;

    // TEST before setting the first rx packet
    {
        struct coretse_desc __iomem * rx_descs = priv->rx_dma_desc;
        short index = dma_inst->first_rx_desc_index;
        dma_addr_t rx_phys = priv->rxdescphys + index*sizeof(struct coretse_desc);
        netdev_dbg(priv->dev, "%s: Desc=0x%llx\n",__func__, rx_phys);
        netdev_dbg(priv->dev, "%s: Buffer=0x%x Len=0x%x\n",__func__, rx_descs[index].pkt_start_addr, rx_descs[index].pkt_size);
    }

    /* Make first descriptor to point to the packet requested */
    coretsedma_set_rx(priv, dma_inst->first_rx_desc_index);
}

/* 
 * This function is setting the descriptor number index in the DMA register
 * for reception.
 */
static void coretsedma_set_rx(struct altera_tse_private *priv, short index)
{
    struct tmp {
        struct altera_tse_mac __iomem *base_addr;
    } _tmp = {priv->mac_dev}, *this_tse = &_tmp;
    dma_addr_t rx_phys;

    rx_phys = priv->rxdescphys + index*sizeof(struct coretse_desc);
    rx_phys &= 0xFFFFFFFF;
    HAL_set_32bit_reg(this_tse->base_addr, DMARXDESC, rx_phys);

    HAL_set_32bit_reg_field(this_tse->base_addr, DMARXSTATUS_RXPKT_OVR, 0x01);
    HAL_set_32bit_reg_field(this_tse->base_addr, DMARXCTRL_RX_EN, 0x01);
}
