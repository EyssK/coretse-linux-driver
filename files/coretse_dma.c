

#include <linux/netdevice.h>
#include "altera_utils.h"
#include "altera_tse.h"

#include "core_tse.h"
#include "coretse_dma.h"
#include "coretse_dmahw.h"
#include "portage.h"

extern int debug;
extern int dma_tx_num;
extern int dma_rx_num;

static void tx_desc_ring_init(struct altera_tse_private *priv)
{
    int inc;    
    struct coretse_desc* tx_dma_desc = priv->tx_dma_desc;
    
    for(inc = 0; inc < dma_tx_num; ++inc)
    {
        tx_dma_desc[inc].pkt_start_addr = 0u;
        tx_dma_desc[inc].pkt_size = DMA_DESC_EMPTY_FLAG_MASK;
        
        if((dma_tx_num - 1) == inc)
        {
            tx_dma_desc[inc].next_desriptor =
            (struct coretse_desc*)&tx_dma_desc[0];
        }
        else
        {
            tx_dma_desc[inc].next_desriptor =
            (struct coretse_desc*)&tx_dma_desc[inc + 1];
        }
        
        tx_dma_desc[inc].index = inc;
    }
}

static void rx_desc_ring_init(struct altera_tse_private *priv)
{
    int inc;
    struct coretse_desc* rx_dma_desc = priv->rx_dma_desc;
    
    for(inc = 0; inc < dma_rx_num; ++inc)
    {
        rx_dma_desc[inc].pkt_start_addr = 0u;
        rx_dma_desc[inc].pkt_size = 0u;
        
        if((dma_rx_num - 1) == inc)
        {
            rx_dma_desc[inc].next_desriptor =
            (struct coretse_desc*)&rx_dma_desc[0];
        }
        else
        {
            rx_dma_desc[inc].next_desriptor =
            (struct coretse_desc*)&rx_dma_desc[inc + 1];
        }
        
        rx_dma_desc[inc].index = inc;
    }
}

int coretsedma_initialize(struct altera_tse_private *priv)
{
    struct coretse_dma_instance *dma_inst;
    dev_dbg(priv->device, "%s, priv->rx_dma_desc=0x%p priv->rxdescmem=0x%lx\n",__func__,priv->rx_dma_desc,priv->rxdescmem );
    
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
    dma_inst->nb_available_rx_desc = TSE_RX_RING_SIZE;
    dma_inst->next_free_rx_desc_index = 0;
    dma_inst->first_rx_desc_index = INVALID_INDEX;
    
    /// TX/RX callback functions
    // TODO
//     TSE_set_tx_callback(&g_tse_0, packet_tx_complete_handler);
//     TSE_set_tx_callback(&g_tse_1, packet_tx_complete_handler);
//     
//     TSE_set_rx_callback(&g_tse_0, mac_rx_callback);
//     TSE_set_rx_callback(&g_tse_1, mac_rx_callback);
    

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

inline void
coretse_dma_enable_txirq
(
    struct altera_tse_private * priv
)
{
    struct tmp {
        struct altera_tse_mac __iomem *base_addr;
    } _tmp = {priv->mac_dev}, *this_tse = &_tmp;
    
    dev_dbg(priv->device, "%s\n",__func__);

    HAL_set_32bit_reg_field(this_tse->base_addr, DMAINTRMASK_TXPKT_SENT, 0x1);    
}

inline void
coretse_dma_disable_txirq
(
    struct altera_tse_private * priv
)
{
    struct tmp {
        struct altera_tse_mac __iomem *base_addr;
    } _tmp = {priv->mac_dev}, *this_tse = &_tmp;
    
    dev_dbg(priv->device, "%s\n",__func__);

    HAL_set_32bit_reg_field(this_tse->base_addr, DMAINTRMASK_TXPKT_SENT, 0x0);
}

inline void
coretse_dma_enable_rxirq
(
    struct altera_tse_private * priv
)
{
    struct tmp {
        struct altera_tse_mac __iomem *base_addr;
    } _tmp = {priv->mac_dev}, *this_tse = &_tmp;
    
    dev_dbg(priv->device, "%s\n",__func__);

    HAL_set_32bit_reg_field(this_tse->base_addr, DMAINTRMASK_RXPKT_RCVD, 0x1);    
}

inline void
coretse_dma_disable_rxirq
(
    struct altera_tse_private * priv
)
{
    struct tmp {
        struct altera_tse_mac __iomem *base_addr;
    } _tmp = {priv->mac_dev}, *this_tse = &_tmp;
    
    dev_dbg(priv->device, "%s\n",__func__);

    HAL_set_32bit_reg_field(this_tse->base_addr, DMAINTRMASK_RXPKT_RCVD, 0x0);
}

int
TSE_send_pkt
(struct altera_tse_private *priv, struct tse_buffer *buffer)
{
    struct tmp {
        struct altera_tse_mac __iomem *base_addr;
    } _tmp = {priv->mac_dev}, *this_tse = &_tmp;
    u8 status = 0;
    
    struct coretse_dma_instance *dma_inst = priv->coretse_dma_instance;
    struct coretse_desc __iomem * tx_descs = priv->tx_dma_desc;
    struct coretse_desc __iomem * p_next_tx_desc;
    
    if(INVALID_INDEX == dma_inst->first_tx_index)
    {
        dma_inst->first_tx_index = dma_inst->next_tx_index;
    }
    
    dma_inst->last_tx_index = dma_inst->next_tx_index;
    
    p_next_tx_desc = &(tx_descs[dma_inst->next_tx_index]);
    csrwr32(buffer->dma_addr, p_next_tx_desc, offsetof(struct coretse_desc, pkt_start_addr));
    csrwr32(buffer->len,      p_next_tx_desc, offsetof(struct coretse_desc, pkt_size));
    //p_next_tx_desc->caller_info = p_user_data;
    
    dev_dbg(priv->device, "p_next_tx_desc=0x%p tx_descs=0x%p, dma_inst->next_tx_index=%d\n", p_next_tx_desc, tx_descs, dma_inst->next_tx_index);
    
    dma_sync_single_for_device(priv->device, priv->txdescphys,
                               sizeof(struct coretse_desc), DMA_TO_DEVICE);
    
    /*
        *         If TX is found disabled, this might be because this is the first
        *         time packet is being sent or the DMA completed previous transfer and
        *         stopped transmission by itself caused by TX underrun or bus error.
        *         This function neglects the errors and tries to send the current packet.
        */
//     if(0 == HAL_get_32bit_reg_field(this_tse->base_addr, DMATXCTRL_TX_EN))
//     {
//         HAL_set_32bit_reg(this_tse->base_addr,
//                             DMATXDESC,
//                             (u32)p_next_tx_desc);
//     }
    
    /*
        *         Enable DMA transmit anyway to cover the case where Tx completed after
        *         the read of DMA_TX_CTRL.
        */
    HAL_set_32bit_reg_field(this_tse->base_addr, DMATXCTRL_TX_EN, 0x01);
    
    /*
    *         Point the next_tx_desc to next free descriptor in the ring. Wrap around
    *         in case next descriptor is pointing to last in the ring
    */
    if((TSE_TX_RING_SIZE - 1) == dma_inst->next_tx_index)
    {
        dma_inst->next_tx_index = 0;
    }
    else
    {
        ++dma_inst->next_tx_index;
    }
    
    /*
        *         Packet scheduled for transmission successfully only when there is no
        *         TX bus error
        */
    if(0 == HAL_get_32bit_reg_field(this_tse->base_addr, DMARXSTATUS_BUSERR))
    {
        status = 1;
    }
    
    /* TxPktSent Interrupt Enable. */
    HAL_set_32bit_reg_field(this_tse->base_addr, DMAINTRMASK_TXPKT_SENT, 0x1);
    
    dev_dbg(priv->device,"%s: sending buffer\n",__func__);
    
    return status;
}


