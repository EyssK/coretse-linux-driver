
#ifndef __CORETSE_DMAHW_H__
#define __CORETSE_DMAHW_H__

struct coretse_desc
{
    u32 pkt_start_addr;    /* Packet start address */
    volatile u32 pkt_size; /* Packet size & Per packet override flags */
    struct coretse_desc *next_desriptor;    /* Link to next descriptor */
    u32 index;             /* Index: helps in handling interrupts */
    void * caller_info;         /* Pointer to user specific data */
};

struct coretse_dma_instance
{
//     tse_transmit_callback_t     tx_complete_handler;
//     tse_receive_callback_t      pckt_rx_callback;
    short                       nb_available_tx_desc;
    short                       first_tx_index;
    short                       last_tx_index;
    short                       next_tx_index;
    short                       nb_available_rx_desc;
    short                       next_free_rx_desc_index;
    short                       first_rx_desc_index;
    u8                          phy_addr; /* PHY address for this instance of CoreTSE*/
    //tse_wol_callback_t          wol_callback;
};

/*------------------------------------------------------------------------------
 * DMA descriptor definitions
 */
#define DMA_DESC_EMPTY_FLAG_MASK                ((uint32_t)1u << 31u)
#define DMA_DESC_PKT_SIZE_MASK                  0xFFFu

/*------------------------------------------------------------------------------
 * DMA descriptor packet size
 */
#define DMA_PKTCOUNT_MASK                       ((uint32_t)0xFFu << 16)

#endif /* __CORETSE_DMAHW_H__ */
