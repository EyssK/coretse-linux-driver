
#ifndef __CORETSE_DMA_H__
#define __CORETSE_DMA_H__

int coretsedma_initialize(struct altera_tse_private *priv);
void coretsedma_uninitialize(struct altera_tse_private *priv);
void coretse_dma_reset(struct altera_tse_private *priv);
void coretse_dma_enable_txirq( struct altera_tse_private * priv );
void coretse_dma_disable_txirq( struct altera_tse_private * priv);
void coretse_dma_enable_rxirq( struct altera_tse_private * priv);
void coretse_dma_disable_rxirq( struct altera_tse_private * priv);
void coretsedma_clear_rxirq(struct altera_tse_private *priv);
void coretsedma_clear_txirq(struct altera_tse_private *priv);
u32 coretsedma_tx_completions(struct altera_tse_private *priv);
int TSE_send_pkt(struct altera_tse_private *priv, struct tse_buffer *buffer);
u32 coretsedma_rx_status(struct altera_tse_private *priv);
void coretsedma_add_rx_desc(struct altera_tse_private *priv, struct tse_buffer *rxbuffer);

#endif /*  __CORETSE_DMA_H__ */
