
#ifndef __CORETSE_DMA_H__
#define __CORETSE_DMA_H__

#include "coretse_dma.h"

int coretsedma_initialize(struct altera_tse_private *priv);
void coretsedma_uninitialize(struct altera_tse_private *priv);
void coretse_dma_enable_txirq( struct altera_tse_private * priv );
void coretse_dma_disable_txirq( struct altera_tse_private * priv);
void coretse_dma_enable_rxirq( struct altera_tse_private * priv);
void coretse_dma_disable_rxirq( struct altera_tse_private * priv);
int TSE_send_pkt(struct altera_tse_private *priv, struct tse_buffer *buffer);

#endif /*  __CORETSE_DMA_H__ */
