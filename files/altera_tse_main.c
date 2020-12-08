// SPDX-License-Identifier: GPL-2.0-only
/* Altera Triple-Speed Ethernet MAC driver
 * Copyright (C) 2008-2014 Altera Corporation. All rights reserved
 *
 * Contributors:
 *   Dalon Westergreen
 *   Thomas Chou
 *   Ian Abbott
 *   Yuriy Kozlov
 *   Tobias Klauser
 *   Andriy Smolskyy
 *   Roman Bulgakov
 *   Dmytro Mytarchuk
 *   Matthew Gerlach
 *
 * Original driver contributed by SLS.
 * Major updates contributed by GlobalLogic
 */

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/netdevice.h>
#include <linux/of_device.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <asm/cacheflush.h>

#include "altera_utils.h"
#include "altera_tse.h"
#include "altera_sgdma.h"
#include "altera_msgdma.h"

#include "core_tse.h"
#include "coretse_dma.h"

static atomic_t instance_count = ATOMIC_INIT(~0);
/* Module parameters */
int debug = -1;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Message Level (-1: default, 0: no output, 16: all)");

static const u32 default_msg_level = (NETIF_MSG_DRV | NETIF_MSG_PROBE |
					NETIF_MSG_LINK | NETIF_MSG_IFUP |
					NETIF_MSG_IFDOWN);

#define RX_DESCRIPTORS 64
int dma_rx_num = RX_DESCRIPTORS;
module_param(dma_rx_num, int, 0644);
MODULE_PARM_DESC(dma_rx_num, "Number of descriptors in the RX list");

#define TX_DESCRIPTORS 64
int dma_tx_num = TX_DESCRIPTORS;
module_param(dma_tx_num, int, 0644);
MODULE_PARM_DESC(dma_tx_num, "Number of descriptors in the TX list");


#define POLL_PHY (-1)

/* Make sure DMA buffer size is larger than the max frame size
 * plus some alignment offset and a VLAN header. If the max frame size is
 * 1518, a VLAN header would be additional 4 bytes and additional
 * headroom for alignment is 2 bytes, 2048 is just fine.
 */
#define ALTERA_RXDMABUFFER_SIZE	2048

/* Allow network stack to resume queueing packets after we've
 * finished transmitting at least 1/4 of the packets in the queue.
 */
#define TSE_TX_THRESH(x)	(x->tx_ring_size / 4)

#define TXQUEUESTOP_THRESHHOLD	2

static const struct of_device_id coretse_ids[];

static inline u32 tse_tx_avail(struct altera_tse_private *priv)
{
	return priv->tx_cons + priv->tx_ring_size - priv->tx_prod - 1;
}

/* PCS Register read/write functions
 */
static u16 sgmii_pcs_read(struct altera_tse_private *priv, int regnum)
{
    netdev_err(priv->dev, "%s not handled",__func__);
	return csrrd32(priv->mac_dev,
		       tse_csroffs(mdio_phy0) + regnum * 4) & 0xffff;
}

static void sgmii_pcs_write(struct altera_tse_private *priv, int regnum,
				u16 value)
{
    netdev_err(priv->dev, "%s not handled",__func__);
	csrwr32(value, priv->mac_dev, tse_csroffs(mdio_phy0) + regnum * 4);
}

/* Check PCS scratch memory */
static int sgmii_pcs_scratch_test(struct altera_tse_private *priv, u16 value)
{
    netdev_err(priv->dev, "%s not handled",__func__);
	sgmii_pcs_write(priv, SGMII_PCS_SCRATCH, value);
	return (sgmii_pcs_read(priv, SGMII_PCS_SCRATCH) == value);
}

/* MDIO specific functions
 */
static int altera_tse_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	struct net_device *ndev = bus->priv;
	struct altera_tse_private *priv = netdev_priv(ndev);

    netdev_err(ndev, "%s not handled",__func__);
	/* set MDIO address */
	csrwr32((mii_id & 0x1f), priv->mac_dev,
		tse_csroffs(mdio_phy1_addr));

	/* get the data */
	return csrrd32(priv->mac_dev,
		       tse_csroffs(mdio_phy1) + regnum * 4) & 0xffff;
}

static int altera_tse_mdio_write(struct mii_bus *bus, int mii_id, int regnum,
				 u16 value)
{
	struct net_device *ndev = bus->priv;
	struct altera_tse_private *priv = netdev_priv(ndev);

    netdev_err(ndev, "%s not handled",__func__);
	/* set MDIO address */
	csrwr32((mii_id & 0x1f), priv->mac_dev,
		tse_csroffs(mdio_phy1_addr));

	/* write the data */
	csrwr32(value, priv->mac_dev, tse_csroffs(mdio_phy1) + regnum * 4);
	return 0;
}

static int altera_tse_mdio_create(struct net_device *dev, unsigned int id)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	int ret;
	struct device_node *mdio_node = NULL;
	struct mii_bus *mdio = NULL;
	struct device_node *child_node = NULL;

	for_each_child_of_node(priv->device->of_node, child_node) {
		if (of_device_is_compatible(child_node, "altr,tse-mdio")) {
			mdio_node = child_node;
			break;
		}
	}

	if (mdio_node) {
		netdev_dbg(dev, "FOUND MDIO subnode\n");
	} else {
		netdev_dbg(dev, "NO MDIO subnode\n");
		return 0;
	}

    netdev_err(priv->dev, "%s not handled",__func__);

	mdio = mdiobus_alloc();
	if (mdio == NULL) {
		netdev_err(dev, "Error allocating MDIO bus\n");
		return -ENOMEM;
	}

	mdio->name = MICROCHIP_TSE_RESOURCE_NAME;
	mdio->read = &altera_tse_mdio_read;
	mdio->write = &altera_tse_mdio_write;
	snprintf(mdio->id, MII_BUS_ID_SIZE, "%s-%u", mdio->name, id);

	mdio->priv = dev;
	mdio->parent = priv->device;

	ret = of_mdiobus_register(mdio, mdio_node);
	if (ret != 0) {
		netdev_err(dev, "Cannot register MDIO bus %s\n",
			   mdio->id);
		goto out_free_mdio;
	}

	if (netif_msg_drv(priv))
		netdev_info(dev, "MDIO bus %s: created\n", mdio->id);

	priv->mdio = mdio;
	return 0;
out_free_mdio:
	mdiobus_free(mdio);
	mdio = NULL;
	return ret;
}

static void altera_tse_mdio_destroy(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);

	if (priv->mdio == NULL)
		return;

    netdev_err(priv->dev, "%s not handled",__func__);

	if (netif_msg_drv(priv))
		netdev_info(dev, "MDIO bus %s: removed\n",
			    priv->mdio->id);

	mdiobus_unregister(priv->mdio);
	mdiobus_free(priv->mdio);
	priv->mdio = NULL;
}

static int tse_init_rx_buffer(struct altera_tse_private *priv,
			      struct tse_buffer *rxbuffer, int len)
{
	rxbuffer->skb = netdev_alloc_skb_ip_align(priv->dev, len);
	if (!rxbuffer->skb)
		return -ENOMEM;

	rxbuffer->dma_addr = dma_map_single(priv->device, rxbuffer->skb->data,
						len,
						DMA_FROM_DEVICE);

	if (dma_mapping_error(priv->device, rxbuffer->dma_addr)) {
		netdev_err(priv->dev, "%s: DMA mapping error\n", __func__);
		dev_kfree_skb_any(rxbuffer->skb);
		return -EINVAL;
	}
	rxbuffer->dma_addr &= (dma_addr_t)~3;
	rxbuffer->len = len;
	return 0;
}

static void tse_free_rx_buffer(struct altera_tse_private *priv,
			       struct tse_buffer *rxbuffer)
{
	struct sk_buff *skb = rxbuffer->skb;
	dma_addr_t dma_addr = rxbuffer->dma_addr;

	if (skb != NULL) {
		if (dma_addr)
			dma_unmap_single(priv->device, dma_addr,
					 rxbuffer->len,
					 DMA_FROM_DEVICE);
		dev_kfree_skb_any(skb);
		rxbuffer->skb = NULL;
		rxbuffer->dma_addr = 0;
	}
}

/* Unmap and free Tx buffer resources
 */
static void tse_free_tx_buffer(struct altera_tse_private *priv,
			       struct tse_buffer *buffer)
{
	if (buffer->dma_addr) {
		if (buffer->mapped_as_page)
			dma_unmap_page(priv->device, buffer->dma_addr,
				       buffer->len, DMA_TO_DEVICE);
		else
			dma_unmap_single(priv->device, buffer->dma_addr,
					 buffer->len, DMA_TO_DEVICE);
		buffer->dma_addr = 0;
	}
	if (buffer->skb) {
		dev_kfree_skb_any(buffer->skb);
		buffer->skb = NULL;
	}
}

static int alloc_init_skbufs(struct altera_tse_private *priv)
{
	unsigned int rx_descs = priv->rx_ring_size;
	unsigned int tx_descs = priv->tx_ring_size;
	int ret = -ENOMEM;
	int i;

	/* Create Rx ring buffer */
	priv->rx_ring = kcalloc(rx_descs, sizeof(struct tse_buffer),
				GFP_KERNEL);
	if (!priv->rx_ring)
		goto err_rx_ring;

	/* Create Tx ring buffer */
	priv->tx_ring = kcalloc(tx_descs, sizeof(struct tse_buffer),
				GFP_KERNEL);
	if (!priv->tx_ring)
		goto err_tx_ring;

	priv->tx_cons = 0;
	priv->tx_prod = 0;

	/* Init Rx ring */
	for (i = 0; i < rx_descs; i++) {
		ret = tse_init_rx_buffer(priv, &priv->rx_ring[i],
					 priv->rx_dma_buf_sz);
		if (ret)
			goto err_init_rx_buffers;
	}

	priv->rx_cons = 0;
	priv->rx_prod = 0;

	return 0;
err_init_rx_buffers:
	while (--i >= 0)
		tse_free_rx_buffer(priv, &priv->rx_ring[i]);
	kfree(priv->tx_ring);
err_tx_ring:
	kfree(priv->rx_ring);
err_rx_ring:
	return ret;
}

static void free_skbufs(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	unsigned int rx_descs = priv->rx_ring_size;
	unsigned int tx_descs = priv->tx_ring_size;
	int i;

	/* Release the DMA TX/RX socket buffers */
	for (i = 0; i < rx_descs; i++)
		tse_free_rx_buffer(priv, &priv->rx_ring[i]);
	for (i = 0; i < tx_descs; i++)
		tse_free_tx_buffer(priv, &priv->tx_ring[i]);


	kfree(priv->tx_ring);
}

/* Reallocate the skb for the reception process
 */
static inline void tse_rx_refill(struct altera_tse_private *priv)
{
	unsigned int rxsize = priv->rx_ring_size;
	unsigned int entry;
	int ret;

	for (; priv->rx_cons - priv->rx_prod > 0;
			priv->rx_prod++) {
		entry = priv->rx_prod % rxsize;
		if (likely(priv->rx_ring[entry].skb == NULL)) {
			ret = tse_init_rx_buffer(priv, &priv->rx_ring[entry],
				priv->rx_dma_buf_sz);
			if (unlikely(ret != 0))
				break;
			priv->dmaops->add_rx_desc(priv, &priv->rx_ring[entry]);
		}
	}
}

/* Pull out the VLAN tag and fix up the packet
 */
static inline void tse_rx_vlan(struct net_device *dev, struct sk_buff *skb)
{
	struct ethhdr *eth_hdr;
	u16 vid;
	if ((dev->features & NETIF_F_HW_VLAN_CTAG_RX) &&
	    !__vlan_get_tag(skb, &vid)) {
		eth_hdr = (struct ethhdr *)skb->data;
		memmove(skb->data + VLAN_HLEN, eth_hdr, ETH_ALEN * 2);
		skb_pull(skb, VLAN_HLEN);
		__vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q), vid);
	}
}

/* Receive a packet: retrieve and pass over to upper levels
 */
static int tse_rx(struct altera_tse_private *priv, int limit)
{
	unsigned int count = 0;
	unsigned int next_entry;
	struct sk_buff *skb;
	unsigned int entry = priv->rx_cons % priv->rx_ring_size;
	u32 rxstatus;
	u16 pktlength;
	u16 pktstatus;

	/* Check for count < limit first as get_rx_status is changing
	* the response-fifo so we must process the next packet
	* after calling get_rx_status if a response is pending.
	* (reading the last byte of the response pops the value from the fifo.)
	*/
	while ((count < limit) &&
	       ((rxstatus = priv->dmaops->get_rx_status(priv)) != 0)) {
		pktstatus = rxstatus >> 16;
		pktlength = rxstatus & 0xffff;

		if ((pktstatus & 0xFF) || (pktlength == 0))
			netdev_err(priv->dev,
				   "RCV pktstatus %08X pktlength %08X\n",
				   pktstatus, pktlength);

		/* DMA trasfer from TSE starts with 2 aditional bytes for
		 * IP payload alignment. Status returned by get_rx_status()
		 * contains DMA transfer length. Packet is 2 bytes shorter.
		 */
// 		pktlength -= 2;

		count++;
		next_entry = (++priv->rx_cons) % priv->rx_ring_size;

		skb = priv->rx_ring[entry].skb;
		if (unlikely(!skb)) {
			netdev_err(priv->dev,
				   "%s: Inconsistent Rx descriptor chain\n",
				   __func__);
			priv->dev->stats.rx_dropped++;
			break;
		}
		priv->rx_ring[entry].skb = NULL;

		skb_put(skb, pktlength);

		dma_unmap_single(priv->device, priv->rx_ring[entry].dma_addr,
				 priv->rx_ring[entry].len, DMA_FROM_DEVICE);

		if (netif_msg_pktdata(priv)) {
			netdev_info(priv->dev, "frame received %d bytes\n",
				    pktlength);
			print_hex_dump(KERN_ERR, "data: ", DUMP_PREFIX_OFFSET,
				       16, 1, skb->data, pktlength, true);
		}

		tse_rx_vlan(priv->dev, skb);

		skb->protocol = eth_type_trans(skb, priv->dev);
		skb_checksum_none_assert(skb);

		napi_gro_receive(&priv->napi, skb);

		priv->dev->stats.rx_packets++;
		priv->dev->stats.rx_bytes += pktlength;

		entry = next_entry;

		tse_rx_refill(priv);
	}

	return count;
}

/* Reclaim resources after transmission completes
 */
static int tse_tx_complete(struct altera_tse_private *priv)
{
	unsigned int txsize = priv->tx_ring_size;
	u32 ready;
	unsigned int entry;
	struct tse_buffer *tx_buff;
	int txcomplete = 0;

	spin_lock(&priv->tx_lock);

	ready = priv->dmaops->tx_completions(priv);

	/* Free sent buffers */
	while (ready && (priv->tx_cons != priv->tx_prod)) {
		entry = priv->tx_cons % txsize;
		tx_buff = &priv->tx_ring[entry];

		if (netif_msg_tx_done(priv))
			netdev_dbg(priv->dev, "%s: curr %d, dirty %d\n",
				   __func__, priv->tx_prod, priv->tx_cons);

		if (likely(tx_buff->skb))
			priv->dev->stats.tx_packets++;

		tse_free_tx_buffer(priv, tx_buff);
		priv->tx_cons++;

		txcomplete++;
		ready--;
	}

	if (unlikely(netif_queue_stopped(priv->dev) &&
		     tse_tx_avail(priv) > TSE_TX_THRESH(priv))) {
		if (netif_queue_stopped(priv->dev) &&
		    tse_tx_avail(priv) > TSE_TX_THRESH(priv)) {
			if (netif_msg_tx_done(priv))
				netdev_dbg(priv->dev, "%s: restart transmit\n",
					   __func__);
			netif_wake_queue(priv->dev);
		}
	}

	spin_unlock(&priv->tx_lock);
	return txcomplete;
}

/* NAPI polling function
 */
static int tse_poll(struct napi_struct *napi, int budget)
{
	struct altera_tse_private *priv =
			container_of(napi, struct altera_tse_private, napi);
	int rxcomplete = 0;
	unsigned long int flags;
    
	tse_tx_complete(priv);

	rxcomplete = tse_rx(priv, budget);

	if (rxcomplete < budget) {

		napi_complete_done(napi, rxcomplete);

		netdev_dbg(priv->dev,
			   "NAPI Complete, did %d packets with budget %d\n",
			   rxcomplete, budget);

		spin_lock_irqsave(&priv->rxdma_irq_lock, flags);
		priv->dmaops->enable_rxirq(priv);
		priv->dmaops->enable_txirq(priv);
		spin_unlock_irqrestore(&priv->rxdma_irq_lock, flags);
	}
	return rxcomplete;
}

/* DMA TX & RX FIFO interrupt routing
 */
static irqreturn_t altera_isr(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct altera_tse_private *priv;

	if (unlikely(!dev)) {
		pr_err("%s: invalid dev pointer\n", __func__);
		return IRQ_NONE;
	}
	priv = netdev_priv(dev);

    u32 dma_irq = HAL_get_32bit_reg(priv->mac_dev, DMAINTR);
    // netdev_dbg(priv->dev, "%s: dma_irq=0x%x\n", __func__, dma_irq);
    if (!dma_irq) {
        netdev_err(priv->dev,"%s: IRQ but DMA interrupt register is 0\n", __func__);
    }
    else {
        if (dma_irq & TSE_RXPKTRCVD_IRQ_MASK) {
            spin_lock(&priv->rxdma_irq_lock);
            priv->dma_irq_rx = HAL_get_32bit_reg(priv->mac_dev, DMARXSTATUS); 
            priv->dmaops->clear_rxirq(priv);
            spin_unlock(&priv->rxdma_irq_lock);
        }
        if (dma_irq & TSE_TXPKTSENT_IRQ_MASK) {
            spin_lock(&priv->rxdma_irq_lock);
            priv->dma_irq_tx = HAL_get_32bit_reg(priv->mac_dev, DMATXSTATUS); 
            priv->dmaops->clear_txirq(priv);
            spin_unlock(&priv->rxdma_irq_lock);
        }
        // Check errors
        if(dma_irq & (1u << TSE_RXOVRFLOW_IRQ)) {
            spin_lock(&priv->rxdma_irq_lock);
            HAL_set_32bit_reg_field(priv->mac_dev, DMARXSTATUS_RXPKT_OVR, 0x1);
            HAL_set_32bit_reg_field(priv->mac_dev, DMARXCTRL_RX_EN, 0x1);
            spin_unlock(&priv->rxdma_irq_lock);
            netdev_dbg(priv->dev,"%s: TSE_RXOVRFLOW_IRQ\n", __func__);
        }
        if(dma_irq & (1u << TSE_RXBUSERR_IRQ)) {
            /*
             *     Clear the rx packet received interrupt once. If this bit still persists,
             *     then another rx packet received interrupt will be generated. Rx count
             *     will be decremented.
             */
            spin_lock(&priv->rxdma_irq_lock);
            HAL_set_32bit_reg_field(priv->mac_dev, DMARXSTATUS_BUSERR, 0x1);
            spin_unlock(&priv->rxdma_irq_lock);
            netdev_dbg(priv->dev,"%s: TSE_RXBUSERR_IRQ\n", __func__);
        }
    }

	if (likely(napi_schedule_prep(&priv->napi))) {
		spin_lock(&priv->rxdma_irq_lock);
		priv->dmaops->disable_rxirq(priv);
		priv->dmaops->disable_txirq(priv);
		spin_unlock(&priv->rxdma_irq_lock);
		__napi_schedule(&priv->napi);
	}

	return IRQ_HANDLED;
}

/* Transmit a packet (called by the kernel). Dispatches
 * either the SGDMA method for transmitting or the
 * MSGDMA method, assumes no scatter/gather support,
 * implying an assumption that there's only one
 * physically contiguous fragment starting at
 * skb->data, for length of skb_headlen(skb).
 */
static netdev_tx_t tse_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	unsigned int txsize = priv->tx_ring_size;
	unsigned int entry;
	struct tse_buffer *buffer = NULL;
	int nfrags = skb_shinfo(skb)->nr_frags;
	unsigned int nopaged_len = skb_headlen(skb);
	netdev_tx_t ret = NETDEV_TX_OK;
	dma_addr_t dma_addr;

	spin_lock_bh(&priv->tx_lock);

	if (unlikely(tse_tx_avail(priv) < nfrags + 1)) {
		if (!netif_queue_stopped(dev)) {
			netif_stop_queue(dev);
			/* This is a hard error, log it. */
			netdev_err(priv->dev,
				   "%s: Tx list full when queue awake\n",
				   __func__);
		}
		ret = NETDEV_TX_BUSY;
		goto out;
	}

	/* Map the first skb fragment */
	entry = priv->tx_prod % txsize;
	buffer = &priv->tx_ring[entry];

	dma_addr = dma_map_single(priv->device, skb->data, nopaged_len,
				  DMA_TO_DEVICE);
	if (dma_mapping_error(priv->device, dma_addr)) {
		netdev_err(priv->dev, "%s: DMA mapping error\n", __func__);
		ret = NETDEV_TX_OK;
		goto out;
	}

	buffer->skb = skb;
	buffer->dma_addr = dma_addr;
	buffer->len = nopaged_len;

	priv->dmaops->tx_buffer(priv, buffer);

	skb_tx_timestamp(skb);

	priv->tx_prod++;
	dev->stats.tx_bytes += skb->len;

	if (unlikely(tse_tx_avail(priv) <= TXQUEUESTOP_THRESHHOLD)) {
		if (netif_msg_hw(priv))
			netdev_dbg(priv->dev, "%s: stop transmitted packets\n",
				   __func__);
		netif_stop_queue(dev);
	}

out:
	spin_unlock_bh(&priv->tx_lock);

	return ret;
}

/* Called every time the controller might need to be made
 * aware of new link state.  The PHY code conveys this
 * information through variables in the phydev structure, and this
 * function converts those variables into the appropriate
 * register values, and can bring down the device if needed.
 */
static void altera_tse_adjust_link(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	struct phy_device *phydev = dev->phydev;
	int new_state = 0;

	/* only change config if there is a link */
	spin_lock(&priv->mac_cfg_lock);
	if (phydev->link) {
		/* Read old config */
		u32 cfg_reg = ioread32(&priv->mac_dev->command_config);

		/* Check duplex */
		if (phydev->duplex != priv->oldduplex) {
			new_state = 1;
			if (!(phydev->duplex))
				cfg_reg |= MAC_CMDCFG_HD_ENA;
			else
				cfg_reg &= ~MAC_CMDCFG_HD_ENA;

			netdev_dbg(priv->dev, "%s: Link duplex = 0x%x\n",
				   dev->name, phydev->duplex);

			priv->oldduplex = phydev->duplex;
		}

		/* Check speed */
		if (phydev->speed != priv->oldspeed) {
			new_state = 1;
			switch (phydev->speed) {
			case 1000:
				cfg_reg |= MAC_CMDCFG_ETH_SPEED;
				cfg_reg &= ~MAC_CMDCFG_ENA_10;
				break;
			case 100:
				cfg_reg &= ~MAC_CMDCFG_ETH_SPEED;
				cfg_reg &= ~MAC_CMDCFG_ENA_10;
				break;
			case 10:
				cfg_reg &= ~MAC_CMDCFG_ETH_SPEED;
				cfg_reg |= MAC_CMDCFG_ENA_10;
				break;
			default:
				if (netif_msg_link(priv))
					netdev_warn(dev, "Speed (%d) is not 10/100/1000!\n",
						    phydev->speed);
				break;
			}
			priv->oldspeed = phydev->speed;
		}
		iowrite32(cfg_reg, &priv->mac_dev->command_config);

		if (!priv->oldlink) {
			new_state = 1;
			priv->oldlink = 1;
		}
	} else if (priv->oldlink) {
		new_state = 1;
		priv->oldlink = 0;
		priv->oldspeed = 0;
		priv->oldduplex = -1;
	}

	if (new_state && netif_msg_link(priv))
		phy_print_status(phydev);

	spin_unlock(&priv->mac_cfg_lock);
}
static struct phy_device *connect_local_phy(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	struct phy_device *phydev = NULL;
	char phy_id_fmt[MII_BUS_ID_SIZE + 3];

	if (priv->phy_addr != POLL_PHY) {
		snprintf(phy_id_fmt, MII_BUS_ID_SIZE + 3, PHY_ID_FMT,
			 priv->mdio->id, priv->phy_addr);

		netdev_dbg(dev, "trying to attach to %s\n", phy_id_fmt);

		phydev = phy_connect(dev, phy_id_fmt, &altera_tse_adjust_link,
				     priv->phy_iface);
		if (IS_ERR(phydev)) {
			netdev_err(dev, "Could not attach to PHY\n");
			phydev = NULL;
		}

	} else {
		int ret;
		phydev = phy_find_first(priv->mdio);
		if (phydev == NULL) {
			netdev_err(dev, "No PHY found\n");
			return phydev;
		}

		ret = phy_connect_direct(dev, phydev, &altera_tse_adjust_link,
				priv->phy_iface);
		if (ret != 0) {
			netdev_err(dev, "Could not attach to PHY\n");
			phydev = NULL;
		}
	}
	return phydev;
}

static int altera_tse_phy_get_addr_mdio_create(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	struct device_node *np = priv->device->of_node;
	int ret;

	ret = of_get_phy_mode(np, &priv->phy_iface);

	/* Avoid get phy addr and create mdio if no phy is present */
	if (ret)
		return 0;

	/* try to get PHY address from device tree, use PHY autodetection if
	 * no valid address is given
	 */

	if (of_property_read_u32(priv->device->of_node, "phy-addr",
			 &priv->phy_addr)) {
		priv->phy_addr = POLL_PHY;
	}

	if (!((priv->phy_addr == POLL_PHY) ||
		  ((priv->phy_addr >= 0) && (priv->phy_addr < PHY_MAX_ADDR)))) {
		netdev_err(dev, "invalid phy-addr specified %d\n",
			priv->phy_addr);
		return -ENODEV;
	}

	/* Create/attach to MDIO bus */
	ret = altera_tse_mdio_create(dev,
					 atomic_add_return(1, &instance_count));

	if (ret)
		return -ENODEV;

	return 0;
}

/* Initialize driver's PHY state, and attach to the PHY
 */
int init_phy(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	struct phy_device *phydev;
	struct device_node *phynode;
	bool fixed_link = false;
	int rc = 0;

    netdev_dbg(dev, "init_phy\n");
    
	/* Avoid init phy in case of no phy present */
	if (!priv->phy_iface){
        netdev_info(dev, "no phy present\n");
		return 0;
    }

	priv->oldlink = 0;
	priv->oldspeed = 0;
	priv->oldduplex = -1;

	phynode = of_parse_phandle(priv->device->of_node, "phy-handle", 0);

	if (!phynode) {
		/* check if a fixed-link is defined in device-tree */
		if (of_phy_is_fixed_link(priv->device->of_node)) {
			rc = of_phy_register_fixed_link(priv->device->of_node);
			if (rc < 0) {
				netdev_err(dev, "cannot register fixed PHY\n");
				return rc;
			}

			/* In the case of a fixed PHY, the DT node associated
			 * to the PHY is the Ethernet MAC DT node.
			 */
			phynode = of_node_get(priv->device->of_node);
			fixed_link = true;

			netdev_dbg(dev, "fixed-link detected\n");
			phydev = of_phy_connect(dev, phynode,
						&altera_tse_adjust_link,
						0, priv->phy_iface);
		} else {
			netdev_dbg(dev, "no phy-handle found\n");
			if (!priv->mdio) {
				netdev_err(dev, "No phy-handle nor local mdio specified\n");
				return -ENODEV;
			}
			phydev = connect_local_phy(dev);
		}
	} else {
		netdev_dbg(dev, "phy-handle found\n");
		phydev = of_phy_connect(dev, phynode,
			&altera_tse_adjust_link, 0, priv->phy_iface);
	}
	of_node_put(phynode);

	if (!phydev) {
		netdev_err(dev, "Could not find the PHY\n");
		if (fixed_link)
			of_phy_deregister_fixed_link(priv->device->of_node);
		return -ENODEV;
	}

	/* Stop Advertising 1000BASE Capability if interface is not GMII
	 */
	if ((priv->phy_iface == PHY_INTERFACE_MODE_MII) ||
	    (priv->phy_iface == PHY_INTERFACE_MODE_RMII))
		phy_set_max_speed(phydev, SPEED_100);

	/* Broken HW is sometimes missing the pull-up resistor on the
	 * MDIO line, which results in reads to non-existent devices returning
	 * 0 rather than 0xffff. Catch this here and treat 0 as a non-existent
	 * device as well. If a fixed-link is used the phy_id is always 0.
	 * Note: phydev->phy_id is the result of reading the UID PHY registers.
	 */
	if ((phydev->phy_id == 0) && !fixed_link) {
		netdev_err(dev, "Bad PHY UID 0x%08x\n", phydev->phy_id);
		phy_disconnect(phydev);
		return -ENODEV;
	}

	netdev_dbg(dev, "attached to PHY %d UID 0x%08x Link = %d\n",
		   phydev->mdio.addr, phydev->phy_id, phydev->link);

	return 0;
}

static void tse_update_mac_addr(struct altera_tse_private *priv, u8 *addr)
{
	u32 msb;
	u32 lsb;

	msb = (addr[3] << 24) | (addr[2] << 16) | (addr[1] << 8) | addr[0];
	lsb = ((addr[5] << 8) | addr[4]) & 0xffff;

    netdev_err(priv->dev, "%s not handled",__func__);
	/* Set primary MAC address */
	csrwr32(msb, priv->mac_dev, tse_csroffs(mac_addr_0));
	csrwr32(lsb, priv->mac_dev, tse_csroffs(mac_addr_1));
    
    
}

/* Initialize MAC core registers
*/
static int init_mac(struct altera_tse_private *priv)
{
	unsigned int cmd = 0;
	u32 frm_length;

    netdev_err(priv->dev, "%s not handled",__func__);

	/* Setup Rx FIFO */
	csrwr32(priv->rx_fifo_depth - ALTERA_TSE_RX_SECTION_EMPTY,
		priv->mac_dev, tse_csroffs(rx_section_empty));

	csrwr32(ALTERA_TSE_RX_SECTION_FULL, priv->mac_dev,
		tse_csroffs(rx_section_full));

	csrwr32(ALTERA_TSE_RX_ALMOST_EMPTY, priv->mac_dev,
		tse_csroffs(rx_almost_empty));

	csrwr32(ALTERA_TSE_RX_ALMOST_FULL, priv->mac_dev,
		tse_csroffs(rx_almost_full));

	/* Setup Tx FIFO */
	csrwr32(priv->tx_fifo_depth - ALTERA_TSE_TX_SECTION_EMPTY,
		priv->mac_dev, tse_csroffs(tx_section_empty));

	csrwr32(ALTERA_TSE_TX_SECTION_FULL, priv->mac_dev,
		tse_csroffs(tx_section_full));

	csrwr32(ALTERA_TSE_TX_ALMOST_EMPTY, priv->mac_dev,
		tse_csroffs(tx_almost_empty));

	csrwr32(ALTERA_TSE_TX_ALMOST_FULL, priv->mac_dev,
		tse_csroffs(tx_almost_full));

	/* MAC Address Configuration */
	tse_update_mac_addr(priv, priv->dev->dev_addr);

	/* MAC Function Configuration */
	frm_length = ETH_HLEN + priv->dev->mtu + ETH_FCS_LEN;
	csrwr32(frm_length, priv->mac_dev, tse_csroffs(frm_length));

	csrwr32(ALTERA_TSE_TX_IPG_LENGTH, priv->mac_dev,
		tse_csroffs(tx_ipg_length));

	/* Disable RX/TX shift 16 for alignment of all received frames on 16-bit
	 * start address
	 */
	tse_set_bit(priv->mac_dev, tse_csroffs(rx_cmd_stat),
		    ALTERA_TSE_RX_CMD_STAT_RX_SHIFT16);

	tse_clear_bit(priv->mac_dev, tse_csroffs(tx_cmd_stat),
		      ALTERA_TSE_TX_CMD_STAT_TX_SHIFT16 |
		      ALTERA_TSE_TX_CMD_STAT_OMIT_CRC);

	/* Set the MAC options */
	cmd = csrrd32(priv->mac_dev, tse_csroffs(command_config));
	cmd &= ~MAC_CMDCFG_PAD_EN;	/* No padding Removal on Receive */
	cmd &= ~MAC_CMDCFG_CRC_FWD;	/* CRC Removal */
	cmd |= MAC_CMDCFG_RX_ERR_DISC;	/* Automatically discard frames
					 * with CRC errors
					 */
	cmd |= MAC_CMDCFG_CNTL_FRM_ENA;
	cmd &= ~MAC_CMDCFG_TX_ENA;
	cmd &= ~MAC_CMDCFG_RX_ENA;

	/* Default speed and duplex setting, full/100 */
	cmd &= ~MAC_CMDCFG_HD_ENA;
	cmd &= ~MAC_CMDCFG_ETH_SPEED;
	cmd &= ~MAC_CMDCFG_ENA_10;

	csrwr32(cmd, priv->mac_dev, tse_csroffs(command_config));

	csrwr32(ALTERA_TSE_PAUSE_QUANTA, priv->mac_dev,
		tse_csroffs(pause_quanta));

	if (netif_msg_hw(priv))
		dev_dbg(priv->device,
			"MAC post-initialization: CMD_CONFIG = 0x%08x\n", cmd);

	return 0;
}

/* Start/stop MAC transmission logic
 */
static void tse_set_mac(struct altera_tse_private *priv, bool enable)
{
    struct tmp {
        struct altera_tse_mac __iomem *base_addr;
    } _tmp = {priv->mac_dev}, *this_tse = &_tmp;

    netdev_dbg(priv->dev,"%s:%d: before enable CFG1 = 0x%x\n",__func__,__LINE__, HAL_get_32bit_reg(priv->mac_dev,CFG1));

    /* Enable tx and rx at MAC level. */
    HAL_set_32bit_reg_field(this_tse->base_addr, CFG1_TX_EN, 0x01u);

    /* Enable reception at MAC level.    */
    HAL_set_32bit_reg_field(this_tse->base_addr, CFG1_RX_EN, 0x01u);

    netdev_dbg(priv->dev,"%s:%d: after enable CFG1 = 0x%x\n",__func__,__LINE__, HAL_get_32bit_reg(priv->mac_dev,CFG1));
}

/* Change the MTU
 */
static int tse_change_mtu(struct net_device *dev, int new_mtu)
{
	if (netif_running(dev)) {
		netdev_err(dev, "must be stopped to change its MTU\n");
		return -EBUSY;
	}

	dev->mtu = new_mtu;
	netdev_update_features(dev);

	return 0;
}

static void altera_tse_set_mcfilter(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	int i;
	struct netdev_hw_addr *ha;

    netdev_err(dev, "%s not handled",__func__);

	/* clear the hash filter */
	for (i = 0; i < 64; i++)
		csrwr32(0, priv->mac_dev, tse_csroffs(hash_table) + i * 4);

	netdev_for_each_mc_addr(ha, dev) {
		unsigned int hash = 0;
		int mac_octet;

		for (mac_octet = 5; mac_octet >= 0; mac_octet--) {
			unsigned char xor_bit = 0;
			unsigned char octet = ha->addr[mac_octet];
			unsigned int bitshift;

			for (bitshift = 0; bitshift < 8; bitshift++)
				xor_bit ^= ((octet >> bitshift) & 0x01);

			hash = (hash << 1) | xor_bit;
		}
		csrwr32(1, priv->mac_dev, tse_csroffs(hash_table) + hash * 4);
	}
}


static void altera_tse_set_mcfilterall(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	int i;

    netdev_err(dev, "%s not handled",__func__);
	/* set the hash filter */
	for (i = 0; i < 64; i++)
		csrwr32(1, priv->mac_dev, tse_csroffs(hash_table) + i * 4);
}

/* Set or clear the multicast filter for this adaptor
 */
static void tse_set_rx_mode_hashfilter(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);

    netdev_err(dev, "%s not handled",__func__);
	spin_lock(&priv->mac_cfg_lock);

	if (dev->flags & IFF_PROMISC)
		tse_set_bit(priv->mac_dev, tse_csroffs(command_config),
			    MAC_CMDCFG_PROMIS_EN);

	if (dev->flags & IFF_ALLMULTI)
		altera_tse_set_mcfilterall(dev);
	else
		altera_tse_set_mcfilter(dev);

	spin_unlock(&priv->mac_cfg_lock);
}

/* Set or clear the multicast filter for this adaptor
 */
static void tse_set_rx_mode(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);

	/* TODO */
	return;
	
    netdev_err(dev, "%s not handled",__func__);
	spin_lock(&priv->mac_cfg_lock);

	if ((dev->flags & IFF_PROMISC) || (dev->flags & IFF_ALLMULTI) ||
	    !netdev_mc_empty(dev) || !netdev_uc_empty(dev))
		tse_set_bit(priv->mac_dev, tse_csroffs(command_config),
			    MAC_CMDCFG_PROMIS_EN);
	else
		tse_clear_bit(priv->mac_dev, tse_csroffs(command_config),
			      MAC_CMDCFG_PROMIS_EN);

	spin_unlock(&priv->mac_cfg_lock);
}

/* Initialise (if necessary) the SGMII PCS component
 */
static int init_sgmii_pcs(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	int n;
	unsigned int tmp_reg = 0;

	if (priv->phy_iface != PHY_INTERFACE_MODE_SGMII)
		return 0; /* Nothing to do, not in SGMII mode */

    netdev_err(dev, "%s not handled",__func__);
	/* The TSE SGMII PCS block looks a little like a PHY, it is
	 * mapped into the zeroth MDIO space of the MAC and it has
	 * ID registers like a PHY would.  Sadly this is often
	 * configured to zeroes, so don't be surprised if it does
	 * show 0x00000000.
	 */

	if (sgmii_pcs_scratch_test(priv, 0x0000) &&
		sgmii_pcs_scratch_test(priv, 0xffff) &&
		sgmii_pcs_scratch_test(priv, 0xa5a5) &&
		sgmii_pcs_scratch_test(priv, 0x5a5a)) {
		netdev_info(dev, "PCS PHY ID: 0x%04x%04x\n",
				sgmii_pcs_read(priv, MII_PHYSID1),
				sgmii_pcs_read(priv, MII_PHYSID2));
	} else {
		netdev_err(dev, "SGMII PCS Scratch memory test failed.\n");
		return -ENOMEM;
	}

	/* Starting on page 5-29 of the MegaCore Function User Guide
	 * Set SGMII Link timer to 1.6ms
	 */
	sgmii_pcs_write(priv, SGMII_PCS_LINK_TIMER_0, 0x0D40);
	sgmii_pcs_write(priv, SGMII_PCS_LINK_TIMER_1, 0x03);

	/* Enable SGMII Interface and Enable SGMII Auto Negotiation */
	sgmii_pcs_write(priv, SGMII_PCS_IF_MODE, 0x3);

	/* Enable Autonegotiation */
	tmp_reg = sgmii_pcs_read(priv, MII_BMCR);
	tmp_reg |= (BMCR_SPEED1000 | BMCR_FULLDPLX | BMCR_ANENABLE);
	sgmii_pcs_write(priv, MII_BMCR, tmp_reg);

	/* Reset PCS block */
	tmp_reg |= BMCR_RESET;
	sgmii_pcs_write(priv, MII_BMCR, tmp_reg);
	for (n = 0; n < SGMII_PCS_SW_RESET_TIMEOUT; n++) {
		if (!(sgmii_pcs_read(priv, MII_BMCR) & BMCR_RESET)) {
			netdev_info(dev, "SGMII PCS block initialised OK\n");
			return 0;
		}
		udelay(1);
	}

	/* We failed to reset the block, return a timeout */
	netdev_err(dev, "SGMII PCS block reset failed.\n");
	return -ETIMEDOUT;
}

void
TSE_cfg_struct_def_init
(
        tse_cfg_t * cfg
)
{
    const u8 g_default_mac_address[6] =
    {
        0xC0u, 0xB1u, 0x3Cu, 0x88u, 0x88u, 0x88u
    };

    if(0 != cfg)
    {
        cfg->framefilter        = TSE_FC_DEFAULT_MASK;
        cfg->aneg_enable        = TSE_ENABLE;
        cfg->speed_duplex_select = TSE_ANEG_ALL_SPEEDS;
        cfg->phy_addr           = 0u;
        cfg->tx_edc_enable      = TSE_ERR_DET_CORR_ENABLE;
        cfg->rx_edc_enable      = TSE_ERR_DET_CORR_ENABLE;
        cfg->preamble_length    = TSE_PREAMLEN_DEFVAL;
        cfg->hugeframe_enable   = TSE_HUGE_FRAME_DISABLE;
        cfg->length_field_check = TSE_LENGTH_FILED_CHECK_ENABLE;
        cfg->pad_n_CRC          = TSE_PAD_N_CRC_ENABLE;
        cfg->append_CRC         = TSE_CRC_ENABLE;
        cfg->fullduplex         = TSE_FULLDUPLEX_ENABLE;
        cfg->loopback           = TSE_LOOPBACK_DISABLE;
        cfg->rx_flow_ctrl       = TSE_RX_FLOW_CTRL_DISABLE;
        cfg->tx_flow_ctrl       = TSE_TX_FLOW_CTRL_DISABLE;
        cfg->min_IFG            = TSE_MINIFG_DEFVAL;
        cfg->btb_IFG            = TSE_BTBIFG_DEFVAL;
        cfg->max_retx_tries     = TSE_MAXRETX_DEFVAL;
        cfg->excessive_defer    = TSE_EXSS_DEFER_DISABLE;
        cfg->nobackoff          = TSE_NO_BACKOFF_DISABLE;
        cfg->backpres_nobackoff = TSE_BACKPRESS_NO_BACKOFF_DISABLE;
        cfg->ABEB_enable        = TSE_ABEB_DISABLE;
        cfg->ABEB_truncvalue    = TSE_ABEBTRUNC_DEFVAL;
        cfg->phyclk             = TSE_DEF_PHY_CLK;
        cfg->supress_preamble   = TSE_SUPPRESS_PREAMBLE_DISABLE;
        cfg->autoscan_phys      = TSE_PHY_AUTOSCAN_DISABLE;
        cfg->max_frame_length   = TSE_MAXFRAMELEN_DEFVAL;
        cfg->non_btb_IFG        = TSE_NONBTBIFG_DEFVAL;
        cfg->slottime           = TSE_SLOTTIME_DEFVAL;
        cfg->framedrop_mask     = TSE_DEFVAL_FRAMEDROP_MASK;
        cfg->wol_enable         = TSE_WOL_DETECT_DISABLE;

        cfg->mac_addr[0] = g_default_mac_address[0];
        cfg->mac_addr[1] = g_default_mac_address[1];
        cfg->mac_addr[2] = g_default_mac_address[2];
        cfg->mac_addr[3] = g_default_mac_address[3];
        cfg->mac_addr[4] = g_default_mac_address[4];
        cfg->mac_addr[5] = g_default_mac_address[5];
    }
}


static void
mac_reset
(
    struct altera_tse_private * priv
)
{
    struct tmp {
        struct altera_tse_mac __iomem *base_addr;
    } _tmp = {priv->mac_dev}, *this_tse = &_tmp;
        
    // TODO
    // clear pending Tx
    // reset statisitics

    /* Reset MCXMAC TX functionality */
    HAL_set_32bit_reg_field(this_tse->base_addr, CFG1_TX_RST, 0x01u);

    /* Reset MCXMAC RX functionality */
    HAL_set_32bit_reg_field(this_tse->base_addr, CFG1_RX_RST, 0x01u);

    /* Reset MCXMAC TX control */
    HAL_set_32bit_reg_field(this_tse->base_addr, CFG1_TXCTL_RST, 0x01u);

    /* Reset MCXMAC RX control */
    HAL_set_32bit_reg_field(this_tse->base_addr, CFG1_RXCTL_RST, 0x01u);

    /* Reset MIIMGMT  */
    HAL_set_32bit_reg_field(this_tse->base_addr, MIIMGMT_RESET_MII_MGMT, 0x01u);

    /* Reset MIIMGMT  */
    HAL_set_32bit_reg_field(this_tse->base_addr, IFC_RESET, 0x01u);

    /* Reset FIFO watermark module */
    HAL_set_32bit_reg_field(this_tse->base_addr, FIFOCFG0_WMM_RST, 0x01u);

    /* Reset FIFO Rx system module */
    HAL_set_32bit_reg_field(this_tse->base_addr, FIFOCFG0_RSYS_RST, 0x01u);

    /* Reset FIFO Rx fab module */
    HAL_set_32bit_reg_field(this_tse->base_addr, FIFOCFG0_RFAB_RST, 0x01u);

    /* Reset FIFO Tx system module */
    HAL_set_32bit_reg_field(this_tse->base_addr, FIFOCFG0_TSYS_RST, 0x01u);

    /* Reset FIFO Tx system module */
    HAL_set_32bit_reg_field(this_tse->base_addr, FIFOCFG0_TFAB_RST, 0x01u);

}


static void config_mac_hw(struct altera_tse_private * priv, const tse_cfg_t * cfg)
{
    struct tmp {
        struct altera_tse_mac __iomem *base_addr;
    } _tmp = {priv->mac_dev}, *this_tse = &_tmp;

    uint32_t tempreg;

    /* Check for validity of configuration parameters */
//     HAL_ASSERT(IS_STATE(cfg->tx_edc_enable));
//     HAL_ASSERT(IS_STATE(cfg->rx_edc_enable));
//     HAL_ASSERT(TSE_PREAMLEN_MAXVAL >= cfg->preamble_length);
//     HAL_ASSERT(IS_STATE(cfg->hugeframe_enable));
//     HAL_ASSERT(IS_STATE(cfg->length_field_check));
//     HAL_ASSERT(IS_STATE(cfg->pad_n_CRC));
//     HAL_ASSERT(IS_STATE(cfg->append_CRC));
//     HAL_ASSERT(IS_STATE(cfg->loopback));
//     HAL_ASSERT(IS_STATE(cfg->rx_flow_ctrl));
//     HAL_ASSERT(IS_STATE(cfg->tx_flow_ctrl));
// 
//     HAL_ASSERT(TSE_BTBIFG_MAXVAL >= cfg->btb_IFG);
//     HAL_ASSERT(TSE_MAXRETX_MAXVAL >= cfg->max_retx_tries);
//     HAL_ASSERT(IS_STATE(cfg->excessive_defer));
//     HAL_ASSERT(IS_STATE(cfg->nobackoff));
//     HAL_ASSERT(IS_STATE(cfg->backpres_nobackoff));
//     HAL_ASSERT(IS_STATE(cfg->ABEB_enable));
//     HAL_ASSERT(TSE_ABEBTRUNC_MAXVAL >= cfg->ABEB_truncvalue);
//     HAL_ASSERT(TSE_BY28_PHY_CLK >= cfg->phyclk);
//     HAL_ASSERT(IS_STATE(cfg->supress_preamble));
//     HAL_ASSERT(IS_STATE(cfg->autoscan_phys));
//     HAL_ASSERT(TSE_MAXFRAMELEN_MAXVAL >= cfg->max_frame_length);
//     HAL_ASSERT(TSE_SLOTTIME_MAXVAL >= cfg->slottime);

    /* Configure PHY related MII MGMT registers */
//     tempreg = (uint32_t)cfg->phyclk & MII_CLOCK_SELECT_MASK;
// 
//     if(TSE_ENABLE == cfg->supress_preamble)
//     {
//         tempreg |= MII_PREAM_SUPRESS_MASK;
//     }
// 
//     if(TSE_ENABLE == cfg->autoscan_phys)
//     {
//         tempreg |= MII_SCAN_AUTO_INC_MASK;
//     }
// 
//     HAL_set_32bit_reg(this_tse->base_addr, MIIMGMT, tempreg);

    /* Clear all reset bits */
    /* Clear soft reset for MCXMAC, Tx function, Rx function, Tx MAC control and
       Rx MAC control.
     */
    HAL_set_32bit_reg_field(this_tse->base_addr, CFG1_SOFT_RST, 0x00u);
    HAL_set_32bit_reg_field(this_tse->base_addr, CFG1_TXCTL_RST, 0x00u);
    HAL_set_32bit_reg_field(this_tse->base_addr, CFG1_RXCTL_RST, 0x00u);
    HAL_set_32bit_reg_field(this_tse->base_addr, CFG1_TX_RST, 0x00u);
    HAL_set_32bit_reg_field(this_tse->base_addr, CFG1_RX_RST, 0x00u);

    /* Clear MIIMGMT */
//     HAL_set_32bit_reg_field(this_tse->base_addr, MIIMGMT_RESET_MII_MGMT, 0x00u);

    HAL_set_32bit_reg_field(this_tse->base_addr, IFC_RESET, 0x00u);

    /* Clear FIFO resets. */
    HAL_set_32bit_reg_field(this_tse->base_addr, FIFOCFG0_WMM_RST, 0x00u);
    HAL_set_32bit_reg_field(this_tse->base_addr, FIFOCFG0_RSYS_RST, 0x00u);
    HAL_set_32bit_reg_field(this_tse->base_addr, FIFOCFG0_RFAB_RST, 0x00u);
    HAL_set_32bit_reg_field(this_tse->base_addr, FIFOCFG0_TSYS_RST, 0x00u);
    HAL_set_32bit_reg_field(this_tse->base_addr, FIFOCFG0_TFAB_RST, 0x00u);

    /* configure loppback and flow control enables. */
    if(TSE_ENABLE == cfg->loopback)
    {
        tempreg = CFG1_LOOPBACK_MASK;
    }
    else
    {
        tempreg = 0u;
    }

    if(TSE_ENABLE == cfg->rx_flow_ctrl)
    {
        tempreg |= CFG1_RX_FCTL_MASK;
    }

    if(TSE_ENABLE == cfg->tx_flow_ctrl)
    {
        tempreg |= CFG1_TX_FCTL_MASK;
    }
    
    HAL_set_32bit_reg(this_tse->base_addr, CFG1, tempreg);

    if(TSE_ENABLE == cfg->fullduplex)
    {
        tempreg = CFG2_FDX_MASK;
    }
    else
    {
        tempreg = 0u;
    }

    if(TSE_ENABLE == cfg->append_CRC)
    {
        tempreg |= CFG2_CRC_EN_MASK;
    }

    if(TSE_ENABLE == cfg->pad_n_CRC)
    {
        tempreg |= CFG2_PAD_CRC_EN_MASK;
    }

    if(TSE_ENABLE == cfg->length_field_check)
    {
        tempreg |= CFG2_LEN_CHECK_MASK;
    }

    if(TSE_ENABLE == cfg->hugeframe_enable)
    {
        tempreg |= CFG2_HUGE_FRAME_EN_MASK;
    }

//     if((TSE_PHY_INTERFACE == TSE_MII) || (TSE_PHY_INTERFACE == TSE_RMII))
//     {
//         /* MII and RMII use nibble mode interface. */
//         tempreg |= CFG2_NIBBLE_MASK;
//     }
    else
    {
        /* TBI and GMII use byte interface. */
        tempreg |= CFG2_BYTE_MASK;
    }

    tempreg |= (((uint32_t)cfg->preamble_length) << CFG2_PREAM_LEN);
    HAL_set_32bit_reg(this_tse->base_addr, CFG2, tempreg);

    tempreg = cfg->btb_IFG;
    tempreg |= ((uint32_t)cfg->min_IFG << IFG_MINIFGENF);
    tempreg |= ((uint32_t)cfg->non_btb_IFG << IFG_NONBTBIPG);

    HAL_set_32bit_reg(this_tse->base_addr, IFG, tempreg);
    
    tempreg = (uint32_t)cfg->slottime & HALF_DUPLEX_SLOTTIME_MASK;

    tempreg |= (uint32_t)cfg->max_retx_tries << HALF_DUPLEX_RETX_MAX_OFFSET;

    if(TSE_ENABLE == cfg->excessive_defer)
    {
        tempreg |= HALF_DUPLEX_EXCS_DEFER_MASK;
    }

    if(TSE_ENABLE == cfg->nobackoff)
    {
        tempreg |= HALF_DUPLEX_NO_BACKOFF_MASK;
    }

    if(TSE_ENABLE == cfg->backpres_nobackoff)
    {
        tempreg |= HALF_DUPLEX_BACKPRES_NOBACKOFF_MASK;
    }

    if(TSE_ENABLE == cfg->ABEB_enable)
    {
        tempreg |= HALF_DUPLEX_ABEB_ENABLE_MASK;
    }

    tempreg |= (uint32_t)cfg->ABEB_truncvalue << HALF_DUPLEX_ABEB_TUNC_OFFSET;

    HAL_set_32bit_reg(this_tse->base_addr, HDX, tempreg);
        
    HAL_set_32bit_reg(this_tse->base_addr, FREMLEN, (uint32_t)cfg->max_frame_length);
    
    /*Enable WoL*/
    if(TSE_WOL_DETECT_DISABLE != cfg->wol_enable)
    {
        if(TSE_WOL_UNICAST_FRAME_DETECT_EN & cfg->wol_enable)
        {
            HAL_set_32bit_reg_field(this_tse->base_addr, IFC_MCXWOL_UMATCH_EN, 0x01u);
        }

        if(TSE_WOL_MAGIC_FRAME_DETECT_EN & cfg->wol_enable)
        {
            HAL_set_32bit_reg_field(this_tse->base_addr, IFC_MCXWOL_MAGIC_EN, 0x01u);
        }
    }

    /*--------------------------------------------------------------------------
     * Configure FIFOs
     */
    HAL_set_32bit_reg_field(this_tse->base_addr, FIFOCFG0_WMM_EN, 0x01);
    HAL_set_32bit_reg_field(this_tse->base_addr, FIFOCFG0_RSYS_EN, 0x01);
    HAL_set_32bit_reg_field(this_tse->base_addr, FIFOCFG0_RFAB_EN, 0x01);
    HAL_set_32bit_reg_field(this_tse->base_addr, FIFOCFG0_TSYS_EN, 0x01);
    HAL_set_32bit_reg_field(this_tse->base_addr, FIFOCFG0_TFAB_EN, 0x01);

    /* RX FIFO size : 8KB  */
    HAL_set_32bit_reg(this_tse->base_addr, FIFOCFG1, FIFO_CFG1_DEFVAL);

    /* Rx FIFO watermark */
    HAL_set_32bit_reg(this_tse->base_addr, FIFOCFG2, FIFO_CFG2_DEFVAL);

    /* Tx FIFO watermark: 4KB Tx FIFO */
    HAL_set_32bit_reg(this_tse->base_addr, FIFOCFG3, FIFO_CFG3_DEFVAL);

    tempreg = HAL_get_32bit_reg_field(this_tse->base_addr, FIFOCFG5_HSTFLTRFRMDC);

    tempreg |= 0x0003FFFF;

    tempreg &= ~(cfg->framedrop_mask);

    HAL_set_32bit_reg_field(this_tse->base_addr, FIFOCFG5_HSTFLTRFRMDC, tempreg);

    HAL_set_32bit_reg_field(this_tse->base_addr, FIFOCFG4_HSTFLTRFRM, cfg->framedrop_mask);

    if(cfg->framedrop_mask & TSE_SHORT_FRAME_FRAMEDROP_MASK)
    {
        HAL_set_32bit_reg_field(this_tse->base_addr, FIFOCFG5_HSTDRPLT64, 0x01);
    }
    netdev_dbg(priv->dev,"%s:%d: CFG1 = 0x%x\n",__func__,__LINE__, HAL_get_32bit_reg(priv->mac_dev,CFG1));
}

static void update_mac_cfg(struct altera_tse_private * priv, u8 phy_addr)
{
    struct tmp {
        struct altera_tse_mac __iomem *base_addr;
    } _tmp = {priv->mac_dev}, *this_tse = &_tmp;

    tse_speed_t speed;
    u8 fullduplex;
    u8 link_up;
    netdev_dbg(priv->dev,"%s:%d: CFG1 = 0x%x\n",__func__,__LINE__, HAL_get_32bit_reg(priv->mac_dev,CFG1));

    // NO PHY
// #if (TSE_PHY_INTERFACE == TSE_1000BASEX)
    // link_up = msgmii_get_link_status(this_tse, &speed, &fullduplex);
	link_up = TSE_LINK_UP;
	fullduplex = TSE_FULL_DUPLEX;
	speed = TSE_MAC1000MBPS;
// #else
//     // link_up = TSE_phy_get_link_status(this_tse, phy_addr, &speed, &fullduplex);
// #endif

    if(TSE_LINK_DOWN != link_up)
    {

#if (TSE_PHY_INTERFACE == TSE_RGMII)
        // rgmii_set_link_speed(this_tse, speed);
#endif

        /* Set byte/nibble mode based on interface type and link speed. */
        HAL_set_32bit_reg_field(this_tse->base_addr, CFG2_IF_MODE, 0x00u);

        if(TSE_MAC1000MBPS == speed)
        {
            /* Set interface to byte mode. */
            HAL_set_32bit_reg_field(this_tse->base_addr, CFG2_IF_MODE, 0x02);

            /* RB */
            HAL_set_32bit_reg(this_tse->base_addr, MISCC, SPEED_1000M);
            HAL_set_32bit_reg_field(this_tse->base_addr, FIFOCFG5_CFGBYTEMODE, 0x01);
        }
        else
        {
            /* Set interface to nibble mode. */
            HAL_set_32bit_reg_field(this_tse->base_addr, CFG2_IF_MODE, 0x00u);

            if(TSE_MAC100MBPS == speed)
            {
                HAL_set_32bit_reg(this_tse->base_addr, MISCC, SPEED_100M);
                HAL_set_32bit_reg_field(this_tse->base_addr, CFG2_IF_MODE, 0x01);
            }
            else
            {
                HAL_set_32bit_reg(this_tse->base_addr, MISCC, SPEED_10M);
            }

            HAL_set_32bit_reg_field(this_tse->base_addr, FIFOCFG5_CFGBYTEMODE, 0x00);
        }

        /* Configure duplex mode */
        if(TSE_HALF_DUPLEX == fullduplex)
        {
            /* half duplex */
            HAL_set_32bit_reg_field(this_tse->base_addr, CFG2_FDX, 0x0);
        }
        else
        {
            /* full duplex */
            HAL_set_32bit_reg_field(this_tse->base_addr, CFG2_FDX, 0x01);
        }
    }
}

static void assign_station_addr
(
    struct altera_tse_private *priv,
    const uint8_t mac_addr[6]
)
{
    struct tmp {
        struct altera_tse_mac __iomem *base_addr;
    } _tmp = {priv->mac_dev}, *this_tse = &_tmp;
    uint32_t address32;

    address32 = ((uint32_t)mac_addr[5]) << 24u;
    address32 |= ((uint32_t)mac_addr[4]) << 16u;
    address32 |= ((uint32_t)mac_addr[3]) << 8u;
    address32 |= ((uint32_t)mac_addr[2]);

    HAL_set_32bit_reg(this_tse->base_addr, STADDRL, address32);

    address32 = ((uint32_t)mac_addr[1]) << 24u;
    address32 |= ((uint32_t)mac_addr[0]) << 16u;

    HAL_set_32bit_reg(this_tse->base_addr, STADDRH, address32);

}

void
TSE_init
(
    struct altera_tse_private *priv,
    tse_cfg_t * cfg
)
{
	struct coretse_mac *mac = (struct coretse_mac *)(priv->mac_dev);
    void __iomem *base_addr = mac;
    
    netdev_dbg(priv->dev,"%s:%d: CFG1 = 0x%x\n",__func__,__LINE__, HAL_get_32bit_reg(priv->mac_dev,CFG1));
    
    HAL_set_32bit_reg_field(base_addr, CFG2_IF_MODE, 0x00u);

    if(cfg != 0)
    {
        HAL_set_32bit_reg(base_addr, DMAINTRMASK, 0x00000000UL);

        mac_reset(priv);

        HAL_set_32bit_reg_field(base_addr, CFG2_IF_MODE, 0x00u);

        config_mac_hw(priv, cfg);

        /* Assign MAC station address */
        assign_station_addr(priv, cfg->mac_addr);

        // init descriptors
        // coretse_dma_init(priv);
        
         /* initialize default interrupt handlers */
        // TODO
        // this_tse->tx_complete_handler = NULL_POINTER;
        // this_tse->pckt_rx_callback = NULL_POINTER;

        update_mac_cfg(priv, priv->phy_addr);

        /*Enable Stats module*/
        HAL_set_32bit_reg_field(base_addr, IFC_STATS_EN, 0x01u);

        /*Need to write 1 and then 0 to correctly clear and start all counters*/
        HAL_set_32bit_reg_field(base_addr, IFC_STATS_CLR_ALL, 0x01u);
        HAL_set_32bit_reg_field(base_addr, IFC_STATS_CLR_ALL, 0x00u);

        /* Enable default Flow Control at MAC level.    */
        HAL_set_32bit_reg(base_addr, FPC, cfg->framefilter);

    }
    netdev_info(priv->dev, "TSE_init completed\n");
}

static int coretse_open(struct altera_tse_private *priv) 
{
	static tse_cfg_t g_tse_config_1;
	static u8* mac_address_1;
    mac_address_1 = priv->dev->dev_addr;

    TSE_cfg_struct_def_init(&g_tse_config_1);

    g_tse_config_1.phy_addr = 0;
    g_tse_config_1.aneg_enable = 0;
    g_tse_config_1.speed_duplex_select = TSE_ANEG_1000M_FD;
    g_tse_config_1.pad_n_CRC = 0;
    g_tse_config_1.append_CRC = 1;
    g_tse_config_1.loopback = 0;
    g_tse_config_1.mac_addr[0] = mac_address_1[0];
    g_tse_config_1.mac_addr[1] = mac_address_1[1];
    g_tse_config_1.mac_addr[2] = mac_address_1[2];
    g_tse_config_1.mac_addr[3] = mac_address_1[3];
    g_tse_config_1.mac_addr[4] = mac_address_1[4];
    g_tse_config_1.mac_addr[5] = mac_address_1[5];
    g_tse_config_1.framefilter = TSE_FC_PROMISCOUS_MODE_MASK;

    TSE_init(priv, &g_tse_config_1);
    netdev_dbg(priv->dev,"%s:%d: CFG1 = 0x%x\n",__func__,__LINE__, HAL_get_32bit_reg(priv->mac_dev,CFG1));
    return 0;
}
    
    
/* Open and initialize the interface
 */
static int tse_open(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);
	int ret = 0;
	int i;
	unsigned long int flags;

	/* Reset and configure TSE MAC and probe associated PHY */
	ret = priv->dmaops->init_dma(priv);
	if (ret != 0) {
		netdev_err(dev, "Cannot initialize DMA\n");
		goto phy_error;
	}

	if (netif_msg_ifup(priv))
		netdev_warn(dev, "device MAC address %pM\n",
			    dev->dev_addr);

	if ((priv->revision < 0xd00) || (priv->revision > 0xe00))
		netdev_warn(dev, "TSE revision %x\n", priv->revision);

	spin_lock(&priv->mac_cfg_lock);
	/* no-op if MAC not operating in SGMII mode*/
	ret = init_sgmii_pcs(dev);
	if (ret) {
		netdev_err(dev,
			   "Cannot init the SGMII PCS (error: %d)\n", ret);
		spin_unlock(&priv->mac_cfg_lock);
		goto phy_error;
	}

	// TODO
    mac_reset(priv);
// 	/* Note that reset_mac will fail if the clocks are gated by the PHY
// 	 * due to the PHY being put into isolation or power down mode.
// 	 * This is not an error if reset fails due to no clock.
// 	 */
// 	if (ret)
// 		netdev_dbg(dev, "Cannot reset MAC core (error: %d)\n", ret);

    // ret = init_mac(priv);
    ret = coretse_open(priv);
	spin_unlock(&priv->mac_cfg_lock);
	if (ret) {
		netdev_err(dev, "Cannot init MAC core (error: %d)\n", ret);
		goto alloc_skbuf_error;
	}

	priv->dmaops->reset_dma(priv);

	/* Create and initialize the TX/RX descriptors chains. */
	priv->rx_ring_size = dma_rx_num;
	priv->tx_ring_size = dma_tx_num;
	ret = alloc_init_skbufs(priv);
	if (ret) {
		netdev_err(dev, "DMA descriptors initialization failed\n");
		goto alloc_skbuf_error;
	}

	/* Register RX interrupt */
	ret = request_irq(priv->dma_irq, altera_isr, IRQF_SHARED,
			  dev->name, dev);
	if (ret) {
		netdev_err(dev, "Unable to register RX interrupt %d: %d for dev %p\n",
                   priv->dma_irq, ret, dev);
		goto init_error;
	}

	/* Enable DMA interrupts */
	spin_lock_irqsave(&priv->rxdma_irq_lock, flags);
	priv->dmaops->enable_rxirq(priv);
	priv->dmaops->enable_txirq(priv);

	/* Setup RX descriptor chain */
	for (i = 0; i < priv->rx_ring_size; i++)
		priv->dmaops->add_rx_desc(priv, &priv->rx_ring[i]);

	spin_unlock_irqrestore(&priv->rxdma_irq_lock, flags);

	if (dev->phydev)
		phy_start(dev->phydev);

	napi_enable(&priv->napi);
	netif_start_queue(dev);

	priv->dmaops->start_rxdma(priv);

	/* Start MAC Rx/Tx */
	spin_lock(&priv->mac_cfg_lock);
	tse_set_mac(priv, true);
	spin_unlock(&priv->mac_cfg_lock);

	return 0;

init_error:
	free_skbufs(dev);
alloc_skbuf_error:
phy_error:
	return ret;
}

/* Stop TSE MAC interface and put the device in an inactive state
 */
static int tse_shutdown(struct net_device *dev)
{
	struct altera_tse_private *priv = netdev_priv(dev);
// 	int ret;
	unsigned long int flags;
    
	/* Stop the PHY */
	if (dev->phydev){
        phy_stop(dev->phydev);
        if (of_phy_is_fixed_link(priv->device->of_node))
            of_phy_deregister_fixed_link(priv->device->of_node);
    }

	netif_stop_queue(dev);
	napi_disable(&priv->napi);

	/* Disable DMA interrupts */
	spin_lock_irqsave(&priv->rxdma_irq_lock, flags);
	priv->dmaops->disable_rxirq(priv);
	priv->dmaops->disable_txirq(priv);
	spin_unlock_irqrestore(&priv->rxdma_irq_lock, flags);

	/* Free the IRQ lines */
	free_irq(priv->dma_irq, dev);

	/* disable and reset the MAC, empties fifo */
	spin_lock(&priv->mac_cfg_lock);
	spin_lock(&priv->tx_lock);

    // TODO
// 	ret = reset_mac(priv);
    mac_reset(priv);
	/* Note that reset_mac will fail if the clocks are gated by the PHY
	 * due to the PHY being put into isolation or power down mode.
	 * This is not an error if reset fails due to no clock.
	 */
// 	if (ret)
// 		netdev_dbg(dev, "Cannot reset MAC core (error: %d)\n", ret);
	priv->dmaops->reset_dma(priv);
	free_skbufs(dev);

	spin_unlock(&priv->tx_lock);
	spin_unlock(&priv->mac_cfg_lock);

	priv->dmaops->uninit_dma(priv);

	return 0;
}

static struct net_device_ops altera_tse_netdev_ops = {
	.ndo_open		= tse_open,
	.ndo_stop		= tse_shutdown,
	.ndo_start_xmit		= tse_start_xmit,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_set_rx_mode	= tse_set_rx_mode,
	.ndo_change_mtu		= tse_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
};

static int request_and_map(struct platform_device *pdev, const char *name,
			   struct resource **res, void __iomem **ptr)
{
	struct resource *region;
	struct device *device = &pdev->dev;

	*res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (*res == NULL) {
		dev_err(device, "resource %s not defined\n", name);
		return -ENODEV;
	}

	region = devm_request_mem_region(device, (*res)->start,
					 resource_size(*res), dev_name(device));
	if (region == NULL) {
		dev_err(device, "unable to request %s\n", name);
		return -EBUSY;
	}

	*ptr = devm_ioremap(device, region->start,
				    resource_size(region));
	if (*ptr == NULL) {
		dev_err(device, "ioremap of %s failed!", name);
		return -ENOMEM;
	}

	return 0;
}

static void key_test(struct platform_device *pdev) {
    //struct resource *region = pdev->resource;
    //struct device *device = &pdev->dev;
    int i;
    //tse_instance_t coretse;
    //tse_cfg_t cfg;

    /* Map the MAC */    
    
	/*
	dev_info(&pdev->dev,"KEY pointers %p %d %d\n", device, region->start, resource_size(region));
    coretse.base_addr = (unsigned int)devm_ioremap(device, region->start,
            resource_size(region));
	*/
    

    
    /* Dump mapped area */
    
    /*
	dev_info(&pdev->dev,"Accessing %s 0x%llx 0x%llx\n",region->name, region->start, region->end);
    dev_info(&pdev->dev,"Mem at 0x%llx\n",region->start);
    for (i =0; i<8 ; i++) {
        dev_info(&pdev->dev,"0x%x 0x%x 0x%x 0x%x\n", ((int*)macdata)[i*4], ((int*)macdata)[i*4+1], ((int*)macdata)[i*4+2], ((int*)macdata)[i*4+3]);
    }
    */

    /* Send a frame */
    
	/* list resources available */
//     for (i = 0; i < pdev->num_resources; i++) {
// 		struct resource *r = &pdev->resource[i];
//         dev_dbg(&pdev->dev,"%d ressource %s type=0x%lx IORESOURCE_MEM=0x%x\n",i, r->name,   resource_type(r), IORESOURCE_MEM);
//         dev_dbg(&pdev->dev,"start 0x%llx size 0x%llx\n", r->start, resource_size(r));
//
// 	}
	
}

// void phy_stub(struct net_device *ndev){
//     struct phy_device phydev;
//     int phy_connect_direct(ndev, &phydev,
//                            void (*handler)(struct net_device *),
//                            phy_interface_t interface)
// }

/* Probe Altera TSE MAC device
 */
static int coretse_probe(struct platform_device *pdev)
{
	struct net_device *ndev;
	int ret = -ENODEV;
	struct resource *control_port;
	struct resource *dma_res;
	struct altera_tse_private *priv;
	const unsigned char *macaddr;
	void __iomem *descmap;
	const struct of_device_id *of_id = NULL;

    dev_info(&pdev->dev,"Coretse probing starts\n");
    

    ndev = alloc_etherdev(sizeof(struct altera_tse_private));
    if (!ndev) {
        dev_err(&pdev->dev, "Could not allocate network device\n");
        return -ENODEV;
	}

	SET_NETDEV_DEV(ndev, &pdev->dev);

	priv = netdev_priv(ndev);
	priv->device = &pdev->dev;
	priv->dev = ndev;
	priv->msg_enable = netif_msg_init(debug, default_msg_level);

	of_id = of_match_device(coretse_ids, &pdev->dev);

	if (of_id)
		priv->dmaops = (struct coretse_dmaops *)of_id->data;


    key_test(pdev);
    
	if (priv->dmaops &&
	    priv->dmaops->coretse_dtype == CORETSE_DMA) {
		
		/* Get the mapped address to the SGDMA descriptor memory */
		ret = request_and_map(pdev, "s1", &dma_res, &descmap);
		if (ret)
			goto err_free_netdev;

		/* Start of that memory is for transmit descriptors */
		priv->tx_dma_desc = descmap;

		/* First half is for tx descriptors, other half for tx */
        priv->txdescmem = resource_size(dma_res)/2;

		priv->txdescmem_busaddr = (dma_addr_t)dma_res->start;

		priv->rx_dma_desc = (void __iomem *)((uintptr_t)(descmap +
						     priv->txdescmem));
        priv->rxdescmem = resource_size(dma_res)/2;
		priv->rxdescmem_busaddr = dma_res->start;
		priv->rxdescmem_busaddr += priv->txdescmem;

		if (upper_32_bits(priv->rxdescmem_busaddr)) {
			dev_dbg(priv->device,
				"CORETSE DMA bus addresses greater than 32-bits\n");
			ret = -EINVAL;
			goto err_free_netdev;
		}
		if (upper_32_bits(priv->txdescmem_busaddr)) {
			dev_dbg(priv->device,
				"CORETSE DMA bus addresses greater than 32-bits\n");
			ret = -EINVAL;
			goto err_free_netdev;
		}
		dev_dbg(&pdev->dev,"tx_dma_desc=0x%p\n", priv->tx_dma_desc);
        dev_dbg(&pdev->dev,"txdescmem=0x%x\n", priv->txdescmem);
        dev_dbg(&pdev->dev,"txdescmem_busaddr=0x%llx\n", priv->txdescmem_busaddr);

	} else if (priv->dmaops &&
		   priv->dmaops->coretse_dtype == CORETSE_NODMA) {
        
        dev_info(&pdev->dev,"CORETSE_NODMA\n");
        // not handled yet
		dev_info(&pdev->dev,"not handled\n");
        goto err_free_netdev;
        
	/*
		ret = request_and_map(pdev, "rx_resp", &dma_res,
				      &priv->rx_dma_resp);
		if (ret)
			goto err_free_netdev;

		ret = request_and_map(pdev, "tx_desc", &dma_res,
				      &priv->tx_dma_desc);
		if (ret)
			goto err_free_netdev;

		priv->txdescmem = resource_size(dma_res);
		priv->txdescmem_busaddr = dma_res->start;

		ret = request_and_map(pdev, "rx_desc", &dma_res,
				      &priv->rx_dma_desc);
		if (ret)
			goto err_free_netdev;

		priv->rxdescmem = resource_size(dma_res);
		priv->rxdescmem_busaddr = dma_res->start;
		*/

	} else {
		goto err_free_netdev;
	}

	if (!dma_set_mask(priv->device, DMA_BIT_MASK(priv->dmaops->dmamask)))
		dma_set_coherent_mask(priv->device,
				      DMA_BIT_MASK(priv->dmaops->dmamask));
	else if (!dma_set_mask(priv->device, DMA_BIT_MASK(32)))
		dma_set_coherent_mask(priv->device, DMA_BIT_MASK(32));
	else
		goto err_free_netdev;

	/* MAC address space */
// 	control_port = pdev->resource;

// 	priv->mac_dev = (struct altera_tse_mac*)devm_ioremap(&pdev->dev, control_port->start, resource_size(control_port));
	ret = request_and_map(pdev, "mac_registers", &control_port,
			      (void __iomem **)&priv->mac_dev);
    dev_info(&pdev->dev,"control port start 0x%llx size 0x%llx priv->mac_dev 0x%p\n",control_port->start,resource_size(control_port), priv->mac_dev);
	if (!priv->mac_dev) {
		dev_err(&pdev->dev,"Cannot map MAC address space\n");
		goto err_free_netdev;
	}

	/* xSGDMA Rx Dispatcher address space */
	/*ret = request_and_map(pdev, "rx_csr", &dma_res,
			      &priv->rx_dma_csr);
	if (ret)
		goto err_free_netdev;
	*/


	/* xSGDMA Tx Dispatcher address space */
	/* ret = request_and_map(pdev, "tx_csr", &dma_res,
			      &priv->tx_dma_csr);
	if (ret)
		goto err_free_netdev;
	*/


	/* DMA IRQ */
	priv->dma_irq = platform_get_irq_byname(pdev, "dma_irq");
    if (priv->dma_irq == -ENXIO) {
		dev_err(&pdev->dev, "cannot obtain DMA IRQ\n");
		ret = -ENXIO;
		goto err_free_netdev;
	}

	/* get FIFO depths from device tree */
	/*
	if (of_property_read_u32(pdev->dev.of_node, "rx-fifo-depth",
				 &priv->rx_fifo_depth)) {
		dev_err(&pdev->dev, "cannot obtain rx-fifo-depth\n");
		ret = -ENXIO;
		goto err_free_netdev;
	}
	*/

	/*
	if (of_property_read_u32(pdev->dev.of_node, "tx-fifo-depth",
				 &priv->tx_fifo_depth)) {
		dev_err(&pdev->dev, "cannot obtain tx-fifo-depth\n");
		ret = -ENXIO;
		goto err_free_netdev;
	}
	*/

	/* get hash filter settings for this instance */
	/*
	priv->hash_filter =
		of_property_read_bool(pdev->dev.of_node,
				      "altr,has-hash-multicast-filter");
				      */

	/* Set hash filter to not set for now until the
	 * multicast filter receive issue is debugged
	 */
	priv->hash_filter = 0;

	/* get supplemental address settings for this instance */
	/*
	 * priv->added_unicast =
		of_property_read_bool(pdev->dev.of_node,
				      "altr,has-supplementary-unicast");
				      */

	priv->dev->min_mtu = ETH_ZLEN + ETH_FCS_LEN;
	/* Max MTU is 1500, ETH_DATA_LEN */
	priv->dev->max_mtu = ETH_DATA_LEN;

	/* Get the max mtu from the device tree. Note that the
	 * "max-frame-size" parameter is actually max mtu. Definition
	 * in the ePAPR v1.1 spec and usage differ, so go with usage.
	 */
	/*
	of_property_read_u32(pdev->dev.of_node, "max-frame-size",
			     &priv->dev->max_mtu);
			     */

	/* The DMA buffer size already accounts for an alignment bias
	 * to avoid unaligned access exceptions for the NIOS processor,
	 */
	priv->rx_dma_buf_sz = ALTERA_RXDMABUFFER_SIZE;

	/* get default MAC address from device tree */
	macaddr = of_get_mac_address(pdev->dev.of_node);
	if (!IS_ERR(macaddr)) {
        dev_dbg(&pdev->dev, "MAC found in DT\n");
        ether_addr_copy(ndev->dev_addr, macaddr);
    }
	else {
        dev_dbg(&pdev->dev, "No MAC found in DT\n");
        eth_hw_addr_random(ndev);
    }

	/* get phy addr and create mdio */
	ret = altera_tse_phy_get_addr_mdio_create(ndev);

	if (ret)
		goto err_free_netdev;

	/* initialize netdev */
	ndev->mem_start = control_port->start;
	ndev->mem_end = control_port->end;
	ndev->netdev_ops = &altera_tse_netdev_ops;
	altera_tse_set_ethtool_ops(ndev);

	altera_tse_netdev_ops.ndo_set_rx_mode = tse_set_rx_mode;

	if (priv->hash_filter)
		altera_tse_netdev_ops.ndo_set_rx_mode =
			tse_set_rx_mode_hashfilter;

	/* Scatter/gather IO is not supported,
	 * so it is turned off
	 */
	ndev->hw_features &= ~NETIF_F_SG;
	ndev->features |= ndev->hw_features | NETIF_F_HIGHDMA;

	/* VLAN offloading of tagging, stripping and filtering is not
	 * supported by hardware, but driver will accommodate the
	 * extra 4-byte VLAN tag for processing by upper layers
	 */
	ndev->features |= NETIF_F_HW_VLAN_CTAG_RX;

	/* setup NAPI interface */
	netif_napi_add(ndev, &priv->napi, tse_poll, NAPI_POLL_WEIGHT);

	spin_lock_init(&priv->mac_cfg_lock);
	spin_lock_init(&priv->tx_lock);
	spin_lock_init(&priv->rxdma_irq_lock);

	netif_carrier_off(ndev);
	ret = register_netdev(ndev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register TSE net device\n");
		goto err_register_netdev;
	}

	platform_set_drvdata(pdev, ndev);

	//priv->revision = ioread32(&priv->mac_dev->megacore_revision);

	if (netif_msg_probe(priv))
		dev_info(&pdev->dev, "CoreTSE MAC version %d.%d at 0x%08lx irq %d\n",
			 (priv->revision >> 8) & 0xff,
			 priv->revision & 0xff,
			 (unsigned long) control_port->start, priv->dma_irq);

        
	ret = init_phy(ndev);
	if (ret != 0) {
		netdev_err(ndev, "Cannot attach to PHY (error: %d)\n", ret);
	}
    

	return 0;

// err_init_phy:
// 	unregister_netdev(ndev);
err_register_netdev:
	netif_napi_del(&priv->napi);
	altera_tse_mdio_destroy(ndev);
err_free_netdev:
	free_netdev(ndev);
	return ret;
}

/* Remove Altera TSE MAC device
 */
static int coretse_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct altera_tse_private *priv = netdev_priv(ndev);

    dev_info(&pdev->dev,"Coretse removing starts\n");

	if (ndev->phydev) {
		phy_disconnect(ndev->phydev);

		if (of_phy_is_fixed_link(priv->device->of_node))
			of_phy_deregister_fixed_link(priv->device->of_node);
	}

	platform_set_drvdata(pdev, NULL);
// 	altera_tse_mdio_destroy(ndev);
	unregister_netdev(ndev);
	free_netdev(ndev);

	return 0;
}

static const struct coretse_dmaops coretse_dma = {
    .coretse_dtype = CORETSE_DMA,
    .dmamask = 64,
    .reset_dma = coretse_dma_reset,
    .enable_txirq = coretse_dma_enable_txirq,
    .enable_rxirq = coretse_dma_enable_rxirq,
    .disable_txirq = coretse_dma_disable_txirq,
    .disable_rxirq = coretse_dma_disable_rxirq,
    .clear_txirq = coretsedma_clear_txirq,
    .clear_rxirq = coretsedma_clear_rxirq,
    .tx_buffer = TSE_send_pkt,
    .tx_completions = coretsedma_tx_completions,
    .add_rx_desc = coretsedma_add_rx_desc,
    .get_rx_status = coretsedma_rx_status,
    .init_dma = coretsedma_initialize,
    .uninit_dma = coretsedma_uninitialize,
    .start_rxdma = coretsedma_start_rxdma,
};

static const struct coretse_dmaops coretse_nodma = {
    .coretse_dtype = CORETSE_NODMA,
};

static const struct of_device_id coretse_ids[] = {
	{ .compatible = "coretse,dma", .data = &coretse_dma, },
    { .compatible = "coretse,nodma", &coretse_nodma,},
	{},
};
MODULE_DEVICE_TABLE(of, coretse_ids);

static struct platform_driver coretse_driver = {
	.probe		= coretse_probe,
	.remove		= coretse_remove,
	.suspend	= NULL,
	.resume		= NULL,
	.driver		= {
		.name	= MICROCHIP_TSE_RESOURCE_NAME,
		.of_match_table = coretse_ids,
	},
};

module_platform_driver(coretse_driver);

MODULE_AUTHOR("Thales R&T");
MODULE_DESCRIPTION("Microchip Triple Speed Ethernet MAC driver");
MODULE_LICENSE("GPL v2");
