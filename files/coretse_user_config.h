/*******************************************************************************
 * (c) Copyright 2007 Actel Corporation.  All rights reserved.
 *
 * Actel:Firmware:CoreTSE_Driver:2.5.100 configuration.
 *
 */


#ifndef ACTEL__FIRMWARE__CORETSE_DRIVER__2_5_100_CONFIGURATION_HEADER
#define ACTEL__FIRMWARE__CORETSE_DRIVER__2_5_100_CONFIGURATION_HEADER

extern int dma_tx_num;
extern int dma_rx_num;

// no phy
// #define TSE_MSGMII_ADDR 18
// #define TSE_PHY 1
// //#define TSE_PHY_INTERFACE 2
// #define TSE_PHY_INTERFACE 1
// #define TSE_RGMII_MDIO_ADDR 20

#define TSE_RX_RING_SIZE dma_rx_num
#define TSE_TX_RING_SIZE dma_tx_num

#endif // ACTEL__FIRMWARE__CORETSE_DRIVER__2_5_100_CONFIGURATION_HEADER

