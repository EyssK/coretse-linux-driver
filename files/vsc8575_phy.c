/*******************************************************************************
 * (c) Copyright 2014-2018 Microsemi SoC Products Group.  All rights reserved.
 *
 * Marvell's M88E1111 PHY interface driver source code.
 *
 * SVN $Revision: 9728 $
 * SVN $Date: 2018-01-29 18:03:40 +0530 (Mon, 29 Jan 2018) $
 */
#include "phy.h"

#include "core_tse.h"
#include "coretse_regs.h"
#include "coretse_user_config.h"
#include "hal.h"
#include "hal_assert.h"

//#include "../../CMSIS/system_m2sxxx.h"

#ifdef __cplusplus
extern "C" {
#endif
/**************************************************************************/
/* Preprocessor Macros                                                    */
/**************************************************************************/
#define TSE_RGMII                               0x04

#define SF2_MSGMII_PHY_ADDR                     TSE_MSGMII_ADDR
#define SF2_MSGMII_PHY_RESET_MASK               0x8000u
#define SF2_MSGMII_TBI_CONTROL                  0x11

#define BMSR_AUTO_NEGOTIATION_COMPLETE          0x0020u

#define ANEG_REQUESTED                          0x80000000u
#define FORCED_CFG_REQUESTED                    0x40000000u

/*------------------------------------------------------------------------------
 * M88E1111 PHY specific register offsets and bit definitions
 */
#define VCS8575_EXT_ADDR_PAGE_CR                0x1f
#define VCS8575_EXT_PAGE_0                      0x00
#define VCS8575_EXT_PAGE_1                      0x01
#define VCS8575_EXT_PAGE_2                      0x02
#define VCS8575_EXT_PAGE_3                      0x03
#define VCS8575_EXT_PAGE_4                      0x04
#define VCS8575_EXT_PAGE_GP                     0x10


#define VSC8575_AUX_PHY_STATUS             0x1cu
#define VSC8575_PHY_STATUS_1000            0x0010u
#define VSC8575_PHY_STATUS_100             0x0008u
#define VSC8575_PHY_STATUS_10              0x0000u
#define VSC8575_PHY_STATUS_SPD_MASK        0x0018u
#define VSC8575_PHY_STATUS_FULLDUPLEX      0x0020u


//#define M88E1111_PHY_EXT_SR                     0x1b
//
//#define M88E1111_PHY_STATUS                     0x11
//#define M88E1111_PHY_STATUS_1000                0x8000
//#define M88E1111_PHY_STATUS_100                 0x4000
//#define M88E1111_PHY_STATUS_10                  0x0000
//#define M88E1111_PHY_STATUS_SPD_MASK            0xc000
//#define M88E1111_PHY_STATUS_FULLDUPLEX          0x2000
//#define M88E1111_PHY_STATUS_RESOLVED            0x0800
//#define M88E1111_PHY_STATUS_LINK                0x0400

/**************************************************************************//**
 * See m88e1111_phy.h for details of how to use this function.
 */
void
TSE_phy_init
(
    tse_instance_t * this_tse,
    uint8_t phy_addr
)
{
    volatile uint16_t phy_reg;
    volatile uint16_t temp;

//#if (TSE_INTERFACE == TSE_RGMII)
//    /*
//    * Enable phy-local clock shift
//    */
//    phy_reg = TSE_read_phy_reg(this_tse, phy_addr, MII_EXTEND);
//    phy_reg |= 0x0002;
//    TSE_write_phy_reg(this_tse, phy_addr, MII_EXTEND, phy_reg);
//#endif /*(TSE_INTERFACE == TSE_RGMII)*/

    /* Extended Control and Status, SMI broadcast write enabled */
    TSE_write_phy_reg(this_tse, phy_addr, VCS8575_EXT_ADDR_PAGE_CR, VCS8575_EXT_PAGE_0);

    phy_reg = TSE_read_phy_reg(this_tse, phy_addr, 0x16);
    phy_reg |= 0x01;
    TSE_write_phy_reg(this_tse, phy_addr, 0x16, phy_reg);


    /* 16E3 bit 7 setting to 1 for SERDES MAC AN EN */
    TSE_write_phy_reg(this_tse, phy_addr, VCS8575_EXT_ADDR_PAGE_CR, VCS8575_EXT_PAGE_3);

    phy_reg = TSE_read_phy_reg(this_tse, phy_addr, 0x10);
    phy_reg |= 0x80;
    TSE_write_phy_reg(this_tse, phy_addr, 0x10, phy_reg);

    TSE_write_phy_reg(this_tse, phy_addr, VCS8575_EXT_ADDR_PAGE_CR, VCS8575_EXT_PAGE_GP);

    phy_reg = 0x80F0;
    TSE_write_phy_reg(this_tse, phy_addr, 0x12, phy_reg);

    while(1)
    {
        temp = TSE_read_phy_reg(this_tse, phy_addr, 0x12);

        if((temp & 0x8000) == 0)
            {
              break;
            }
        }
    TSE_write_phy_reg(this_tse, phy_addr, VCS8575_EXT_ADDR_PAGE_CR, VCS8575_EXT_PAGE_0);

    phy_reg = TSE_read_phy_reg(this_tse, phy_addr, MII_BMCR);
       phy_reg |= BMCR_RESET;
       TSE_write_phy_reg(this_tse, phy_addr, MII_BMCR, phy_reg);
    while(1)
      {
    	temp = TSE_read_phy_reg(this_tse, phy_addr, MII_BMCR);
        if((temp & 0x8000) == 0)
        {
          break;
        }
      }

    /* Reset the PHY. */
//    phy_reg = TSE_read_phy_reg(this_tse, phy_addr, MII_BMCR);
//    phy_reg |= BMCR_RESET;
//    TSE_write_phy_reg(this_tse, phy_addr, MII_BMCR, phy_reg);

    /* Power up the PHY. */
//    phy_reg = TSE_read_phy_reg(this_tse, phy_addr, MII_BMCR);
//    phy_reg &= ~BMCR_PDOWN;
//    TSE_write_phy_reg(this_tse, phy_addr, MII_BMCR, phy_reg);
}

/**************************************************************************//**
 *
 */
//advertiser
void
TSE_phy_set_link_speed
(
    tse_instance_t * this_tse,
    uint8_t phy_addr,
    uint32_t speed_duplex_select
)
{
    uint16_t phy_reg;
    uint32_t inc;
    uint32_t speed_select;
    uint16_t const mii_advertise_bits[4] = {ADVERTISE_10FULL, ADVERTISE_10HALF,
                                            ADVERTISE_100FULL, ADVERTISE_100HALF};

    /* Set 10Mbps and 100Mbps advertisement. */
    phy_reg = TSE_read_phy_reg(this_tse, phy_addr, MII_ADVERTISE);
    phy_reg &= ~(ADVERTISE_10HALF | ADVERTISE_10FULL |
                 ADVERTISE_100HALF | ADVERTISE_100FULL);

    speed_select = speed_duplex_select;
    for(inc = 0u; inc < 4u; ++inc)
    {
        uint32_t advertise;
        advertise = speed_select & 0x00000001u;
        if(advertise != 0u)
        {
            phy_reg |= mii_advertise_bits[inc];
        }
        speed_select = speed_select >> 1u;
    }
    //printf("phy_reg=0x%x\n",phy_reg);
    TSE_write_phy_reg(this_tse, phy_addr, MII_ADVERTISE, phy_reg);

    /* Set 1000Mbps advertisement. */
    phy_reg = TSE_read_phy_reg(this_tse, phy_addr, MII_CTRL1000);
    phy_reg &= ~(ADVERTISE_1000FULL | ADVERTISE_1000HALF);

    if((speed_duplex_select & TSE_ANEG_1000M_FD) != 0u)
    {
        phy_reg |= ADVERTISE_1000FULL;
    }

    if((speed_duplex_select & TSE_ANEG_1000M_HD) != 0u)
    {
        phy_reg |= ADVERTISE_1000HALF;
    }

    TSE_write_phy_reg(this_tse, phy_addr, MII_CTRL1000, phy_reg);
}

/**************************************************************************//**
 *
 */
void
TSE_phy_autonegotiate
(
    tse_instance_t * this_tse,
    uint8_t phy_addr
)
{
    volatile uint16_t phy_reg;
    uint16_t autoneg_complete;
    uint32_t volatile copper_aneg_timeout = 1000000u;

    /* Enable auto-negotiation. */
    TSE_write_phy_reg(this_tse, phy_addr, VCS8575_EXT_ADDR_PAGE_CR, VCS8575_EXT_PAGE_0);

    phy_reg = TSE_read_phy_reg(this_tse, phy_addr, MII_BMCR);
    phy_reg |= (BMCR_ANENABLE | BMCR_ANRESTART);
    TSE_write_phy_reg(this_tse, phy_addr, MII_BMCR, phy_reg);

    /* Wait for copper auto-negotiation to complete. */
    do
    {
        phy_reg = TSE_read_phy_reg(this_tse, phy_addr, MII_BMSR);
        autoneg_complete = phy_reg & BMSR_AUTO_NEGOTIATION_COMPLETE;
        --copper_aneg_timeout;
    } while((!autoneg_complete  && copper_aneg_timeout != 0u) || (0xFFFF == phy_reg));

    for (volatile uint32_t i = 0; i < 100000; i++);

//    {
//        volatile uint32_t delay = SystemCoreClock / 256u;
//        while(delay != 0)
//        {
//            --delay;
//        }
//    }
}

/***************************************************************************//**
 *
 */
uint8_t
TSE_phy_get_link_status
(
    tse_instance_t * this_tse,
    uint8_t phy_addr,
    tse_speed_t * speed,
    uint8_t * fullduplex
)
{
    uint16_t phy_reg;
    uint16_t link_up;
    uint8_t link_status;

    /* Find out if link is up between Marvell PHY and remote device.*/
    phy_reg = TSE_read_phy_reg(this_tse, phy_addr, MII_BMSR);
    link_up = phy_reg & BMSR_LSTATUS;

    if(link_up != TSE_LINK_DOWN)
    {
        uint16_t duplex;
        uint16_t phy_speed;

        /* Link is up. */
        link_status = TSE_LINK_UP;

        phy_reg = TSE_read_phy_reg(this_tse, phy_addr, VSC8575_AUX_PHY_STATUS);
        duplex = phy_reg & VSC8575_PHY_STATUS_FULLDUPLEX;
        phy_speed = phy_reg & VSC8575_PHY_STATUS_SPD_MASK;

        if(TSE_HALF_DUPLEX == duplex)
        {
            *fullduplex = TSE_HALF_DUPLEX;
        }
        else
        {
            *fullduplex = TSE_FULL_DUPLEX;
        }

        switch(phy_speed)
        {
            case VSC8575_PHY_STATUS_1000:
                *speed = TSE_MAC1000MBPS;
                break;

            case VSC8575_PHY_STATUS_100:
                *speed = TSE_MAC100MBPS;
                break;

            case VSC8575_PHY_STATUS_10:
                *speed = TSE_MAC10MBPS;
                break;
            default:
                *speed = TSE_INVALID_SPEED;
                break;
        }
    }
    else
    {
        /* Link is down. */
        link_status = TSE_LINK_DOWN;
    }

    return link_status;
}

#ifdef __cplusplus
}
#endif

/******************************** END OF FILE ******************************/
