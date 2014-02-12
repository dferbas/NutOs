#ifndef PHY_GENERIC_H_
#define PHY_GENERIC_H_

/*
 * Phy Register List
 */
#define PHY_REG_BMCR      0x0000  /* Basic Mode Control Register */
#define PHY_REG_BMSR      0x0001  /* Basic Mode Status Register */
#define PHY_REG_IDR1      0x0002  /* PHY Identifier Register #1 */
#define PHY_REG_IDR2      0x0003  /* PHY Identifier Register #2 */
#define PHY_REG_ANAR      0x0004  /* Auto-Negotiation Advertisement Register */
#define PHY_REG_ANLPAR    0x0005  /* Auto-Negotiation Link Partner Ability Register */
#define PHY_REG_ANER      0x0006  /* Auto-Negotiation Expansion Register */

/*
 * Register Bit Definitions
 */

/* PHY_BMCR */
#define PHY_REG_BMCR_FORCE_SPEED_1000    0x0040
#define PHY_REG_BMCR_COLLISION_TEST      0x0080
#define PHY_REG_BMCR_FORCE_FULL_DUP      0x0100
#define PHY_REG_BMCR_RESTART_AUTONEG     0x0200
#define PHY_REG_BMCR_ISOLATE             0x0400
#define PHY_REG_BMCR_POWER_DOWN          0x0800
#define PHY_REG_BMCR_AUTO_NEG_ENABLE     0x1000
#define PHY_REG_BMCR_FORCE_SPEED_100     0x2000
#define PHY_REG_BMCR_FORCE_SPEED_10      0x0000
#define PHY_REG_BMCR_FORCE_SPEED_MASK    0x2040
#define PHY_REG_BMCR_LOOPBACK            0x4000
#define PHY_REG_BMCR_RESET               0x8000

/* PHY_BMSR */
#define PHY_REG_BMSR_EXTENDED_CAPABLE    0x0001
#define PHY_REG_BMSR_JABBER_DETECT       0x0002
#define PHY_REG_BMSR_LINK_STATUS         0x0004
#define PHY_REG_BMSR_AUTO_NEG_ABILITY    0x0008
#define PHY_REG_BMSR_REMOTE_FAULT        0x0010
#define PHY_REG_BMSR_AUTO_NEG_COMPLETE   0x0020
#define PHY_REG_BMSR_PREAMBLE_SUPPRESS   0x0040
#define PHY_REG_BMSR_RESERVED            0x0080
#define PHY_REG_BMSR_1000T_EXT_STATUS    0x0100
#define PHY_REG_BMSR_100T2_HALF_DUP      0x0200
#define PHY_REG_BMSR_100T2_FULL_DUP      0x0400
#define PHY_REG_BMSR_10T_HALF_DUP        0x0800
#define PHY_REG_BMSR_10T_FULL_DUP        0x1000
#define PHY_REG_BMSR_100X_HALF_DUP       0x2000
#define PHY_REG_BMSR_100X_FULL_DUP       0x4000
#define PHY_REG_BMSR_100T4_CAPABLE       0x8000

/* PHY_IDR */
#define PHY_ID(id1_val, id2_val) (((id1_val) << 6) | ((id2_val) >> 10))
#define PHY_REG_IDR1_OUI_MSB_MASK        0xFFFF
#define PHY_REG_IDR2_OUI_LSB_MASK        0xFC00
#define PHY_REG_IDR2_MODEL_MASK          0x03F0
#define PHY_REG_IDR2_REVISION_MASK       0x000F

/* PHY_ANAR */
#define PHY_REG_ANAR_PROTO_SEL_MASK      0x001F
#define PHY_REG_ANAR_PROTO_8023          0x0001
#define PHY_REG_ANAR_10T_HALF_DUP        0x0020
#define PHY_REG_ANAR_10T_FULL_DUP        0x0040
#define PHY_REG_ANAR_100T_HALF_DUP       0x0080
#define PHY_REG_ANAR_100T_FULL_DUP       0x0100
#define PHY_REG_ANAR_100T4_SUPPORT       0x0200
#define PHY_REG_ANAR_PAUSE_SUPPORT       0x0400
#define PHY_REG_ANAR_ASY_PAUSE_SUPPORT   0x0800
#define PHY_REG_ANAR_RESERVED0           0x1000
#define PHY_REG_ANAR_REMOTE_FAULT        0x2000
#define PHY_REG_ANAR_RESERVED1           0x4000
#define PHY_REG_ANAR_NEXT_PAGE_IND       0x8000

/* PHY_ANLPAR */
#define PHY_REG_ANLPAR_PROTO_SEL_MASK    0x001F
#define PHY_REG_ANLPAR_10T_HALF_DUP      0x0020
#define PHY_REG_ANLPAR_10T_FULL_DUP      0x0040
#define PHY_REG_ANLPAR_100T_HALF_DUP     0x0080
#define PHY_REG_ANLPAR_100T_FULL_DUP     0x0100
#define PHY_REG_ANLPAR_100T4_SUPPORT     0x0200
#define PHY_REG_ANLPAR_PAUSE_SUPPORT     0x0400
#define PHY_REG_ANLPAR_ASY_PAUSE         0x0800
#define PHY_REG_ANLPAR_RESERVED0         0x1000
#define PHY_REG_ANLPAR_REMOTE_FAULT      0x2000
#define PHY_REG_ANLPAR_ACK               0x4000
#define PHY_REG_ANLPAR_NEXT_PAGE_IND     0x8000

/* PHY_ANER */
#define PHY_REG_ANER_AUTO_NEG_CAPABLE    0x0001
#define PHY_REG_ANER_PAGE_RX             0x0002
#define PHY_REG_ANER_NEXT_PAGE_ABLE      0x0004
#define PHY_REG_ANER_PRT_NEXT_PAGE_ABLE  0x0008
#define PHY_REG_ANER_PARALLEL_DET_FAULT  0x0010

#endif /* PHY_GENERIC_H_ */
