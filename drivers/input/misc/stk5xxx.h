
/* STK5XXX_REG_CHIPID */
#define STK5013_ID							0x2164

#define STK_SAR_THD_0						0x00000024
#define STK_SAR_THD_1						0x00000024
#define STK_SAR_THD_2						0x00000024
#define STK_SAR_THD_3						0x00000024

#define STK_POLLING_TIME				    100000/*us*/

typedef struct stk5xxx_register_table {
	u16 address;
	u32 value;
} stk5xxx_register_table;

typedef enum {
	STK5XXX_PRX_NEAR_BY,
	STK5XXX_PRX_FAR_AWAY,
	STK5XXX_PRX_NEAR_BY_UNKNOWN
} stk5xxx_prx_nearby_type;

/*****************************************************************************
 * stk5xxx register start
 *****************************************************************************/

#define STK_CHIP_INDEX_ADDR                     0x0038
#define STK_CHIP_INDEX_CHIP_ID_MASK             0xFFFF0000
#define STK_CHIP_INDEX_CHIP_ID_SHIFT            16
#define STK_CHIP_INDEX_VER_MASK                 0x000000FF


#define STK_IRQ_SOURCE_ADDR                     0x0000
#define STK_IRQ_SOURCE_LVR_RST_IRQ__SHIFT       10
#define STK_IRQ_SOURCE_LVR_RST_IRQ__MASK        0x1

/*TRIGGER_REG*/
#define STK_TRIGGER_REG_ADDR                    0x0118
#define STK_TRIGGER_REG_PHEN_SHIFT              0
#define STK_TRIGGER_REG_PHEN_MASK               0xFF
#define STK_TRIGGER_REG_PHEN_DISABLE_ALL        0x00000000
#define STK_TRIGGER_REG_PHEN_ENABLE_PH0123      0xF

#define STK_TRIGGER_REG_COMPEN_SHIFT            8
#define STK_TRIGGER_REG_COMPEN_MASK             0xFF
#define STK_TRIGGER_REG_COMPEN_DISABLE_ALL      0x00000000
#define STK_TRIGGER_REG_COMPEN_ENABLE_PH0123    0xF
#define STK_TRIGGER_REG_INIT_ALL                0x00000101

/*TRIGGER_CMD*/
#define STK_TRIGGER_CMD_REG_ADDR                0x0114
#define STK_TRIGGER_CMD_REG_SHIFT               0
#define STK_TRIGGER_CMD_REG_MASK                0xF
#define STK_TRIGGER_CMD_REG_EXIT_PAUSE_MODE     0x0000000C
#define STK_TRIGGER_CMD_REG_ENTER_PAUSE_MODE    0x0000000D
#define STK_TRIGGER_CMD_REG_PHEN_ENABLE         0xFFFFFFFF
#define STK_TRIGGER_CMD_REG_INIT_ALL            0xFFFFFFFF


#define STK_SOFT_RESET_REG_ADDR                 0x002C
#define STK_SOFT_RESET_REG_SOFT_RESET_CMD       0x000000A5


#define STK_TRIM_LOCK_REG_ADDR                  0x0008
#define STK_TRIM_LOCK_REG_VALUE                 0x000000A5


#define STK_TRIM_REG_ADDR                       0x0024
#define STK_TRIM_REG_VALUE                      0x1B008073


#define STK_DISABLE_TRIM_RX_WEIGHT_REG_ADDR     0x0080
#define STK_DISABLE_TRIM_RX_WEIGHT_REG_VALUE    0x0
/*DEGLITCH*/
#define STK_ADC_DEGLITCH_ADDR                   0x0090
#define STK_ADC_DEGLITCH_VALUE                  0x00000002
/*MOVEMENT*/
#define STK_MOVEMENT_0_REG_ADDR                 0x0094
#define STK_MOVEMENT_0_REG_VALUE                0x005A0000
#define STK_MOVEMENT_2_REG_ADDR                 0x009C
#define STK_MOVEMENT_2_REG_VALUE                0x04040404
#define STK_MOVEMENT_3_REG_ADDR                 0x00A0
#define STK_MOVEMENT_3_REG_VALUE                0x00000404
/*SCAN_FACTOR*/
#define STK_SCAN_FACTOR_ADDR                    0x011C
#define STK_SCAN_FACTOR_VALUE                   0x000007D0
/*DOZE_MODE*/
#define STK_DOZE_MODE_ADDR                      0x0120
#define STK_DOZE_MODE_VALUE                     0x00000030
#define STK_DOZE_MODE_SHIFT                     0
#define STK_DOZE_MODE_MASK                      0xFFFFFFFF
#define STK_DOZE_MODE_TURN_OFF                  0x00000030
#define STK_DOZE_MODE_TURN_ON                   0x00000001
/*SCAN_PERIOD*/
#define STK_SCAN_PERIOD_ADDR                    0x0124
#define STK_SCAN_PERIOD_VALUE                   0x000007D0
/*AFECTRL_PH0*/
#define STK_AFE_CTRL_PH0_REG_ADDR               0x0128
#define STK_AFE_CTRL_PH0_REG_VALUE              0x0020071A
#define STK_AFE_CTRL_PH00_REG_ADDR              0x012C
#define STK_AFE_CTRL_PH00_REG_VALUE             0x00000070
/*AFECTRL_PH1*/
#define STK_AFE_CTRL_PH1_REG_ADDR               0x0130
#define STK_AFE_CTRL_PH1_REG_VALUE              0x00201710
#define STK_AFE_CTRL_PH11_REG_ADDR              0x0134
#define STK_AFE_CTRL_PH11_REG_VALUE             0x00000070
/*AFECTRL_PH2*/
#define STK_AFE_CTRL_PH2_REG_ADDR               0x0138
#define STK_AFE_CTRL_PH2_REG_VALUE              0x00201710
#define STK_AFE_CTRL_PH22_REG_ADDR              0x013C
#define STK_AFE_CTRL_PH22_REG_VALUE             0x00001070
/*AFECTRL_PH3*/
#define STK_AFE_CTRL_PH3_REG_ADDR               0x0140
#define STK_AFE_CTRL_PH3_REG_VALUE              0x00201710
#define STK_AFE_CTRL_PH33_REG_ADDR              0x0144
#define STK_AFE_CTRL_PH33_REG_VALUE             0x00001070
/*AFECTRL_PH4*/
#define STK_AFE_CTRL_PH4_REG_ADDR               0x0148
#define STK_AFE_CTRL_PH4_REG_VALUE              0x00201710
#define STK_AFE_CTRL_PH44_REG_ADDR              0x014C
#define STK_AFE_CTRL_PH44_REG_VALUE             0x00001070
/*AFECTRL_PH5*/
#define STK_AFE_CTRL_PH5_REG_ADDR               0x0150
#define STK_AFE_CTRL_PH5_REG_VALUE              0x00201710
#define STK_AFE_CTRL_PH55_REG_ADDR              0x0154
#define STK_AFE_CTRL_PH55_REG_VALUE             0x00001070
/*ANA_CTRL1*/
#define STK_ANA_CTRL0_REG_ADDR                  0x019C
#define STK_ANA_CTRL0_REG_VALUE                 0x02320213
#define STK_ANA_CTRL1_REG_ADDR                  0x01A0
#define STK_ANA_CTRL1_REG_VALUE                 0x01208011
#define STK_S_AND_H_REG_ADDR                    0x01A4
#define STK_S_AND_H_REG_VALUE                   0x00000124
/*RX_TIMING*/
#define STK_RX_TIMING0_REG_ADDR                 0x01A8
#define STK_RX_TIMING0_REG_VALUE                0x20C60C66
#define STK_RX_TIMING1_REG_ADDR                 0x01AC
#define STK_RX_TIMING1_REG_VALUE                0x000001FF
#define STK_RX_TIMING2_REG_ADDR                 0x01B0
#define STK_RX_TIMING2_REG_VALUE                0x0F020000
#define STK_RX_TIMING3_REG_ADDR                 0x01B4
#define STK_RX_TIMING3_REG_VALUE                0x00000110
#define STK_RX_TIMING4_REG_ADDR                 0x01B8
#define STK_RX_TIMING4_REG_VALUE                0x00001146
/*RX_TIMING  END*/

#define STK_TCON0_REG_ADDR                      0x01C4
#define STK_TCON0_REG_VALUE                     0x00053010

#define STK_TCON1_REG_ADDR                      0x01C8
#define STK_TCON1_REG_VALUE                     0x00000B00

/*FILT_PH0*/
#define STK_REGFILT0PH0_REG_ADDR                0x0300
#define STK_REGFILT0PH0_REG_VALUE               0x00011340
#define STK_REGSTATEDET0PH0_REG_ADDR            0x0308
#define STK_REGSTATEDET0PH0_REG_VALUE           0x0000201C
#define STK_REGSTATEDET2PH0_REG_ADDR            0x0310
#define STK_REGSTATEDET2PH0_REG_VALUE           0x00000030
#define STK_REGSTATEDET3PH0_REG_ADDR            0x0314
#define STK_REGSTATEDET3PH0_REG_VALUE           0x00000000
#define STK_RX_GAIN_REG_ADDR                    0x0320
#define STK_RX_GAIN_REG_VALUE                   0x00010000
#define STK_REGADVDIG1PH0_REG_ADDR              0x0324
#define STK_REGADVDIG1PH0_REG_VALUE             0x00000000

/*
#define STK_CADC_REG_ADDR                       0x0330
#define STK_CADC_REG_VALUE                      0x000004E1
*/
/*F0ILT_PH1*/
#define STK_REGFILT0PH1_REG_ADDR                0x0340
#define STK_REGFILT0PH1_REG_VALUE               0x00011100
#define STK_REGSTATEDET0PH1_REG_ADDR            0x0348
#define STK_REGSTATEDET0PH1_REG_VALUE           0x00000000
#define STK_REGSTATEDET2PH1_REG_ADDR            0x0350
#define STK_REGSTATEDET2PH1_REG_VALUE           0x00000000
#define STK_REGSTATEDET3PH1_REG_ADDR            0x0354
#define STK_REGSTATEDET3PH1_REG_VALUE           0x00000000
#define STK_REGADVDIG0PH1_REG_ADDR              0x0360
#define STK_REGADVDIG0PH1_REG_VALUE             0x00010000
#define STK_REGADVDIG1PH1_REG_ADDR              0x0364
#define STK_REGADVDIG1PH1_REG_VALUE             0x00000000
/*FILT_PH2*/
#define STK_REGFILT0PH2_REG_ADDR                0x0380
#define STK_REGFILT0PH2_REG_VALUE               0x00011100
#define STK_REGSTATEDET0PH2_REG_ADDR            0x0388
#define STK_REGSTATEDET0PH2_REG_VALUE           0x00000000
#define STK_REGSTATEDET2PH2_REG_ADDR            0x0390
#define STK_REGSTATEDET2PH2_REG_VALUE           0x00000000
#define STK_REGSTATEDET3PH2_REG_ADDR            0x0394
#define STK_REGSTATEDET3PH2_REG_VALUE           0x00000000
#define STK_REGADVDIG0PH2_REG_ADDR              0x03A0
#define STK_REGADVDIG0PH2_REG_VALUE             0x00010000
#define STK_REGADVDIG1PH2_REG_ADDR              0x03A4
#define STK_REGADVDIG1PH2_REG_VALUE             0x00000000
/*FILT_PH3*/
#define STK_REGFILT0PH3_REG_ADDR                0x03C0
#define STK_REGFILT0PH3_REG_VALUE               0x00011100
#define STK_REGSTATEDET0PH3_REG_ADDR            0x03C8
#define STK_REGSTATEDET0PH3_REG_VALUE           0x00000000
#define STK_REGSTATEDET2PH3_REG_ADDR            0x03D0
#define STK_REGSTATEDET2PH3_REG_VALUE           0x00000000
#define STK_REGSTATEDET3PH3_REG_ADDR            0x03D4
#define STK_REGSTATEDET3PH3_REG_VALUE           0x00000000
#define STK_REGADVDIG0PH3_REG_ADDR              0x03E0
#define STK_REGADVDIG0PH3_REG_VALUE             0x00010000
#define STK_REGADVDIG1PH3_REG_ADDR              0x03E4
#define STK_REGADVDIG1PH3_REG_VALUE             0x00000000
/*FILT_PH4*/
#define STK_REGFILT0PH4_REG_ADDR                0x0400
#define STK_REGFILT0PH4_REG_VALUE               0x00011100
#define STK_REGSTATEDET0PH4_REG_ADDR            0x0408
#define STK_REGSTATEDET0PH4_REG_VALUE           0x00000000
#define STK_REGSTATEDET2PH4_REG_ADDR            0x0410
#define STK_REGSTATEDET2PH4_REG_VALUE           0x00000000
#define STK_REGSTATEDET3PH4_REG_ADDR            0x0414
#define STK_REGSTATEDET3PH4_REG_VALUE           0x00000000
#define STK_REGADVDIG0PH4_REG_ADDR              0x0420
#define STK_REGADVDIG0PH4_REG_VALUE             0x00010000
#define STK_REGADVDIG1PH4_REG_ADDR              0x0424
#define STK_REGADVDIG1PH4_REG_VALUE             0x00000000
/*FILT_PH5*/
#define STK_REGFILT0PH5_REG_ADDR                0x0440
#define STK_REGFILT0PH5_REG_VALUE               0x00011100
#define STK_REGSTATEDET0PH5_REG_ADDR            0x0448
#define STK_REGSTATEDET0PH5_REG_VALUE           0x00000000
#define STK_REGSTATEDET2PH5_REG_ADDR            0x0450
#define STK_REGSTATEDET2PH5_REG_VALUE           0x00000000
#define STK_REGSTATEDET3PH5_REG_ADDR            0x0454
#define STK_REGSTATEDET3PH5_REG_VALUE           0x00000000
#define STK_REGADVDIG0PH5_REG_ADDR              0x0460
#define STK_REGADVDIG0PH5_REG_VALUE             0x00010000
#define STK_REGADVDIG1PH5_REG_ADDR              0x0464
#define STK_REGADVDIG1PH5_REG_VALUE             0x00000000
/*REF_CORR*/
#define STK_REGREFCORRA_REG_ADDR                0x0480
#define STK_REGREFCORRA_REG_VALUE               0x01000000
#define STK_REGDBGVARSEL_REG_ADDR               0x04D4
#define STK_REGDBGVARSEL_REG_VALUE              0x00000000



#define STK_INHOUSE_CMD0_REG_ADDR               0x0600
#define STK_INHOUSE_CMD0_REG_VALUE              0x0000000A


#define STK_INHOUSE_CMD1_REG_ADDR               0x0604
#define STK_INHOUSE_CMD1_REG_VALUE              0x00000000


#define STK_INHOUSE_CMD2_REG_ADDR               0x0608
#define STK_INHOUSE_CMD2_REG_VALUE              0x00000001

/*RXIO0*/
#define STK_RXIO0_MUX_REG_ADDR                  0x0058
#define STK_RXIO0_MUX_REG_AFE_PH0_SHIFT         0
#define STK_RXIO0_MUX_REG_AFE_PH0_MASK          0x3

#define STK_RXIO0_MUX_REG_AFE_PH1_SHIFT         4
#define STK_RXIO0_MUX_REG_AFE_PH1_MASK          0x7

#define STK_RXIO0_MUX_REG_AFE_PH2_SHIFT         8
#define STK_RXIO0_MUX_REG_AFE_PH2_MASK          0x7

#define STK_RXIO0_MUX_REG_AFE_PH3_SHIFT         12
#define STK_RXIO0_MUX_REG_AFE_PH3_MASK          0x7

#define STK_RXIO0_MUX_REG_AFE_PH4_SHIFT         16
#define STK_RXIO0_MUX_REG_AFE_PH4_MASK          0x7

#define STK_RXIO0_MUX_REG_AFE_PH5_SHIFT         20
#define STK_RXIO0_MUX_REG_AFE_PH5_MASK          0x7

#define STK_RXIO0_MUX_REG_VALUE                 0x00777777


#define STK_RXIO1_MUX_REG_ADDR                  0x005C
#define STK_RXIO1_MUX_REG_AFE_PH0_SHIFT         0
#define STK_RXIO1_MUX_REG_AFE_PH0_MASK          0x3

#define STK_RXIO1_MUX_REG_AFE_PH1_SHIFT         4
#define STK_RXIO1_MUX_REG_AFE_PH1_MASK          0x7

#define STK_RXIO1_MUX_REG_AFE_PH2_SHIFT         8
#define STK_RXIO1_MUX_REG_AFE_PH2_MASK          0x7

#define STK_RXIO1_MUX_REG_AFE_PH3_SHIFT         12
#define STK_RXIO1_MUX_REG_AFE_PH3_MASK          0x7

#define STK_RXIO1_MUX_REG_AFE_PH4_SHIFT         16
#define STK_RXIO1_MUX_REG_AFE_PH4_MASK          0x7

#define STK_RXIO1_MUX_REG_AFE_PH5_SHIFT         20
#define STK_RXIO1_MUX_REG_AFE_PH5_MASK          0x7

#define STK_RXIO1_MUX_REG_VALUE                 0x00777775

#define STK_RXIO2_MUX_REG_ADDR                  0x0060
#define STK_RXIO2_MUX_REG_AFE_PH0_SHIFT         0
#define STK_RXIO2_MUX_REG_AFE_PH0_MASK          0x3

#define STK_RXIO2_MUX_REG_AFE_PH1_SHIFT         4
#define STK_RXIO2_MUX_REG_AFE_PH1_MASK          0x7

#define STK_RXIO2_MUX_REG_AFE_PH2_SHIFT         8
#define STK_RXIO2_MUX_REG_AFE_PH2_MASK          0x7

#define STK_RXIO2_MUX_REG_AFE_PH3_SHIFT         12
#define STK_RXIO2_MUX_REG_AFE_PH3_MASK          0x7

#define STK_RXIO2_MUX_REG_AFE_PH4_SHIFT         16
#define STK_RXIO2_MUX_REG_AFE_PH4_MASK          0x7

#define STK_RXIO2_MUX_REG_AFE_PH5_SHIFT         20
#define STK_RXIO2_MUX_REG_AFE_PH5_MASK          0x7

#define STK_RXIO2_MUX_REG_VALUE                 0x00777777

#define STK_RXIO3_MUX_REG_ADDR                  0x0064
#define STK_RXIO3_MUX_REG_AFE_PH0_SHIFT         0
#define STK_RXIO3_MUX_REG_AFE_PH0_MASK          0x3

#define STK_RXIO3_MUX_REG_AFE_PH1_SHIFT         4
#define STK_RXIO3_MUX_REG_AFE_PH1_MASK          0x7

#define STK_RXIO3_MUX_REG_AFE_PH2_SHIFT         8
#define STK_RXIO3_MUX_REG_AFE_PH2_MASK          0x7

#define STK_RXIO3_MUX_REG_AFE_PH3_SHIFT         12
#define STK_RXIO3_MUX_REG_AFE_PH3_MASK          0x7

#define STK_RXIO3_MUX_REG_AFE_PH4_SHIFT         16
#define STK_RXIO3_MUX_REG_AFE_PH4_MASK          0x7

#define STK_RXIO3_MUX_REG_AFE_PH5_SHIFT         20
#define STK_RXIO3_MUX_REG_AFE_PH5_MASK          0x7

#define STK_RXIO3_MUX_REG_VALUE                 0x00777777


#define STK_RXIO4_MUX_REG_ADDR                  0x0068
#define STK_RXIO4_MUX_REG_AFE_PH0_SHIFT         0
#define STK_RXIO4_MUX_REG_AFE_PH0_MASK          0x3

#define STK_RXIO4_MUX_REG_AFE_PH1_SHIFT         4
#define STK_RXIO4_MUX_REG_AFE_PH1_MASK          0x7

#define STK_RXIO4_MUX_REG_AFE_PH2_SHIFT         8
#define STK_RXIO4_MUX_REG_AFE_PH2_MASK          0x7

#define STK_RXIO4_MUX_REG_AFE_PH3_SHIFT         12
#define STK_RXIO4_MUX_REG_AFE_PH3_MASK          0x7

#define STK_RXIO4_MUX_REG_AFE_PH4_SHIFT         16
#define STK_RXIO4_MUX_REG_AFE_PH4_MASK          0x7

#define STK_RXIO4_MUX_REG_AFE_PH5_SHIFT         20
#define STK_RXIO4_MUX_REG_AFE_PH5_MASK          0x7

#define STK_RXIO4_MUX_REG_VALUE                 0x00777777

#define STK_RXIO5_MUX_REG_ADDR                  0x006C
#define STK_RXIO5_MUX_REG_AFE_PH0_SHIFT         0
#define STK_RXIO5_MUX_REG_AFE_PH0_MASK          0x3

#define STK_RXIO5_MUX_REG_AFE_PH1_SHIFT         4
#define STK_RXIO5_MUX_REG_AFE_PH1_MASK          0x7

#define STK_RXIO5_MUX_REG_AFE_PH2_SHIFT         8
#define STK_RXIO5_MUX_REG_AFE_PH2_MASK          0x7

#define STK_RXIO5_MUX_REG_AFE_PH3_SHIFT         12
#define STK_RXIO5_MUX_REG_AFE_PH3_MASK          0x7

#define STK_RXIO5_MUX_REG_AFE_PH4_SHIFT         16
#define STK_RXIO5_MUX_REG_AFE_PH4_MASK          0x7

#define STK_RXIO5_MUX_REG_AFE_PH5_SHIFT         20
#define STK_RXIO5_MUX_REG_AFE_PH5_MASK          0x7

#define STK_RXIO5_MUX_REG_VALUE                 0x00777777





#define STK_IRQ_SOURCE_ENABLE_REG_ADDR          0x0004
#define STK_IRQ_SOURCE_ENABLE_REG_VALUE          0x00001003
#define STK_IRQ_SOURCE_ENABLE_REG_CLOSE_ANY_IRQ_EN_SHIFT    0
#define STK_IRQ_SOURCE_ENABLE_REG_CLOSE_ANY_IRQ_EN_MASK     0x1
#define STK_IRQ_SOURCE_ENABLE_REG_CLOSE_ANY_IRQ_EN_ENABLE   1

#define STK_IRQ_SOURCE_ENABLE_REG_FAR_ANY_IRQ_EN_SHIFT      1
#define STK_IRQ_SOURCE_ENABLE_REG_FAR_ANY_IRQ_EN_MASK       0x2
#define STK_IRQ_SOURCE_ENABLE_REG_FAR_ANY_IRQ_EN_ENABLE     1

#define STK_IRQ_SOURCE_ENABLE_REG_CONVDONE_IRQ_EN_SHIFT     3
#define STK_IRQ_SOURCE_ENABLE_REG_CONVDONE_IRQ_EN_MASK      0x4
#define STK_IRQ_SOURCE_ENABLE_REG_CONVDONE_IRQ_EN_ENABLE    1

#define STK_IRQ_SOURCE_ENABLE_REG_PROG2_IRQ_EN_SHIFT        4
#define STK_IRQ_SOURCE_ENABLE_REG_PROG2_IRQ_EN_MASK         0x8
#define STK_IRQ_SOURCE_ENABLE_REG_PROG2_IRQ_EN_ENABLE       1

#define STK_IRQ_SOURCE_ENABLE_REG_PAUSE_IRQ_EN_SHIFT        7
#define STK_IRQ_SOURCE_ENABLE_REG_PAUSE_IRQ_EN_MASK         0x00000080
#define STK_IRQ_SOURCE_ENABLE_REG_PAUSE_IRQ_EN_ENABLE       1

#define STK_REG_RAW_PH0_REG_ADDR                          0x0500
#define STK_REG_RAW_PH1_REG_ADDR                          0x0504
#define STK_REG_RAW_PH2_REG_ADDR                          0x0508
#define STK_REG_RAW_PH3_REG_ADDR                          0x050C
#define STK_REG_RAW_PH4_REG_ADDR                          0x0510
#define STK_REG_RAW_PH5_REG_ADDR                          0x0514

#define STK_REG_BASE_PH0_REG_ADDR                          0x0518
#define STK_REG_BASE_PH1_REG_ADDR                          0x051C
#define STK_REG_BASE_PH2_REG_ADDR                          0x0520
#define STK_REG_BASE_PH3_REG_ADDR                          0x0524
#define STK_REG_BASE_PH4_REG_ADDR                          0x0528
#define STK_REG_BASE_PH5_REG_ADDR                          0x052C

#define STK_REG_IRQ_STATE0_REG_ADDR                         0x016C
#define STK_REG_IRQ_STATE0_REG_PROX_STATE_MASK              0x00000001
#define STK_REG_IRQ_STATE0_REG_TABLE_STATE_MASK             0x0000FF00
#define STK_REG_IRQ_STATE0_REG_BODY_STATE_MASK              0x00FF0000
#define STK_REG_IRQ_STATE0_REG_STEADY_STATE_MASK            0xFF000000


#define STK_REG_DELTA_PH0_REG_ADDR                          0x0530

#define STK_REG_DELTA_PH1_REG_ADDR                          0x0534

#define STK_REG_DELTA_PH2_REG_ADDR                          0x0538

#define STK_REG_DELTA_PH3_REG_ADDR                          0x053C

#define STK_REG_DELTA_PH4_REG_ADDR                          0x0540

#define STK_REG_DELTA_PH5_REG_ADDR                          0x0544


/*****************************************************************************
 * stk5xxx register end
 *****************************************************************************/
