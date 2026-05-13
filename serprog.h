#pragma once

/* serprog return codes */
#define S_ACK 0x06
#define S_NAK 0x15

/* serprog command codes */
#define S_CMD_NOP 0x00
#define S_CMD_Q_IFACE 0x01
#define S_CMD_Q_CMDMAP 0x02
#define S_CMD_Q_PGMNAME 0x03
#define S_CMD_Q_SERBUF 0x04
#define S_CMD_Q_BUSTYPE 0x05
#define S_CMD_Q_CHIPSIZE 0x06
#define S_CMD_Q_OPBUF 0x07
#define S_CMD_Q_WRNMAXLEN 0x08
#define S_CMD_R_BYTE 0x09
#define S_CMD_R_NBYTES 0x0A
#define S_CMD_O_INIT 0x0B
#define S_CMD_O_WRITEB 0x0C
#define S_CMD_O_WRITEN 0x0D
#define S_CMD_O_DELAY 0x0E
#define S_CMD_O_EXEC 0x0F
#define S_CMD_SYNCNOP 0x10
#define S_CMD_Q_RDNMAXLEN 0x11
#define S_CMD_S_BUSTYPE 0x12
#define S_CMD_O_SPIOP 0x13
#define S_CMD_S_SPI_FREQ 0x14
#define S_CMD_S_PIN_STATE 0x15
#define S_CMD_S_SPI_CS 0x16
#define S_CMD_S_SPI_MODE 0x17
#define S_CMD_S_CS_MODE 0x18

/* flashrom bus bit definitions */
#define S_BUS_PARALLEL (1u << 0)
#define S_BUS_LPC      (1u << 1)
#define S_BUS_FWH      (1u << 2)
#define S_BUS_SPI      (1u << 3)
