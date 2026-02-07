/*
 * protocol.h - Frame protocol definitions and parser state machine
 *
 * Frame layout:
 *   [SOF1=0xAA][SOF2=0x55][LEN][TYPE][SEQ][PAYLOAD...][CRC_H][CRC_L]
 *
 * CRC16-CCITT covers: LEN + TYPE + SEQ + PAYLOAD
 */
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <string.h>

/* ── Constants ────────────────────────────────────────────── */
#define FRAME_SOF1       0xAA
#define FRAME_SOF2       0x55
#define FRAME_MAX_PAYLOAD 128
#define MSG_TYPE_SNAPSHOT 0x01

/* ── CRC16-CCITT (0xFFFF initial, poly 0x1021) ───────────── */
static inline uint16_t crc16_update(uint16_t crc, uint8_t byte)
{
    crc ^= (uint16_t)byte << 8;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x8000)
            crc = (crc << 1) ^ 0x1021;
        else
            crc <<= 1;
    }
    return crc;
}

static inline uint16_t crc16_calc(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++)
        crc = crc16_update(crc, data[i]);
    return crc;
}

/* ── Frame packing (Sensor side) ─────────────────────────── */

/*
 * frame_pack - Build a complete frame into `out_buf`.
 *
 * Returns total frame length (header + payload + CRC).
 * Caller must ensure out_buf is large enough: payload_len + 7.
 */
static inline uint32_t frame_pack(uint8_t *out_buf,
                                  uint8_t type,
                                  uint8_t seq,
                                  const uint8_t *payload,
                                  uint8_t payload_len)
{
    uint32_t idx = 0;

    /* SOF */
    out_buf[idx++] = FRAME_SOF1;
    out_buf[idx++] = FRAME_SOF2;

    /* Header fields covered by CRC */
    out_buf[idx++] = payload_len;  /* LEN */
    out_buf[idx++] = type;         /* TYPE */
    out_buf[idx++] = seq;          /* SEQ */

    /* Payload */
    memcpy(&out_buf[idx], payload, payload_len);
    idx += payload_len;

    /* CRC16 over LEN+TYPE+SEQ+PAYLOAD (bytes [2..idx-1]) */
    uint16_t crc = crc16_calc(&out_buf[2], payload_len + 3);
    out_buf[idx++] = (uint8_t)(crc >> 8);    /* CRC high */
    out_buf[idx++] = (uint8_t)(crc & 0xFF);  /* CRC low  */

    return idx;
}

/* ── Frame parser (Control side) ─────────────────────────── */

typedef enum {
    PS_SOF1,
    PS_SOF2,
    PS_LEN,
    PS_TYPE,
    PS_SEQ,
    PS_PAYLOAD,
    PS_CRC_H,
    PS_CRC_L
} parser_state_t;

typedef struct {
    parser_state_t state;
    uint8_t  len;
    uint8_t  type;
    uint8_t  seq;
    uint8_t  payload[FRAME_MAX_PAYLOAD];
    uint8_t  payload_idx;
    uint8_t  crc_h;

    /* Statistics */
    uint32_t good_frames;
    uint32_t bad_crc;
    uint32_t sync_resets;
} frame_parser_t;

static inline void parser_init(frame_parser_t *p)
{
    memset(p, 0, sizeof(*p));
    p->state = PS_SOF1;
}

/*
 * parser_feed - Feed one byte into the state machine.
 *
 * Returns 1 when a complete, CRC-valid frame has been received.
 * After return==1, read p->type, p->seq, p->payload[], p->len.
 */
static inline int parser_feed(frame_parser_t *p, uint8_t byte)
{
    switch (p->state) {
    case PS_SOF1:
        if (byte == FRAME_SOF1)
            p->state = PS_SOF2;
        return 0;

    case PS_SOF2:
        if (byte == FRAME_SOF2) {
            p->state = PS_LEN;
        } else {
            p->sync_resets++;
            p->state = (byte == FRAME_SOF1) ? PS_SOF2 : PS_SOF1;
        }
        return 0;

    case PS_LEN:
        if (byte > FRAME_MAX_PAYLOAD) {
            p->sync_resets++;
            p->state = PS_SOF1;
            return 0;
        }
        p->len = byte;
        p->state = PS_TYPE;
        return 0;

    case PS_TYPE:
        p->type = byte;
        p->state = PS_SEQ;
        return 0;

    case PS_SEQ:
        p->seq = byte;
        p->payload_idx = 0;
        p->state = (p->len > 0) ? PS_PAYLOAD : PS_CRC_H;
        return 0;

    case PS_PAYLOAD:
        p->payload[p->payload_idx++] = byte;
        if (p->payload_idx >= p->len)
            p->state = PS_CRC_H;
        return 0;

    case PS_CRC_H:
        p->crc_h = byte;
        p->state = PS_CRC_L;
        return 0;

    case PS_CRC_L: {
        uint16_t rx_crc = ((uint16_t)p->crc_h << 8) | byte;

        /* Rebuild CRC check buffer: LEN + TYPE + SEQ + PAYLOAD */
        uint8_t check_buf[3 + FRAME_MAX_PAYLOAD];
        check_buf[0] = p->len;
        check_buf[1] = p->type;
        check_buf[2] = p->seq;
        memcpy(&check_buf[3], p->payload, p->len);

        uint16_t calc_crc = crc16_calc(check_buf, 3 + p->len);

        p->state = PS_SOF1;

        if (calc_crc == rx_crc) {
            p->good_frames++;
            return 1;   /* frame ready! */
        } else {
            p->bad_crc++;
            return 0;
        }
    }

    default:
        p->state = PS_SOF1;
        return 0;
    }
}

#endif /* PROTOCOL_H */
