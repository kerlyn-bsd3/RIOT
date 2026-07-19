/*
 * Copyright (C) 2026 WPI MQP (6LoBAC) — Kerry Lynn
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * CLEAN-ROOM MS/TP Receive Frame state machine, transcribed from
 * ANSI/ASHRAE 135-2024 Clause 9.5.4 (states/transitions), 9.5.8 (CheckHeader)
 * and 9.10.3 (COBS decode). Every transition below corresponds 1:1 to a named
 * transition in the standard; the clause and transition name are cited.
 *
 * NOT derived from bacnet-stack (GPL) or the contiki-claude tree. COBS and
 * CRC-32K come from draft-lynn-6lo-rfc8163-bis-01 (IETF Trust / Simplified BSD).
 */

#include <string.h>

#include "mstp_fsm.h"
#include "mstp_crc.h"
#include "mstp_cobs.h"

/* Non-encoded frames: max Data field length (135-2024 9.3). */
#define MSTP_MAX_NONENCODED_LEN (501U)

/* ------------------------------------------------------------------------- */

void mstp_rx_fsm_init(mstp_rx_fsm_t *fsm, uint8_t this_station)
{
    memset(fsm, 0, sizeof(*fsm));
    fsm->state = MSTP_RX_IDLE;
    fsm->this_station = this_station;
}

void mstp_rx_fsm_note_overrun(mstp_rx_fsm_t *fsm)
{
    fsm->stats.receive_error++;
}

/** True if this frame's Frame Type denotes a COBS-encoded frame (9.3). */
static bool is_cobs_type(uint8_t frame_type)
{
    return (frame_type >= MSTP_NMIN_COBS_TYPE) &&
           (frame_type <= MSTP_NMAX_COBS_TYPE);
}

/** True if the frame is addressed to us or broadcast (9.5.4.4). */
static bool for_us(const mstp_rx_fsm_t *fsm)
{
    uint8_t dst = fsm->header[1];
    return (dst == fsm->this_station) || (dst == 255U);
}

/** CheckHeader (9.5.8) outcome, split by cause for diagnostics. */
typedef enum {
    HDR_OK = 0,       /**< GoodHeader                                        */
    HDR_BAD_CRC,      /**< HeaderCRC residue mismatch                        */
    HDR_SRC_255,      /**< Source Address == 255 (illegal, 9.3)              */
    HDR_BAD_LENGTH,   /**< DataLength illegal for the frame type             */
} hdr_check_t;

/**
 * The CheckHeader procedure — 135-2024 Clause 9.5.8.
 * Sets GoodHeader FALSE on a bad header CRC residue, a broadcast source, or a
 * DataLength that is illegal for the frame's type. Returns the *cause* so the
 * caller can account each distinctly (a CRC bit flip vs. a shifted octet).
 */
static hdr_check_t check_header(const mstp_rx_fsm_t *fsm)
{
    uint8_t frame_type = fsm->header[0];
    uint8_t source = fsm->header[2];
    uint16_t len = fsm->data_length;

    if (fsm->header_crc != MSTP_HEADER_CRC_RESIDUE) {
        return HDR_BAD_CRC;
    }
    if (source == 255U) {
        return HDR_SRC_255;   /* a Source Address of 255 is not allowed (9.3) */
    }
    if (!is_cobs_type(frame_type) && (len > MSTP_MAX_NONENCODED_LEN)) {
        return HDR_BAD_LENGTH;
    }
    if (is_cobs_type(frame_type) &&
        ((len < MSTP_NMIN_COBS_LENGTH) || (len > MSTP_NMAX_COBS_LENGTH))) {
        return HDR_BAD_LENGTH;
    }
    return HDR_OK;
}

/** Hand the assembled frame up. */
static mstp_rx_result_t deliver(mstp_rx_fsm_t *fsm, uint16_t msdu_len)
{
    fsm->frame.frame_type  = fsm->header[0];
    fsm->frame.destination = fsm->header[1];
    fsm->frame.source      = fsm->header[2];
    fsm->frame.length      = msdu_len;
    fsm->state = MSTP_RX_IDLE;
    fsm->stats.frames_ok++;
    fsm->received_valid_frame = true;   /* 9.5.4.4 NoData/GoodCRC: ReceivedValidFrame */
    return MSTP_RX_FRAME;
}

static mstp_rx_result_t invalid(mstp_rx_fsm_t *fsm)
{
    fsm->state = MSTP_RX_IDLE;
    fsm->received_invalid_frame = true; /* 9.5.4: BadHeader/BadCRC/Timeout/Error */
    fsm->stats.frames_inv++;            /* diagnostic: every ReceivedInvalidFrame */
    return MSTP_RX_INVALID;
}

static mstp_rx_result_t idle(mstp_rx_fsm_t *fsm)
{
    fsm->state = MSTP_RX_IDLE;
    return MSTP_RX_NONE;
}

/*
 * HEADER_CRC (9.5.4.4). Transient: entered after the header CRC octet and
 * resolved immediately — none of its transitions consume an octet.
 */
static mstp_rx_result_t enter_header_crc(mstp_rx_fsm_t *fsm)
{
    uint16_t len = fsm->data_length;

    hdr_check_t hc = check_header(fsm);
    if (hc != HDR_OK) {                                         /* BadHeader */
        switch (hc) {
            case HDR_SRC_255:    fsm->stats.src_invalid++;    break;
            case HDR_BAD_LENGTH: fsm->stats.bad_length++;     break;
            case HDR_BAD_CRC:
            default:             fsm->stats.header_crc_err++; break;
        }
        return invalid(fsm);
    }
    if (!for_us(fsm)) {
        if (len == 0U) {
            return idle(fsm);                                   /* NotForUs */
        }
        fsm->index = 0;                                     /* DataNotForUs */
        fsm->state = MSTP_RX_SKIP_DATA;
        return MSTP_RX_NONE;
    }
    if (len > MSTP_INPUT_BUFFER_SIZE) {                      /* FrameTooLong */
        fsm->stats.frame_too_long++;
        fsm->index = 0;
        fsm->state = MSTP_RX_SKIP_DATA;
        return MSTP_RX_INVALID;
    }
    if (len == 0U) {
        return deliver(fsm, 0);                                   /* NoData */
    }
    if (is_cobs_type(fsm->header[0])) {                  /* EncodedFields */
        fsm->index = 0;
        fsm->crc32k = CRC32K_INITIAL_VALUE;
        fsm->state = MSTP_RX_RECEIVE_ENCODED_FIELDS;
        return MSTP_RX_NONE;
    }
    fsm->index = 0;                                                 /* Data */
    fsm->data_crc = 0xFFFFU;
    fsm->state = MSTP_RX_DATA;
    return MSTP_RX_NONE;
}

/*
 * VALIDATE_ENCODED_FIELDS (9.5.4.9). Transient. Entered from FinalOctet after
 * the fields have been decoded per 9.10.3.
 */
static mstp_rx_result_t enter_validate_encoded(mstp_rx_fsm_t *fsm)
{
    uint16_t enc_len = (uint16_t)(fsm->data_length - 3U);  /* Encoded Data len */
    uint8_t crc_bytes[4];

    /*
     * 9.10.3 / draft-lynn-6lo-rfc8163-bis-01: the Encoded CRC-32K field is
     * decoded and the resulting four octets are accumulated by calc_crc32K().
     * fsm->crc32k already holds the accumulation over the Encoded Data field.
     */
    if (cobs_decode(crc_bytes, &fsm->input_buffer[enc_len],
                    MSTP_ENCODED_CRC_LEN, MSTP_COBS_MASK) != sizeof(crc_bytes)) {
        fsm->stats.cobs_err++;
        return invalid(fsm);                                      /* BadCRC */
    }
    for (size_t i = 0; i < sizeof(crc_bytes); i++) {
        fsm->crc32k = calc_crc32K(crc_bytes[i], fsm->crc32k);
    }
    if (fsm->crc32k != CRC32K_RESIDUE) {                          /* BadCRC */
        fsm->stats.data_crc_err++;
        return invalid(fsm);
    }

    /* GoodCRC: decode the Encoded Data field into the MSDU. */
    size_t msdu_len = cobs_decode(fsm->frame.data, fsm->input_buffer,
                                  enc_len, MSTP_COBS_MASK);
    if ((msdu_len == 0) || (msdu_len > MSTP_MAX_MSDU)) {
        fsm->stats.cobs_err++;
        return invalid(fsm);
    }
    return deliver(fsm, (uint16_t)msdu_len);                     /* GoodCRC */
}

/* ------------------------------------------------------------------------- */

/*
 * EventCount (9.5.2) counts link activity. Per 9.5.4 it is incremented on every
 * octet or ReceiveError event handled while the FSM is in IDLE, PREAMBLE, or
 * HEADER (9.5.4.1 EatAnError/EatAnOctet/Preamble1; 9.5.4.2 Error/RepeatedPreamble1/
 * NotPreamble/Preamble2; 9.5.4.3 Error/FrameType/…/HeaderCRC) — and NOT during
 * the DATA/SKIP_DATA/RECEIVE_ENCODED_FIELDS phases nor on a silence timeout. It
 * is used by the Manager Node FSM (PASS_TOKEN SawTokenUser, NO_TOKEN) to detect
 * that another node has begun transmitting.
 */
static bool counts_event(mstp_rx_state_t state)
{
    return (state == MSTP_RX_IDLE) ||
           (state == MSTP_RX_PREAMBLE) ||
           (state == MSTP_RX_HEADER);
}

mstp_rx_result_t mstp_rx_fsm_error(mstp_rx_fsm_t *fsm)
{
    /* Every ReceiveError transition (9.5.4) clears SilenceTimer (activity). */
    fsm->silence_timer = 0;
    if (counts_event(fsm->state)) {
        fsm->event_count++;
    }
    fsm->stats.receive_error++;

    switch (fsm->state) {
        /* 9.5.4.3/.5/.7/.8 Error: a frame was in progress. */
        case MSTP_RX_HEADER:
        case MSTP_RX_DATA:
        case MSTP_RX_SKIP_DATA:
        case MSTP_RX_RECEIVE_ENCODED_FIELDS:
            return invalid(fsm);
        /* 9.5.4.1 EatAnError, 9.5.4.2 Error: no frame in progress. */
        default:
            return idle(fsm);
    }
}

mstp_rx_result_t mstp_rx_fsm_silence(mstp_rx_fsm_t *fsm)
{
    fsm->stats.frame_abort++;

    switch (fsm->state) {
        /* 9.5.4.2 Timeout: "a correct preamble has not been received" — no flag. */
        case MSTP_RX_PREAMBLE:
            return idle(fsm);
        /* 9.5.4.3/.5/.7/.8 Timeout: ReceivedInvalidFrame. */
        case MSTP_RX_HEADER:
        case MSTP_RX_DATA:
        case MSTP_RX_SKIP_DATA:
        case MSTP_RX_RECEIVE_ENCODED_FIELDS:
            return invalid(fsm);
        default:
            fsm->stats.frame_abort--;   /* IDLE: nothing to abort */
            return MSTP_RX_NONE;
    }
}

mstp_rx_result_t mstp_rx_fsm_octet(mstp_rx_fsm_t *fsm, uint8_t octet)
{
    /*
     * Every octet-consuming transition in 9.5.4 "set[s] SilenceTimer to zero"
     * (activity detected); IDLE/PREAMBLE/HEADER events also increment EventCount.
     * Both are done once here on entry — the state is read before any transition.
     */
    fsm->silence_timer = 0;
    if (counts_event(fsm->state)) {
        fsm->event_count++;
    }

    switch (fsm->state) {

    case MSTP_RX_IDLE:                                        /* 9.5.4.1 */
        if (octet == MSTP_PREAMBLE_55) {                     /* Preamble1 */
            fsm->state = MSTP_RX_PREAMBLE;
        }
        /* else EatAnOctet: remain in IDLE */
        return MSTP_RX_NONE;

    case MSTP_RX_PREAMBLE:                                    /* 9.5.4.2 */
        if (octet == MSTP_PREAMBLE_55) {             /* RepeatedPreamble1 */
            return MSTP_RX_NONE;
        }
        if (octet == MSTP_PREAMBLE_FF) {                     /* Preamble2 */
            fsm->index = 0;
            fsm->header_crc = 0xFFU;
            fsm->state = MSTP_RX_HEADER;
            return MSTP_RX_NONE;
        }
        return idle(fsm);                                  /* NotPreamble */

    case MSTP_RX_HEADER:                                      /* 9.5.4.3 */
        fsm->header_crc = calc_header_crc(octet, fsm->header_crc);
        switch (fsm->index) {
        case 0:                                              /* FrameType */
        case 1:                                            /* Destination */
        case 2:                                                 /* Source */
            fsm->header[fsm->index] = octet;
            fsm->index++;
            return MSTP_RX_NONE;
        case 3:                                                /* Length1 */
            fsm->header[3] = octet;
            fsm->data_length = (uint16_t)octet * 256U;
            fsm->index = 4;
            return MSTP_RX_NONE;
        case 4:                                                /* Length2 */
            fsm->header[4] = octet;
            fsm->data_length = (uint16_t)(fsm->data_length + octet);
            fsm->index = 5;
            return MSTP_RX_NONE;
        default:                                             /* HeaderCRC */
            return enter_header_crc(fsm);
        }

    case MSTP_RX_DATA:                                        /* 9.5.4.5 */
        if (fsm->index < fsm->data_length) {                 /* DataOctet */
            fsm->input_buffer[fsm->index] = octet;
            fsm->index++;
            /* TODO: accumulate DataCRC (CRC-16) — needs 9.6.2 / Annex G.
             * Unreachable for 6LoBAC, whose frames are all COBS-encoded. */
            return MSTP_RX_NONE;
        }
        if (fsm->index == fsm->data_length) {                     /* CRC1 */
            fsm->index++;
            return MSTP_RX_NONE;
        }
        /* CRC2 -> DATA_CRC (9.5.4.6), transient */
        fsm->stats.data_crc_err++;
        return invalid(fsm);   /* TODO: check DataCRC == X'F0B8' */

    case MSTP_RX_SKIP_DATA:                                   /* 9.5.4.7 */
        if (fsm->index < (uint16_t)(fsm->data_length + 1U)) { /* DataOctet */
            fsm->index++;
            return MSTP_RX_NONE;
        }
        return idle(fsm);                                         /* Done */

    case MSTP_RX_RECEIVE_ENCODED_FIELDS:                      /* 9.5.4.8 */
        if (fsm->index < (uint16_t)(fsm->data_length - 3U)) { /* DataOctet */
            fsm->crc32k = calc_crc32K(octet, fsm->crc32k);
            fsm->input_buffer[fsm->index] = octet;
            fsm->index++;
            return MSTP_RX_NONE;
        }
        if (fsm->index < (uint16_t)(fsm->data_length + 1U)) {  /* CRCOctet */
            fsm->input_buffer[fsm->index] = octet;   /* not accumulated */
            fsm->index++;
            return MSTP_RX_NONE;
        }
        fsm->input_buffer[fsm->index] = octet;               /* FinalOctet */
        return enter_validate_encoded(fsm);

    default:
        return idle(fsm);
    }
}
