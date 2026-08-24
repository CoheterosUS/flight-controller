#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "Utils/configuration.h"

#define PROTOCOL_MAX_PAYLOAD    48

typedef enum {
    COMMAND_NONE = 0x00,
    COMMAND_RESET = 0x01,
    COMMAND_GROUND_ABORT = 0x02,
    COMMAND_CALIBRATION = 0x03,
    COMMAND_DROGUE = 0x04,
    COMMAND_HIL_DATA = 0x10,
    COMMAND_GPS_DATA = 0x20
} CommandType_t;

typedef enum {
    PARSE_HEADER_1,
    PARSE_HEADER_2,
    PARSE_CMD,
    PARSE_LEN,
    PARSE_PAYLOAD,
    PARSE_FOOTER,
} ParseState_t;

typedef struct {
    ParseState_t State;
    uint8_t Command;
    uint8_t Length;
    uint8_t Payload[PROTOCOL_MAX_PAYLOAD];
    uint8_t PayloadIndex;
} ProtocolParser_t;

void ProtocolInitParser(ProtocolParser_t *Parser);
bool ProtocolFeed(ProtocolParser_t *Parser, uint8_t Byte, uint8_t *Command);

#endif //PROTOCOL_H
