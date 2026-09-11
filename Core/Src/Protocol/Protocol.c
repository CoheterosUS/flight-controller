#include "Protocol/Protocol.h"

void ProtocolInitParser(ProtocolParser_t *Parser) {
    Parser->State = PARSE_HEADER_1;
    Parser->Command = 0;
    Parser->Length = 0;
    Parser->PayloadIndex = 0;
}

bool ProtocolFeed(ProtocolParser_t *Parser, uint8_t Byte, uint8_t *Command, uint8_t *Length) {
    switch (Parser->State) {
        case PARSE_HEADER_1:
            if (Byte == PACKET_HEADER_LSB) {
                Parser->State = PARSE_HEADER_2;
            }
            break;

        case PARSE_HEADER_2:
            if (Byte == PACKET_HEADER_MSB) {
                Parser->State = PARSE_CMD;
            } else {
                Parser->State = PARSE_HEADER_1;
            }
            break;

        case PARSE_CMD:
            Parser->Command = Byte;
            Parser->State = PARSE_LEN;
            break;

        case PARSE_LEN:
            Parser->Length = Byte;
            Parser->PayloadIndex = 0;
            if (Parser->Length > PROTOCOL_MAX_PAYLOAD) {
                ProtocolInitParser(Parser);
            } else if (Parser->Length == 0) {
                Parser->State = PARSE_FOOTER;
            } else {
                Parser->State = PARSE_PAYLOAD;
            }
            break;

        case PARSE_PAYLOAD:
            Parser->Payload[Parser->PayloadIndex++] = Byte;
            if (Parser->PayloadIndex >= Parser->Length) {
                Parser->State = PARSE_FOOTER;
            }
            break;

        case PARSE_FOOTER:
            if (Byte == PACKET_FOOTER) {
                *Command = Parser->Command;
                *Length = Parser->Length;
                ProtocolInitParser(Parser);
                return true;
            }
            ProtocolInitParser(Parser);
            break;
    }

    return false;
}
