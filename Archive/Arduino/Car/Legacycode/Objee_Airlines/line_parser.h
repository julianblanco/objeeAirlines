#ifndef LINE_PARSER
    #define LINE_PARSER

    #include <stdint.h>

    uint8_t line_parser(char *line, char delim);
    uint8_t get_field(char *field, uint8_t N);

#endif
