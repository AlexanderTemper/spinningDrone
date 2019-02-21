#include "debug.h"

#ifdef DEBUG_LOG_ENABLE
char debugBuffer[MAX_STRING_LENGTH] = { 0 };

uint32_t _debug_modul_level = MODUL_ALL;

void debugUart(uint32_t module, const char *format, ...) {

    if ((module & _debug_modul_level) > 0) {
        va_list arg_list;
        va_start(arg_list, format);
        vsnprintf(debugBuffer, MAX_STRING_LENGTH, format, arg_list);
        va_end(arg_list);
        usart_write_buffer_wait(&usart_instance, (uint8_t *) "ABD ", 4);
        usart_write_buffer_wait(&usart_instance, (uint8_t *) debugBuffer, sizeof(debugBuffer));
        usart_write_buffer_wait(&usart_instance, (uint8_t *) "AB", 2);
        memset(debugBuffer,0,sizeof(debugBuffer));
    }

}
#endif
