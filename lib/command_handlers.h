#ifndef _H_COMMAND_HANDLERS_
#define _H_COMMAND_HANDLERS_

#include "command_list.h"

void com_echo_handler(char *args);

/*
 * Array of handlers for terminal manager
 * index in array corresponds to the number of command
 */
void (* const commands_handlers[UPPER_BOUND_COMMANDS])(char *) = {
        [COM_ECHO] = com_echo_handler,
};

#endif
