#ifndef COLLECTOR_H
#define COLLECTOR_H



typedef enum {
    SELF_INITIALIZING,
    WAITING_A_BIT,
    WAITING_NODE_INIT,
    WAITING_A_BIT_MORE,
    IN_OPERATION,
    WAITING_UART_EMPTY
} CollectorState;


extern CollectorState collector_state;

void run_collector_state_machine() ;

#endif // COLLECTOR_H
