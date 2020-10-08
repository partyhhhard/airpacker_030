#ifndef PCCONN_H
#define PCCONN_H

extern void pcconnTaskFunc( void const *argument );


#define TX_BUFFER_SIZE        30
#define RX_BUFFER_SIZE        30
#define SEND_STATE_PERIOD       5000

void debug_( char *msg, float value );

#endif