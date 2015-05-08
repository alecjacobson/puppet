
#include "uart_functions.h"
#include "uart_queue.h"
#include "stm32f10x_it.h"
#include "node_functions.h"
#include "i2c_functions.h"
#include "stdio.h"
#include "platform_config.h"
#include "collector.h"

#include "configuration.h"


CollectorState collector_state = SELF_INITIALIZING;
static unsigned int collector_target = 0;
static unsigned int new_counter = 0;
static unsigned int iteration_counter = 0;

static uint8_t old_topo_counter = 0;

extern  char TxBuffer[32];

extern uint8_t not_interrupted;
extern uint8_t in_debug;

extern uint8_t ID;
extern NodeType TYPE;
extern uint8_t *UID;
extern uint16_t *CALIB_VALUES;

extern uint16_t topology[];
extern uint16_t topology_error[];
extern uint8_t topo_counter;
extern uint8_t next_splitter_completed;
extern uint8_t next_splitter_ID;

extern uint8_t initialized;

extern I2cError i2c_error_code;

// i2c variables
/* Buffer of data to be received by I2C1 */
extern uint8_t Buffer_Rx2[255];
/* Buffer of data to be transmitted by I2C1 */
extern uint8_t Buffer_Tx2[255];

void run_collector_state_machine() {
    Line lout;
    uint8_t TxBuffer[32];
    // Then implement actions for a given state
    switch (collector_state) {
        case SELF_INITIALIZING:
			// reset some variables
			next_splitter_completed = 0;
			next_splitter_ID = 0;
			topo_counter=0;

			// set ID to 0x01 for collector
			ID = 0x01;

            // inform user about start init phase
            lout.length = snprintf(lout.text, UART_QUEUE_LINE_LENGTH, "A\r\n");
            uart_queue_push_line(&uart_output_queue,&lout);

            // wait, maybe unnecessary
            wait(10000);

            char TxBuffer[8];
            TxBuffer[0] = SET_LED; //led
            TxBuffer[1] = 1 >> 8;
            TxBuffer[2] = 1;
            TxBuffer[3] = 0 >> 8;
            TxBuffer[4] = 0;
            TxBuffer[5] = 0 >> 8;
            TxBuffer[6] = 0;

            // warning, we should mask interrupts here
            send_i2c(I2C1, TxBuffer, 0, 7);

            // broadcast collector i2c id, init initialization phase
			TxBuffer[0] = I2C_ID_BROADCAST;
			TxBuffer[1] = ID;
			TxBuffer[2] = TYPE;

			if(!send_i2c(I2C1, TxBuffer, 0, 3))
			{
				//TODO: do something;
				//clear_busy_i2c();
				restart_slaves();

				send_i2c(I2C1, TxBuffer, 0, 3);
			}

			topology[topo_counter++] = ID;

            // send collector id to serial
            lout.length = snprintf(lout.text,UART_QUEUE_LINE_LENGTH, "I,%02x,%02x,%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\r\n", ID, TYPE, UID[0], UID[1], UID[2], UID[3], UID[4], UID[5], UID[6], UID[7], UID[8], UID[9], UID[10], UID[11]);
            uart_queue_push_line(&uart_output_queue,&lout);

            // wait, maybe unnecessary
            iteration_counter = 50;
            collector_state = WAITING_A_BIT;
            break;

        case WAITING_A_BIT:
            if (iteration_counter) {
                wait(10000);
                iteration_counter --;
            } else {
                // send pulse (token) to first module
                send_topo(TOPO_TX_1);
                iteration_counter = 50;
                old_topo_counter = topo_counter;
                collector_state = WAITING_NODE_INIT;
            }
            break;

        case WAITING_NODE_INIT:
            if (next_splitter_completed)
            {
				// initialization is done
				initialized = 1;

				// broadcast 'init done'
				TxBuffer[0] = I2C_COMPLETE;
				send_i2c(I2C1, TxBuffer, 0, 1);

				// send serial 'init done' - do not send
	            lout.length = snprintf(lout.text,UART_QUEUE_LINE_LENGTH, "Z\r\n");
	            uart_queue_push_line(&uart_output_queue,&lout);

                iteration_counter = 20;
                collector_state = WAITING_A_BIT_MORE;
                break;
            }

            if (iteration_counter)
            {
                wait(10000);
                iteration_counter --;
            }
            else if((old_topo_counter == topo_counter) && (next_splitter_ID == 0))
            {
            	next_splitter_completed = 1;
            }
            else
            {
                old_topo_counter = topo_counter;
                iteration_counter = 50;
            }
            break;
        
        case WAITING_A_BIT_MORE:
            if (iteration_counter) {
                wait(10000);
                iteration_counter --;
            } else {
                collector_target = topo_counter+1;
                collector_state = IN_OPERATION;
            }
            break;

        case IN_OPERATION:
            if (!initialized) {
                collector_state = SELF_INITIALIZING;
                break;
            }
            else
            {
                // send i2c to new_node_id, to get acknowledgement of new nodes
                if(new_counter>100)
    			{
    				TxBuffer[0] = NEW_NODE;

    				if(send_i2c(I2C1, TxBuffer, 88, 1))
    				{
    					initialized = 0;
    				}
    				else
    				{
    					if(in_debug)
    					{
                            lout.length = snprintf(lout.text,UART_QUEUE_LINE_LENGTH, "#E,nobody new\r\n");
                            uart_queue_push_line(&uart_output_queue,&lout);
    					}
    				}

    				new_counter = 0;
    			}

                new_counter++;
            }

            if (!not_interrupted)
            {
                // nothing to do...
                break;
            }
            if (collector_target >= topo_counter)
            {
	            lout.length = snprintf(lout.text,UART_QUEUE_LINE_LENGTH, "Z\r\n");
	            uart_queue_push_line(&uart_output_queue,&lout);

	            lout.length = snprintf(lout.text,UART_QUEUE_LINE_LENGTH, "A\r\n");
	            uart_queue_push_line(&uart_output_queue,&lout);

	            collector_target = 1;
                collector_state = WAITING_UART_EMPTY;
                break;
            } 
            if (uart_queue_length(&uart_output_queue) >= UART_QUEUE_LENGTH)
            {
                // The queue is full, wait until it empties
            	if(in_debug)
            	{
                    lout.length = snprintf(lout.text,UART_QUEUE_LINE_LENGTH, "#E,queue will be emptied\r\n");
                    uart_queue_push_line(&uart_output_queue,&lout);
            	}
                collector_state = WAITING_UART_EMPTY;
                break;
            }
                        
            // check if really joint module or something else (has to be solved different
            if (not_interrupted && is_joint(topology[collector_target] >> 8))
            {
                if(receive_i2c(I2C1, Buffer_Rx2, topology[collector_target], 6))
                {
                    unsigned int angle_to_send[3];
                    // make angle package
                    angle_to_send[0] = (Buffer_Rx2[0] << 8 | Buffer_Rx2[1]);
                    angle_to_send[1] = (Buffer_Rx2[2] << 8 | Buffer_Rx2[3]);
                    angle_to_send[2] = (Buffer_Rx2[4] << 8 | Buffer_Rx2[5]);//*0.00549316406;

                    // spit out via UART to computer, sprintf takes 0.1 ms
                    lout.length = snprintf(lout.text,UART_QUEUE_LINE_LENGTH, 
                            "m,%02x,%04x,%04x,%04x\r\n",
                            topology[collector_target], 
                            angle_to_send[0], angle_to_send[1], angle_to_send[2]);
                    uart_queue_push_line(&uart_output_queue,&lout);
                    topology_error[topology[collector_target] & 0xFF] = 0;
                }
                else
                {
                    // error message
                	if(in_debug)
                	{
                        lout.length = snprintf(lout.text,UART_QUEUE_LINE_LENGTH, "#E,%02x,I2C,%02x \r\n", topology[collector_target] & 0xFF, i2c_error_code);
                        uart_queue_push_line(&uart_output_queue,&lout);
                	}

					if(topology_error[topology[collector_target] & 0xFF]++>=5)
					{
						topology_error[topology[collector_target] & 0xFF] = 0;
						initialized = 0;
					}

					if(i2c_error_code == BUSY)
					{
						//TODO: do something;
						clear_busy_i2c();
						//restart_slaves();
					}
                }
            }

            collector_target += 1;
            //TODO: remove to get maximal speed
            //wait_ms(10);
            break;

        case WAITING_UART_EMPTY:
            if (!initialized)
            {
                collector_state = SELF_INITIALIZING;
                break;
            }
            if (uart_queue_is_empty(&uart_output_queue))
            {
                collector_state = IN_OPERATION;
            }
            break;
        default:
            break;
    }
}
