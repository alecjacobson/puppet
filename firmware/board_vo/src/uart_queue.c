
#include "uart_functions.h"
#include "uart_queue.h"
#include "stm32f10x_it.h"
#include "node_functions.h"
#include "i2c_functions.h"
#include "stdio.h"
#include "string.h"
#include "platform_config.h"

#include "configuration.h"

UartQueue uart_input_queue;
UartQueue uart_output_queue;

extern uint16_t *CALIB_VALUES;
extern uint8_t ID;
extern uint8_t TYPE;
extern uint8_t in_debug;
extern uint8_t initialized;
extern uint8_t not_interrupted;

void uart_queue_initialize(UartQueue * q) {
    int i;
    q->read_index = 0;
    q->write_index = 0;
    for (i=0;i<UART_QUEUE_LENGTH;i++) {
        q->line[i].length = 0;
        q->line[i].text[0] = 0;
        q->line[i].text[UART_QUEUE_LINE_LENGTH-1] = 0;
    }
}

int uart_queue_is_empty(UartQueue *q) {
    return (q->read_index == q->write_index);
}

unsigned int uart_queue_length(UartQueue * q) {
    if (q->write_index < q->read_index) {
        return q->write_index + UART_QUEUE_LENGTH - q->read_index;
    } else {
        return q->write_index - q->read_index;
    }
}

void uart_queue_pop_line(UartQueue *q, Line *l) {
	__disable_irq();
    if (uart_queue_is_empty(q)) {
        l->length=0;
        l->text[0]=0;
    } else {
        memcpy(l,&(q->line[q->read_index]),sizeof(Line));
        q->line[q->read_index].length = 0;
        q->read_index = (q->read_index + 1) % UART_QUEUE_LENGTH;
    }
	__enable_irq();
}

void uart_queue_push_line(UartQueue *q, Line * line) {
	__disable_irq();
    memcpy(&(q->line[q->write_index]),line,sizeof(Line));
    q->write_index = (q->write_index + 1) % UART_QUEUE_LENGTH;
    if (q->read_index == q->write_index) {
        q->read_index = (q->read_index + 1) % UART_QUEUE_LENGTH;
    }
	__enable_irq();
}

void uart_queue_push_charp(UartQueue *q, char * line) {
	__disable_irq();
    q->line[q->write_index].length=strlen(line);
    strncpy(q->line[q->write_index].text,line,(UART_QUEUE_LINE_LENGTH-1));
    q->write_index = (q->write_index + 1) % UART_QUEUE_LENGTH;
    if (q->read_index == q->write_index) {
        q->read_index = (q->read_index + 1) % UART_QUEUE_LENGTH;
    }
	__enable_irq();
}

static Line current_output;

void process_uart_output(int blocking) {
    //Line l;
    if (blocking) {
    	if (!current_output.length)
    	{
        	uart_output_interrupt();
    	}
        while (1) {
        	if((uart_queue_is_empty(&uart_output_queue)))
        		return;
        }
    } else {
        if (current_output.length) {
            // we're already sending
            return;
        }
    }
    uart_output_interrupt();
}


void uart_output_interrupt() {
    uart_queue_pop_line(&uart_output_queue,&current_output);
    if (!current_output.length) {
        // Nothing to send
        return;
    }
    // Assuming such a function exists, it will send the buffer, and then 
    // call uart_output_interrupt 
    send_uart_dma(current_output.text, current_output.length);
}


void process_uart_input() {
    Line lin,lout;
    uart_queue_pop_line(&uart_input_queue,&lin);
    if (!lin.length) {
        return;
    }

    static uint8_t TxBuffer[32];

	int n, module_id = 0, joint_type;
	int rate_r, rate_g, rate_b;
	int splitter_output, roll, pitch, yaw;

    switch (lin.text[0]) {
        case 'b':
            not_interrupted = !not_interrupted;

            if(!not_interrupted)
            {
            	in_debug = 0;
            }
            break;
        case 'l':
            n = sscanf(lin.text, "l, %d, %d, %d, %d", &module_id, &rate_r, &rate_g, &rate_b);

            if(module_id!=ID)
            {
                char TxBuffer[8];
                TxBuffer[0] = SET_LED; //led
                TxBuffer[1] = rate_r >> 8;
                TxBuffer[2] = rate_r;
                TxBuffer[3] = rate_g >> 8;
                TxBuffer[4] = rate_g;
                TxBuffer[5] = rate_b >> 8;
                TxBuffer[6] = rate_b;

                // warning, we should mask interrupts here 
                send_i2c(I2C1, TxBuffer, module_id, 7);
            }
            else
            {
                set_leds(rate_r, rate_g, rate_b);
            }

            break;
        case 't':
        	// t for type
            n = sscanf(lin.text,"t,%02x",&joint_type);
            set_type(joint_type);

            lout.length = snprintf(lout.text,UART_QUEUE_LINE_LENGTH, "new type set: 0x%02x\r\n", joint_type);
            uart_queue_push_line(&uart_output_queue,&lout);
            break;
        case 'g':
        	// g for geometry
        	// g,x,roll,pitch,yaw - (in deg) defines the transformation from input to output x in a splitter
        	n = sscanf(lin.text, "g, %d, %d, %d, %d", &splitter_output, &roll, &pitch, &yaw);


        case 's':
        	// s for status
            not_interrupted = 0;

            lout.length = snprintf(lout.text,UART_QUEUE_LINE_LENGTH, "my id is: 0x%02x\r\n", ID);
            uart_queue_push_line(&uart_output_queue,&lout);

            if(TYPE==NODE_COLLECTOR)
            {
                lout.length = snprintf(lout.text,UART_QUEUE_LINE_LENGTH, "and COLLECTOR (0x%02x)\r\n", TYPE);
                uart_queue_push_line(&uart_output_queue,&lout);
            }
            else
            {
                lout.length = snprintf(lout.text,UART_QUEUE_LINE_LENGTH, "and NOT COLLECTOR (0x%02x)\r\n", TYPE);
                uart_queue_push_line(&uart_output_queue,&lout);
            }

            break;
        case 'h':
            not_interrupted = 0;

            lout.length = snprintf(lout.text,UART_QUEUE_LINE_LENGTH, "b\t\tbreak\r\n");
            uart_queue_push_line(&uart_output_queue,&lout);
            lout.length = snprintf(lout.text,UART_QUEUE_LINE_LENGTH, "l,<id>,<r[ms]>,<g[ms]>,<b[ms]>\tleds (id=0 for gc)\r\n");
            uart_queue_push_line(&uart_output_queue,&lout);
            lout.length = snprintf(lout.text,UART_QUEUE_LINE_LENGTH, "i,<id>\t\tset new id\r\n");
            uart_queue_push_line(&uart_output_queue,&lout);
            lout.length = snprintf(lout.text,UART_QUEUE_LINE_LENGTH, "s\t\tget summary\r\n");
            uart_queue_push_line(&uart_output_queue,&lout);
            lout.length = snprintf(lout.text,UART_QUEUE_LINE_LENGTH, "h\t\thelp menu\r\n");
            uart_queue_push_line(&uart_output_queue,&lout);
            break;
		case 'd':
			in_debug = !in_debug;
			if(in_debug)
			{
				lout.length = snprintf(lout.text,UART_QUEUE_LINE_LENGTH, "#IN DEBUG\r\n");
				uart_queue_push_line(&uart_output_queue,&lout);
			}
			else
			{
				lout.length = snprintf(lout.text,UART_QUEUE_LINE_LENGTH, "#NOT IN DEBUG\r\n");
				uart_queue_push_line(&uart_output_queue,&lout);
			}
			break;
        case 'i':
            initialized = 0;
            break;
		case 'o':
			if(is_joint(TYPE))
			{
				calibrate_angles();

				lout.length = snprintf(lout.text,UART_QUEUE_LINE_LENGTH, "#calibration done\r\n");
				uart_queue_push_line(&uart_output_queue,&lout);

				lout.length = snprintf(lout.text,UART_QUEUE_LINE_LENGTH, "#values: %d, %d, %d\r\n", CALIB_VALUES[0], CALIB_VALUES[1], CALIB_VALUES[2]);
				uart_queue_push_line(&uart_output_queue,&lout);
			}
			else if(TYPE==NODE_COLLECTOR)
			{
				n = sscanf(lin.text,"o,%d",&module_id);

				TxBuffer[0] = CALIB;

				if(module_id)
				{
					send_i2c(I2C1, TxBuffer, module_id, 1);
				}
				else
				{
					send_i2c(I2C1, TxBuffer, 0, 1);
				}

				wait_ms(100);

				lout.length = snprintf(lout.text,UART_QUEUE_LINE_LENGTH, "#calibration command sent\r\n");
	            uart_queue_push_line(&uart_output_queue,&lout);
			}

			break;
		case 'k':
			restart_slaves();
            lout.length = snprintf(lout.text,UART_QUEUE_LINE_LENGTH, "slaves restarted\r\n");
            uart_queue_push_line(&uart_output_queue,&lout);
			break;
        default:
            lout.length = snprintf(lout.text,UART_QUEUE_LINE_LENGTH, "unknown command: '%c'\r\n", lin.text[0]);
            uart_queue_push_line(&uart_output_queue,&lout);
            break;
    }
}

