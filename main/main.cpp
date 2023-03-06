

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include <cmath>


#define UART_ID   uart0
#define BAUD_RATE 115200
#define DATA_BITS 8  // c.f. pico-examples, bc idk why
#define STOP_BITS 1  // c.f. pico-examples, bc idk why
#define PARITY    UART_PARITY_NONE // c.f. pico-examples, bc idk why

// feedbacks
#define ACKNOWLEDGE 0
#define TERMINATE   1
#define VARIABLE    2
#define TRACKING    3


// orders
#define LIDAR      0
#define MOVE       1
#define ROTATE     2
#define CANCELMOVE 3
#define MOTORVALUE 4
#define MOTORTIME  5
#define PUMPS      6
#define MOTORS     7
#define MOTORSARG  8
#define SETVAR     9
#define GETVAR     10
#define TRACK      11


bool trackingPID(false);
bool lidarStopped(false);


// each componenent of the message
typedef struct OrderArgs {
    unsigned char id;
    unsigned char complement;
    unsigned short arg0;
    unsigned short arg1;
    float floatArg;
    unsigned int tmp;
} OrderArgs; 
OrderArgs ARGS;

typedef struct Buffer {
    int messageLength;
    unsigned char* receivedMessage;
    int head;
} Buffer;

Buffer BUFFER;

void makeArgs() {
    ARGS.id = BUFFER.receivedMessage[0] >> 4;
    ARGS.complement = BUFFER.receivedMessage[0] & 0xf; 
    ARGS.arg0 = (((unsigned short) BUFFER.receivedMessage[1]) << 8) + ((unsigned short) BUFFER.receivedMessage[2]);
    ARGS.arg1 = (((unsigned short) BUFFER.receivedMessage[3]) << 8) + ((unsigned short) BUFFER.receivedMessage[4]); 
    ARGS.tmp = (((unsigned int) ARGS.arg0) << 16) + (unsigned int) ARGS.arg1;
    ARGS.floatArg = *(float*) &ARGS.tmp;
}

void bufferInit(int length) {
    BUFFER.messageLength = length;
    BUFFER.receivedMessage = new unsigned char[length];
    BUFFER.head = 0;
}

bool bufferPush() {
    BUFFER.receivedMessage[BUFFER.head] = uart_getc(UART_ID);
    BUFFER.head++;
    return BUFFER.head == BUFFER.messageLength;
}

void sendBuffer() {
    for (int i=0; i < BUFFER.messageLength; i++) uart_putc_raw(UART_ID, BUFFER.receivedMessage[i]);
}

void sendFeedback(unsigned char feedbackId, unsigned char orderId) {
    uart_putc_raw(UART_ID, (char) ((feedbackId << 4) | orderId));
    for (int i = 1; i < BUFFER.messageLength; i++) uart_putc_raw(UART_ID, 0);
}

void sendVar(unsigned char varID, float value) {
    uart_putc_raw(UART_ID, (VARIABLE << 4) | varID);
    // "evil" trick to get floating point representation as int
    unsigned int tmp = *(unsigned int*) &value;
    for (int i = 0; i < 4; i++) uart_putc_raw(UART_ID, (char) (tmp >> (8 * (3 - i)) & 0xff));
}

void sendTrackedValue(float trackedValue) {
    uart_putc_raw(UART_ID, TRACKING << 4);
    // "evil" trick to get floating point representation as int
    unsigned int tmp = *(unsigned int*) &trackedValue;
    for (int i = 0; i < 4; i++) uart_putc_raw(UART_ID, (char) (tmp >> (8 * (3 - i)) & 0xff));
}

typedef void (*VoidFunc) (void);

typedef struct ProcessState {
    // loop bodies
    VoidFunc bodies[16];

    bool lidarStopped;
    // movement (MOVE or ROTATE) target in ticks
    short tickTarget;
    

    char motorID;
    // motor Value
    short value;
    // motor Time
    unsigned short time;

    unsigned short pumps;
    
    // Motors
    // number of arguments remaining
    unsigned char motorArgsCountDown;
    char* motorsID;

    float vars[16];

    bool tracking;
} ProcessState;
ProcessState STATE;


void doNothing () {}

void processInit() {
    for (int i = 0; i < 16; i++) STATE.bodies[i] = &doNothing;
    STATE.motorArgsCountDown = 0;
    STATE.lidarStopped = false;
    STATE.tracking = false;
    for (int i = 0; i < 16; i++) STATE.vars[i] = 1.f;
    STATE.pumps = 0;
}

// -------------------------------------------

void lidar() {
    STATE.lidarStopped = ARGS.complement;
    sendFeedback(TERMINATE, LIDAR);
}


// -------------------------------------------

void move_body() {
    if (lidarStopped) return;
    sendFeedback(TERMINATE, MOVE);
    STATE.bodies[MOVE] = &doNothing;
}

void move() {
    STATE.bodies[MOVE] = &move_body;
    STATE.tickTarget = ARGS.arg1;
}

// -------------------------------------------

void rotate_body() {
    if (lidarStopped) return;
    sendFeedback(TERMINATE, ROTATE);
    STATE.bodies[ROTATE] = &doNothing;
}

void rotate() {
    STATE.bodies[ROTATE] = &rotate_body;
    STATE.tickTarget = ARGS.arg1;
}

// -------------------------------------------

void cancelMove() {
    STATE.bodies[MOVE] = &doNothing;
    STATE.bodies[ROTATE] = &doNothing;
    sendFeedback(TERMINATE, CANCELMOVE);
}

// -------------------------------------------

void motorValue_body() {
    sendFeedback(TERMINATE, MOTORVALUE);
    STATE.bodies[MOTORVALUE] = &doNothing;
}

void motorValue() {
    STATE.bodies[MOTORVALUE] = &motorValue_body;
    if (STATE.tracking) {
        for (int i=0; i < 500; i++) {
            sendTrackedValue(1. - expf(- (float)i * 0.005));
        }
    }
    STATE.motorID = ARGS.complement;
    STATE.value = ARGS.arg1;
}

// -------------------------------------------

void motorTime_body() {
    sendFeedback(TERMINATE, MOTORTIME);
    STATE.bodies[MOTORTIME] = &doNothing;
}

void motorTime() {    
    STATE.bodies[MOTORTIME] = &motorTime_body;
    STATE.motorID = ARGS.complement;
    STATE.time = ARGS.arg1;

}

// -------------------------------------------

void pumps() {
    STATE.pumps = ARGS.arg1;
    sendFeedback(TERMINATE, PUMPS);
}

// -------------------------------------------


void motors() {
    STATE.motorArgsCountDown = ARGS.complement;
    STATE.motorsID = new char[ARGS.complement];
}

void motors_body() {
    sendFeedback(TERMINATE, MOTORS);
    STATE.bodies[MOTORS] = &doNothing;
}

// -------------------------------------------

void motorsArg() {
    STATE.motorsID[STATE.motorArgsCountDown - 1] = ARGS.complement;
    STATE.motorArgsCountDown--;
    if (!STATE.motorArgsCountDown) {
        STATE.bodies[MOTORS] = &motors_body;
    }
}

// -------------------------------------------

void setVar() {
    STATE.vars[ARGS.complement] = ARGS.floatArg;
    sendFeedback(TERMINATE, SETVAR);
}

// -------------------------------------------

void getVar() {
    sendVar(ARGS.complement, STATE.vars[ARGS.complement]);
    sendFeedback(TERMINATE, GETVAR);
}

// -------------------------------------------

void track() {
    STATE.tracking = ARGS.complement;
    sendFeedback(TERMINATE, TRACK);
}

// -------------------------------------------

VoidFunc commands[] = {
    &lidar, 
    &move,
    &rotate,
    &cancelMove,
    &motorValue,
    &motorTime,
    &pumps,
    &motors,
    &motorsArg,
    &setVar,
    &getVar,
    &track,
    NULL,
    NULL,
    NULL,
    NULL
};


int first(3);
void on_uart_rx() {
    // weird bytes appear at the beggining
    if (first) {uart_getc(UART_ID); first--; return;}
    if (bufferPush()) {
        makeArgs();
        BUFFER.head = 0;
        sendFeedback(ACKNOWLEDGE, ARGS.id);
        commands[ARGS.id]();
    }
}



int main() {    // Set up our UART with a basic baud rate.
    uart_init(UART_ID, BAUD_RATE);
    bufferInit(5);
    processInit();

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ID, false, false);

    // Set our data format
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, true);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);
    BUFFER.head = 0;
    while (1) {
        for (int i = 0; i < 16; i++) STATE.bodies[i]();
    } 
}
