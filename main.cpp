
#include <stdio.h>
#include "pico/stdlib.h"
#include <cmath>

const int messageLength(5);
unsigned char receivedMessage[messageLength] = {0};
bool trackingPID(false);
bool lidarStopped(false);

int reception;


// each componenent of the message
unsigned char id;
unsigned char complement;
unsigned short arg0;
unsigned short arg1;
float floatArg;
unsigned int tmp;


void sendFeedback(unsigned char feedbackId, unsigned char orderId) {
    putchar_raw((char) ((feedbackId << 4) | orderId));
    for (int i = 1; i < messageLength; i++) putchar_raw(0);
}

void sendFeedback(float trackedValue) {
    putchar_raw(0x30);
    tmp = *(unsigned int*) &trackedValue;
    for (int i = 0; i < 4; i++) putchar_raw((char) (tmp >> (8 * (3 - i)) & 0xff));
}

void sendFeedback(unsigned char varID, float value) {
    putchar_raw(0x20 | varID);
    tmp = *(unsigned int*) &value;
    for (int i = 0; i < 4; i++) putchar_raw((char) (tmp >> (8 * (3 - i)) & 0xff));
}

float PIDVars[16] = {
    1.0f,
    3.1415f,
    2.72f,
    1.688f,
    0.1f,
    0.f,
    -15.f,
    0.45f,
    0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f
};

unsigned int timers[16] = {0};
int motorsCountDown;
unsigned short chosenMotors; 
unsigned short pumpStates;

// order fucntions
void doNothing() {}

void lidar() {
    lidarStopped = complement;
}

void move() {
    timers[id] = 160;
}

void rotate() {
    timers[id] = 110;
}

void cancelMove() {
    timers[id - 1] = 0;
    timers[id - 2] = 0;
    timers[id]++;
}
void motorValue() {
    if (trackingPID) {
        for (int i = 0; i < 600; i++) {
            sendFeedback(1.f - expf(-.005f * (float)i));
        }
    }
    timers[id] = 150;
}

void motorTime() {
    timers[id] = 100;
}

void pumps() {
    pumpStates = arg1;
    timers[id]++;
}

void motors() {
    motorsCountDown = complement;
    chosenMotors = arg1;
}

void motorsArg() {
    if (motorsCountDown && ((1 << complement) & chosenMotors)) {
        motorsCountDown--;
        if (!motorsCountDown) {
            timers[id - 1] = 190;
        }
    }
}

void setVar() {
    PIDVars[complement] = floatArg;
    timers[id]++;
}


void getVar() {
    sendFeedback(complement, PIDVars[complement]);
    timers[id]++;
}

void track() {
    trackingPID = complement;
    timers[id]++;
}

void (*commands[16])() = {
    *lidar,
    *move,
    *rotate,
    *cancelMove,
    *motorValue,
    *motorTime,
    *pumps,
    *motors,
    *motorsArg,
    *setVar,
    *getVar,
    *track,
    *doNothing,
    *doNothing,
    *doNothing,
    *doNothing
};


bool readInput() {
    reception = getchar_timeout_us(0);
    if (reception != PICO_ERROR_TIMEOUT) {
        receivedMessage[0] = (unsigned char) reception;
        for (int i = 1; i < messageLength; i++) {
            receivedMessage[i] = getchar();
        }
        return true;
    }
    return false;
} 

void makeArgs() {
    id = receivedMessage[0] >> 4;
    complement = receivedMessage[0] & 0xf;
    arg0 = (((unsigned short) receivedMessage[1]) << 8) + ((unsigned short) receivedMessage[2]);
    arg1 = (((unsigned short) receivedMessage[3]) << 8) + ((unsigned short) receivedMessage[4]); 
    tmp = (((unsigned int) arg0) << 16) + (unsigned int) arg1;
    floatArg = *(float*) &tmp;
}



int main() {
    stdio_init_all();
    while (1) {
        if (readInput()) {
            makeArgs();
            sendFeedback(0, id);
            commands[id]();
        }
        if (!lidarStopped) for (unsigned char i=0; i < 16; i++) {
            if (timers[i]) {
                timers[i]--;
                if (!timers[i]){
                    sendFeedback(1, i);
                } 
            }
        }    
        sleep_ms(50);
    } 
}
