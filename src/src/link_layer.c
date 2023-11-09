// Link layer protocol implementation
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "../include/link_layer.h"

typedef enum {
    INFO,
    SET,
    DISC,
    UA,
    DUP,
} type_of_frame;

typedef struct{

    type_of_frame type;

    unsigned char number;
    unsigned int size;

} frame_info;

frame_info check_frame(int fd, unsigned char* buffer, unsigned char last_received_frame);

// MISC
#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source

// Common frame fields

#define F 0x7e

#define A 0x03
#define A_Tx 0x03
#define A_Rx 0x01

#define C_0 0x00
#define C_1 0x40

#define C_SET 0x03
#define C_UA 0x07
#define C_RR0 0x05
#define C_RR1 0x85
#define C_REJ0 0x01
#define C_REJ1 0x81
#define C_DISC 0x0b

#define ESC 0x7d

#define BIT(n) (1 << n)

// Message reception state machine
typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    C_SET_RCV,
    C_UA_RCV,
    C_DISC_RCV,
    C_INFO_RCV,
    READING_DATA,
    ESC_OK,
    BCC2_OK,
    ESC_BCC2,
    FLAG_ESC_BCC2,
    BCC_OK,
    STOP
} ReceiveState;

struct termios oldtio;

LinkLayerRole role;
int nRetransmissions;
int timeout;

int fd;

int alarmTriggered = FALSE;

int info_frame_number = 0;

unsigned char set_frame[5] = {F, A_Tx, C_SET, A_Tx ^ C_SET, F};
unsigned char ua_frame[5] = {F, A_Tx, C_UA, A_Tx ^ C_UA, F};
unsigned char disc_frame[5] = {F, A_Tx, C_DISC, A_Tx ^ C_DISC, F};

unsigned char rr_frame_0[5] = {F, A_Tx, 0x05, A_Tx ^ 0x05, F};
unsigned char rr_frame_1[5] = {F, A_Tx, 0x85, A_Tx ^ 0x85, F};

unsigned char rej_frame_0[5] = {F, A_Tx, 0x01, A_Tx ^ 0x01, F};
unsigned char rej_frame_1[5] = {F, A_Tx, 0x81, A_Tx ^ 0x81, F};

unsigned char mask = ~((unsigned char) BIT(6));

void alarmHandler(int signal) {
    alarmTriggered = TRUE;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{

    role = connectionParameters.role;
    nRetransmissions = connectionParameters.nRetransmissions;
    timeout = connectionParameters.timeout;
    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);
    if (fd < 0) return -1;
    if (tcgetattr(fd, &oldtio) == -1) return -1;

    struct termios newtio;
    memset(&newtio, 0, sizeof(newtio));
    newtio.c_cflag = connectionParameters.baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_cc[VTIME] = 0;
    if(connectionParameters.role == LlTx)
    	newtio.c_cc[VMIN] = 0;
    else if(connectionParameters.role == LlRx)
    	newtio.c_cc[VMIN] = 1;

    tcflush(fd, TCIOFLUSH);
    if (tcsetattr(fd, TCSANOW, &newtio) == -1) return -1;

    ReceiveState state = START;
    int tries = nRetransmissions;
    unsigned char byte;
    
    if (connectionParameters.role == LlTx) {
        if (write(fd, set_frame, 5) < 0) return -1;

        (void) signal(SIGALRM, alarmHandler);
        alarm(timeout);

        while (tries > 0) {
            if (read(fd, &byte, 1)) {
                switch (state) {
                    case START:
                        if (byte == F) state = FLAG_RCV;
                        break;
                    case FLAG_RCV:
                        if (byte == A_Tx) state = A_RCV;
                        else if (byte == F) continue;
                        else state = START;
                        break;
                    case A_RCV:
                        if (byte == C_UA) state = C_RCV;
                        else if (byte == F) state = FLAG_RCV;
                        else state = START;
                        break;
                    case C_RCV:
                        if (byte == (A_Tx ^ C_UA)) state = BCC_OK;
                        else if (byte == F) state = FLAG_RCV;
                        else state = START;
                        break;
                    case BCC_OK:
                        if (byte == F) {
                            state = STOP;
                        } else state = START;
                        break;
                    default: break;
                }
            }
            if (state == STOP) break;
            else if (alarmTriggered == TRUE) {
                tries--;
                printf("llopen(): Alarm triggered, %i tries remaining.\n", tries);
                
                alarmTriggered = FALSE;
                (void) signal(SIGALRM, alarmHandler);
                alarm(timeout);
                if (write(fd, set_frame, 5) < 0) return -1;
            }
        }
        if (tries == 0 && state != STOP) return -1;
    } else if (connectionParameters.role == LlRx) {
        while (1) {
            if (read(fd, &byte, 1)) {
                switch (state) {
                    case START:
                        if (byte == F) state = FLAG_RCV;
                        break;
                    case FLAG_RCV:
                        if (byte == A_Tx) state = A_RCV;
                        else if (byte == F) continue;
                        else state = START;
                        break;
                    case A_RCV:
                        if (byte == C_SET) state = C_RCV;
                        else if (byte == F) state = FLAG_RCV;
                        else state = START;
                        break;
                    case C_RCV:
                        if (byte == (A_Tx ^ C_SET)) state = BCC_OK;
                        else if (byte == F) state = FLAG_RCV;
                        else state = START;
                        break;
                    case BCC_OK:
                        if (byte == F) {
                            state = STOP;
                        } else state = START;
                        break;
                    default: break;
                }
            }
            if (state == STOP) break;
        }
        if (write(fd, ua_frame, 5) < 0) return -1;
    }
    tries = connectionParameters.nRetransmissions + 1;

    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    int oldFrameSize = bufSize + 6;
    unsigned char *oldFrame = (unsigned char*) malloc (oldFrameSize);

    oldFrame[0] = F;
    oldFrame[1] = A_Tx;
    oldFrame[2] = info_frame_number == 0 ? C_0 : C_1;
    oldFrame[3] = A_Tx ^ (info_frame_number == 0 ? C_0 : C_1);
    int i;
    unsigned char BCC2 = 0;
    for (i = 4; i < bufSize + 4; i++) {
        oldFrame[i] = buf[i - 4];
        BCC2 ^= buf[i - 4];
    }
    oldFrame[i] = BCC2;
    i++;
    oldFrame[i] = F;

    unsigned char *newFrame = (unsigned char*) malloc (oldFrameSize * 2);
    for (int j = 0; j < 4; j++) newFrame[j] = oldFrame[j];
    int newFrameSize = 4;
    for (int j = 4; j < oldFrameSize - 1; j++) {
        if (oldFrame[j] == F) {
            newFrame[newFrameSize] = ESC; newFrameSize++;
            newFrame[newFrameSize] = 0x5e; newFrameSize++;
        } else if (oldFrame[j] == ESC) {
            newFrame[newFrameSize] = ESC; newFrameSize++;
            newFrame[newFrameSize] = 0x5d; newFrameSize++;
        } else {
            newFrame[newFrameSize] = oldFrame[j];
            newFrameSize++;
        }
    }
    newFrameSize++;
    newFrame[newFrameSize - 1] = F;
    newFrame = realloc(newFrame, newFrameSize);

    ReceiveState state = START;
    int tries = nRetransmissions;
    unsigned char byte;

    if (write(fd, newFrame, newFrameSize) < 0) return -1;

    (void) signal(SIGALRM, alarmHandler);
    alarm(timeout);

    while (tries > 0) {
        if (read(fd, &byte, 1)) {
            switch (state) {
                case START:
                    if (byte == F) state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (byte == A_Tx) state = A_RCV;
                    else if (byte == F) continue;
                    else state = START;
                    break;
                case A_RCV:
                    if (byte == F) {state = FLAG_RCV; break;}
                    if (info_frame_number == 0) {
                        if (byte == C_RR1) state = C_RCV;
                        else if (byte == C_REJ0) state = START;
                    } else if (info_frame_number == 1) {
                        if (byte == C_RR0) state = C_RCV;
                        else if (byte == C_REJ1) state = START;
                    }
                    break;
                case C_RCV:
                    if (byte == (A_Tx ^ (info_frame_number == 0 ? C_RR1 : C_RR0))) state = BCC_OK;
                    else if (byte == F) state = FLAG_RCV;
                    else state = START;
                    break;
                case BCC_OK:
                    if (byte == F) {
                        state = STOP;
                    } else state = START;
                    break;
                default: break;
            }
        }
        if (state == STOP) {
            break;
        }
        else if (alarmTriggered == TRUE) {
            tries--;
            printf("llwrite(): Alarm triggered, %i tries remaining.\n", tries);

            alarmTriggered = FALSE;
            (void) signal(SIGALRM, alarmHandler);
            alarm(timeout);

            if (write(fd, newFrame, newFrameSize) < 0) return -1;
        } else if (state == START) {
            alarmTriggered = FALSE;
            (void) signal(SIGALRM, alarmHandler);
            alarm(timeout);
            
            if (write(fd, newFrame, newFrameSize) < 0) return -1;
        }
    }
    if (tries == 0 && state != STOP) return -1;

    info_frame_number = info_frame_number == 0 ? 1 : 0;
    return bufSize;
}

unsigned char last_received_frame = 1;

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet){
    frame_info info = check_frame(fd, packet, last_received_frame);
    switch (info.type){
    case SET:
        write(fd, ua_frame, 5);
        break;
    case UA:
        break;

    case INFO:
        last_received_frame = info.number;
        if(info.number == 0) write(fd, rr_frame_1, 5);
        else if(info.number == 1) write(fd, rr_frame_0, 5);
        return info.size;
        break;
        
    case DUP:
    	if(info.number == 0) write(fd, rr_frame_1, 5);
        else if(info.number == 1) write(fd, rr_frame_0, 5);
        return -1;
        break;
        
    default:
        if(info.number == 0) write(fd, rej_frame_0, 5);
        else if (info.number == 1) write(fd, rej_frame_1, 5);

        return -1;
    }
    return 0;
}

int block_info = FALSE;

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics){
    block_info = TRUE;
    if(role == LlTx) {
        write(fd, disc_frame, 5);
        frame_info info = check_frame(fd, NULL, 0);
        if(info.type == DISC) write(fd, ua_frame, 5);
        else return -1;
    }
    else if(role == LlRx) {
        frame_info info = check_frame(fd, NULL, 0);
        if(info.type == DISC){
            write(fd, disc_frame, 5);
            (void) signal(SIGALRM, alarmHandler);
            alarm(timeout);
            int tries = nRetransmissions;
            while(info.type != UA && tries > 0){
                info = check_frame(fd, NULL, 0);
                if(alarmTriggered){
                    write(fd, disc_frame, 5);
                
                    alarmTriggered = FALSE;
                    (void) signal(SIGALRM, alarmHandler);
                    alarm(timeout);
                    
                }
            }
            if(tries <= 0){
                return -1;
            }
        }
        else return -1;
    }
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1){
        perror("tcsetattr");
        exit(-1);
    }
    close(fd);
    return 1;
}

frame_info check_frame(int fd, unsigned char* buffer, unsigned char last_received_frame) {
    ReceiveState state = START;
    frame_info info;
    info.type = -1; info.number = 2; info.size=0;
    

    unsigned char current_C;

    unsigned char BCC2 = 0x00;

    unsigned char byte_read;
    unsigned int current_buffer_byte = 0;

    unsigned int initial_header_size, info_field_size, final_header_size;

    while(state != STOP){
        if (read(fd, &byte_read, 1) == -1) continue;
        switch(state){
            case START:
                initial_header_size = 0; info_field_size = 0; final_header_size = 0;
                if(byte_read == F) state = FLAG_RCV;
                else state = START;
                break;

            case FLAG_RCV:
                initial_header_size++;
                if(byte_read == A) state = A_RCV;
                else if(byte_read == F) state = FLAG_RCV;
                else state = START;
                break;

            case A_RCV:
                initial_header_size++;
                if(byte_read == C_SET) state = C_SET_RCV;
                else if(byte_read == C_UA) state = C_UA_RCV;
                else if(byte_read == C_DISC) state = C_DISC_RCV;
                else if(!(byte_read & mask)) { state = C_INFO_RCV; current_C = byte_read; }
                else if(byte_read == F) state = FLAG_RCV;
                else state = START;
                break;

            case C_SET_RCV:
                initial_header_size++;
                info.type = SET;
                if(byte_read == (A^C_SET)) state = BCC_OK;
                else if(byte_read == F) state = FLAG_RCV;
                else state = START;
                break;

            case C_UA_RCV:
                initial_header_size++;
                info.type = UA;
                if(byte_read == (A^C_UA)) state = BCC_OK;
                else if(byte_read == F) state = FLAG_RCV;
                else state = START;
                break;

            case C_DISC_RCV:
                initial_header_size++;
                info.type = DISC;
                if(byte_read == (A^C_DISC)) state = BCC_OK;
                else if(byte_read == F) state = FLAG_RCV;
                else state = START;
                break;

            case C_INFO_RCV:
            	if(block_info) { info.type = -1; return info; }
                initial_header_size++;
                info.type = INFO;
                info.number = current_C >> 6;
                if(last_received_frame == info.number && 
                	byte_read == (A^current_C)) { info.type = DUP; return info; }
                if(byte_read == (A^current_C)) state = READING_DATA;
                else if(byte_read == F) state = FLAG_RCV;
                else state = START;
                break;

            case READING_DATA:
                if(byte_read == ESC && (BCC2 == 0x7e || BCC2 == 0x7d)) state = ESC_BCC2; 
                else if(byte_read == ESC) state = ESC_OK; 
                else if(byte_read == BCC2) state = BCC2_OK;
                else if(byte_read == F){ info.type = -1; return info; }
                else {
                    info_field_size++;
                    BCC2 ^= byte_read; 
                    buffer[current_buffer_byte++] = byte_read;
                    state = READING_DATA; 
                }
                break;

            case ESC_OK:
                if(byte_read == 0x5d) { 
                    BCC2 ^= 0x7d;
                    info_field_size++;
                    buffer[current_buffer_byte++] = 0x7d;
                    state = READING_DATA;
                } else if(byte_read == 0x5e) { 
                    BCC2 ^= 0x7e;
                    info_field_size++;
                    buffer[current_buffer_byte++] = 0x7e;
                    state = READING_DATA; 
                } else { info.type = -1; return info; }
                break;

            case BCC2_OK:
                final_header_size++;
                if(byte_read == F) state = STOP;
                else {
                    buffer[current_buffer_byte++] = BCC2;
                    BCC2 ^= BCC2;

                    if(byte_read == ESC && (BCC2 == 0x7e || BCC2 == 0x7d)) state = ESC_BCC2; 
                    else if(byte_read == ESC) state = ESC_OK; 
                    else if(byte_read == BCC2) state = BCC2_OK;

                    else { 
                        BCC2 ^= byte_read; 
                        buffer[current_buffer_byte++] = byte_read;
                        state = READING_DATA;
                    }

                }
                break;

            case ESC_BCC2:
                final_header_size++;
                if(BCC2 == 0x7d && byte_read == 0x5d) state = FLAG_ESC_BCC2;
                else if(BCC2 == 0x7e && byte_read == 0x5e) state = FLAG_ESC_BCC2;
                else { info.type = -1; return info; }
                break;

            case FLAG_ESC_BCC2:
                final_header_size++;
                if(byte_read == F) state = STOP;
                else { 
                    buffer[current_buffer_byte++] = BCC2;
                    BCC2 ^= BCC2;
                    state = READING_DATA; 
                }

            case BCC_OK:
                final_header_size++;
                if(byte_read == F) state = STOP;
                else state = START;
                break;

            case STOP:
                break;
            default:
                break;
        }      
    }
    info.size = current_buffer_byte;
    return info;
}
