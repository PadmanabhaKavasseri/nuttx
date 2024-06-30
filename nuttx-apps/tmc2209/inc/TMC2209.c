#include "TMC2209.h"



void printDG(WriteReadReplyDatagram* dg){
    printf("Bytes: %d ",dg->bytes);
    printf("Sync: %d ",dg->sync);
    printf("Serial Address: %d ",dg->serial_address);
    printf("register_address: %d ",dg->register_address);
    printf("RW: %d ",dg->rw);
    printf("data: %d ",dg->data);
    printf("crc: %d ",dg->crc);
}


uint8_t serial_address_ = 0;

void serialWrite(uint8_t c){
    // write(fd_,&c,sizeof(c));
    // printf("%d\n");
}

void sendDatagramUnidirectional(WriteReadReplyDatagram* datagram, uint8_t datagram_size)
{
    uint8_t byte;

    printf("SendUnidgg\n");
    printDG(datagram);

    for (uint8_t i=0; i<datagram_size; ++i)
    {
        byte = (datagram->bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
        printf("%d ",byte);
        serialWrite(byte);
    }
}

uint8_t calculateCrc(WriteReadReplyDatagram* datagram, uint8_t datagram_size)
{
    uint8_t crc = 0;
    uint8_t byte;
    for (uint8_t i=0; i<(datagram_size - 1); ++i){
        
        if (datagram_size == sizeof(WriteReadReplyDatagram)) {
            byte = (((WriteReadReplyDatagram*)datagram)->bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
        }
        for (uint8_t j=0; j<BITS_PER_BYTE; ++j){
            if ((crc >> 7) ^ (byte & 0x01)){
                crc = (crc << 1) ^ 0x07;
            }
            else{
                crc = crc << 1;
            }
            byte = byte >> 1;
        }
    }
    return crc;
}

uint32_t reverseData(uint32_t data)
{
    uint32_t reversed_data = 0;
    uint8_t right_shift;
    uint8_t left_shift;
    for (uint8_t i=0; i<DATA_SIZE; ++i){
        right_shift = (DATA_SIZE - i - 1) * BITS_PER_BYTE;
        left_shift = i * BITS_PER_BYTE;
        reversed_data |= ((data >> right_shift) & BYTE_MAX_VALUE) << left_shift;
    }
    return reversed_data;
}



void pkwrite(uint8_t register_address, uint32_t data){
    printf("data: %d\n", data);
    WriteReadReplyDatagram write_datagram;
    write_datagram.bytes = 0;
    write_datagram.sync = SYNC;
    write_datagram.serial_address = serial_address_;
    write_datagram.register_address = register_address;
    write_datagram.rw = RW_WRITE;
    write_datagram.data = reverseData(data);
    write_datagram.crc = calculateCrc(&write_datagram, WRITE_READ_REPLY_DATAGRAM_SIZE);

    printDG(&write_datagram);
    sendDatagramUnidirectional(&write_datagram, WRITE_READ_REPLY_DATAGRAM_SIZE);
}

void moveAtVelocity(int32_t microsteps_per_period){
    printf("mv@Vel:%d\n",microsteps_per_period);
    pkwrite(ADDRESS_VACTUAL, microsteps_per_period);
}




void init(int fd){
    fd = fd_;
    serial_address_ = 0;
}