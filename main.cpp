#include "uart.h"
#include <stdio.h>
#include <unistd.h>
#include <string.h>

int main(int argc, char *argv[]) {
    Uart u;
    int i = 0;
    unsigned char tx_msg[] = "STM32\n"; 
    unsigned char rx_msg[256]; //buffer for received message

    while (1){
        //send to stm
        u.sendUart(tx_msg);
        printf("Sent: %s\n", tx_msg);

        //wait for received
        u.readUart();

        //print the received message (split by bytes)
        printf("Recieved: ");
        while( u.serial_message[i] != '#' ){
            printf("%c", u.serial_message[i]);
            i++;
        }
        printf("\n");

        i = 0; //reset
        usleep(1000000); //1 second delay
    }
    
    return 0;
}