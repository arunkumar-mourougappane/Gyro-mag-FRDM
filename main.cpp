#include "mbed.h"
#include "MPU9250.h"
DigitalOut myled(LED_GREEN);
Serial pc(USBTX, USBRX);

int main()
{
    int i = 0;
    pc.printf("Hello World!\n");
    MPU9250 mpu9250;
    while (true) {
        wait(0.5f); // wait a small period of time
        pc.printf("%d \n", i); // print the value of variable i
        i++; // increment the variable
        myled = !myled; // toggle a led
    }
}
