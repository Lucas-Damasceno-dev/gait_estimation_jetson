#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include "wiringPi.h"
#include "wiringPiI2C.h"

int wiringPiSetup(void) { return 0; }
void pinMode(int pin, int mode) { (void)pin; (void)mode; }
void digitalWrite(int pin, int value) { (void)pin; (void)value; }
int digitalRead(int pin) { (void)pin; return 0; }
void delay(unsigned int howLong) { usleep(howLong * 1000); }
void delayMicroseconds(unsigned int howLong) { usleep(howLong); }

int wiringPiI2CSetup(const int devId) { (void)devId; return 1; }
int wiringPiI2CRead(int fd) { (void)fd; return 0; }
int wiringPiI2CReadReg8(int fd, int reg) { (void)fd; (void)reg; return 0xA0; }
int wiringPiI2CReadReg16(int fd, int reg) { (void)fd; (void)reg; return 0; }
int wiringPiI2CWrite(int fd, int data) { (void)fd; (void)data; return 0; }
int wiringPiI2CWriteReg8(int fd, int reg, int data) { (void)fd; (void)reg; (void)data; return 0; }
int wiringPiI2CWriteReg16(int fd, int reg, int data) { (void)fd; (void)reg; (void)data; return 0; }
