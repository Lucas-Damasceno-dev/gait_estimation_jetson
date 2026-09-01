#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#define INPUT 0
#define OUTPUT 1
#define LOW 0
#define HIGH 1

int wiringPiSetup(void);
void pinMode(int pin, int mode);
void digitalWrite(int pin, int value);
int digitalRead(int pin);
void delay(unsigned int howLong);
void delayMicroseconds(unsigned int howLong);

#ifdef __cplusplus
}
#endif
