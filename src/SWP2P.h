#ifndef SWP2P_H
#define SWP2P_H

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#ifndef SWP2P_FIFO_DEPTH
#define SWP2P_FIFO_DEPTH 16
#endif

class SWP2P {
public:
    enum TxState { TX_IDLE, TX_PENDING, TX_ARB, TX_DATA, TX_RELEASE, TX_DONE, TX_LOST };
    enum RxState { RX_IDLE, RX_ADDR, RX_DATA, RX_ACK };

    SWP2P(uint8_t nodeId);

    void begin(bool clkIsOutput, uint8_t* dataPins = nullptr, uint8_t dataWidth = 4, unsigned long clkFreq = 40000UL);
    bool send(uint8_t destId, uint8_t data);
    bool available();
    uint8_t read();

    bool isSending() const;
    bool isBusy() const;

private:
    void setupTimer1(unsigned long freq);
    void stopTimer1();
};

#endif
