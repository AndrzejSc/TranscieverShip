#include<SPI.h>
#include<nRF24L01.h>
#include<RF24.h>
#include <arduino.h>
RF24 radio(2, 3);
byte ackMessg[5] = {0};
byte x[5] = {10, 11, 12, 13, 14};
boolean state = true;
const uint64_t pipe[1] = {0xF0F0F0F0E1LL};
void setup()
{
    Serial.begin(9600);
    radio.begin();
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(76);
    radio.enableAckPayload();
    radio.setRetries(2, 5);
}
void loop()
{
    //radio.openWritingPipe(pipe[0]);
    if (state)
    {
        radio.stopListening();
        radio.write(x, sizeof(x));
        for (int i = 0; i < sizeof(x); i++)
        {
            Serial.print("x[");
            Serial.print(i, DEC);
            Serial.print("]=");
            Serial.println(x[i]);
            delay (200);
            radio.startListening();
            state = false;
        }
    }
    else
    {
        if ( radio.isAckPayloadAvailable())
        {
            radio.read(ackMessg, sizeof(ackMessg));
            for (int i = 0; i < sizeof(ackMessg); i++)
            {
                Serial.print("ackMessg[");
                Serial.print(i, DEC);
                Serial.print("]=");
                Serial.println(ackMessg[i]);
                delay (200);
            }
        }
        state = true;
    }
}
