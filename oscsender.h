#ifndef OSCSENDER_H
#define OSCSENDER_H

#include "osc/OscOutboundPacketStream.h"
#include "ip/UdpSocket.h"


#define ADDRESS "127.0.0.1"
#define PORT 7000

#define OUTPUT_BUFFER_SIZE 1024


class oscSender
{
private:
    UdpTransmitSocket *transmitSocket;

public:
    oscSender();
    ~oscSender();
    void init();

    void send();
};

#endif // OSCSENDER_H
