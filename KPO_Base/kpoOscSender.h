#ifndef OSCSENDER_H
#define OSCSENDER_H

#include "osc/OscOutboundPacketStream.h"
#include "ip/UdpSocket.h"

#include <vector>
#include <boost/variant.hpp>


#define ADDRESS "127.0.0.1"
#define PORT 7000

#define OUTPUT_BUFFER_SIZE 1024


class kpoOscSender
{
private:
    bool setup_;

    UdpTransmitSocket *transmitSocket;

public:
    kpoOscSender();
    ~kpoOscSender();

    void setNetworkTarget(const char *ip, int port);

    void send(const char *path, int value);
    void sendObject(int object_id, float x, float y, float z);

    void sendBlob(float x, float y, float size);

};

#endif // OSCSENDER_H
