#include "kpoOscSender.h"


kpoOscSender::kpoOscSender()
{
}

kpoOscSender::~kpoOscSender()
{
    delete transmitSocket;
}

void kpoOscSender::setNetworkTarget(const char *ip, int port)
{
    transmitSocket = new UdpTransmitSocket( IpEndpointName( ADDRESS, PORT ) );
}

void kpoOscSender::send()
{
    if (!transmitSocket) return;

    char buffer[OUTPUT_BUFFER_SIZE];
    osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );

    p << osc::BeginBundleImmediate
        << osc::BeginMessage( "/test1" )
            << true << 23 << (float)3.1415 << "hello" << osc::EndMessage
        << osc::BeginMessage( "/test2" )
            << true << 24 << (float)10.8 << "world" << osc::EndMessage
        << osc::EndBundle;

    transmitSocket->Send( p.Data(), p.Size() );
}
