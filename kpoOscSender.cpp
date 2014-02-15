#include "kpoOscSender.h"
#include <iostream>

kpoOscSender::kpoOscSender()
{
    setup_ = false;
}

kpoOscSender::~kpoOscSender()
{
    delete transmitSocket;
}

void kpoOscSender::setNetworkTarget(const char *ip, int port)
{
    transmitSocket = new UdpTransmitSocket( IpEndpointName( ip, port ) );
    setup_ = true;
}

void kpoOscSender::send()
{
    if (!setup_) return;

    char buffer[OUTPUT_BUFFER_SIZE];

    osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );

    p << osc::BeginBundleImmediate
      << osc::BeginMessage( "/test1" )
          << (float)3.1 << osc::EndMessage;
//          << osc::EndBundle;

/*
          << osc::BeginMessage( "/test1" )
              << true << 23 << (float)3.1415 << "hello" << osc::EndMessage
        << osc::BeginMessage( "/test2" )
            << true << 24 << (float)10.8 << "world" << osc::EndMessage
*/

    transmitSocket->Send( p.Data(), p.Size() );
}
