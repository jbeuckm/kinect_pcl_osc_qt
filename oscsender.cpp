#include "oscsender.h"


oscSender::oscSender()
{
    transmitSocket = new UdpTransmitSocket( IpEndpointName( ADDRESS, PORT ) );
}

oscSender::~oscSender()
{
    delete transmitSocket;
}


void oscSender::init()
{

}

void oscSender::send()
{

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
