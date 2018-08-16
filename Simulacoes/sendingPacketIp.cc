#include "ns3/command-line.h"
#include "ns3/node.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/node-container.h"
#include "ns3/net-device-container.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/seq-ts-header.h"
#include "ns3/wave-net-device.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/wave-helper.h"
#include "ns3/internet-module.h"

//ns2-mobility
#include "ns3/netanim-module.h"
#include <fstream>
#include <sstream>
#include "ns3/core-module.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/csma-module.h"
#include "ns3/applications-module.h"


using namespace ns3;

/*
 * ===  VARIÁVEIS GLOBAIS ==============================================================
 *      
 * =====================================================================================
 */
NodeContainer nodes; ///< the nodes
NetDeviceContainer devices; ///< the devices
int numIP=0;
int numWSMP=0;


//###########################################################################################################
/*
 * ===  FUNCTION  ======================================================================
 *         Name:  sendOneIpPacket e sendPacketIp
 *  Description: Funções que irão realizar o envio de pacotes IP 
 * =====================================================================================
 */


void sendOneIpPacket (uint32_t seq, bool ipv6,   int send, int rec)
{

  Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices.Get (send));
  Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices.Get (rec));

  const Address dest = receiver->GetAddress ();

  // send IPv4 packet or IPv6 packet
  const static uint16_t IPv4_PROT_NUMBER = 0x0800;
  const static uint16_t IPv6_PROT_NUMBER = 0x86DD;
  
  uint16_t protocol = ipv6 ? IPv6_PROT_NUMBER : IPv4_PROT_NUMBER;
  
  Ptr<Packet> p  = Create<Packet> (100);
  SeqTsHeader seqTs;
  seqTs.SetSeq (seq);
  p->AddHeader (seqTs);
  sender->Send (p, dest, protocol);
}

void  sendPacketIp(double sec, int send, int rec){

  Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices.Get (send));
  Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices.Get (rec));

  // Alternating access without immediate channel switch
  const SchInfo schInfo = SchInfo (SCH1, false, EXTENDED_ALTERNATING);
  Simulator::Schedule (Seconds (sec), &WaveNetDevice::StartSch, sender, schInfo);
  // An important point is that the receiver should also be assigned channel
  // access for the same channel to receive packets.
  Simulator::Schedule (Seconds (sec), &WaveNetDevice::StartSch, receiver, schInfo);

  // both IPv4 and IPv6 packets below will not be inserted to internal queue because of no tx profile registered
  Simulator::Schedule (Seconds (sec+1.0), &sendOneIpPacket, numIP+1, true, send, rec);
  Simulator::Schedule (Seconds (sec+1.050), &sendOneIpPacket, numIP+2, false, send, rec);
  //register txprofile
  // IP packets will automatically be sent with txprofile parameter

  const TxProfile txProfile = TxProfile (SCH1);
  Simulator::Schedule (Seconds (sec+2.0), &WaveNetDevice::RegisterTxProfile, sender, txProfile);
  
  // both IPv4 and IPv6 packet are transmitted successfully
  Simulator::Schedule (Seconds (sec+2.05), &sendOneIpPacket, numIP+3, true, send, rec);
  Simulator::Schedule (Seconds (sec+2.2), &sendOneIpPacket, numIP+4, false, send, rec);

  // unregister TxProfile or release channel access
  Simulator::Schedule (Seconds (sec+2.5),&WaveNetDevice::DeleteTxProfile, sender,SCH1);
  Simulator::Schedule (Seconds (sec+2.5),&WaveNetDevice::StopSch, sender,SCH1);
  Simulator::Schedule (Seconds (sec+2.5),&WaveNetDevice::StopSch, receiver, SCH1);
  // these packets will be dropped again because of no channel access assigned and no tx profile registered
  Simulator::Schedule (Seconds (sec+3.00), &sendOneIpPacket, numIP+5, true, send, rec);
  Simulator::Schedule (Seconds (sec+3.05), &sendOneIpPacket, numIP+6, false, send, rec);
  numIP+=6;

  

} 

//###########################################################################################################


//###########################################################################################################
/*
 * ===  FUNCTION  ======================================================================
 *         Name:  SendOneWsmpPacket e sendWsmpPacket
 *  Description: Funções que irão realizar o envio de pacotes WSMP
 * =====================================================================================
 */

void SendOneWsmpPacket  (uint32_t channel, uint32_t seq, int sen)
{

  Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices.Get (sen));
  const static uint16_t WSMP_PROT_NUMBER = 0x88DC;
  Mac48Address bssWildcard = Mac48Address::GetBroadcast ();

  const TxInfo txInfo = TxInfo (channel);
  Ptr<Packet> p  = Create<Packet> (100);
  SeqTsHeader seqTs;
  seqTs.SetSeq (seq);
  p->AddHeader (seqTs);
  sender->SendX  (p, bssWildcard, WSMP_PROT_NUMBER, txInfo);
}

void sendWsmpPacket(double sec,  int sen, int rec){
  Ptr<WaveNetDevice>  sender = DynamicCast<WaveNetDevice> (devices.Get (sen));
  Ptr<WaveNetDevice>  receiver = DynamicCast<WaveNetDevice> (devices.Get (rec));
  
  // Alternating access without immediate channel switch
  const SchInfo schInfo = SchInfo (SCH1, false, EXTENDED_ALTERNATING);
  Simulator::Schedule (Seconds (sec+0.0), &WaveNetDevice::StartSch,sender,schInfo);
  // An important point is that the receiver should also be assigned channel
  // access for the same channel to receive packets.
  Simulator::Schedule (Seconds (sec+0.0), &WaveNetDevice::StartSch, receiver, schInfo);

  // send WSMP packets
  // the first packet will be queued currently and be transmitted in next SCH interval
  Simulator::Schedule (Seconds (sec+1.0), &SendOneWsmpPacket, SCH1, numWSMP+1, sen);
  // the second packet will be queued currently and then be transmitted , because of in the CCH interval.
  Simulator::Schedule (Seconds (sec+1.0), &SendOneWsmpPacket, CCH, numWSMP+2, sen);
  // the third packet will be dropped because of no channel access for SCH2.
  Simulator::Schedule (Seconds (sec+1.0), &SendOneWsmpPacket, SCH2, numWSMP+3, sen);

  // release SCH access
  Simulator::Schedule (Seconds (sec+2.0), &WaveNetDevice::StopSch, sender, SCH1);
  Simulator::Schedule (Seconds (sec+2.0), &WaveNetDevice::StopSch, receiver, SCH1);
  // the fourth packet will be queued and be transmitted because of default CCH access assigned automatically.
  Simulator::Schedule (Seconds (sec+3.0), &SendOneWsmpPacket, CCH, numWSMP+4, sen);
  // the fifth packet will be dropped because of no SCH1 access assigned
  Simulator::Schedule (Seconds (sec+3.0), &SendOneWsmpPacket, SCH1, numWSMP+5, sen);
  numWSMP+=5;
}


//##################################################################################################################

bool Receive (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender)
{
  SeqTsHeader seqTs;
  pkt->PeekHeader (seqTs);
  std::cout << "receive a packet: " << std::endl
            << "  sequence = " << seqTs.GetSeq () << "," << std::endl
            << "  sendTime = " << seqTs.GetTs ().GetSeconds () << "s," << std::endl
            << "  recvTime = " << Now ().GetSeconds () << "s," << std::endl
            << "  protocol = 0x" << std::hex << mode << std::dec  << std::endl;
  return true;
}

bool ReceiveVsa (Ptr<const Packet> pkt,const Address & address, uint32_t, uint32_t)
{
  std::cout << "receive a VSA management frame: recvTime = " << Now ().GetSeconds () << "s." << std::endl;
  return true;
}

int 
main (int argc, char *argv[])
{

  std::string traceFile;
  std::string logFile;
  int    nodeNum;
  double duration;

  // Enable logging from the ns2 helper
  LogComponentEnable ("Ns2MobilityHelper",LOG_LEVEL_DEBUG);

  // Parse command line attribute
  CommandLine cmd;
  cmd.AddValue ("traceFile", "Ns2 movement trace file", traceFile);
  cmd.AddValue ("nodeNum", "Number of nodes", nodeNum);
  cmd.AddValue ("duration", "Duration of Simulation", duration);
  cmd.AddValue ("logFile", "Log file", logFile);
  cmd.Parse (argc,argv);

  // Check command line arguments
  if (traceFile.empty () || nodeNum <= 0 || duration <= 0 || logFile.empty ())
    {
      std::cout << "Usage of " << argv[0] << " :\n\n"
      "./waf --run \"ns2-mobility-trace"
      " --traceFile=src/mobility/examples/default.ns_movements"
      " --nodeNum=2 --duration=100.0 --logFile=ns2-mob.log\" \n\n"
      "NOTE: ns2-traces-file could be an absolute or relative path. You could use the file default.ns_movements\n"
      "      included in the same directory of this example file.\n\n"
      "NOTE 2: Number of nodes present in the trace file must match with the command line argument and must\n"
      "        be a positive number. Note that you must know it before to be able to load it.\n\n"
      "NOTE 3: Duration must be a positive number. Note that you must know it before to be able to load it.\n\n";

      return 0;
    }

  //-----------------------------------------------------------------------------------------------------------
  
  nodes = NodeContainer ();
  nodes.Create (nodeNum);//20 nós


  // Create Ns2MobilityHelper with the specified trace log file as parameter
  Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);

  // open log file for output
  std::ofstream os;
  os.open (logFile.c_str ());
  ns2.Install (); // configure movements for each node, while reading trace file

  YansWifiChannelHelper waveChannel = YansWifiChannelHelper::Default ();
  YansWavePhyHelper wavePhy =  YansWavePhyHelper::Default ();
  wavePhy.SetChannel (waveChannel.Create ());
  wavePhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);
  QosWaveMacHelper waveMac = QosWaveMacHelper::Default ();
  WaveHelper waveHelper = WaveHelper::Default ();
  devices = waveHelper.Install (wavePhy, waveMac, nodes);

  for (uint32_t i = 0; i != devices.GetN (); ++i)
    {
      Ptr<WaveNetDevice> device = DynamicCast<WaveNetDevice> (devices.Get (i));
      device->SetReceiveCallback (MakeCallback (&Receive));
      device->SetWaveVsaCallback (MakeCallback  (&ReceiveVsa));
    }

  // Tracing
  wavePhy.EnablePcap ("wave-simple-device", devices);

  //ENVIANDO PACOTES IPs
   sendPacketIp( 10.0,0,1);
  // sendPacketIp(10.0,1,2);
  sendPacketIp(15.0,2,3);
  sendPacketIp(20.0,3,4);
  sendPacketIp(25.0,4,5);
  sendPacketIp(30.0,5,6);
  sendPacketIp(35.0,6,7);
  sendPacketIp(40.0,7,8);
  sendPacketIp(45.0,8,9);
  sendPacketIp(50.0,9,10);
   sendPacketIp(55.0,10,11);
  sendPacketIp(60.0,11,12);
  sendPacketIp(65.0,12,13);

  // float tempo=0.0;
  // for (int i = 0; i < 49; ++i)
  //   {
  //     sendPacketIp(tempo,i,i+1);
  //     tempo+=5.0;
  //   }

  // //ENVIANDO PACOTES WSMPs
  // sendWsmpPacket(40.0,0,11);
  // sendWsmpPacket(50.0,0,11);
  // sendWsmpPacket(80.0,0,11);

  //---------------------------------------------------------------------------
  
  AnimationInterface anim("simulations/sendingPacketIP.xml");

Simulator::Stop (Seconds (600.0));
  Simulator::Run ();
  Simulator::Destroy ();

  Simulator::Stop (Seconds (600));
  Simulator::Run ();
  Simulator::Destroy ();

  os.close (); // close log file

  return 0;
}
