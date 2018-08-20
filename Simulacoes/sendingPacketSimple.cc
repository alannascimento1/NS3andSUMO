#include "ns3/vector.h"
#include "ns3/string.h"
#include "ns3/socket.h"
#include "ns3/double.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/command-line.h"
#include "ns3/mobility-model.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-interface-container.h"
#include <iostream>
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"

#include "ns3/netanim-module.h"

//ns2-mobility
#include <fstream>
#include <sstream>
#include "ns3/core-module.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/csma-module.h"
#include "ns3/applications-module.h"

NS_LOG_COMPONENT_DEFINE ("WifiSimpleOcb");
using namespace ns3;


void ReceivePacket (Ptr<Socket> socket)
{
  NS_LOG_UNCOND ("Received one packet!");
}

static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
      socket->Send (Create<Packet> (pktSize));
      Simulator::Schedule (pktInterval, &GenerateTraffic,
                           socket, pktSize,pktCount - 1, pktInterval);
    }
  else
    {
      socket->Close ();
    }
}

int main (int argc, char *argv[])
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

  // Create all nodes.
  NodeContainer stas;
  stas.Create (nodeNum);
  NodeContainer d;
  d.Create (2);

  //---------------------------------------------------------------------------------------------------
  std::string phyMode ("OfdmRate6MbpsBW10MHz");
  uint32_t packetSize = 1000; // bytes
  uint32_t numPackets = 800;
  double interval = 1.0; // seconds
  bool verbose = false;

  Time interPacketInterval = Seconds (interval);


  // The below set of helpers will help us to put together the wifi NICs we want
  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiPhy.SetChannel (channel);
  // ns-3 supports generate a pcap trace
  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);

  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  if (verbose)
    {
      wifi80211p.EnableLogComponents ();      // Turn on all Wifi 802.11p logging
    }

  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (phyMode),
                                      "ControlMode",StringValue (phyMode));
  NetDeviceContainer devices = wifi80211p.Install (wifiPhy, wifi80211pMac, stas);


  InternetStackHelper internet;
  internet.Install (stas);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);

  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  
  Ptr<Socket> recvSink = Socket::CreateSocket (stas.Get (0), tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
  recvSink->Bind (local);
  recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));

  Ptr<Socket> source = Socket::CreateSocket (stas.Get (1), tid);
  InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
  source->SetAllowBroadcast (true);
  source->Connect (remote);

  Simulator::ScheduleWithContext (source->GetNode ()->GetId (),
                                  Seconds (1.0), &GenerateTraffic,
                                  source, packetSize, numPackets, interPacketInterval);
//----------------------------------------------------------------------------
  AnimationInterface anim("simulations/wave-simple.xml");
   anim.SetConstantPosition(d.Get(0),125.0,125.0); 

 // Create Ns2MobilityHelper with the specified trace log file as parameter
  Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);

  // open log file for output
  std::ofstream os;
  os.open (logFile.c_str ());
  ns2.Install (); // configure movements for each node, while reading trace file

  // // Configure callback for logging
  // Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange",
  //                  MakeBoundCallback (&CourseChange, &os));

  Simulator::Stop (Seconds (duration));
  Simulator::Run ();
  Simulator::Destroy ();

  os.close (); // close log file
  return 0;

}