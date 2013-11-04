#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/config-store-module.h"
#include "ns3/csma-module.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/topology-read-module.h"
#include "ns3/general-udp-client-server-helper.h"
#include "ns3/random-variable-stream.h"
#include <math.h>
#include "ns3/packet-socket.h"
#include "ns3/udp-socket.h"
#include "ns3/lte-rlc-tm.h"
#include "ns3/lte-rlc-um.h"
#include "ns3/PPBP-helper.h"
#include "ns3/flow-monitor-helper.h"
#include <iostream>
#include <fstream>

using namespace ns3;

void
NotifyConnectionEstablishedUe (std::string context,
                               uint64_t imsi,
                               uint16_t cellid,
                               uint16_t rnti)
{
  std::cout << context
            << " UE IMSI " << imsi
            << ": connected to CellId " << cellid
            << " with RNTI " << rnti
            << std::endl;
}

void
NotifyHandoverStartUe (std::string context,
                       uint64_t imsi,
                       uint16_t cellid,
                       uint16_t rnti,
                       uint16_t targetCellId)
{
  std::cout << context
            << " UE IMSI " << imsi
            << ": previously connected to CellId " << cellid
            << " with RNTI " << rnti
            << ", doing handover to CellId " << targetCellId
            << std::endl;
}

void
NotifyHandoverEndOkUe (std::string context,
                       uint64_t imsi,
                       uint16_t cellid,
                       uint16_t rnti)
{
  std::cout << context
            << " UE IMSI " << imsi
            << ": successful handover to CellId " << cellid
            << " with RNTI " << rnti
            << std::endl;
}

void
NotifyConnectionEstablishedEnb (std::string context,
                                uint64_t imsi,
                                uint16_t cellid,
                                uint16_t rnti)
{
  std::cout << context
            << " eNB CellId " << cellid
            << ": successful connection of UE with IMSI " << imsi
            << " RNTI " << rnti
            << std::endl;
}

void
NotifyHandoverStartEnb (std::string context,
                        uint64_t imsi,
                        uint16_t cellid,
                        uint16_t rnti,
                        uint16_t targetCellId)
{
  std::cout << context
            << " eNB CellId " << cellid
            << ": start handover of UE with IMSI " << imsi
            << " RNTI " << rnti
            << " to CellId " << targetCellId
            << std::endl;
}

void
NotifyHandoverEndOkEnb (std::string context,
                        uint64_t imsi,
                        uint16_t cellid,
                        uint16_t rnti)
{
  std::cout << context
            << " eNB CellId " << cellid
            << ": completed handover of UE with IMSI " << imsi
            << " RNTI " << rnti
            << std::endl;
}


NS_LOG_COMPONENT_DEFINE ("SimpleVmMigration");
int
main (int argc, char *argv[])
{
  uint16_t numberOfUeCcn   = 5;
  uint16_t numberOfUeMove  = 1;
  uint16_t numberOfEnbs    = 2;
  double initTime          = 0.0;
  double distance          = 1000.0; // meter
  double clientMinStart    = 0.1 + initTime;
  double clientMaxStart    = 0.3 + initTime;
  double mySimTime         = 30;
  uint32_t numOfPacket     = 100000;
  uint32_t vmMigration     = 1; // 0: no migration

  CommandLine cmd;
  cmd.AddValue("simTime", "Simulation time", mySimTime);
  cmd.AddValue("numOfPacket", "number of VM chunks", numOfPacket);
  cmd.AddValue("vmMigration", "VM migration indicator", vmMigration);
  cmd.Parse(argc, argv);

  double clientStop        = mySimTime + initTime;
  double simTime           = mySimTime + initTime;

  Config::SetDefault ("ns3::EpcX2::NumOfPacket", UintegerValue (numOfPacket));
  Config::SetDefault ("ns3::EpcX2::VmMigration", UintegerValue (vmMigration));
  Config::SetDefault ("ns3::UdpEchoServer2::VmMigration", UintegerValue (vmMigration));

  Config::SetDefault ("ns3::UdpEchoClient::Interval", TimeValue (MilliSeconds(10)));
  Config::SetDefault ("ns3::UdpEchoClient::MaxPackets", UintegerValue(1000000));
  Config::SetDefault ("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue(320));

  Config::SetDefault ("ns3::LteEnbNetDevice::DlEarfcn", UintegerValue (3100)); // band 7
  Config::SetDefault ("ns3::LteEnbNetDevice::UlEarfcn", UintegerValue (21100)); // band7

  /*
   *  CREATE BACKBONE NETWORK TOPOLOGY
   */
  std::string format ("Rocketfuel");
  std::string input ("src/topology-read/examples/sim6hop-vcp-target.txt");

  Ptr<TopologyReader> inFile = 0;
  TopologyReaderHelper topoHelp;

  NodeContainer backboneNodes;

  topoHelp.SetFileName (input);
  topoHelp.SetFileType (format);
  inFile = topoHelp.GetTopologyReader ();

  if (inFile != 0)
    {
      backboneNodes = inFile->Read ();
    }

  if (inFile->LinksSize () == 0)
    {
      NS_LOG_ERROR ("Problems reading the topology file. Failing.");
      return -1;
    }

  NS_LOG_INFO ("creating internet stack");
  InternetStackHelper stack;

  stack.Install (backboneNodes);

  NS_LOG_INFO ("creating ip4 addresses");
  Ipv4AddressHelper bbAddress;
  bbAddress.SetBase ("10.0.0.0", "255.0.0.0");
  Ipv4AddressHelper internetAddress;
  internetAddress.SetBase ("167.0.0.0", "255.255.0.0");

  int totlinks = inFile->LinksSize ();

  NS_LOG_INFO ("creating node containers");
  NodeContainer* nc = new NodeContainer[totlinks];
  TopologyReader::ConstLinksIterator iter;
  int i = 0;
  for ( iter = inFile->LinksBegin (); iter != inFile->LinksEnd (); iter++, i++ )
    {
      nc[i] = NodeContainer (iter->GetFromNode (), iter->GetToNode ());
    }

  NS_LOG_INFO ("creating net device containers");
  NetDeviceContainer* ndc = new NetDeviceContainer[totlinks];
  PointToPointHelper p2p;

  p2p.SetDeviceAttribute ("Mtu", UintegerValue (1500));

  for (int j = 0; j < totlinks; j++)
    {
        p2p.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("1Gb/s")));
        ndc[j] = p2p.Install (nc[j]);
    }

  NS_LOG_INFO ("creating ipv4 interfaces");
  Ipv4InterfaceContainer* ipic = new Ipv4InterfaceContainer[totlinks];
  for (int i = 0; i < totlinks; i++)
    {
      if (i < 12)
      {
        ipic[i] = bbAddress.Assign (ndc[i]);
        bbAddress.NewNetwork ();
      }
      else
      {
        ipic[i] = internetAddress.Assign (ndc[i]);
        internetAddress.NewNetwork ();
      }
    }

  // Create a single RemoteHost
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  stack.Install (remoteHostContainer);

  // Create the Internet
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("1Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));

  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("90.0.0.0", "255.0.0.0");

  NetDeviceContainer internetDevices = p2ph.Install(remoteHost, backboneNodes.Get(16));
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (0);

  // Global Routing
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  // Create LTE Nodes
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<EpcHelper> epcHelper = CreateObject<EpcHelper> (backboneNodes.Get(0));
  lteHelper->SetEpcHelper (epcHelper);
  //lteHelper->SetSchedulerType("ns3::RrFfMacScheduler");

  NodeContainer ueNodesEnb1;
  NodeContainer ueNodesMove;
  NodeContainer enbNodes;

  enbNodes.Add(backboneNodes.Get(2));
  enbNodes.Add(backboneNodes.Get(4));
  ueNodesEnb1.Create(numberOfUeCcn);
  ueNodesMove.Create(numberOfUeMove);

  // Install Mobility Model
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  for (uint16_t i = 0; i < numberOfEnbs; i++)
    {
      positionAlloc->Add (Vector(2* i * distance - distance, 0, 0));
    }

  for (uint16_t i = 0; i < ueNodesMove.GetN(); i++)
    {
      positionAlloc->Add (Vector(0, 0, 0));
    }

  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator(positionAlloc);
  mobility.Install(enbNodes);
  mobility.Install(ueNodesMove);

  // Position of UEs and Background Nodes attached to eNodeB 1
  MobilityHelper uemobilityEnb1;
  uemobilityEnb1.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
			"X", DoubleValue (-distance),
			"Y", DoubleValue (0.0),
			"rho", DoubleValue (distance));

  uemobilityEnb1.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  uemobilityEnb1.Install(ueNodesEnb1);

  // Install LTE Devices in eNB and UEs
  NetDeviceContainer enbLteDevs    = lteHelper->InstallEnbDevice (enbNodes.Get(0));
  Config::SetDefault ("ns3::LteEnbNetDevice::DlEarfcn", UintegerValue (3150)); // band 7
  Config::SetDefault ("ns3::LteEnbNetDevice::UlEarfcn", UintegerValue (21150)); // band7
  NetDeviceContainer enbLteDevs2    = lteHelper->InstallEnbDevice (enbNodes.Get(1));
  NetDeviceContainer ueLteDevsEnb1 = lteHelper->InstallUeDevice (ueNodesEnb1);
  NetDeviceContainer ueLteDevsMove = lteHelper->InstallUeDevice (ueNodesMove);

  // Install the IP stack on the UEs
  InternetStackHelper internet;
  internet.Install (ueNodesEnb1);
  internet.Install (ueNodesMove);

  Ipv4InterfaceContainer ueIpIfacesEnb1;
  Ipv4InterfaceContainer ueIpIfacesMove;

  ueIpIfacesEnb1     = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueLteDevsEnb1));
  ueIpIfacesMove     = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueLteDevsMove));

  // Set default gateways for UEs

  Ipv4StaticRoutingHelper ipv4RoutingHelper;

  for (uint32_t u = 0; u < ueNodesEnb1.GetN(); ++u)
    {
      Ptr<Node> ueNode = ueNodesEnb1.Get (u);
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }

  for (uint32_t u = 0; u < ueNodesMove.GetN(); ++u)
    {
      Ptr<Node> ueNode = ueNodesMove.Get (u);
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }

  // Attach UEs to the enb1
  for (uint16_t i = 0; i < ueNodesEnb1.GetN() ; i++)
    {
          lteHelper->Attach (ueLteDevsEnb1.Get(i), enbLteDevs.Get(0));
    }

  // Attach UEs to the enb1
  for (uint16_t i = 0; i < ueNodesMove.GetN() ; i++)
    {
          lteHelper->Attach (ueLteDevsMove.Get(i), enbLteDevs.Get(0));
    }

  NS_LOG_LOGIC ("setting up applications");

  // Install application into VCP node
  uint16_t vcpPort = 4444;
  ApplicationContainer vcpApp;
  UdpEchoServerHelper2 vcpServer (vcpPort);
  vcpApp.Add (vcpServer.Install (backboneNodes.Get(6)));
  vcpApp.Start (Seconds (initTime));

  // Install and start applications on UEs and remote host
  uint16_t dlPort = 10000;
  uint16_t ulPort = 20000;

  Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable> ();
  startTimeSeconds->SetAttribute ("Min", DoubleValue (clientMinStart));
  startTimeSeconds->SetAttribute ("Max", DoubleValue (clientMaxStart));

  // install and start application on UEs attached to enb1
  for (uint32_t u = 0; u < ueNodesEnb1.GetN(); ++u)
    {
      Ptr<Node> ue = ueNodesEnb1.Get (u);

      ApplicationContainer clientApps;
      ApplicationContainer serverApps;

      UdpEchoServerHelper ulServer (ulPort);
      serverApps.Add (ulServer.Install (remoteHost));

      UdpEchoClientHelper ulClient (remoteHostAddr, ulPort);
      clientApps.Add (ulClient.Install (ue));

      Ptr<EpcTft> tft = Create<EpcTft> ();
      EpcTft::PacketFilter dlpf;
      dlpf.localPortStart = dlPort;
      dlpf.localPortEnd = dlPort;
      tft->Add (dlpf);
      EpcTft::PacketFilter ulpf;
      ulpf.remotePortStart = ulPort;
      ulpf.remotePortEnd = ulPort;
      tft->Add (ulpf);
      EpsBearer bearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
      lteHelper->ActivateDedicatedEpsBearer (ueLteDevsEnb1.Get (u), bearer, tft);

      Time startTime = Seconds (startTimeSeconds->GetValue ());

      serverApps.Start (Seconds(initTime));
      clientApps.Start (startTime);
      clientApps.Stop (Seconds(clientStop));

    }

  // install and start application on UEs that will move
  for (uint32_t u = 0; u < ueNodesMove.GetN(); ++u)
    {
      Ptr<Node> ue = ueNodesMove.Get (u);

      ApplicationContainer clientApps;
      ApplicationContainer serverApps;

      UdpEchoServerHelper ulServer (ulPort);
      serverApps.Add (ulServer.Install (remoteHost));

      UdpEchoClientHelper ulClient (remoteHostAddr, ulPort);
      clientApps.Add (ulClient.Install (ue));

      Ptr<EpcTft> tft = Create<EpcTft> ();
      EpcTft::PacketFilter dlpf;
      dlpf.localPortStart = dlPort;
      dlpf.localPortEnd = dlPort;
      tft->Add (dlpf);
      EpcTft::PacketFilter ulpf;
      ulpf.remotePortStart = ulPort;
      ulpf.remotePortEnd = ulPort;
      tft->Add (ulpf);
      EpsBearer bearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
      lteHelper->ActivateDedicatedEpsBearer (ueLteDevsMove.Get (u), bearer, tft);

      Time startTime = Seconds (startTimeSeconds->GetValue ());

      serverApps.Start (Seconds(initTime));
      clientApps.Start (Seconds(clientMinStart));
      clientApps.Stop (Seconds(clientStop));
    }

  // Add X2 inteface
  lteHelper->AddX2Interface (enbNodes);
  lteHelper->HandoverRequest (Seconds (1.0), ueLteDevsMove.Get (0), enbLteDevs.Get (0), enbLteDevs2.Get (0));

  lteHelper->EnableMacTraces ();
  lteHelper->EnableRlcTraces ();
  lteHelper->EnablePdcpTraces ();
  Ptr<RadioBearerStatsCalculator> rlcStats = lteHelper->GetRlcStats ();
  rlcStats->SetAttribute ("EpochDuration", TimeValue (Seconds (0.05)));
  Ptr<RadioBearerStatsCalculator> pdcpStats = lteHelper->GetPdcpStats ();
  pdcpStats->SetAttribute ("EpochDuration", TimeValue (Seconds (0.05)));

  // connect custom trace sinks for RRC connection establishment and handover notification
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
                   MakeCallback (&NotifyConnectionEstablishedEnb));
  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
                   MakeCallback (&NotifyConnectionEstablishedUe));
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
                   MakeCallback (&NotifyHandoverStartEnb));
  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
                   MakeCallback (&NotifyHandoverStartUe));
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                   MakeCallback (&NotifyHandoverEndOkEnb));
  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
                   MakeCallback (&NotifyHandoverEndOkUe));


  Simulator::Stop(Seconds(simTime));
  Simulator::Run();

  Simulator::Destroy();
  return 0;

}
