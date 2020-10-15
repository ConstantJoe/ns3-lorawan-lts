/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2017 IDLab-imec
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Floris Van den Abeele <floris.vandenabeele@ugent.be>
 */

/*
 * Try to send data from two class A end devices to a gateway, data is
 * unconfirmed upstream data. Chain is LoRaWANMac -> LoRaWANPhy ->
 * SpectrumChannel -> LoRaWANPhy -> LoRaWANMac
 *
 * Trace Phy state changes, and Mac DataIndication and DataConfirm events
 * to stdout
 */
#include <ns3/log.h>
#include <ns3/core-module.h>
#include <ns3/network-module.h>
#include <ns3/ipv4-address.h>
#include <ns3/lorawan-module.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/mobility-module.h>
#include <ns3/applications-module.h>
#include <ns3/simulator.h>
#include <ns3/single-model-spectrum-channel.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/node.h>
#include <ns3/packet.h>

#include <iostream>

using namespace ns3;

void
ReceivePacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  uint64_t bytes = 0;
  while ((packet = socket->Recv ()))
    {
      bytes += packet->GetSize ();
    }

  //std::cout << "SOCKET received " << bytes << " bytes" << std::endl;
}

Ptr<Socket>
SetupPacketReceive (Ptr<Node> node)
{
  TypeId tid = TypeId::LookupByName ("ns3::PacketSocketFactory");
  Ptr<Socket> sink = Socket::CreateSocket (node, tid);
  sink->Bind ();
  sink->SetRecvCallback (MakeCallback (&ReceivePacket));
  return sink;
}

int main (int argc, char *argv[])
{
  //LogComponentEnable ("LoRaWANGatewayApplication", LOG_LEVEL_DEBUG);
  // LogComponentEnable ("LoRaWANEndDeviceApplication", LOG_LEVEL_DEBUG);
  //LogComponentEnable ("LoRaWANPhy", LOG_LEVEL_DEBUG);
  // LogComponentEnableAll (LOG_PREFIX_TIME);
  
  uint32_t nNodes = 4;
  //uint8_t  dr = 0;
  //uint32_t stream = 0;

  CommandLine cmd;
  cmd.AddValue("nNodes", "Number of nodes to add to simulation", nNodes);
  //cmd.AddValue("dr", "Data rate to be used (up and down, a and b)", dr);
  //cmd.AddValue("stream", "Random stream var", stream);
  cmd.Parse (argc, argv);


  NodeContainer endDeviceNodes;
  NodeContainer gatewayNodes;
  NodeContainer allNodes;

  endDeviceNodes.Create (nNodes);
  gatewayNodes.Create (1);
  allNodes.Add (endDeviceNodes);
  allNodes.Add (gatewayNodes);

  double m_discRadius = 4000.0;
  MobilityHelper edMobility;
  edMobility.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
                                    "X", DoubleValue (0.0),
                                    "Y", DoubleValue (0.0),
                                    "rho", DoubleValue (m_discRadius));
  edMobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  edMobility.Install (endDeviceNodes);

  // the gateway is placed at 0,0,0
  MobilityHelper gwMobility;
  Ptr<ListPositionAllocator> nodePositionList = CreateObject<ListPositionAllocator> ();
  nodePositionList->Add (Vector (0.0, 0.0, 0.0));  // gateway
  gwMobility.SetPositionAllocator (nodePositionList);
  gwMobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  gwMobility.Install (gatewayNodes);

  LoRaWANHelper lorawanHelper;
  uint8_t reps = 1;
  lorawanHelper.SetNbRep(reps); // no retransmissions
  NetDeviceContainer lorawanEDDevices = lorawanHelper.Install (endDeviceNodes);

  lorawanHelper.SetDeviceType (LORAWAN_DT_GATEWAY);
  NetDeviceContainer lorawanGWDevices = lorawanHelper.Install (gatewayNodes);

  // Note to self: PacketSocketHelper::Install adds a PacketSocketFactory
  // object as an aggregate object to each of the nodes in the NodeContainer
  PacketSocketHelper packetSocket;
  packetSocket.Install (endDeviceNodes);
  packetSocket.Install (gatewayNodes);

  //LoRaWANEndDeviceHelper enddevicehelper;
  //enddevicehelper.SetAttribute ("DataRateIndex", UintegerValue (0));
  //ApplicationContainer enddeviceApps = enddevicehelper.Install (endDeviceNodes);

  NodeContainer EDDR5_devices;
  NodeContainer EDDR4_devices;
  NodeContainer EDDR3_devices;
  NodeContainer EDDR2_devices;
  NodeContainer EDDR1_devices;
  NodeContainer EDDR0_devices;

  NodeContainer::Iterator d;
  for (d = endDeviceNodes.Begin(); d != endDeviceNodes.End(); ++d) { 
    Ptr<MobilityModel> mobility = (*d)->GetObject<MobilityModel>();
    Vector current = mobility->GetPosition();
    float dist = sqrt((current.x * current.x) + (current.y * current.y)); //note: assumes GW is at 0.0
    //std::cout << (*d) dist << std::endl;

    Ptr<LoRaWANEndDeviceApplication> app = (*d)->GetObject<LoRaWANEndDeviceApplication>();
    if(dist < 1500) {
        EDDR5_devices.Add (*d);
    } else if(dist < 2000) {
        EDDR4_devices.Add (*d);
    } else if(dist < 2500) {
        EDDR3_devices.Add (*d);
    } else if(dist < 3000) {
        EDDR2_devices.Add (*d);
    } else if(dist < 3500) {
        EDDR1_devices.Add (*d);
    } else {
        EDDR0_devices.Add (*d);
    }
  }

  LoRaWANEndDeviceHelper enddevicehelper;
  enddevicehelper.SetAttribute ("DataRateIndex", UintegerValue (5));
  ApplicationContainer enddeviceAppsDR5 = enddevicehelper.Install (EDDR5_devices);

  enddevicehelper.SetAttribute ("DataRateIndex", UintegerValue (4));
  ApplicationContainer enddeviceAppsDR4 = enddevicehelper.Install (EDDR4_devices);

  enddevicehelper.SetAttribute ("DataRateIndex", UintegerValue (3));
  ApplicationContainer enddeviceAppsDR3 = enddevicehelper.Install (EDDR3_devices);

  enddevicehelper.SetAttribute ("DataRateIndex", UintegerValue (2));
  ApplicationContainer enddeviceAppsDR2 = enddevicehelper.Install (EDDR2_devices);

  enddevicehelper.SetAttribute ("DataRateIndex", UintegerValue (1));
  ApplicationContainer enddeviceAppsDR1 = enddevicehelper.Install (EDDR1_devices);

  enddevicehelper.SetAttribute ("DataRateIndex", UintegerValue (0));
  ApplicationContainer enddeviceAppsDR0 = enddevicehelper.Install (EDDR0_devices);

  LoRaWANGatewayHelper gatewayhelper;
  ApplicationContainer gatewayApps = gatewayhelper.Install (gatewayNodes);

  ApplicationContainer enddeviceApps;
  enddeviceApps.Add (enddeviceAppsDR5);
  enddeviceApps.Add (enddeviceAppsDR4);
  enddeviceApps.Add (enddeviceAppsDR3);
  enddeviceApps.Add (enddeviceAppsDR2);
  enddeviceApps.Add (enddeviceAppsDR1);
  enddeviceApps.Add (enddeviceAppsDR0);

  std::cout << "LOCATIONS START" << std::endl;
  //NodeContainer::Iterator d;
  for (d = endDeviceNodes.Begin(); d != endDeviceNodes.End(); ++d) {
    Ptr<MobilityModel> mobility = (*d)->GetObject<MobilityModel>();
    Vector current = mobility->GetPosition();
    float dist = sqrt((current.x * current.x) + (current.y * current.y));
    std::cout << (*d)->GetId() << " " << current.x << " " << current.y << " " << current.z << " " << dist << std::endl;
  }
  std::cout << "LOCATIONS END" << std::endl;

  enddeviceApps.Start (Seconds (0.0));
  enddeviceApps.Stop (Seconds (600.0*150));

  //run for a day
  gatewayApps.Start (Seconds (0.0));
  gatewayApps.Stop (Seconds (600.0*150 + 1));

  Ptr<Socket> recvSink = SetupPacketReceive (gatewayNodes.Get (0));

  Simulator::Stop (Seconds (600.0*150 + 1));

  Simulator::Run ();

  Simulator::Destroy ();
  return 0;
}
