#!/usr/bin/env python3
"""Generate CycloneDDS XML configs bridging macOS host CARLA and Docker Autoware.

On macOS Docker Desktop, `-p` UDP forwarding passes packets through `com.docker.backend`,
which silently drops high/many ephemeral UDP port mappings when ~70+ Autoware nodes
bind dynamic participant indices. To avoid this completely:
  1. `cyclonedds_host.xml`: macOS host CARLA (ROS_DOMAIN_ID=0, ParticipantIndex=0, UDP 7410-7411)
  2. `cyclonedds_bridge.xml`: Container Domain 0 relay side (ParticipantIndex=1, UDP 7412-7413)
     bridged to macOS via `-p 7412-7413:7412-7413/udp`
  3. `cyclonedds_container.xml`: Container Autoware stack (ROS_DOMAIN_ID=1 on `lo`,
     MaxAutoParticipantIndex=200)
"""

from __future__ import annotations

import argparse
import socket
import sys
from pathlib import Path


def detect_host_ip() -> str:
    """Return the primary non-loopback IPv4 address of the macOS host."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        return sock.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        sock.close()


HOST_TEMPLATE = """<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface address="{host_ip}" multicast="false" />
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Discovery>
      <ParticipantIndex>0</ParticipantIndex>
      <Peers>
        <Peer address="127.0.0.1:7410" />
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
"""

BRIDGE_TEMPLATE = """<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="eth0" multicast="false" />
      </Interfaces>
      <ExternalNetworkAddress>127.0.0.1</ExternalNetworkAddress>
      <AllowMulticast>false</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Discovery>
      <ParticipantIndex>1</ParticipantIndex>
      <Peers>
        <Peer address="{host_ip}:7410" />
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
"""

CONTAINER_TEMPLATE = """<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="lo" />
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
"""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host-ip", default=None, help="Override detected macOS IPv4 address")
    parser.add_argument(
        "--out-dir",
        default=str(Path(__file__).resolve().parent),
        help="Output directory for cyclonedds_{host,bridge,container}.xml",
    )
    args = parser.parse_args()

    host_ip = args.host_ip or detect_host_ip()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    host_xml = out_dir / "cyclonedds_host.xml"
    bridge_xml = out_dir / "cyclonedds_bridge.xml"
    container_xml = out_dir / "cyclonedds_container.xml"
    host_xml.write_text(HOST_TEMPLATE.format(host_ip=host_ip), encoding="utf-8")
    bridge_xml.write_text(BRIDGE_TEMPLATE.format(host_ip=host_ip), encoding="utf-8")
    container_xml.write_text(CONTAINER_TEMPLATE, encoding="utf-8")

    print(f"Detected host IP : {host_ip}")
    print(f"Host DDS config  : {host_xml}")
    print(f"Bridge DDS config: {bridge_xml}")
    print(f"Docker DDS config: {container_xml}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
