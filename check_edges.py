import subprocess, sys, os

sumo_home = os.environ.get("SUMO_HOME", "")
if sumo_home:
    sys.path.append(os.path.join(sumo_home, "tools"))

import sumolib

net = sumolib.net.readNet("network.net.xml")
print("=== ALL EDGE IDs IN YOUR NETWORK ===")
for edge in net.getEdges():
    print(f"  {edge.getID()}  from={edge.getFromNode().getID()}  to={edge.getToNode().getID()}")