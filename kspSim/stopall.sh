#!/bin/sh
# Stop every simulated instance (idle sims keep simulating in real time).
for p in $(ps -eo pid,args | grep "[k]spSim.run --instance" | awk '{print $1}'); do kill $p; done
