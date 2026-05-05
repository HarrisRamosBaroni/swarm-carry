REPO_ROOT := $(shell git -C $(dir $(abspath $(lastword $(MAKEFILE_LIST)))) rev-parse --show-toplevel)
OWL_INC   := $(REPO_ROOT)/src/swarm_mocap/include/swarm_mocap
OWL_LIB   := $(REPO_ROOT)/src/swarm_mocap/lib

owl_bridge.so: owl_bridge.cpp
	g++ -shared -fPIC -std=c++17 -O2 \
	    -I$(OWL_INC) \
	    -L$(OWL_LIB) -lowlsock \
	    -Wl,-rpath,$(OWL_LIB) \
	    -o $@ $<

clean:
	rm -f owl_bridge.so
