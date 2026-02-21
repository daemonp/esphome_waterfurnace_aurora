CXX ?= g++
CXXFLAGS = -std=c++17 -Wall -Wextra -Wpedantic \
           -I../components/waterfurnace_aurora \
           -I.

# Hub tests need mock headers to intercept ESPHome includes
HUB_CXXFLAGS = -std=c++17 -Wall -Wextra -Wpedantic \
               -Imocks \
               -I../components/waterfurnace_aurora \
               -I.

COMPONENT_DIR = ../components/waterfurnace_aurora

.PHONY: test clean all

all: test

test: test_protocol test_registers test_hub
	@echo "=== Running protocol tests ==="
	./test_protocol
	@echo ""
	@echo "=== Running register tests ==="
	./test_registers
	@echo ""
	@echo "=== Running hub tests ==="
	./test_hub
	@echo ""
	@echo "All tests passed!"

test_protocol: test_protocol.cpp $(COMPONENT_DIR)/protocol.cpp $(COMPONENT_DIR)/protocol.h catch_amalgamated.cpp catch_amalgamated.hpp
	$(CXX) $(CXXFLAGS) -o $@ test_protocol.cpp $(COMPONENT_DIR)/protocol.cpp catch_amalgamated.cpp

test_registers: test_registers.cpp $(COMPONENT_DIR)/registers.cpp $(COMPONENT_DIR)/registers.h catch_amalgamated.cpp catch_amalgamated.hpp
	$(CXX) $(CXXFLAGS) -o $@ test_registers.cpp $(COMPONENT_DIR)/registers.cpp catch_amalgamated.cpp

test_hub: test_hub.cpp mock_impl.cpp $(COMPONENT_DIR)/waterfurnace_aurora.cpp $(COMPONENT_DIR)/waterfurnace_aurora.h \
          $(COMPONENT_DIR)/protocol.cpp $(COMPONENT_DIR)/protocol.h \
          $(COMPONENT_DIR)/registers.cpp $(COMPONENT_DIR)/registers.h \
          catch_amalgamated.cpp catch_amalgamated.hpp
	$(CXX) $(HUB_CXXFLAGS) -o $@ test_hub.cpp mock_impl.cpp \
		$(COMPONENT_DIR)/waterfurnace_aurora.cpp \
		$(COMPONENT_DIR)/protocol.cpp \
		$(COMPONENT_DIR)/registers.cpp \
		catch_amalgamated.cpp

clean:
	rm -f test_protocol test_registers test_hub
