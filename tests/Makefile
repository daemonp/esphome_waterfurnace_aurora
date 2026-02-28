CXX ?= g++

# Base flags with debug info for meaningful stack traces
BASE_FLAGS = -std=gnu++20 -Wall -Wextra -Wpedantic -g -O0

# Enable sanitizers when available (disable with SANITIZE=0)
SANITIZE ?= 1
ifeq ($(SANITIZE),1)
  SANITIZE_FLAGS = -fsanitize=address,undefined -fno-omit-frame-pointer
else
  SANITIZE_FLAGS =
endif

CXXFLAGS = $(BASE_FLAGS) $(SANITIZE_FLAGS) \
           -I../components/waterfurnace_aurora \
           -I.

# Hub tests need mock headers to intercept ESPHome includes
HUB_CXXFLAGS = $(BASE_FLAGS) $(SANITIZE_FLAGS) \
               -Imocks \
               -I../components/waterfurnace_aurora \
               -I.

LDFLAGS = $(SANITIZE_FLAGS)

COMPONENT_DIR = ../components/waterfurnace_aurora
MOCK_HEADERS = $(wildcard mocks/esphome/core/*.h mocks/esphome/components/*/*.h)

.PHONY: test clean all

all: test

test: test_protocol test_registers test_hub test_climate_math
	@echo "=== Running protocol tests ==="
	./test_protocol
	@echo ""
	@echo "=== Running register tests ==="
	./test_registers
	@echo ""
	@echo "=== Running hub tests ==="
	./test_hub
	@echo ""
	@echo "=== Running climate math tests ==="
	./test_climate_math
	@echo ""
	@echo "All tests passed!"

test_protocol: test_protocol.cpp $(COMPONENT_DIR)/protocol.cpp $(COMPONENT_DIR)/protocol.h catch_amalgamated.cpp catch_amalgamated.hpp
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ test_protocol.cpp $(COMPONENT_DIR)/protocol.cpp catch_amalgamated.cpp

test_registers: test_registers.cpp $(COMPONENT_DIR)/registers.cpp $(COMPONENT_DIR)/registers.h catch_amalgamated.cpp catch_amalgamated.hpp
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ test_registers.cpp $(COMPONENT_DIR)/registers.cpp catch_amalgamated.cpp

test_hub: test_hub.cpp mock_impl.cpp $(COMPONENT_DIR)/waterfurnace_aurora.cpp $(COMPONENT_DIR)/waterfurnace_aurora.h \
          $(COMPONENT_DIR)/protocol.cpp $(COMPONENT_DIR)/protocol.h \
          $(COMPONENT_DIR)/registers.cpp $(COMPONENT_DIR)/registers.h \
          $(MOCK_HEADERS) \
          catch_amalgamated.cpp catch_amalgamated.hpp
	$(CXX) $(HUB_CXXFLAGS) $(LDFLAGS) -o $@ test_hub.cpp mock_impl.cpp \
		$(COMPONENT_DIR)/waterfurnace_aurora.cpp \
		$(COMPONENT_DIR)/protocol.cpp \
		$(COMPONENT_DIR)/registers.cpp \
		catch_amalgamated.cpp

test_climate_math: test_climate_math.cpp $(COMPONENT_DIR)/climate/climate_math.h catch_amalgamated.cpp catch_amalgamated.hpp
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ test_climate_math.cpp catch_amalgamated.cpp

clean:
	rm -f test_protocol test_registers test_hub test_climate_math
