CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra
BUILD    := build
TARGET   := $(BUILD)/gcode_stepper_sim
SRC      := src/gcode_stepper_sim.cpp

.PHONY: all clean fit

all: $(TARGET)

$(TARGET): $(SRC)
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) $(SRC) -o $@

fit:
	python scripts/backlash_fit.py data/reversal_encoder_log.csv

clean:
	rm -rf $(BUILD)
