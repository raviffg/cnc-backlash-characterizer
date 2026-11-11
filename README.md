# cnc-backlash-characterizer

Measures leadscrew backlash on a 3-axis router and cross-checks it against a
stepper simulation.

## Layout

    src/gcode_stepper_sim.cpp   G00/G01/G02 parser + lash/hysteresis model
    scripts/backlash_fit.py     fits reversal deadband from encoder captures
    data/reversal_encoder_log.csv

## Build

    make

## Run

    ./build/gcode_stepper_sim path/to/file.gcode
    python scripts/backlash_fit.py data/reversal_encoder_log.csv
    make fit

The simulator steps each axis on the TMC2209 1/16 microstep grid
(0.000625 mm on a 2 mm pitch screw) and consumes the nut deadband before any
carriage motion is transmitted, so direction reversals reproduce the hysteresis
loop seen on the optical scale.

`backlash_fit.py` reports median and peak deadband per axis plus corner
overshoot. Divide the median by two to get the per-side compensation constant
for the controller.

Tested on g++ 13 / Python 3.11. No external dependencies.
