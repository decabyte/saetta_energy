# saetta_energy/EnergyStatus.msg
#   message for reporting the status of a monitored subsystem
#
# if the subsystem is producing energy (like a battery or a generator) the energy counter will be negative
# indicating that the energy is leaving the subsystem thus giving energy to other system up in the components tree
#
# if the subsystem is using energy (like a sensors or an actuator) the energy counter will be positive and the
# the energy_left indicator defaults to zero (indicating no energy storage)

uint8 PRODUCER=0
uint8 CONSUMER=1

uint32 sid                      # subsystem number
string name                     # subsystem name
uint8 type                      # subsystem type (producer, consumer, etc.)

float32 energy_used             # energy usage over time (Wh)
float32 energy_left             # actual energy stored in this subsystem (Wh)
float32 energy_full             # maximum energy stored in this subsystem (Wh)

float32 avg_voltage             # average voltage (volts)
float32 avg_current             # average current (amps)
float32 avg_power               # average power (watts)

# Work in Progress:
#   describe the relationship between subsystems
#   i.e. tree structure or parent-child one
#
#       EnergyStatus[] subsystems       # (optional) detailed energy usage for subsystems
#
#       uint32 parent
#       uint32 child
