# saetta_energy/EnergyReport.msg
#   message for reporting the global status of a monitored system

Header header

float32 energy_used             # total energy usage counter (watt-hours)
float32 energy_left             # total energy available for the system (watt-hours)
float32 energy_full             # design total energy (maximum energy storage)

float32 avg_power               # average power used by the system (watts)
EnergyStatus[] subsystems       # (optional) detailed energy usage for subsystems

# optional fields (work in progress)
# float32 time_depletion          # estimate of residual useful time
