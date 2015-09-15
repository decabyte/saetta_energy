# saetta_energy/BatteryStatus.msg
#   message for reporting the status of the vehicle battery system

Header header

float64 energy_max              # battery system maximum energy
float64 energy_residual         # battery system residual energy
float64 soc                     # state of charge

float64 energy_std              # estimation std
