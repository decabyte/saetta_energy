# saetta_energy/TrackerStatus.msg
#   message for reporting the status of the mission tracking system

Header header

float64 projected_time          # estimated time at mission completion
float64 projected_energy        # estimated energy at mission completion

float64 planned_time            # original time (planned before mission start)
float64 planned_energy          # original energy (planned before mission start)

float64 residual_actions        # remaining actions
float64 residual_energy         # remaining energy before mission completion
float64 residual_time           # remaining time before mission completion

float64[] actions_time             # estimated time at each trajectory leg
float64[] actions_energy           # estimated energy at each trajectory leg

float64 rmse_time               # uncertainty on time
float64 rmse_energy             # uncertainty on energy

float64 pomc                    # probability of mission completion
