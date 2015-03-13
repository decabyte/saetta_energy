; mission_problem.pddl
(define (problem inspection)
	
	(:domain mission)

	(:objects
		auv - vehicle
		wp0 - waypoint
		wp1 - waypoint
		wp2 - waypoint
		wp3 - waypoint
		wp4 - waypoint
	)

	(:init
		(at auv wp0)
		(= (energy-level auv) 100000)
		(= (energy-required wp0 wp1) 10)
		(= (energy-required wp1 wp2) 10)
		(= (energy-required wp3 wp4) 10)
		(= (energy-required wp0 wp2) 5)
		(= (energy-required wp1 wp4) 5)
		(= (energy-required wp4 wp3) 5)

		(= (distance wp0 wp1) 10)
		(= (distance wp1 wp2) 10)
		(= (distance wp2 wp3) 10)
		(= (distance wp3 wp4) 10)
		(= (distance wp0 wp2) 5)
		(= (distance wp1 wp4) 30)

		(connected wp0 wp1)
		(connected wp1 wp2)
		(connected wp2 wp3)
		(connected wp3 wp4)
		(connected wp0 wp2)
		(connected wp1 wp4)
		(connected wp4 wp3)
	)

	(:goal
		(at auv wp4)
;		(and
;			(at auv wp4)
;		)
	)

	(:metric minimize (and (energy-usage auv))
)