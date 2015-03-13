; mission_problem.pddl
(define (domain mission)

	(:requirements 
		:adl 
		:fluents
		:durative-actions 
		:duration-inequalities 
		:continuous-effects
	)

	(:types
		vehicle
		waypoint
		speed
	)

	(:predicates
		(at ?v - vehicle ?w - waypoint)
		(connected ?a ?b - waypoint)
	)

	(:functions
		(distance ?a ?b - waypoint)
		(energy-level ?v - vehicle)
		(energy-required ?a ?b - waypoint)
	)

	(:durative-action move
		:parameters (?from ?to - waypoint ?v - vehicle)
		:duration (= ?duration (* 1 (distance ?from ?to)))
		:condition (and
			(at start (at ?v ?from))
			(at start (connected ?from ?to))
		)
		:effect (and
			(at start (not (at ?v ?from))) 
			(at end (at ?v ?to))
			(decrease (energy-level ?v) (* #t (energy-required ?from ?to)))
		)
	)
)
