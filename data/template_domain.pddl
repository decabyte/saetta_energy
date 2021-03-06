; domain.pddl
;   this is auto-generated by the planner node
;   do not edit manually unless sure of it
;
; generated on: {{ time }}

(define (domain mission)

    ; define the domain requirements
	(:requirements 
		:adl 
		:fluents
		:durative-actions 
		:duration-inequalities 
		:continuous-effects
	)

    ; define the types
	(:types
		vehicle
		waypoint
		speed
	)

    ; define the predicates
	(:predicates
		(at ?v - vehicle ?w - waypoint)
		(connected ?a ?b - waypoint)
	)

    ; define the fluents
	(:functions
		(distance ?a ?b - waypoint)
		(energy-usage ?v - vehicle)
		(energy-required ?a ?b - waypoint)
	)

    ; define the actions
	(:durative-action move
		:parameters (?from ?to - waypoint ?v - vehicle)
		:duration (= ?duration (* 3.3 (distance ?from ?to)))
		:condition (and
			(at start (at ?v ?from))
			(at start (connected ?from ?to))
		)
		:effect (and
			(at start (not (at ?v ?from))) 
			(at end (at ?v ?to))
			(increase (energy-usage ?v) (* #t (energy-required ?from ?to)))
		)
	)
)
