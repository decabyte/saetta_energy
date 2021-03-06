; problem.pddl
;   this is auto-generated by the planner node
;   do not edit manually unless sure of it
;
; generated on: {{ time }}

(define (problem {{ problem_name }})
	; reference the domain domain
	(:domain {{ domain_name }})

    ; define the planning objects
	(:objects
    {% for obj_label, obj_type in objects %}
        {{ obj_label }} - {{ obj_type }}
    {% endfor %}
	)

    ; define the initial planning state
	(:init
    {% for name, values in init_literals %}
        ({{ name }} {{ ' '.join(values) }})
    {% endfor %}
    {% for function, number in init_fluents %}
        (= ({{ function[0] }} {{ ' '.join(function[1]) }}) {{ number }})
    {% endfor %}
    )

    ; define the planning goal
	(:goal
		(and
        {% for name, values in goals %}
            ({{ name }} {{ ' '.join(values) }})
        {% endfor %}
            ()
 		)
	)

	(:metric minimize (energy-usage auv))
)