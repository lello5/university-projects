(define (problem global-problem)
    (:domain global-domain)
    
	(:init
			(unknown (wantStart))
			(unknown (newUser))
    )
    
    (:goal (interactionDone))
)
