(define (domain global-domain)

	(:predicates	(wantStart)			; true if user wants to start the interaction
					(newUser)			; true if the user is new (i.e. not registered in the database)
					(classifiedUser)	; true if the user has been classified (i.e. interview + quiz)
					(players)			; true if user has chosen his/her favourite players
					(warmUp)			; true if user has completed the warm up
					(levelChecked)		; true if the level of the user has been checked
					(lessonDone)		; true if the user has completed the lessons
					(interactionDone)	; true if the interaction can be terminated
	)
	
	(:action ask_start
		:observe (wantStart)
	)
	
	(:action check_new
		:precondition (wantStart)
		:observe (newUser)
	)
	
	(:action classify_evaluate_user
		:precondition (newUser)
		:effect (classifiedUser)
	)
	
	(:action ask_players
		:precondition (classifiedUser)
		:effect (players)
	)
	
	(:action warm_up
		:precondition (players)
		:effect (warmUp)
	)
	
	(:action warm_up
		:precondition (not (newUser))
		:effect (warmUp)
	)
	
	(:action start_lessons
		:precondition (warmUp)
		:effect (lessonDone)
	)
	
	(:action terminate
		:precondition (lessonDone)
		:effect (interactionDone)
	)
	
	(:action terminate
		:precondition (not (wantStart))
		:effect (interactionDone)
	)
)
