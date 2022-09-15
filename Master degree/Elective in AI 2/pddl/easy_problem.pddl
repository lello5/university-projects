(define (problem quizEasy-problem)
    (:domain pepper-domain)
    (:objects
                doYouLikeBasketball - question doYouPlayInWeek - question doYouWatchMatches - question
				
                howManyPlayersInTeam - quiz canHitBallWithFeet - quiz howManyPointsInsideArea - quiz		
	)
	
	(:init
			(classificationDone)
			
			(unknown (correctQuiz howManyPlayersInTeam))
			(unknown (correctQuiz canHitBallWithFeet))
			(unknown (correctQuiz howManyPointsInsideArea))
    )
    
    (:goal (quizDone))
)
