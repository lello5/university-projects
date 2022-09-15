(define (problem classifyUser-problem)
    (:domain pepper-domain)
    (:objects	
				doYouLikeBasketball - question doYouPlayInWeek - question doYouWatchMatches - question
				
				howManyPlayersInTeam - quiz canHitBallWithFeet - quiz howManyPointsInsideArea - quiz
				howManySecondsKeepBall - quiz howManyPointsFreeThrow - quiz whatIsMaxNumFouls - quiz
	)
	
	(:init
			(unknown (correctAnswer doYouLikeBasketball))
			(unknown (correctAnswer doYouPlayInWeek))
			(unknown (correctAnswer doYouWatchMatches))
    )
    
    (:goal (classificationDone))
)
