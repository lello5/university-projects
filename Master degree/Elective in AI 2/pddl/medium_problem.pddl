(define (problem quizHard-problem)
    (:domain pepper-domain)
    (:objects
                doYouLikeBasketball - question doYouPlayInWeek - question doYouWatchMatches - question
				
                howManyPlayersInTeam - quiz howManyPointsInsideArea - quiz howManySecondsKeepBall - quiz		
	)
	
	(:init
			(classificationDone)
			
			(unknown (correctQuiz howManyPlayersInTeam))
			(unknown (correctQuiz howManyPointsInsideArea))
			(unknown (correctQuiz howManySecondsKeepBall))
    )
    
    (:goal (quizDone))
)
