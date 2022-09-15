(define (problem quizHard-problem)
    (:domain pepper-domain)
    (:objects
                doYouLikeBasketball - question doYouPlayInWeek - question doYouWatchMatches - question
				
                howManySecondsKeepBall - quiz howManyPointsFreeThrow - quiz whatIsMaxNumFouls - quiz
	)
	
	(:init
			(classificationDone)
			
			(unknown (correctQuiz howManySecondsKeepBall))
			(unknown (correctQuiz howManyPointsFreeThrow))
			(unknown (correctQuiz whatIsMaxNumFouls))
    )
    
    (:goal (quizDone))
)
