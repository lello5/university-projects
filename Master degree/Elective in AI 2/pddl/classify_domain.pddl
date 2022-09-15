(define (domain pepper-domain)
	
	(:types question
            quiz
    )
    
    (:predicates    (correctAnswer ?x - question)   			; true if the user correctly answers to question x
                    (classificationDone)						; true if the classification/interview has been completed
                    (correctQuiz ?x - quiz)						; true if the user correctly answers to quiz x
                    (quizDone)									; true if the quiz has been completed
    )
    
    ; ASK QUESTION: the robot asks a question and listens to the user's answer.
    ;				It has to observe the correctness of the answer as well.
    (:action ask_question
		:parameters (?q - question)
		:precondition (not (classificationDone))
		:observe (correctAnswer ?q)
    )
    
    ; CLASSIFICATION: we divide the classification into three different actions
    ;					1. beginner - all the 3 answers are wrong
	;					2. intermediate - at least 1 correct and 1 wrong (the third may not be useful)
	;					3. expert - all the 3 answers are correct
	; 1. beginner
    (:action classify_beginner
		:parameters (?q1 - question ?q2 - question ?q3 - question)
		:precondition (and (not (classificationDone))
							(not (= ?q1 ?q2)) (not (= ?q1 ?q3)) (not (= ?q2 ?q3)))
		:effect (when 
					(and (not (correctAnswer ?q1)) (not (correctAnswer ?q2)) (not (correctAnswer ?q3)))
					(classificationDone)
				)
    )
    
    ; 2. intermediate
    (:action classify_intermediate
		:parameters (?q1 - question ?q2 - question ?q3 - question)
		:precondition (and (not (classificationDone))
							(not (= ?q1 ?q2)) (not (= ?q1 ?q3)) (not (= ?q2 ?q3)))
		:effect (when 
					(and (correctAnswer ?q1) (not (correctAnswer ?q2)))
					(classificationDone)
				)
    )
    
    ; 3. expert
    (:action classify_expert
		:parameters (?q1 - question ?q2 - question ?q3 - question)
		:precondition (and (not (classificationDone))
							(not (= ?q1 ?q2)) (not (= ?q1 ?q3)) (not (= ?q2 ?q3)))
		:effect (when 
					(and (correctAnswer ?q1) (correctAnswer ?q2) (correctAnswer ?q3))
					(classificationDone)
				)
    )
	
	; ASK QUIZ: the robot asks a quiz and listens to the user's answer.
    ;			It has to observe the correctness of the answer as well.
	(:action ask_quiz
		:parameters (?q - quiz)
		:precondition (classificationDone)
		:observe (correctQuiz ?q)
	)
    
    ; QUIZ EVALUATION: we divide the evaluation into three different actions
	;				1. bad - all the 3 answers are wrong
	;				2. medium - at least 1 correct and 1 wrong (the third may not be useful)
	;				3. good - all the 3 answers are correct
	; 1. bad
    (:action evaluate_bad
		:parameters (?q1 - quiz ?q2 - quiz ?q3 - quiz)
		:precondition (and (classificationDone)
							(not (= ?q1 ?q2)) (not (= ?q1 ?q3)) (not (= ?q2 ?q3)))
		:effect (when 
						(and (not (correctQuiz ?q1)) (not (correctQuiz ?q2)) (not (correctQuiz ?q3)))
						(quizDone)
				)
    ) 
    
    ; 2.
    (:action evaluate_medium
		:parameters (?q1 - quiz ?q2 - quiz ?q3 - quiz)
		:precondition (and (classificationDone)
							(not (= ?q1 ?q2)) (not (= ?q1 ?q3)) (not (= ?q2 ?q3)))
		:effect (when 
						(and (correctQuiz ?q1) (not (correctQuiz ?q2)))
						(quizDone)
				)
    ) 
    
    ; 3.
    (:action evaluate_good
		:parameters (?q1 - quiz ?q2 - quiz ?q3 - quiz)
		:precondition (and (classificationDone)
							(not (= ?q1 ?q2)) (not (= ?q1 ?q3)) (not (= ?q2 ?q3)))
		:effect (when 
						(and (correctQuiz ?q1) (correctQuiz ?q2) (correctQuiz ?q3))
						(quizDone)
				)
    )  
)
