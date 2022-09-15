import sys
from pepper_cmd_classes import Dialogue
sys.path.append('tablet/scripts')
from modim_classes import Quiz
import pickle


def run_executor():
    with open('dictionaries/p_dict.pickle', 'rb') as handle:
        p_dict = pickle.load(handle)

    with open('dictionaries/a_dict.pickle', 'rb') as handle:
        a_dict = pickle.load(handle)

    with open('dictionaries/q_dict.pickle', 'rb') as handle:
        q_dict = pickle.load(handle)

    dialogue = Dialogue()
    dialogue.say('To teach you better I need to know information about you.')

    next_plan, next_plan_announcement = p_dict['CLS'], a_dict['CLS']
    dialogue.say(next_plan_announcement)

    print("All existing plans: ", p_dict.values())

    while next_plan in list(p_dict.values()):
        print("Opening plan: ", next_plan)
        next_action = '0||0'
        
        with open(next_plan) as f:
            lines = f.readlines()
            for line in lines:
                if next_action in line:
                    # case with inner nodes
                    try:
                        id, action_params, trueson, falseson = line.strip('\n').strip(' ').split(' --- ')
                        action = action_params.split(' ')[0]
                        params = action_params.split(' ')[1:]
                        true_id = trueson.split(': ')[1]
                        false_id = falseson.split(': ')[1]
                        
                        print("Action: ", action)
                
                        question, correct_answer = q_dict[params[0]]['Q'], q_dict[params[0]]['A']
                        print("Question: ", question)
                        print("Anticipated correct answer: ", correct_answer)

                        # section responsible for getting user answer
                        if action == 'ASK_QUESTION':
                            user_answer = dialogue.say(question, require_answer = True, sleeping_time = 1.0)
                        # can only be ASK_QUIZ
                        else:
                            quiz = Quiz(question)
                            f = open('/home/robot/playground/quiz_answer.out', 'r')
                            user_answer = f.read()
                            f.close()

                        # update the next action
                        if user_answer.lower() == correct_answer:
                            next_action = true_id
                        else:
                            next_action = false_id
           
                    # case with a leaf (only one son)
                    except:
                        print("Entered switch plan section.")
                        id, action_params, son = line.strip('\n').strip(' ').split(' --- ')
                        action = action_params.split(' ')[0]
                        params = action_params.split(' ')[1:]
                        son_id = son.split(': ')[1]

                        level = action.split('_')[1]
                        try:
                            next_plan, next_plan_announcement  = p_dict[level], a_dict[level]
                            print("Found the next plan.")
                            print(next_plan)
                            dialogue.say(next_plan_announcement) 
                            dialogue.say('We are almost done with preliminarities. Please, follow the instructions on the tablet to take a little quiz.')
                            quiz = Quiz()
                            classification_result = level       
                        except:
                            print("Didn't find the next plan. Finishing execution.")
                            print(level)
                            quiz = Quiz(level.lower())
                            next_plan = None
                            
    return classification_result


if __name__ == "__main__":
    run_executor()
