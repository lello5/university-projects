import sys
from functions import *
import pickle


def run_executor():
    plan = 'plans/global_problem.txt'
    next_action = '0||0'

    with open('dictionaries/options.pickle', 'rb') as handle:
        options = pickle.load(handle)

    with open(plan) as f:
        lines = f.readlines()
        for line in lines:
            if next_action in line:
                # here we can perform only ASK_START and CHECK_NEW, the actions with two sons
                try:
                    id, action, trueson, falseson = line.strip('\n').strip(' ').split(' --- ')
                    true_id = trueson.split(': ')[1]
                    false_id = falseson.split(': ')[1]
                    
                    print('Action: ', action)
                    res = options[action]()
                    print('Condition was resolved to a {} value.'.format(res))

                    next_action = true_id if res else false_id

                # here we perform all the remaining action with one son only
                except:
                    id, action, son = line.strip('\n').strip(' ').split(' --- ')
                    son_id = son.split(': ')[1]
                    
                    print('Action: ', action)
                    options[action]()

                    next_action = son_id
                    
    return int(next_action.split('||')[1]) 

if __name__ == "__main__":
    run_executor()
