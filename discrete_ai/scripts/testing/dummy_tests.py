import copy

old_selected_action = 1
selected_action = 2

old_goal_parameters = 2
goal_parameters = 2

if (selected_action == old_selected_action and goal_parameters != old_goal_parameters) or (selected_action != old_selected_action):
    if old_selected_action >= 0:
        print('Second and on iteration')
        print('Cancel goal')
    print('Sending new goal')

old_selected_action = copy.copy(selected_action)
old_goal_parameters = copy.copy(goal_parameters)
