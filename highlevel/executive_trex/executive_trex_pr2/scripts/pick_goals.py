#!/usr/bin/python
import time

def write_goal_file(filename, goals):
  f = open(filename, 'w')
  f.write('/*\n *Auto-generated goals file, created at %s\n */\n\n'%time.asctime())
  
  for (goal, index) in zip(goals, range(len(goals))):
    f.write('rejectable(m2_goals.Active g_%d);\ng_%d.outlet_id = %s\n'%(index,index,goal))

def get_goal(current_goals, valid_goals):
  while(True):
    print "Current goals: %s"%str(current_goals)
    print "Available additional goals: %s"%str([x for x in valid_goals if not x in current_goals])
    try:
      x = raw_input("Please enter a goal from the following list, or just hit enter if you're done.\ngoal_id:");
      if(x == ""):
        return None
      if(x in current_goals):
        print "Goal already in list"
      elif(x in valid_goals):
        return x
      else:
        print "%s not in list of valid goals"%x
    except:
      print "Error reading goal"
      
#Read in topological map

valid_goals = [str(x) for x in range(10)]

#Collect user clicks (for now, just ask user to type in IDs) and validate them
goals = []
while(True):
  goal = get_goal(goals, valid_goals)
  if goal == None: break
  goals.append(goal)

#Output goal file
write_goal_file('goals.nddl', goals)
