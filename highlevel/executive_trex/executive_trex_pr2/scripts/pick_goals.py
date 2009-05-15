#!/usr/bin/python
import time

def write_goal_file(filename, goals):

  print "Writnig out file %s with goals %s"%(filename, str(goals))
  f = open(filename, 'w')
  f.write('/*\n *Auto-generated goals file, created at %s\n */\n\n'%time.asctime())
  
  for (goal, index) in zip(goals, range(len(goals))):
    f.write('rejectable(m2_goals.Active g_%d);\ng_%d.outlet_id = %s\n'%(index,index,goal))

def get_goal(current_goals, valid_goals):
  while(True):
    print "Current goals: %s"%str(current_goals)
    print "Available additional goals: %s"%str([x for x in valid_goals if not x in current_goals])
    try:
      x = raw_input("Please enter a new goal, or just hit enter if you're done.\ngoal_id:");
      print "\n"
      if(x == ""):
        return None
      if(int(x) in current_goals):
        print "Goal already in list"
      elif(int(x) in valid_goals):
        return int(x)
      else:
        print "%s not in list of valid goals"%x
    except:
      print "Error reading goal.  Please type just the number of the goal and press enter."
      
#Read in topological map

valid_goals = range(1, 10)

#Collect user clicks (for now, just ask user to type in IDs) and validate them
goals = []
while(True):
  goal = get_goal(goals, valid_goals)
  if goal == None: break
  goals.append(goal)

#Output goal file
write_goal_file('goals.nddl', goals)
