import pickle
import pprint

trajs_file = open('trajs.pkl', 'rb')
gt_file    = open('gt.pkl',    'rb')

trajectory = pickle.load(trajs_file)
gt         = pickle.load(gt_file)
# pprint.pprint(trajectory)

trajs_file.close()

print "loaded ", len(trajectory), "trajector(y|ies)"

for i in range(len(trajectory)):
  for x, y, z in trajectory[i]:
    print x, y, z

