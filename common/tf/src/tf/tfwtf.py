from roswtf.rules import warning_rule, error_rule

#def roswtf_plugin_static(ctx):
#    for r in tf_warnings:
#        warning_rule(r, r[0](ctx), ctx)
#    for r in tf_errors:
#        error_rule(r, r[0](ctx), ctx)
        
import tf.msg

import rospy

_msgs = []

def _tf_handle(msg):
    _msgs.append((msg, rospy.get_rostime(), msg._connection_header['callerid']))

def rostime_delta(ctx):
    deltas = {}
    for m, stamp, callerid in _msgs:
        for t in m.transforms:
            d = t.header.stamp - stamp
            secs = d.to_seconds()
            if abs(secs) > 1.:
                if callerid in deltas:
                    if abs(secs) > abs(deltas[callerid]):
                        deltas[callerid] = secs
                else:
                    deltas[callerid]  = secs

    errors = []
    for k, v in deltas.iteritems():
        errors.append("receiving transform from [%s] that differed from ROS time by %ss"%(k, v))
    return errors
            
def reparenting(ctx):
    errors = []
    parent_id_map = {}
    for m, stamp, callerid in _msgs:
        for t in m.transforms:
            frame_id = t.header.frame_id
            if frame_id in parent_id_map and parent_id_map[frame_id] != t.parent_id:
                msg = "reparenting of [%s] to [%s] by [%s]"%(frame_id, t.parent_id, callerid)
                parent_id_map[frame_id] = t.parent_id
                if msg not in errors:
                    errors.append(msg)
            else:
                parent_id_map[frame_id] = t.parent_id
    return errors

import roslib.scriptutil
def no_msgs(ctx):
    # check to see if tf_message is being published, then warn if there are no messages
    if not _msgs:
        master = roslib.scriptutil.get_master()
        if master is not None:
            code, msg, val = master.getPublishedTopics('/roswtf', '/')
            if code == 1:
                if filter(lambda x: x[0] == '/tf_message', val):
                    return True

tf_warnings = [
  (no_msgs, "No tf messages"),
  (rostime_delta, "Received out-of-date/future transforms:"),  
]
tf_errors = [
  (reparenting, "TF re-parenting contention:"),
]

def roswtf_plugin_online(ctx):
    print "running tf checks, this will take a second..."
    sub = rospy.Subscriber('/tf_message', tf.msg.tfMessage, _tf_handle)
    rospy.sleep(1.0)
    sub.unregister()
    print "... tf checks complete"    

    for r in tf_warnings:
        warning_rule(r, r[0](ctx), ctx)
    for r in tf_errors:
        error_rule(r, r[0](ctx), ctx)

    
