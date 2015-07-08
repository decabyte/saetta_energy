# ActionDispatch.msg
#   message for dispatching an action to task nodes
#
# The action dispatch message is used by the mission executor or an high level autonomy module. It carries info about
# a single (atomical) action. It is sent through the action dispatch topic where task nodes are listening for commands.
#
# Fields not marked as optional are mandatory.
#
#   id          =   can be set to 0 to broadcast a system signal (e.g. emergency stop), normal nodes should reject the message if id = 0
#   duration    =   can be set to 0 to signal a timeless action (e.g. switching things on or off)
#

# action commands
uint8 ACTION_START=0                       # resume action after being paused
uint8 ACTION_PAUSE=1                       # pause the action
uint8 ACTION_ABORT=2                       # cancel the action

# action feedback
uint8 FEED_SINGLE=0
uint8 FEED_STREAM=1

# message body
Header header                                   # header field used for keep track of timing

uint32 id                                       # progressive numerical id for the action
uint8 command                                   # command to trigger on an existing action given its id
uint8 feedback                                  # type of feedback (e.g. single / streaming)
string name                                     # fully qualified name of the action (or signal if id = 0)

float64 timeout                                 # (optional) expected duration in seconds
diagnostic_msgs/KeyValue[] params               # (optional) key-value pairs of action parameters (str:str)
