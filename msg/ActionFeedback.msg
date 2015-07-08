# ActionFeedback.msg
#   message for action feedback from task nodes
#
# The action feedback message is used by task nodes to broadcast the action state. It carries information about
# a single action with a given id. It is sent through the action feedback topic where high level modules are listening
# for feedback about the requested actions.
#
# Fields not marked as optional are mandatory.
#
#   id          =   can be set to 0 to broadcast a system signal (e.g. emergency stop), normal nodes should reject the message if id = 0
#   duration    =   can be set to 0 to signal a timeless action (e.g. switching things on or off)
#

# action states
uint8 ACTION_IDLE=0                         # task node is idle
uint8 ACTION_RUNNING=1                      # task node is executing the action
uint8 ACTION_PAUSED=2                       # action has been paused (by user or by task node)
uint8 ACTION_SUCCESS=3                      # action has been completed
uint8 ACTION_TIMEOUT=4                      # action exceeded the provided duration
uint8 ACTION_FAILED=5                       # action failed to complete (e.g. goal can't be achieved)
uint8 ACTION_ABORT=6                        # action has been cancelled (by user or by executor node)
uint8 ACTION_REJECT=7                       # action has been rejected by task node (e.g. another action is in progress)

# message body
Header header                               # header field used for keep track of timing

uint32 id                                   # progressive numerical id for the action
uint8 status                                # action status at current time (header.stamp)
string name                                 # fully qualified name of the action

float64 duration                            # (optional) action real duration (measured by the task node)
diagnostic_msgs/KeyValue[] info             # (optional) extra information about the feedback (e.g. reason for status)
