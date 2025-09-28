float64 timestamp   # motor state is at this time
int32 id            # motor id
int32 goal          # commanded position (in encoder units)
int32 position      # current position (in encoder units)
int32 error         # difference between current and goal positions
int32 voltage       # current voltage (mv)
