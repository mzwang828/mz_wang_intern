geometry_msgs/Point position   # coordinate of the node on the map matrix, x for row and y for column
int32 F   # F = G+H, total cost
int32 G   # cost from start to current node
int32 H   # heuristic cost for current node