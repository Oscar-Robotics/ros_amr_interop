# Corridor boundaries for a VDA5050 edge (v2.1.0, §6.6.5)
# Defines the area within which the vehicle may deviate from its trajectory for obstacle avoidance.

float64 left_width           # [m] Allowed deviation to the left of the edge trajectory
float64 right_width          # [m] Allowed deviation to the right of the edge trajectory
string corridor_ref_point    # Enum: "KINEMATICCENTER" | "CONTOUR"
                             # Defines whether boundaries apply to the kinematic center or
                             # the vehicle contour. Default (if empty): "KINEMATICCENTER"

bool release_required        # Flag that indicates whether the robot shall request approval from fleet control.

string release_loss_behavior # Enum { 'STOP' , 'RETURN' }
                             # Defines how the robot shall behave in the case of either its release of a corridor
                             # expiring or the release being revoked by the fleet control.
                             # 'STOP': Mobile robot shall stop and await manual intervention. 'RETURN': Mobile robot
                             # shall return to the predefined trajectory of the edge it deviated from
                             # Default: 'STOP'.

# Enum fo corridorRefPoint
string KINEMATICCENTER = "KINEMATICCENTER"
string CONTOUR = "CONTOUR"

# Enum for releaseLossBehavior
string STOP = "STOP"
string RETURN = "RETURN"