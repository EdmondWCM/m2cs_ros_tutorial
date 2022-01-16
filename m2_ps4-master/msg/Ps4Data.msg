float32 hat_lx # left = positive
float32 hat_ly # up = positive
float32 l2_analog # range [0, 1]
float32 hat_rx # left = positive
float32 hat_ry # up = positive
float32 r2_analog # range [0, 1]
float32 dpad_x # left => dpad_x = 1.0, right => dpad_x = -1.0
float32 dpad_y # up => dpad_y = 1.0, down => dpad_y = -1.0

bool cross
bool circle
bool triangle
bool square
bool l1
bool r1
bool l2 # l2 is True iff l2_analog > 0
bool r2 # r2 is True iff r2_analog > 0
bool share
bool options
bool ps
bool l3 # left hat
bool r3 # right hat
bool tpad_click # deprecated, always False
