### Regulatory Element definition ###
int64 id
string type
float64 height                  # meters
float64 width                   # meters
float64 length                  # meters
Transform pose                  # 3D Centroid x,y,z (Map frame)
float64 distance                # meters (To ego-vehicle)
bool occupied                   # fill up by decision-making