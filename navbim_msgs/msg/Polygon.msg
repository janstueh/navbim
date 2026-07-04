# Polygon with holes - for rooms with internal obstacles
# Compatible with Boost.Geometry polygon model (outer + inners)

geometry_msgs/Polygon outer      # Exterior boundary (counter-clockwise)
geometry_msgs/Polygon[] holes    # Interior holes (clockwise, can be empty)
