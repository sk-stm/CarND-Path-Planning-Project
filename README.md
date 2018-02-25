# CarND-Path-Planning-Project

## The code model for generating paths is described in detail. This can be part of the README or a separate doc labeled "Model Documentation".

The path-generation is basically taken from the walk-through video:
* a start point for planning is chosen (car-state of end-point of previous path)
* from the starting point, three Frenet-points are chosen in 30, 60, and 90m distance ahead and with wanted lane as given
* a cubic spline is formed with the chosen points (start point + Frenet points)
* the new path extends the old one by sampling the spline

The behavioral decision making is achieved by cost functions as suggested in the lecture.