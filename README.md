# Path Extrusion Tool for Blender
### Overview
To use this tool, select one polygon curve and another polygon path. The selected curve is extruded along the polygon path. This is achieved by extruding the loop towards each point on the path. The pivot of the curve is translated to the points on the path. The orientation is such that the normal vector of the best-fit plane of the curve is parallel to the path at all points on the path except the first point, for which the original orientation of the curve is used. When the path takes sharp turns, mitering is accomplished by scaling the loop in a manner dependent on the angle of the path.

<img src ="images/ExtrudePath1.jpg" width = "900">

### Technical Details
#### Definitions
The curve (the first polygon to be selected) consists of connected points. For the purposes of the following discussion, the points are considered 3-dimensional column vectors. The points and pivots are named as shown below, as an example.

<img src ="images/ExtrudePath2.jpg" width = "500">

The path also consists of connected points, named below as an example.

<img src ="images/ExtrudePath3.jpg" width = "400">

The curve matrix is defined as the matrix where each point on the curve is a column.

<img src ="images/ExtrudePath8.jpg" width = "200">

#### Translation
The translation vectors define how the curve is translated along the path. First the pivot point is translated to the location of the first point in the curve. Thus the first translation vector is <img src ="images/ExtrudePath10.jpg" width = "80">. For all subsequent translations, the vector is simply the difference between the next point on the path and the point immediately before it <img src ="images/ExtrudePath11.jpg" width = "65">.

#### Rotation

First, the initial normal vector for the curve is determined by finding the normal vector of the plane that best fits all the points on the curve. This is accomplished by first finding the eigenvalues and eigenvectors of the matrix <img src ="images/ExtrudePath9.jpg" width = "35">.

<img src ="images/ExtrudePath4.jpg" width = "400">

The eigenvectors are shown in green in the image above. The eigenvector corresponding to the eigenvalue with the smallest absolute value is the normal vector <img src ="images/ExtrudePath12.jpg" width = "20">.

All subsequent normal vectors are calculated as follows

<img src ="images/ExtrudePath14.jpg" width = "350">

This corresponds to the average of the normalized directions of lines that connect to the vertex of interest.

<img src ="images/ExtrudePath5.jpg" width = "350">

The rotation angle is calculated as

<img src ="images/ExtrudePath18.jpg" width = "200">

The rotation axis is defined as

<img src ="images/ExtrudePath15.jpg" width = "150">

Which is the vector perpendicular to the initial and final normal vectors.

<img src ="images/ExtrudePath6.jpg" width = "350">


#### Scaling
In addition to translation and rotation, scaling is performed during the extrusion process to prevent pinching of the resulting surface at sharp corners. The result is mitered corners.

<img src ="images/ExtrudePath7.jpg" width = "500">

The angle of the corner is calculated as

<img src ="images/ExtrudePath16.jpg" width = "350">

And the scaling factor, (corner width relative to the edge width), is

<img src ="images/ExtrudePath17.jpg" width = "200">


