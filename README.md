Download Link: https://assignmentchef.com/product/solved-ttk4255-homework4-linear-algorithms
<br>
For general information about the assignments, including grading critera and how to get help, consult the assignments.pdf document on BB. To get your assignment approved, you only need to complete 60% (weighting is next to each task). Upload the requested answers and figures as a single PDF. You don’t need to submit your code. You may use any convenient tool to create your report.

<h1>About the assignment</h1>

Several estimation algorithms in computer vision are called “linear algorithms”. These are typically computationally cheap and can provide an estimate of the quantity of interest without an initial guess. On the other hand, the estimate is usually not optimal in a geometrically meaningful sense, and may therefore be suboptimal in the presence of noise. Because of this, linear algorithms are most often used to obtain a rough initial estimate, possibly in combination with RANSAC, which is then further refined by non-linear optimization of a geometric and/or probabilistic objective function.

In this assignment you will learn how to derive linear algorithms using the direct linear transform, and implement an algorithm to estimate the pose of a planar object from 2D-3D point correspondences. As a simple test case, you will apply the algorithm to a paper sheet with fiducial markers. These markers have known positions on the paper and are designed to be easily detected and uniquely identified, which simplifies the problem of establishing correspondences.

Figure 1: Output from the algorithm you will implement, showing the estimated object coordinate frame, along with the detected and predicted location of the markers.

<h1>Background</h1>

Linear algorithms appear several places in Szeliski (2010), including 6.1 (Feature-based alignment), 7.1 (Triangulation) and 7.2 (Two-frame structure from motion). The pose estimation algorithm in this assignment is not presented in Szeliski, however its derivation and implementation is similar to other linear algorithms in the book. For the curious, the algorithm here is based on the camera calibration paper by Zhengyou Zhang, which has had a heavy influence on the development of calibration software:

I Zhengyou Zhang. A Flexible New Technique for Camera Calibration. 2000. <a href="https://www.microsoft.com/en-us/research/wp-content/uploads/2016/02/tr98-71.pdf">(link)</a>

Linear algorithms are based on solving linear systems of equations. A good overview of this topic can be found in Appendix 5 (Least-squares minimization) of Multiple View Geometry by Hartley and Zisserman, which is available on Blackboard.

The first step of the pose estimation algorithm is to estimate a homography between the object plane and the image. Homographies (or projective transformations) are briefly introduced in Szeliski 2.1.2. A more detailed treatment can be found in Hartley and Zisserman, but is not necessary for completing this assignment. In summary, a homography is a mapping between two (homogeneous) 2D coordinates:

<em>x</em><strong>˜</strong><sup>0 </sup>= <strong>H˜</strong><em>x                                                                      </em>(1)

where <strong>H </strong>is an arbitrary 3 × 3 matrix. Besides the perspective image of a planar object, some other transformations that can be described by a homography is the mapping between two images of a planar scene and between two images taken by a rotating camera.

The assignments and projects in this course are copyrighted by Simen Haugo (<a href="/cdn-cgi/l/email-protection" class="__cf_email__" data-cfemail="a0c8c1d5c7cf8ed3c9cdc5cee0c7cdc1c9cc8ec3cfcd">[email protected]</a>) and may not be copied, redistributed, reused or repurposed without written permission.

<h1>Part 1       Transformation between a plane and its image</h1>

Consider a perspective image of a planar object. Without loss of generality, let points on the object have coordinates of the form <strong>X </strong>= (<em>X,Y,</em>0), arbitrarily placing the object plane at <em>Z </em>= 0. The pixel coordinates of a given point on the object are

<em>u </em>= <em>c<sub>x </sub></em>+ <em>s<sub>x</sub>fX<sup>c</sup>/Z<sup>c             </sup></em>(2) <em>v </em>= <em>c<sub>y </sub></em>+ <em>s<sub>y</sub>fY <sup>c</sup>/Z<sup>c          </sup></em>(3)

where

(4)

is the point transformed from object to camera coordinates by <strong>R </strong>and <strong>t</strong>—which we refer to as the “pose”. For convenience, we define the <em>calibrated image coordinates x </em>= (<em>x,y</em>) as

<em>x </em>:= (<em>u </em>− <em>c<sub>x</sub></em>)<em>/s<sub>x</sub>f       </em>(5) <em>y </em>:= (<em>v </em>− <em>c<sub>y</sub></em>)<em>/s<sub>y</sub>f      </em>(6)

orgenerally<em>x</em><strong>˜ </strong>= <strong>K</strong><sup>−1</sup><strong>u˜</strong>foran arbitraryintrinsicmatrix<strong>K</strong>,whichcan bethoughtofas“camera-agnostic” coordinates. Combining the above, we find the following non-linear relationship between points on the object (<em>X,Y </em>) and calibrated image coordinates (<em>x,y</em>):

<em>,                                                    </em>(7)

(8)

<table width="643">

 <tbody>

  <tr>

   <td width="624">If we use homogeneous coordinates, this relationship can be written in the linear form</td>

   <td width="19"> </td>

  </tr>

  <tr>

   <td width="624">      <em>x</em>˜ <em>X r</em>11 <em>r</em>12 <em>t</em><em>x</em><em>y</em>˜ = <strong>H</strong><em>Y </em> where <strong>H </strong>= <em>r</em><sub>21 </sub><em>r</em><sub>22 </sub><em>t<sub>y</sub></em>      <em>z</em>˜ 1 <em>r</em>31 <em>r</em>32 <em>t</em><em>z</em></td>

   <td width="19">(9)</td>

  </tr>

 </tbody>

</table>

with <em>x</em><strong>˜ </strong>= (<em>x,</em>˜ <em>y,</em>˜ <em>z</em>˜) being the homogeneous form of (<em>x,y</em>).

<strong>Task 1.1: </strong>(5%) We say that a homography is “unique up to a scaling factor” because any non-zero scalar multiple of the homography matrix defines the same mapping between the 2D coordinates (after dehomogenization). Show that this is the case for the matrix <strong>H </strong>in Eq. (9).

<strong>Task 1.2: </strong>(5%) A general 3 × 3 homography has eight degrees of freedom (nine arbitrary entries minus scale ambiguity). However, explain why <strong>H </strong>as defined in Eq. (9) has fewer than eight degrees of freedom, i.e. why the entries of <strong>H </strong>are more restricted than in an arbitrary homography.

<h1>Part 2        The direct linear transform</h1>

The direct linear transform (DLT) is a general technique for transforming a set of non-linear equations into a mathematically equivalent <em>linear </em>system that can be solved more easily. Here we will use the DLT to estimate <strong>H </strong>from a set of correspondences (<em>x<sub>i</sub>,y<sub>i</sub></em>) &#x2194; (<em>X<sub>i</sub>,Y<sub>i</sub></em>). Note that although <strong>H </strong>as defined in Eq. (9) is more restricted, the algorithm here can be used to estimate an arbitrary homography.

The key idea of the DLT is to rearrange Eq. (7)-(8) by multiplying by the denominator on both sides, such that the pair of equations becomes

<table width="466">

 <tbody>

  <tr>

   <td width="440">(<em>r</em>31<em>X</em><em>i </em>+ <em>r</em>32<em>Y</em><em>i </em>+ <em>t</em><em>z</em>)<em>x</em><em>i </em>= <em>r</em>11<em>X</em><em>i </em>+ <em>r</em>12<em>Y</em><em>i </em>+ <em>t</em><em>x</em><em>,</em></td>

   <td width="27">(10)</td>

  </tr>

  <tr>

   <td width="440">(<em>r</em>31<em>X</em><em>i </em>+ <em>r</em>32<em>Y</em><em>i </em>+ <em>t</em><em>z</em>)<em>y</em><em>i </em>= <em>r</em>21<em>X</em><em>i </em>+ <em>r</em>22<em>Y</em><em>i </em>+ <em>t</em><em>y</em><em>.</em></td>

   <td width="27">(11)</td>

  </tr>

 </tbody>

</table>

Notably, these equations are <em>linear </em>in the elements of <strong>H</strong>. Hence, if we collect all the unknowns that we wish to estimate into a vector, e.g.

h                                                                 i<em><sup>T</sup></em>

<table width="643">

 <tbody>

  <tr>

   <td width="477">                                                             <strong>h </strong>= <em>r</em>11 <em>r</em>12 <em>t</em><em>x                      </em><em>r</em>21 <em>r</em>22 <em>t</em><em>y                   </em><em>r</em>31 <em>r</em>32 <em>t</em><em>z</em>then you may verify that Eq. (10)-(11) can be written as the linear system</td>

   <td width="140"><em>,</em></td>

   <td width="27">(12)</td>

  </tr>

  <tr>

   <td width="477"><strong>A</strong><em><sub>i</sub></em><strong>h </strong>= <strong>0</strong>where</td>

   <td width="140"> </td>

   <td width="27">(13)</td>

  </tr>

 </tbody>

</table>

<strong>A</strong><em> .                               </em>(14)

Each point correspondence gives rise to one pair of equations. By vertically stacking the equations from <em>n </em>correspondences, we obtain a linear system <strong>Ah </strong>= <strong>0</strong>, where <strong>A </strong>is a 2<em>n</em>×9 matrix. This is called a <em>homogeneous </em>system. Unlike inhomogeneous systems (e.g. <strong>Ah </strong>= <strong>b </strong>where <strong>b </strong>6= <strong>0</strong>), homogeneous systems always have the trivial solution <strong>h </strong>= <strong>0</strong>, which is of no interest. In order to have a non-trivial solution, <strong>A </strong>must have a null-space of dimension one or higher. There will then be an infinite number of solutions given by linear combinations of the null-space vectors. If the null-space is 1-dimensional, then there is a non-trivial solution that is also unique up to a scaling factor. (This reflects the scale ambiguity you showed in Task 1.1, and will be addressed in Part 3.)

The null-space can be made 1-dimensional by stacking the equations from <em>n </em>≥ 4 correspondences. If <em>n </em>= 4, then there is always a unique (up to scale) exact non-trivial solution, as long as no three points on either side are collinear. If <em>n &gt; </em>4, then the system is over-determined, and will generally not have an exact solution apart from <strong>h </strong>= <strong>0 </strong>unless the point locations are free of noise. In absence of an exact solution, we normally seek the “best” solution in the least-squares sense

min||<strong>Ah</strong>||<sub>2 </sub>subject to ||<strong>h</strong>||<sub>2 </sub>= 1                                                 (15)

<strong>h</strong>

where we arbitrarily impose the scale constraint ||<strong>h</strong>||<sub>2 </sub>= 1 to avoid the trivial solution. The leastsquares solution <strong>h </strong>can be obtained from the singular value decomposition (SVD) of <strong>A </strong>= <strong>UΣV</strong><em><sup>T </sup></em>as the column of <strong>V </strong>corresponding to the smallest singular value. (A short proof of this is in section A5.3 of the Hartley and Zisserman chapter on Blackboard.)

<table width="642">

 <tbody>

  <tr>

   <td width="642">The following data is included in this assignment:–     K.txt: Camera intrinsic matrix <strong>K</strong>.–     XY.txt: Markers’ paper coordinates (<em>X,Y </em>).–     image0000.jpg…image0022.jpg: Image sequence.–     detections.txt: Detected markers (one row per image). Each row contains 24 tuples of the form (<em>d<sub>i</sub>,u<sub>i</sub>,v<sub>i</sub></em>), where <em>d<sub>i </sub></em>= 1 if the <em>i</em>’th marker was detected and (<em>u<sub>i</sub>,v<sub>i</sub></em>) are its detected pixel coordinates. Note that only one corner of each marker is used.The main.py/m script has some helper code for loading the data and generating the requested figures. Stub functions are provided in equivalently-named files (Matlab) orin common.py (Python).</td>

  </tr>

 </tbody>

</table>

<strong>Task 2.1: </strong>(25%) Implement estimate_H. This should build the matrix <strong>A</strong>, solve for the vector <strong>h </strong>and reshape the entries of <strong>h </strong>into the 3×3 matrix <strong>H</strong>. Compute the predicted marker locations using <strong>H </strong>and run the main script, which should visualize the marker locations as in Fig. 1. (You will also get a 3D plot visualizing the camera and the object, but this will not work until Task 3.) Check that the predicted locations are close to the detected locations on all images and include the figure for image number 4.

Tip: Remember to convert the detected marker locations into calibrated image coordinates for use in estimate_H. These can be computed as <em>x</em><strong>˜</strong><em><sub>i </sub></em>= <strong>K</strong><sup>−1</sup><strong>u˜</strong><em><sub>i</sub></em>, where <strong>u˜</strong><em><sub>i </sub></em>= (<em>u<sub>i</sub>,v<sub>i</sub>,</em>1). After estimating <strong>H</strong>, the predicted marker locations can be computed using Eq. (9), followed by conversion from calibrated image coordinates back to pixel coordinates: <strong>u˜</strong><em><sub>i,</sub></em><sub>predicted</sub> = <strong>K˜</strong><em>x</em><em><sub>i,</sub></em><sub>predicted</sub>. Note that either conversion produces a homogeneous 3-vector, which must be divided by the last component (and sliced) to obtain the actual 2D coordinates that we measure in the image.

<strong>Task 2.2: </strong>(10%) The distance between a marker’s detected and predicted location is its <em>reprojection error</em>. It is measured in pixels and is defined as the Euclidean distance between pixel coordinates:

q

<em>e</em><em>i </em>= ||<strong>u</strong><em>i </em>− <strong>u</strong><em>i,</em>predicted||2 =         (<strong>u</strong><em>i </em>− <strong>u</strong><em>i,</em>predicted)<em>T </em>(<strong>u</strong><em>i </em>− <strong>u</strong><em>i,</em>predicted)                           (16)

where <strong>u</strong><em><sub>i </sub></em>are the detected pixel coordinates of the <em>i</em>’th marker and <strong>u</strong><em><sub>i,</sub></em><sub>predicted </sub>are computed as above. Modify your script to compute the average, minimum and maximum reprojection error over all markers in each image. Include the numbers for image number 4 in your writeup. The average reprojection error should be less than 1 pixel on this image.

<strong>Task 2.3: (Optional self-study task – 0%) </strong>The matrices <strong>K </strong>and <strong>H </strong>define a mapping from the object plane to the image. When this mapping is invertible, it is possible to go from pixel coordinates back to object coordinates. Use this to extract the texture of the object, i.e. the pattern of markers, into its own image. This is also called a “perspective-free” or “fronto-parallel” image and is highly useful for image processing. For example, parallel lines remain parallel, and circles and other shapes are preserved.

<h1>Part 3      Recover the pose</h1>

We can recover the pose (<strong>R </strong>and <strong>t</strong>) of the planar object by observing from Eq. (9) that <strong>H </strong>contains the translation vector and two of the rotation matrix columns. The last column of the rotation matrix is not present, but if we know any two columns, the third can always be obtained by the cross product, e.g.

<strong>r</strong><sub>3 </sub>= <strong>r</strong><sub>1 </sub>× <strong>r</strong><sub>2                                                                                                                                    </sub>(17)

where <strong>r</strong><em><sub>i </sub></em>= (<em>r</em><sub>1<em>i</em></sub><em>,r</em><sub>2<em>i</em></sub><em>,r</em><sub>3<em>i</em></sub>) is the <em>i</em>’th column of <strong>R</strong>. However, recall that the solution from Part 2 is only unique up to a scaling factor. (We chose the scale ||<strong>h</strong>||<sub>2 </sub>= 1 arbitrarily.) This means that the entries in the solved-for <strong>H </strong>are generally not equal to the rotation matrix and translation vector elements. Instead, there is an unknown scaling factor <em>k </em>such that

<table width="463">

 <tbody>

  <tr>

   <td width="80"><em>h</em>11<strong>H </strong>= <em>h</em><sub>21</sub><em>h</em>31</td>

   <td width="34"><em>h</em>12 <em>h</em>22 <em>h</em>32</td>

   <td width="107"><em>h</em>13    <em>r</em>11 <em>h</em>23 = <em>k </em><em>r</em>21        <em>h</em>33                            <em>r</em>31</td>

   <td width="32"><em>r</em>12 <em>r</em>22 <em>r</em>32</td>

   <td width="183"><em>t<sub>x</sub></em> <em>t</em><em>y</em><em>.</em><em>t<sub>z</sub></em></td>

   <td width="27">(18)</td>

  </tr>

 </tbody>

</table>

We can determine the scaling factor <em>k </em>by imposing the constraint that the columns of the rotation matrix should be of unit length. Hence,

<em>.                                      </em>(19)

Because we can’t recover the sign of <em>k</em>, there are two possible poses (corresponding to +|<em>k</em>| or −|<em>k</em>|).

<strong>Task 3.1: </strong>(15%) Implement decompose_H. It should take in the result from estimate_H and return its two possible pose decompositions as 4×4 transformation matrices. If you run the main script, the generated figure should now draw the object coordinate frame into the image and visualize the camera and the object in 3D. Include the figure for both possible poses on image number 4.

<strong>Task 3.2: </strong>(10%) Only one of the poses is physically plausible in the sense that it places the object in front of the camera. Specifically, every detected marker should have a positive <em>Z </em>coordinate when transformed into the camera frame. Use this criterion to automatically choose the correct pose. Verify that the correct pose is chosen on all images, and include the figure for image number 4, 5 and 21.

<strong>Task3.3: </strong>(10%) Due to noise,the matrixformedby<strong>r</strong><sub>1</sub>,<strong>r</strong><sub>2 </sub>and<strong>r</strong><sub>3 </sub>maynotexactlysatisfythe properties of a rotation matrix. In Appendix C of his paper (see Blackboard), Zhang suggests to replace this matrix with the “closest” valid rotation matrix in the sense of the Frobenius norm. Read Appendix C and implement the function closest_rotation_matrix. Modify decompose_H using this function to return a valid rotation matrix for both poses.

Describe the properties of a rotation matrix, and suggest a way to numerically quantify how well the properties are satisfied by a given 3 × 3 matrix. Quantify how well the properties are satisfied by the rotation matrix of the chosen pose, with and without the above correction, on image number 4.

<h1>Part 4       Derive your own linear algorithm</h1>

Sometimes we have partial information about the pose, either through other sensors or by assumptions about the physically achievable motions of our robot. For example, aerial vehicles can often provide two rotational degrees of freedom from a gyroscope and accelerometer, and, if the ground is flat, one translational degree of freedom from a laser range finder. In such cases, we can derive specialized linear algorithms which may require fewer point correspondences than in the general case.

<strong>Task 4.1: </strong>(10%) Suppose you have a set of 2D-3D point correspondences <strong>u</strong><em><sub>i </sub></em>&#x2194; <strong>X</strong><em><sub>i </sub></em>between an image and a general object (not necessarily planar), which satisfy the pinhole model:

<strong>u˜</strong><em><sub>i </sub></em>= <strong>K</strong>(<strong>RX</strong><em><sub>i </sub></em>+ <strong>t</strong>)<em>,i </em>= 1<em>…n.                                                       </em>(20)

Assume that<strong>K </strong>and<strong>t </strong>are bothknown andshow(using the DLT) that<em>n </em>correspondences can be combined into a linear system of equations <strong>Am </strong>= <strong>b</strong>, where both <strong>A </strong>∈ R<sup>2<em>n</em>×9 </sup>and <strong>b </strong>∈ R<sup>2<em>n </em></sup>are derived purely from known variables or constants, and <strong>m </strong>contains the unknown entries of <strong>R</strong>.

<strong>Task 4.2: </strong>(5%) Is the system homogeneous or inhomogeneous?

<strong>Task 4.3: </strong>(5%) Suggest a way to solve the system for <strong>m </strong>and recover <strong>R</strong>.