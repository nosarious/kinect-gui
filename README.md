# kinect-gui
uses ofxKinect, ofxOpenCV, ofxGui and ofxDelaunay

This example has been modified to make use of two things. The first is that this example uses ofxGui for some of the settings, instead of relying on the keyboard. This program also uses the masking routine to determine which points to include in the pointcloud. Allowing you to 'remove' portions of the camera view by depth.

This example also demonstrates an offset variable for the step value. In the original example one would use a grid from the depth camera for placing points in the pointcloud. This would cause a very structured appearance for the cloud which may not be appealing for the visualizations it is being applied to. BY using the step pattern, the grid is less apparent and, personally, it seems more visually appealing.

In this image you see the grid pattern with a normal offset.
![alt tag](https://raw.githubusercontent.com/nosarious/kinect-gui/master/dotsWithoutOffset.png)

In this image you see the pattern for the depth cloud with an offset for the step value.
![alt tag](https://raw.githubusercontent.com/nosarious/kinect-gui/master/dotsWithOffset.png)

You can see the dot pattern with the offset is more reminiscent of the ben-day dot pattern used in 4-color comics, or paintings by Roy Lichtenstein. What I have noticed is that there is less flickering of a Kinect image when using this method. I haven't the foggist idea why.


This patterning works very well if one was going to use the opintcloud as a triangular mesh, as done with the ofxDelaunay addon. This addon will take the traditional pointcloud and generate a triangular mesh. With the traditional grid system for the step value, one may get triangles which are awkward in some locations, adding to a potential distraction in the visualization. If you use the step offset, however, you are using points that are already in a less-square grid format, you may get better triangles.

This is an exampe of how a traditional triangulation may look without the offset, and with the offset. As I mentioned, the offset seems to generate a more stable pattern of triangles than the triangles made with a grid. ou can see this in hte middle part of the next two images, where there are fewer awkward triangles generated.
![alt tag](https://raw.githubusercontent.com/nosarious/kinect-gui/master/sketch.jpg)

This is an image of a masked and triangulized point cloud without the step offset. All points are read in a grid. Notice the middle image.
![alt tag](https://raw.githubusercontent.com/nosarious/kinect-gui/master/withoutStepOffset.png)

This is an image of a masked and triangulized point cloud with the step offset. Odd lines are offset. Notice the middle image.
![alt tag](https://raw.githubusercontent.com/nosarious/kinect-gui/master/withSetOffset.png)
