# pointcloud_extraction
This repo contains code to interface with the ZED 2i stereo camera and process its output to obtain a point cloud. The code uses the ZED SDK to capture and rectify stereo images, and then applies a depth computation algorithm to obtain the 3D point cloud. The point cloud data is then processed to extract specific features or objects of interest, and displayed in RViz for visualization and analysis.

## parameter for changing 
Parameter              | Description                         |   Hotkey
-----------------------|-------------------------------------|-------------------------------------------------
Save Side by Side      | Save side by side image.            |   's'                             
Save Depth             | Save depth image.                   |   'p'                              
Save Point Cloud       | Save 3D point cloud.                |   'd'
Switch cloud format    | Toggle between point cloud formats. |   'm'
Switch depth format    | Toggle between depth image formats. |   'n'                                                      
Exit                   | Quit the application.               |   'q'
