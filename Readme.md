This repo contains the basic radiation plotting tool for use with the Radiation plugin found here: https://github.com/EEEManchester/gazebo_radiation_plugin . 

To get it working with the repo you have you will need to:

1. Install the radiation layer in the catkin workspace where the radiation plugin exists. 

2. In the radiation_demonstrator launch file for the radiation plugin you will find a launch file commented out called radmap.launch. This needs uncommenting.

3. Replace the radmap_params.yaml file in radiation plugin's launch/params/ folder with the one from this repo.

4. Run the radiation_demonstrator launch file.

5. Add the radiation layer in rviz (it should be of map type) and see the data appear.

This is currently just a simple spreading of the radiation within a radius around where the measurement was taken. We are currently working on a smarter version of this with some fancier ways of combining the data, but it is not ready for wider release yet. 
