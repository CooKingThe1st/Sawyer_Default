As following the tutorial from theconstructsim.com would not work on 16.04 ROS Kinetic,
after struggling for a long time and bug fixing, realized that u gotta use the moveit_packages
legacy and not to create a new one by moveit_configure.
So to sum up, this is the configuration that worked on Ubuntu 16.04 ROS kinetic, as least on my 
computer and I want to share it.

First change is the setup, which should include 5 directories: intera_common, intera sdk, and 3 sawyer_plugin.
It's worth to mention that you have to copy the intera.sh in intera_common to the catkin_root_directory and open
new virtual env every session.

here is the changes:

+ intera common and intera sdk is the original stuff
+ however, sawyer_description in sawyer_robot is the big deal
  gotta change the urdf to work
+ sawyer_moveit have a slightly change from the original too
  which is the IK_Algo changed to TRAC_IK
+sawyer_simulator is the original

my demo video is here btw: https://drive.google.com/file/d/1WcHs4PNcC2_zBVuFIL_E50r0kQ1pmwVg/view?usp=sharing
