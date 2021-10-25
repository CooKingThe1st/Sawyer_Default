import sys
from colorthief import ColorThief

img_file = ('/home/ros/ros_ws/src/planning/scripts/tmp_hand.jpg')
color_thief = ColorThief(img_file)
# print(color_thief.get_color(quality=1))
color_palette = color_thief.get_palette(quality=1)
# rgb_color = 4 #no cube
for d in color_palette:
    if (d[0] >= 2 * d[1] and d[0] >= 2 * d[2]): 
        sys.exit(0) # red
    if (d[1] >= 2 * d[0] and d[1] >= 2 * d[2]): 
        sys.exit(2) # green
    if (d[2] >= 2 * d[0] and d[2] >= 2 * d[1]): 
        sys.exit(1) # blue
sys.exit(4)