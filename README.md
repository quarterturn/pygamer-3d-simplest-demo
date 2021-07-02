# PyGamer 3D Simplest Demo
![Alt text](3d-bunny.JPG?raw=true "Stanford Bunny")


This is a demo of doing 3D graphics on an Adafruit PyGamer board. It makes use of a frambuffer and canvas so that screen writes can be done all at once via DMA. This makes it fast and also allows using the Painter's Algorithm to hide hidden polygons via depth sorting.

The 3D code is based heavily on https://github.com/michaelerule/Uno9341TFT. To make things easy to adapt and understand, the 3D code is rewritten as simple functions, vs a class inheriting a specific TFT driver library.

The .stl to .h converter has been included along with the demo files. It has been hackily modified to scale to numpy.uint16 so that more complex objects can be used. There's no need to recenter and scale in the code now, but I've left it in for now. 

#  Note
Using DMA reverses the byte order to the display, so __builtin_bswap16 function has to be used to swap the RGB565 color information. If you don't do this you'll wonder why your colors are all wrong!

# To Do
Implement hidden-line removal as shown here https://github.com/osresearch/papercraft/blob/master/hiddenwire.c to improve the look of the wireframe rendering mode.

Re-write 3D code as a class usable with Adafruit Arcada.
