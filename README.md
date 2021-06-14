# PyGamer 3D Simplest Demo

This is a demo of doing 3D graphics on an Adafruit PyGamer board. It makes use of a frambuffer and canvas so that screen writes can be done all at once via DMA. This makes it fast and also allows using the Painter's Algorithm to hide hidden polygons via depth sorting.

The 3D code is based heavily on https://github.com/michaelerule/Uno9341TFT. To make things easy to adapt and understand, the 3D code is rewritten as simple functions, vs a class inheriting a specific TFT driver library.

#  Note
Using DMA reverses the byte order to the display, so __builtin_bswap16 function has to be used to swap the RGB565 color information. If you don't do this you'll wonder why your colors are all wrong!

# To Do
Implement hidden-line removal as shown here https://github.com/osresearch/papercraft/blob/master/hiddenwire.c to improve the look of the wireframe rendering mode.

Re-write 3D code as a class usable with Adafruit Arcada.
