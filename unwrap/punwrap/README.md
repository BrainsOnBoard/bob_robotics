# pixpro-unwrap
A simple command line tool to unwrap panoramic photos and videos from Kodak PixPro cameras, written in C++ and using OpenCV. The aim is to create a simple command-line tool that can unwrap panoramic photographs and videos taken with a Kodak PixPro camera.

This program requires OpenCV as a dependency. If you want to copy the audio track when unwrapping a video you also need ffmpeg installed.

To build, run:
```
make
sudo make install
```

Then you can run:
```
pixpro-unwrap *.JPG
pixpro-unwrap *.MP4
```
to unwrap photos and videos, respectively.
