#gcc -o gs gs.c $(pkg-config --cflags --libs gstreamer-1.0)
gcc -o gs gs.c $(pkg-config --cflags --libs gstreamer-1.0)
