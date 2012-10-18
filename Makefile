HEADERS = BrushConfigUI.h correlation.h FltDesignUI.h HelpPageUI.h ImageLib/FileIO.h ImageLib/Image.h ImageLib/RefCntMem.h ImgFilterUI.h ImgFltAux.h imgflt.h ImgView.h iScissor.h PriorityQueue.h ScissorPanelUI.h
CPPS = BrushConfigUI.cpp correlation.cpp FltDesignUI.cpp HelpPageUI.cpp ImageLib/FileIO.cpp ImageLib/Image.cpp ImageLib/RefCntMem.cpp ImgFilterUI.cpp ImgFltAux.cpp ImgFltMain.cpp ImgView.cpp iScissor.cpp ScissorPanelUI.cpp
OBJS = PanoramaMain.o BlendImages.o FeatureAlign.o FeatureSet.o WarpSpherical.o
IMAGELIB=ImageLib/libImage.a


UNAME := $(shell uname)
# Flags for Linux
ifeq ($(UNAME), Linux)
LIBS = -L$(HOME)/local/lib -lfltk_images -lpng -ljpeg -lfltk -L/usr/X11R6/lib -lX11 -L/usr/lib/fltk-1
CFLAGS = -g -I/usr/include/fltk-1 
endif
# Flags for OSX
ifeq ($(UNAME), Darwin)
LIBS = `fltk-config --libs` -framework Cocoa
CFLAGS =  -L$(PNGLIB_PATH) -g -lfltk_images -lpng -ljpeg  `fltk-config --cxxflags `

# Set this variable to the directory where libpng is installed. The
# one bellow is where MacPorts usually puts things, if you used some other
# package manager (e.g. brew) or installed libpng from source you will
# have to modify it.
PNGLIB_PATH=/opt/local/lib 
endif

all: Panorama

Panorama: $(OBJS) $(IMAGELIB)
	g++ -o $@ $(CFLAGS) $(LIBS) $(OBJS) $(IMAGELIB)

PanoramaMain.o: PanoramaMain.cpp
	g++ -c PanoramaMain.cpp $(CFLAGS) $(LIBS)

$(IMAGELIB): 
	make -C ImageLib

clean:
	rm -rf Panorama *.o */*.o *~

.PHONY: clean


