.PHONY: all clean

CC = gcc
CFLAGS = -O2 -DSHM -DHZ=100 -Iinclude

GSL_LIBS = -lgsl -lgslcblas
TARGET = planar
SRC = src/planargl.c

UNAME_S := $(shell uname -s)

ifeq ($(UNAME_S),Linux)
    OPENGL_LIBS = -lglut -lGLU -lGL
    XLIBS = -lXext -lXmu -lXi -lX11
endif

ifeq ($(UNAME_S),CYGWIN_NT-10.0)
    OPENGL_LIBS = -lglut32 -lglu32 -lopengl32
    XLIBS =
endif

LIBS = $(OPENGL_LIBS) $(XLIBS) $(GSL_LIBS) -lm

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $^ $(LIBS) -o $@

clean:
	rm -f $(TARGET) *.o

