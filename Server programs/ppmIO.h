#ifndef PPMIO_H

#define PPMIO_H

typedef struct {
  unsigned char r;
  unsigned char g;
  unsigned char b;
} Pixel;

typedef struct {
	int rows;
	int cols;
	int intensities;
	float gridSize;
	float origin[2];
	float size[2];

	unsigned char *grid;
} Map;

Pixel *readPPM(int *rows, int *cols, int * colors, char *filename);
void writePPM(Pixel *image, int rows, int cols, int colors, char *filename);

unsigned char *readPGM(int *rows, int *cols, int *intensities, char *filename);
void writePGM(unsigned char *image, long rows, long cols, int intensities, char *filename);


#endif
