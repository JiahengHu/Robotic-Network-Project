/*
	Bruce A. Maxwell
	Spring 2017
	CS 363 Project 6

	Include file for map functions

	Also includes sample Particle and ParticleFilter structs and the
	function prototypes I created for the task
 */
#ifndef PARTICLE_H
#define PARTICLE_H

typedef struct {
	float x;
	float y;
	float t;
	double pi;
} Particle;

typedef struct {
	Particle *p;     // particle array
	int N;           // number of active particles
	int MaxN;        // size of the particle array
	float pRandom;   // % of new random particles to create each iteration

	// robot noise
	float sigma_v;   // trans velocity noise (m/s)
	float sigma_w;   // angle velocity noise (rad/s)

	// sensor noise and data
	float dlaser[2]; // x and y offsets for the laser relative to the robot
	float sigma_s;   // sensor noise (m)
	int nReadings; // number of sensor readings to use when assigning probs

} ParticleFilter;

typedef struct {
	int rows;
	int cols;
	int intensities;
	float gridSize;
	float origin[2];
	float size[2];

	unsigned char *grid;
} Map;

double gaussDist(void);

Map *map_read(char *filename);
void map_free( Map *map );
int map_get( Map *map, float x, float y );
int map_getraw( Map *map, int c, int r );
void map_set( Map *map, float x, float y, unsigned char val );
void map_line( Map *map, float xf0, float yf0, float xf1, float yf1, Pixel *src, Pixel value);

ParticleFilter *pf_create(int maxN, float pRandom);
void pf_free(ParticleFilter *pf);
int pf_init(ParticleFilter *pf, Map *map, int N);
void pf_motion( Particle *p, float v, float w, float dt, float sigma_v, float sigma_w );
double pf_sensorModel( float actual_sensor, float expected_sensor, float sigma );
float pf_calcExpected( ParticleFilter *pf, Particle *p, float angle, Map *map, Pixel *test );
void pf_sensor( ParticleFilter *pf, Particle *p, float *sensor, int Nsensor, Map *map );
void pf_resample( ParticleFilter *pf, int N );
void pf_iterate( ParticleFilter *pf, float v, float w, float dt, float *sensor, int Nsensor, Map *map );
void pf_drawMaxState( ParticleFilter *pf, Map *map, char *filename, int N );
void normalize(ParticleFilter *pf);
#endif
