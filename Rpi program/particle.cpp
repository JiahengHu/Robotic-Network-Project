/*
	
	Jiaheng Hu
	Bruce A. Maxwell
	
	Functions for managing a particle filter

	This is intended for localization, it is not a general-purpose PF

	The map is set up so that the (x, y, theta) are to be interpreted as
	a right-handed coordinate system with x positive to the right and y
	positive up.

	The map indexes, of course, are in image coordinates, which are x
	positive to the right and y positive down.  

	map_get takes in world x,y coordinates and returns the map value at
	that location

	map_getraw takes in image coordinates

	map_xy2pix converts from world coordinates to image coordinates.
*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include "ppmIO.h"
#include "particle.h"
#define pie 3.14159265358979323846
#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

static void map_xy2pix( Map *map, float x, float y, int *col, int *row );

// read a map from a PGM file
Map *map_read(char *filename) {
	Map *map;

	map = (Map *)malloc(sizeof(Map));
	if(!map) {
		printf("map_read: unable to malloc map\n");
		return(NULL);
	}

	map->grid = readPGM( &(map->rows), &(map->cols), &(map->intensities), filename );
	if( !map->grid ) {
		printf("map_read: unable to read map %s\n", filename);
		free(map);
		return(NULL);
	}

	// need to read these from the yaml file
	map->gridSize = 0.05;
	map->origin[0] = -10.0;
	map->origin[1] = 10.0; // convert the map to Cartesian coords so (x, y) in world space is x right, y up
	map->size[0] = map->cols * map->gridSize;
	map->size[1] = map->rows * map->gridSize;

	return( map );
}

// free all malloc'd memory in the map structure
void map_free( Map *map ) {
	if( map ) {
		if( map->grid )
			free(map->grid);
		free(map);
	}
}

/*
	Takes in an (x, y) in world coords and returns the corresponding pixel value
 */
int map_get( Map *map, float x, float y ) {
	int r, c;
	map_xy2pix( map, x, y, &c, &r );
	return( map->grid[ r * map->cols + c ] );
}

/*
	Takes in the raw row, column values
 */
int map_getraw( Map *map, int c, int r ) {
	return( map->grid[ r * map->cols + c ] );
}

/*
	Takes in an (x, y) in world coords and sets the corresponding pixel value
 */
void map_set( Map *map, float x, float y, unsigned char val ) {
	int r, c;
	map_xy2pix( map, x, y, &c, &r );
	map->grid[ r * map->cols + c ] = val;
}

/*
	Takes in an (x, y) in world coords and returns the corresponding row/col of the map
 */
static void map_xy2pix( Map *map, float x, float y, int *col, int *row ) {
	*col = (int)((x - map->origin[0]) / map->gridSize  + 0.5);
	*row = (int)((map->origin[1] - y) / map->gridSize  + 0.5);
}

// copy the map data over to the pixels
void map_setPix( Map *map, Pixel *pix ) {
	int i;
	
	for(i=0;i<map->rows*map->cols;i++) {
		pix[i].r = pix[i].g = pix[i].b = map->grid[i];
	}
}

/*
  Generates numbers from a Gaussian distribution with zero mean and
  unit variance.

  Concept and code from NRC

  Modified to use drand48() and return doubles 
*/
double gaussDist() {
  static int iset=0;
  static double gset;
  float fac,rsq,v1,v2;

  // generate a new pair of numbers and return the first
  if(iset == 0) {
    do {
      v1 = 2.0*drand48()-1.0;
      v2 = 2.0*drand48()-1.0;
      rsq = v1*v1+v2*v2;
    } while (rsq >= 1.0 || rsq == 0.0);

    fac = sqrt(-2.0*log(rsq)/rsq);
    gset = v1*fac;
    iset = 1;

    return(v2*fac);
  } 

  // return the last number we generated
  iset = 0;

  return(gset);
}

// takes in two (x, y) locations in world coords and draws a line in the src image
// draws a line that includes the two endpoint pixels
// draws into src using the color value
void map_line( Map *map, float xf0, float yf0, float xf1, float yf1, Pixel *src, Pixel value) {
	int x0, y0, x1, y1;
	int x, y;
	int p;
	int dx, dy;
	int twoDx, twoDy;
	int xstep, ystep;
	int i;

	map_xy2pix( map, xf0, yf0, &x0, &y0 );
	map_xy2pix( map, xf1, yf1, &x1, &y1 );
	
	if(x0 < 0 || x0 >= map->cols || x1 < 0 || x1 >= map->cols || y0 < 0 || y0 >= map->rows || y1 < 0 || y1 >= map->rows)
		return;
	
	dx = x1 - x0;
	dy = y1 - y0;
	x = x0;
	y = y0;
	xstep = dx < 0 ? -1 : 1;
	ystep = dy < 0 ? -1 : 1;
	

	// horizontal and vertical lines
	if(dx == 0) {
		if(dy == 0) {
			src[y*map->cols + x] = value;
			return;
		}
		for(; y != y1;y+=ystep) {
			src[y*map->cols + x] = value;
		}
		return;
	}
	if(dy == 0) {
		for(;x!=x1;x+=xstep) {
			src[y*map->cols + x] = value;
		}
		return;
	}

	twoDx = abs(dx*2);
	twoDy = abs(dy*2);
	
	if( twoDx > twoDy ) {
		p = twoDy - abs(dx);
		for(;x!=x1;x+=xstep) {
			src[y*map->cols + x] = value;
			if(p > 0) {
				y += ystep;
				p -= twoDx;
			}
			p += twoDy;
		}
	}
	else {
		p = twoDx - abs(dy);
		for(;y!=y1;y+=ystep) {
			src[y*map->cols + x] = value;
			if(p > 0) {
				x += xstep;
				p -= twoDy;
			}
			p += twoDx;
		}
	}
	
	return;
}

//draw a circle at the given point in the src image
//use size as the radius
//using the value as the color of the circle
void map_circle( Map *map, float xf, float yf, int size, Pixel *src, Pixel value) {
	int x, y;
	map_xy2pix( map, xf, yf, &x, &y );
	if(x < 0 || x >= map->cols || y < 0 || y >= map->rows)
		return;
	if (size < 0)
		size *= -1;
	for(int i=0; i<size; i++){
		for(int k=0;k<size; k++){
			if(k*k+i*i<=size*size){
				src[(y+k)*map->cols + x+i] = value;
				src[(y+k)*map->cols + x-i] = value;
				src[(y-k)*map->cols + x+i] = value;
				src[(y-k)*map->cols + x-i] = value;
			}			
		}	
	}
	
	return;
}

// sensor model
// returns likelihod that the actual sensor reading matches the expected sensor reading
// uses a Gaussian centered on the expected sensor reading with stdev sigma
double pf_sensorModel( float actual_sensor, float expected_sensor, float sigma ) {
	const double divisor = sqrt( 2.0 * M_PI );
	const double diff = actual_sensor - expected_sensor;
	//printf("divisor: %f, diff: %f\n, result: %f\n",divisor,diff, (1.0 / (sigma * divisor)) * exp( -0.5 * (diff*diff) / (sigma * sigma) ));
	return( (1.0 / (sigma * divisor)) * exp( -0.5 * (diff*diff) / (sigma * sigma) ) );
}

// calc the expected value of the laser at the given angle (rad) at position p
// this needs fixing
float pf_calcExpected( ParticleFilter *pf, Particle *p, float angle, Map *map, Pixel *test ) {
	int x0;
	int y0;
	float a;
	float dx;
	float dy;
	float tx, ty;
	int x, y;
	int i;
	const int maxDistance = (int)(4.0/0.05);
	
	// calculate the start location in pixels
	tx = p->x + pf->dlaser[0] * cos(p->t) - pf->dlaser[1] * sin(p->t);
	ty = p->y + pf->dlaser[0] * sin(p->t) + pf->dlaser[1] * cos(p->t);

	if(tx < map->origin[0] || tx >= map->origin[0] + map->size[0] ||
		 ty > map->origin[1] || ty <= map->origin[1] - map->size[1]) {
		//printf("out of bound\n"); 
		return(0.0);
	}
		 
	map_xy2pix( map, tx, ty, &x0, &y0 );
// 	printf("x: %f, y: %f\n, obValue: %d\n",pf->p[0].x,pf->p[0].y, map_getraw( map, x0, y0 ));
// 	printf("x0: %d, y0: %d\n",x0, y0 );

	if( map_getraw( map, x0, y0 ) < 250 ) { // unknown or inside an obstacle, discard particle
// 		printf("x0: %d, y0: %d\n",x0, y0 );
// 		printf("obstacle\n"); 
		return(0.0);
		
	}

	// calculate the angle
	a = p->t + angle;

	// calculate the gradients, these are in pixel space on the map
	// use a negative angle to adjust from LHS to RHS on the map
	dx = cos(-a);
	dy = sin(-a);

	x = 0;
	y = 0;
	if( fabs(dx) > fabs(dy) ) { // step in x
		for(i=0;i <maxDistance;i++) {
			x = dx >= 0.0 ? i : -i;
			y = (int)((fabs(dy)/fabs(dx)) * i +  0.5);
			y = dy >= 0.0 ? y : -y;

			if( x0 +x < 0 || x0+x >= map->cols || y0+y < 0 || y0+y >= map->rows ) {
				//printf("dont know but x\n"); 
				return(0.0);
			}

			if(test != NULL) {
				test[(y0+y)*map->cols + (x0+x)].r = 255;
				test[(y0+y)*map->cols + (x0+x)].g = 10;
				test[(y0+y)*map->cols + (x0+x)].b = 10;
			}

			if( map_getraw( map, x0+x, y0+y ) < 20 ) { // obstacle
				return( sqrt( x*x + y*y ) * map->gridSize*1000 );
			}
		}
	}
	else { // step in y
		for(i=0;i<maxDistance;i++) {
			y = dy >= 0.0 ? i : -i;
			x = (int)((fabs(dx) / fabs(dy)) * i + 0.5);
			x = dx >= 0.0 ? x : -x;

			if( x0 +x < 0 || x0+x >= map->cols || y0+y < 0 || y0+y >= map->rows ) {
				//printf("dont know but y\n"); 
				return(0.0);
			}

			if(test != NULL) {
				test[(y0+y)*map->cols + (x0+x)].r = 255;
				test[(y0+y)*map->cols + (x0+x)].g = 10;
				test[(y0+y)*map->cols + (x0+x)].b = 10;
			}

			if( map_getraw( map, x0+x, y0+y ) < 20) { // obstacle
				return( sqrt( x*x + y*y ) * map->gridSize*1000 );
			}
		}
	}
	// no obstacle along the line so return 4000
	//printf("no obstacle along the line\n"); 
	return(4000.0);
}

//create a set of random points
ParticleFilter *pf_create(int maxN, float pRandom){
	ParticleFilter *filter;
	filter = (ParticleFilter *)malloc(sizeof(ParticleFilter));

	//Particle *particle;
	filter->p = (Particle *)malloc(maxN*sizeof(Particle));

	//filter->p = particle;
	filter->MaxN = maxN;
	filter->pRandom=pRandom;
	
	return (filter);
}

//free the memory space
void pf_free(ParticleFilter *pf){
	if(pf){
		if(pf->p)
			free(pf->p);
		free(pf);
	}
}

//init the particle filter
int pf_init(ParticleFilter *pf, Map *map, int N){
	srand (time(NULL));
	pf->N = N;
	pf->dlaser[0]=0.05;
	pf->dlaser[1]=0;
	pf->sigma_s=200;
	pf->sigma_v=0.5;
	pf->sigma_w=0.5;
	//printf("before the loop\n");
	for(int i=0;i<N;i++){
		//printf("N:%d\n",i);
		pf->p[i].x=(float)rand()/(float)RAND_MAX*(map->size[0])-10.0;
		pf->p[i].y=-(float)rand()/(float)RAND_MAX*(map->size[1])+10.0;
		pf->p[i].t=(float)rand()/(float)RAND_MAX*pie;
		
		int x0,y0;
		float tx,ty;
		tx = pf->p[i].x + pf->dlaser[0] * cos(pf->p[i].t) - pf->dlaser[1] * sin(pf->p[i].t);
		ty = pf->p[i].y + pf->dlaser[0] * sin(pf->p[i].t) + pf->dlaser[1] * cos(pf->p[i].t);
		map_xy2pix( map, pf->p[i].x, pf->p[i].y, &x0, &y0 );
		
		while( map_getraw( map, x0, y0 ) < 250){
			pf->p[i].x=(float)rand()/(float)RAND_MAX*(map->size[0])-10.0;
			pf->p[i].y=-(float)rand()/(float)RAND_MAX*(map->size[1])+10.0;
			tx = pf->p[i].x + pf->dlaser[0] * cos(pf->p[i].t) - pf->dlaser[1] * sin(pf->p[i].t);
			ty = pf->p[i].y + pf->dlaser[0] * sin(pf->p[i].t) + pf->dlaser[1] * cos(pf->p[i].t);
			map_xy2pix( map, pf->p[i].x, pf->p[i].y, &x0, &y0 );
 		}	
 		
 		//printf("x0: %d, y0: %d\n",x0, y0 );
		
		pf->p[i].pi=1.0/N;
		//printf("x: %f, y: %f\n, obValue: %d\n",pf->p[i].x,pf->p[i].y, map_getraw( map, x0, y0 ));
	}
	
	return 0;
}


//move the points in the filter
void pf_motion( Particle *p, float v, float w, float dt, float sigma_v, float sigma_w ){
	p->t=p->t+w*dt*(1+sigma_w*(drand48()*2-1))+v*dt*(drand48()*2-1)*0.17453;
	p->x=p->x+v*dt*cos(p->t)*(1+sigma_v*(drand48()*2-1));
	p->y=p->y+v*dt*sin(p->t)*(1+sigma_v*(drand48()*2-1));
}		

//update the probability based on sensor reading
void pf_sensor( ParticleFilter *pf, Particle *p, float *sensor, int Nsensor, Map *map ){
	double prob=0;
	for(int i=0;i<Nsensor;i++){
		float angle=i*pie/2.0-1.0/2.0*pie;	
		float distance=pf_calcExpected(pf,p, angle, map, NULL);
		//printf("distance: %f\n",  distance);
		prob+=pf_sensorModel(sensor[i],distance, pf->sigma_s);
		//printf("adding: %f\n",  pf_sensorModel(sensor[i],distance, pf->sigma_s));
	}
	//printf("Prob: %f\n", prob);
	//printf("sigma:%f\n",pf->sigma_s);
	prob/=Nsensor;		
	p->pi=prob;	
}	

//Normalize the probability of the particles
void normalize(ParticleFilter *pf){
	double p=0;
	for(int i = 0; i < pf->N; i++){
		p+=pf->p[i].pi;
	}
	
	for(int i = 0; i < pf->N; i++){
		pf->p[i].pi/=p;
	}
}	

//resample the particles
void pf_resample( ParticleFilter *pf, int N ){
	float sum[pf->N];
	
	for(int i =0; i < pf->N; i++){
		if(i!=0){
			sum[i]=pf->p[i].pi+sum[i-1];
		}
		else{
			sum[i]=	pf->p[i].pi;
		}	
		//printf("sum: %f\n", sum[i]);
		//printf("pi: %f\n", pf->p[i].pi);
	}
	
	Particle *particle;
	particle = (Particle *)malloc(pf->MaxN*sizeof(Particle));
	
	
	for(int i = 0; i < N; i++){
		float num=(float)rand()/(float)RAND_MAX*sum[pf->N-1];
		//printf("random: %f\n", num);
		//printf("random: %f\n", sum[pf->N-1]);
		int count=0;
		while(sum[count]<num){
			count++;
		}
		particle[i]=pf->p[count];
		
		
	}
	
	free(pf->p);
	pf->p=particle;
	
	pf->N=N;
	normalize(pf);
}	

//iterate the updating steps
void pf_iterate( ParticleFilter *pf, float v, float w, float dt, float *sensor, int Nsensor, Map *map ){
	for(int i=0;i<pf->N;i++){
		pf_motion(&(pf->p[i]),v,w,dt,pf->sigma_v, pf->sigma_w);
	}
//	printf("pi motion: %f\n", pf->p[0].pi);


	for(int i=0;i<pf->N;i++){
		pf_sensor(pf,&(pf->p[i]),sensor,Nsensor, map);
	}
// 	for (int j = 0 ; j < 10; j++){
// 			printf("x %d: %f\n", j,pf->p[j].x);
// 			printf("y %d: %f\n", j,pf->p[j].y);
// 			printf("pi %d: %f\n", j,pf->p[j].pi);
// 	}



	pf_resample(pf,pf->N);
	//printf("pi resample: %f\n", pf->p[0].pi);
}

//could be used to return the max particle's index
// int get_max(ParticleFilter *pf){
// 	float max=0;
// 	int maxN=0;
// 	for(int i=0;i<pf->N;i++){
// 		if(pf->p[i].pi>max){
// 			max = pf->p[i].pi;
// 			maxN = i;
// 		}
// 	}	
// 	return maxN;
// }	

//return N different particles with the highest probability
Particle* get_max(ParticleFilter *pf, int N){
	if(N<1)
		return NULL;
	
	Particle *p = new Particle[N];
	
	//initialize P
	for(int i=0;i<N;i++){
		p[i] = pf->p[0];
	}	
	
	//set the first particle to the biggest value
	for(int k=N; k <pf->N; k++){
		if(pf->p[k].pi > p[0].pi){
			p[0]=pf->p[k];
		}	
	}	
	
	//find the Nth biggest value
	for(int i=1;i<N;i++){
		for(int k=0; k <pf->N; k++){
			if(pf->p[k].pi > p[i].pi && pf->p[k].pi < p[i-1].pi){
				p[i]=pf->p[k];
			}	
		}
	}	
	return p;
}	

//create a pixel object which can be drawn
Pixel *pix_createFromMap(Map *map) {
    Pixel *src;
    src = (Pixel *) malloc(map->rows * map->cols * sizeof(Pixel));

    map_setPix(map, src);

    return src;
}

//draw the N highest probability particles onto the map
void pf_drawMaxState( ParticleFilter *pf, Map *map, char *filename, int N ){
	unsigned char value[3] = {255, 0, 0};
	Pixel *pix = pix_createFromMap(map);
	Particle* max = get_max(pf, N);
	for (int i = 0; i< N;i++){
		float x = max[i].x;
		float y = max[i].y;
		float t = max[i].t;
		float x0 = x + 3*cos(t);
		float y0 = y + 3*sin(t);
		printf("x when painting: %f\n", x);
		printf("y when painting: %f\n", y);
		printf("t when painting: %f\n", t);
		printf("x0 when painting: %f\n", x0);
		printf("y0 when painting: %f\n", y0);
		map_line(map, x, y, x0, y0, pix, (Pixel){value[0],value[1],value[2]});
	
	}	
	writePPM(pix, map->rows, map->cols, 255, filename);
	free(pix);
	free(max);
}	

// read in the laser reading and velocity command from the designated file
void read_file( float *sensor, int Nsensor, char *filename, float *velocity){
	FILE *fp;
	fp = fopen(filename,"r");
	float data[Nsensor + 2];	
	for (int j=0; j<Nsensor+2; j++){
		fscanf(fp, "%f", &data[j]);
	}	

	printf("Laser reading: \n");
	for (int j=0; j<Nsensor; j++){
		sensor[j] = data[j];
// 		if(fabs(sensor[j]-4000)<0.1)
// 			sensor[j]=0;
		//this serves to eliminate the 4000 reading

		printf("sensor reading:%f ", sensor[j]);
	}	
	printf("\n");

	for (int j=0; j<2; j++){
		velocity[j] = data [j+Nsensor];
		printf("velocity[%d]:%f \n", j, velocity[j]);
	}
	fclose(fp);
}

//display the error message
void error(char *msg)
{
    perror(msg);
    exit(0);
}
			
int main(int argc, char *argv[]){
	/********************************************set-up process******************************/
	srand (time(NULL));
	srand48(time(NULL));
	ParticleFilter *pf = pf_create(5000,0.2);
	char file[] = "map-test2-crop.pgm";
	//map-edited600x600
	//the map that you want to read

	char data[] = "test.csv";
	//the file you want to read data from

	Map *map=map_read(file);
	
	//the file that enables the program to conmunicate with the python bluetooth program
	
	pf_init(pf,map,3000);
	//should be 3000
	//pf_init(pf,map,10);
	
	int Nsensor =3;
	float *sensor= new float [Nsensor];
	float *velocity= new float [2];

	
	Particle *max;
	float position[3];


    /******************************************socket set-up process******************************/
    int sockfd, portno, n;

    struct sockaddr_in serv_addr;
    struct hostent *server;

    char buffer[256];
    if (argc < 3) {
       fprintf(stderr,"usage %s hostname port\n", argv[0]);
       exit(0);
    }
    portno = atoi(argv[2]);
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");
    server = gethostbyname(argv[1]);
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *)&serv_addr,sizeof(serv_addr)) < 0) 
        error("ERROR connecting");

    printf("successfully connected to the server\n");
    usleep(3500000);//waiting for other programs to set up

	/******************************************iteration process******************************/
	while (1==1){
		
		read_file(sensor, Nsensor, data, velocity);
		
		//should be reading in velocity, time, sensor reading, print out the current position
		pf_iterate(pf, velocity[0], velocity[1], 1.0, sensor, Nsensor, map);
		//the time here is a matter of discussion		

		max = get_max(pf,1);
		position[0]=max[0].x;
		position[1]=max[0].y;
		position[2]=max[0].t;
		
		n = write(sockfd,"start",4);
    			if (n < 0) 
        			error("ERROR writing to socket");
		
		for (int j = 0 ; j < 3; j++){
			printf("position %d: %f\n", j,position[j]);
			n = write(sockfd,(char *)&position[j],4);
    			if (n < 0) 
        			error("ERROR writing to socket");
			usleep(70000);
		}

		usleep(900000);
		//add a delay statement that controls the time
	}

	
	//***********************************finish the loop and end the program******************************/
	free(max);	
	printf("end of particle filter\n");
	pf_free(pf);
	free(map);
	free(sensor);
	return 0;
}	

