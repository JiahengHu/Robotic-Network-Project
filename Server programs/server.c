/* read in input from socket,
   draw pictures through pixel using the input as parameters,
   save the input into mysql database 
   Arthur: Jiaheng Hu */
#include <stdio.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdbool.h>
#include "ppmIO.h"
#include <math.h>
#include <stdlib.h>
#include <my_global.h>
#include <mysql.h>
#include <time.h>
#include <string.h>

//write the current timestamp in the given format(SQL format) into the buffer string
char *timestamp(char *buffer)
{
    time_t ltime; /* calendar time */
    ltime=time(NULL); /* get current cal time */
    
    strftime(buffer,80,"%Y-%m-%d %H:%M:%S",localtime(&ltime));
    return buffer;
}

//function to handle error
void finish_with_error(MYSQL *con)
{
  fprintf(stderr, "%s\n", mysql_error(con));
  mysql_close(con);
  exit(1);        
}

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

//create a pixel object which can be drawn
Pixel *pix_createFromMap(Map *map) {
    Pixel *src;
    src = (Pixel *) malloc(map->rows * map->cols * sizeof(Pixel));

    map_setPix(map, src);

    return src;
}

//function to hanle error
void error(char *msg)
{
    perror(msg);
    exit(1);
}

//draw the robot on the given position
//use the value as color
//write the picture to the given filename
void draw_robot( Map *map, char *filename, unsigned char *value, float *position){
	Pixel *pix = pix_createFromMap(map);
	float x = position[0];
	float y = position[1];
	float t = position[2];
	float x0 = x + cos(t);
	float y0 = y + sin(t);
	map_line(map, x, y, x0, y0, pix, (Pixel){value[0],value[1],value[2]});	
	map_circle(map, x, y, 10, pix, (Pixel){value[0],value[1],value[2]});
	writePPM(pix, map->rows, map->cols, 255, filename);
	free(pix);
}

int main(int argc, char *argv[])
{

/*******************************setting up the server**************************/
     int sockfd, newsockfd, portno, clilen;
     char buffer[256];
     struct sockaddr_in serv_addr, cli_addr;
     int n;
     if (argc < 2) {
         fprintf(stderr,"ERROR, no port provided\n");
         exit(1);
     }
     sockfd = socket(AF_INET, SOCK_STREAM, 0);
     if (sockfd < 0) 
        error("ERROR opening socket");
     bzero((char *) &serv_addr, sizeof(serv_addr));
     portno = atoi(argv[1]);
     serv_addr.sin_family = AF_INET;
     serv_addr.sin_addr.s_addr = INADDR_ANY;
     serv_addr.sin_port = htons(portno);
     if (bind(sockfd, (struct sockaddr *) &serv_addr,
              sizeof(serv_addr)) < 0) 
              error("ERROR on binding");



/*******************************building SQL connections**************************/
  MYSQL *con = mysql_init(NULL);
  char input[80];
  char time_buffer[80];
  if (con == NULL) 
  {
      fprintf(stderr, "%s\n", mysql_error(con));
      exit(1);
  }

  if (mysql_real_connect(con, "localhost", "jhu20", "yuio3fzkd", 
          "yingli", 0, NULL, 0) == NULL) 
  {
      finish_with_error(con);
  }

/*****************************set up the map and preparing for drawing************/
	char file[] = "map-test2-crop.pgm";
	//map-edited600x600
	//the map that you want to read

	Map *map=map_read(file);
	unsigned char value[3] = {255, 0, 0};
	char picture[] = "result.ppm";
float *output;
float position[3];

/*****************************receiving the data******************************/

while(true){
     printf("server running on kili\n");  
     listen(sockfd,5);
     clilen = sizeof(cli_addr);
     newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
     if (newsockfd < 0) 
          error("ERROR on accept");
     printf("connected to the client\n");
while(true){
bzero(buffer,256);
     n = read(newsockfd,buffer,255);
     if (n < 0) error("ERROR reading from socket");
     if(buffer[0]==0 && buffer[1]==0 && buffer[2]==0)
	break;  
     if (buffer[0] == 's' && buffer[1] == 't'){
		printf("start\n");

//get the message and save it to a field called output
for(int i=0;i<3;i++){
     bzero(buffer,256);
     n = read(newsockfd,buffer,255);
     if (n < 0) error("ERROR reading from socket");
     output = (float *)buffer;
	
     printf("Here is the message: %f\n",*output);
     position[i]=*output;
}


//now that the position has been calculated, draw the picture
draw_robot(map, picture, value, position);

/*****************************save the data into SQL database******************************/
  int id=58;//how to get the actual id here?
  bzero(time_buffer,80);
  bzero(input,80);
  sprintf(input,"INSERT INTO robotLocation (robotID,x,y,t,time) VALUES(%d,%f,%f,%f,'",id,position[0],position[1],position[2]);
  strcat(input,timestamp(time_buffer));
  strcat(input,"')");
  printf("inputstring: %s\n",input);
  


  if (mysql_query(con, input)) {
      finish_with_error(con);
  }
}
}
}
/*****************************the end of the loop*************/
     free(map);
     printf("after free map\n");
     mysql_close(con);
     printf("after free con\n");

     return 0; 
}
