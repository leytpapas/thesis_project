#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <math.h>

// Heat dissapation constant
#define K 0.25

// ============================================================================
// Scalar version
// ============================================================================

// One time step
void step(int** map, int** mapOut, int width, int height)
{
  for (int y = 1; y < height-1; y++) {
    for (int x = 1; x < width-1;x++) {
      mapOut[y][x] = ((sqrt((map[y][x]>>24)*(map[y][x]>>24) + (map[y][x]>>16)*(map[y][x]>>16) + (map[y][x]>>8)*(map[y][x]>>8)))/40);
      x++;
      mapOut[y][x] = ((sqrt((map[y][x]>>24)*(map[y][x]>>24) + (map[y][x]>>16)*(map[y][x]>>16) + (map[y][x]>>8)*(map[y][x]>>8)))/40);
      x++;
      mapOut[y][x] = ((sqrt((map[y][x]>>24)*(map[y][x]>>24) + (map[y][x]>>16)*(map[y][x]>>16) + (map[y][x]>>8)*(map[y][x]>>8)))/40);
      x++;
      mapOut[y][x] = ((sqrt((map[y][x]>>24)*(map[y][x]>>24) + (map[y][x]>>16)*(map[y][x]>>16) + (map[y][x]>>8)*(map[y][x]>>8)))/40);
    }
  }
}

// ============================================================================
// Main
// ============================================================================

int main()
{
  // Parameters
  const int WIDTH  = 1280;
  const int HEIGHT = 720;
  const int NSPOTS = 10;
  const int NSTEPS = 1;

  // Timestamps
  timeval tvStart, tvEnd, tvDiff;

  // Allocate
  int* map       = new int [WIDTH*HEIGHT];
  int* mapOut    = new int [WIDTH*HEIGHT];
  int** map2D    = new int* [HEIGHT];
  int** mapOut2D = new int* [HEIGHT];

  // Initialise
  for (int i = 0; i < WIDTH*HEIGHT; i++) map[i] = mapOut[i] = 0.0;
  for (int i = 0; i < HEIGHT; i++) {
    map2D[i]    = &map[i*WIDTH];
    mapOut2D[i] = &mapOut[i*WIDTH];
  }

  // Inject hot spots
  srand(0);
  for (int i = 0; i < NSPOTS; i++) {
    int t = rand() % 256;
    int x = 1 + rand() % (WIDTH-2);
    int y = 1 + rand() % (HEIGHT-2);
    map2D[y][x] = (int) 1000*t;
  }

  // Simulate
  gettimeofday(&tvStart, NULL);
  for (int i = 0; i < NSTEPS; i++) {
    step(map2D, mapOut2D, WIDTH, HEIGHT);
    int** tmp = map2D; map2D = mapOut2D; mapOut2D = tmp;
  }
  gettimeofday(&tvEnd, NULL);
  timersub(&tvEnd, &tvStart, &tvDiff);

  // Display results
  printf("P2\n%i %i\n255\n", WIDTH, HEIGHT);
  for (int y = 0; y < HEIGHT; y++)
    for (int x = 0; x < WIDTH; x++) {
      int t = (int) map2D[y][x];
      t = t < 0   ? 0 : t;
      t = t > 255 ? 255 : t;
      // printf("%d\n", t);
    }
 
  // Run-time of simulation
  printf("# %ld.%06lds\n", tvDiff.tv_sec, tvDiff.tv_usec);

  return 0;
}
