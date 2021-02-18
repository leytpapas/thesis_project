#include <QPULib.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <math.h>

// Heat dissapation constant
#define K 0.25

// ============================================================================
// Vector version
// ============================================================================

double fastPow(double a, double b)
{
    union
    {
        double d;
        int x[2];
    } u = {a};
    u.x[1] = (int)(b * (u.x[1] - 1072632447) + 1072632447);
    u.x[0] = 0;
    return u.d;
}

void step(Ptr<Int> map, Ptr<Int> mapOut, Int pitch, Int width, Int height)
{
  mapOut = mapOut + index() + (me() << 4);
  map = map + index() + (me() << 4);

  Int r=0,g=0,b=0;
  Int pixOut=0;
  Int temp=0;
  For (Int y = me(), y < height, y=y+numQPUs())

    // Point p to the output row
    Ptr<Int> out = mapOut + y*pitch;
    Ptr<Int> in = mapOut + y*pitch;
    // Initilaise three cursors for the three input rows
    // for (int i = 0; i < 3; i++) row[i].init(map + i*pitch);
    // for (int i = 0; i < 3; i++) row[i].prime();
    gather(in);
    in = in + 16;
    gather(in);
    in = in + 16;
    gather(in);
    // Pre-fetch three vectors 

    // Compute one output row
    For (Int x = 0, x < width, x=x+16)
      //Big endian 32-24=R 24-16=B 16-8=G -> for RGB images
      // Pre-fetch three vectors for the *next* iteration
      gather(in);
      in = in + 16;
      gather(in);
      in = in + 16;
      gather(in);
      
      // Receive vectors for *this* iteration
      receive(r);
      receive(g);
      receive(b);
      //store
      temp = r*r+g*g+b*b;
      store(temp, out);
      out = out + 16;
      in = in + 16;
    End
    // Move to the next input rows
    map = map + pitch*numQPUs();
  End
}

// ============================================================================
// Main
// ============================================================================

int main()
{
  // Size of 2D heat map is WIDTH*HEIGHT:
  //   * with zero padding, it is NROWS*NCOLS
  //   * i.e. there is constant cold at the edges
  //   * NCOLs should be a multiple of 16
  //   * HEIGHT should be a multiple of NQPUS
  const int NQPUS  = 4;
  const int WIDTH  = 1280;
  // const int WIDTH  = 640;
  const int NCOLS  = WIDTH;
  const int HEIGHT = 720;
  // const int HEIGHT = 480;
  const int NROWS  = HEIGHT;
  const int NSPOTS = 10;
  const int NSTEPS = 100;

  // Timestamps
  timeval tvStart, tvEnd, tvDiff;

  // Allocate and initialise input and output maps
  
  gettimeofday(&tvStart, NULL);
  SharedArray<int> mapA(NROWS*NCOLS*3), mapB(NROWS*NCOLS);
  for (int y = 0; y < NROWS; y++)
    for (int x = 0; x < NCOLS; x++) {
      mapA[y*NCOLS+(x*3)] = x;
      mapA[y*NCOLS+(x*3)+1] = x;
      mapA[y*NCOLS+(x*3)+2] = x;
      mapB[y*NCOLS+x] = 0;
    }
  gettimeofday(&tvEnd, NULL);
  printf("# %ld.%06lds\n", tvDiff.tv_sec, tvDiff.tv_usec);

  // Inject hot spots
  srand(0);
  for (int i = 0; i < NSPOTS; i++) {
    int t = rand() % 256;
    int x = rand() % WIDTH;
    int y = 1 + rand() % HEIGHT;
    mapA[y*NCOLS+x] = (int) (1000*t);
  }

  // Compile kernel
  auto k = compile(step);

  // Invoke kernel
  k.setNumQPUs(NQPUS);
  gettimeofday(&tvStart, NULL);
  k(&mapA, &mapB, NCOLS, WIDTH, HEIGHT);
  gettimeofday(&tvEnd, NULL);
  timersub(&tvEnd, &tvStart, &tvDiff);

  // Display results
  printf("P2\n%i %i\n255\n", WIDTH, HEIGHT);
  for (int y = 0; y < HEIGHT; y++)
    for (int x = 0; x < WIDTH; x++) {
      int t = (int) mapB[(y+1)*NCOLS+x];
      t = t < 0   ? 0 : t;
      t = t > 255 ? 255 : t;
      // printf("%d\n", t);
    }

  // Run-time of simulation
  printf("# %ld.%06lds\n", tvDiff.tv_sec, tvDiff.tv_usec);

  return 0;
}
