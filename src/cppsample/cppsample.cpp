/*
 * A sample program for Sensapex micro manipulator control library using C++ API.
 * Demonstratin a way to customize the LGPL C++ API without a mandatory requirement to
 * publish the extension
 *
 * Copyright (c) 2012-2013, Sensapex Oy
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/timeb.h>
#include <math.h>

#include "customumctl.h"

#define UNDEF  (-1)
#define VERSION_STR "cppsample v0.111"

typedef struct params_s
{
	int x, y, z, X, Y, Z;
	int dev, verbose, update, loop;
    int speed, delayA, delayB, errorLimit, posReadCount;
	const char *port;
} params_t;

void parseArgs(int argc, char *argv[], params_t *params);
void printStats(const char axis, const int *positions, const int count);

unsigned long int getLongTime()
  {
  struct timeb TimeStamp;
  ftime(&TimeStamp);
  return (unsigned long int)(TimeStamp.time*1000L+TimeStamp.millitm);
  }

#define MAX_POSITIONS 10000;

int main(int argc, char *argv[])
{
	CustomUManipulatorCtl umCtl;
	int i, targetX, targetY, targetZ, distance = 0;
	int loop = 0, homeX = 0, homeY = 0, homeZ = 0;
    bool ret;
	params_t params;

	parseArgs(argc, argv, &params);

	if(!umCtl.open(params.port))
	{
		fprintf(stderr, "Can not open %s - %s\n", params.port, umCtl.lastErrorText());
		exit(1);
	}
    // Retrying the first message seems to be needed in OS X
    for(i = 0; i < 3; i++)
        if((ret = umCtl.select(params.dev)))
           break;
	if(!ret)
	{
		fprintf(stderr, "Select failed for device %d - %s\n", params.dev, umCtl.lastErrorText());
		umCtl.close();
		exit(2);
	}

    if(params.posReadCount > 0)
    {
        int i, errCnt = 0, positions[3][params.posReadCount];
        unsigned long int startTime = getLongTime();

        for(loop = 0; loop < params.posReadCount; loop++)
        {
            bool succeeded = false;
            for(i = 0; i < 3; i++)
            {
                positions[0][loop] = -1;
                positions[1][loop] = -1;
                positions[2][loop] = -1;
                if(!umCtl.getPositions(params.x?&positions[0][loop]:NULL,
                                       params.y?&positions[1][loop]:NULL,
                                       params.z?&positions[2][loop]:NULL,
                                       params.dev, UMANIPULATORCTL_TIMELIMIT_DISABLED))
                {
                    errCnt++;
                    continue;
                }
                succeeded = true;
            }
            if(!succeeded)
            {
                fprintf(stderr, "GetPositions failed - %s\n", umCtl.lastErrorText());
                break;
            }
        }
        unsigned long int elapsed = getLongTime() - startTime;
        printf("Read speed %3.1frps, %3.1fms avg, %d error%s\n",(float)loop*1000.0/(float)elapsed,
               (float)elapsed/(float)loop, errCnt, errCnt>1?"s":"");
        printStats('X',positions[0], loop);
        printStats('Y',positions[1], loop);
        printStats('Z',positions[2], loop);
        umCtl.close();
    }

	if(!umCtl.getPositions(&homeX, &homeY, &homeZ))
	{
		fprintf(stderr, "GetPositions failed - %s\n", umCtl.lastErrorText());
		umCtl.close();
		exit(2);
	}

	if(!umCtl.setMemoryDriveSpeed(params.speed))
	{
		fprintf(stderr, "SetMemoryDriveSpeed failed - %s\n", umCtl.lastErrorText());
		umCtl.close();
		exit(2);
	}

	umCtl.setStatusPollErrorCountLimit(params.errorLimit);
	umCtl.setStatusPollPeriod(params.update);

	targetX = homeX, targetY = homeY, targetZ = homeZ;
	if(params.verbose)
		printf("Initial position: %d %d %d\n", homeX, homeY, homeZ);

	// Relative target pos
	if(params.x)
		targetX = homeX + params.x, distance += params.x;
	if(params.y)
		targetY = homeY + params.y, distance += params.y;
	if(params.z)
		targetZ = homeZ + params.z, distance += params.z;
	// Absolute target pos
	if(params.X != UNDEF)
		targetX = params.X, distance += params.X - homeX;
	if(params.Y != UNDEF)
		targetY = params.Y, distance += params.Y - homeY;
	if(params.Z != UNDEF)
		targetZ = params.Z, distance += params.Z - homeZ;

	do
	{
		int x, y, z;
		// Oscillate between home and target
		if(loop&1)
			x = homeX, y = homeY, z = homeZ;
		else
			x = targetX, y = targetY, z = targetZ;
		if(params.verbose)
			printf("Target position: %d %d %d (%d/%d)\n", x, y, z, loop+1, params.loop);

		unsigned long startTime = getLongTime();

		if(!umCtl.gotoMem(x, y, z) < 0)
		{
			fprintf(stderr, "GotoMem failed - %s\n", umCtl.lastErrorText());
			continue;
		}
		if(!umCtl.waitBusy())
		{
			fprintf(stderr, "WaitBusy failed - %s\n", umCtl.lastErrorText());
			umCtl.close();
			exit(3);
		}

		unsigned int elapsed = (unsigned int)(getLongTime() - startTime);

		if(params.verbose && elapsed > 0)
			printf("Speed: %3.1f (um/s)\n", (float)(1000*distance)/(float)elapsed);

		// Wait on position before next movement
		if(loop&1)
			msSleep(params.delayB);
		else
			msSleep(params.delayA);

	} while(++loop < params.loop);

	umCtl.close();
	exit(0);
}

static void usage(char **argv)
{
	fprintf(stderr,"usage: %s [generic opts] [pos opts] [stimulate opts] \n",argv[0]);
	fprintf(stderr,"Generic options\n");
	fprintf(stderr,"-p\tport (e.g. /dev/ttyUSB1 or com2)\n");
	fprintf(stderr,"-d\tdev\tmanipulator id (def 1)\n");
	fprintf(stderr,"-v\t\tverbose\n");
	fprintf(stderr,"-u\tupdate\tstatus poll period (ms)\n");
	fprintf(stderr,"-e\tlimit\tstatus poll error limit\n");
	fprintf(stderr,"Position change\n");
	fprintf(stderr,"-x\trelative target \n");
	fprintf(stderr,"-y\trelative target \n");
	fprintf(stderr,"-z\trelative target \n");
	fprintf(stderr,"-X\tabs target \n");
	fprintf(stderr,"-Y\tabs target \n");
	fprintf(stderr,"-Z\tabs target \n");
	fprintf(stderr,"Stimulate\n");
	fprintf(stderr,"-s\tspeed\taverage speed (um/s)\n");
	fprintf(stderr,"-a\tdelay\twait time at target position(ms)\n");
	fprintf(stderr,"-b\tdelay\twait time at current position (ms)\n");
	fprintf(stderr,"-n\tcount\toscillate between current and target positions\n");
    fprintf(stderr,"Position read speed test\n");
    fprintf(stderr,"-N\tcount\tloop count\n");
    fprintf(stderr,"\n\n\tsample usage %s -p /dev/ttyUSB1 -d 2 -z 300 -s 500 -a 100 -b 20 -n 100\n", argv[0]);
	fprintf(stderr,"%s\n",VERSION_STR);
  exit(1);
}

// Exits via usage() if an command line error occurs
void parseArgs(int argc, char *argv[], params_t *params)
  {
  int i, v;
  memset(params,0,sizeof(params_t));
  params->X = UNDEF;
  params->Y = UNDEF;
  params->Z = UNDEF;
  params->dev = 1;
  params->port = PORT;
  params->update = 100;
  params->errorLimit = 10;
  params->speed = 1000;

  for(i = 1; i < argc; i++)
	{
	if(argv[i][0] == '-')
	  {
	  switch(argv[i][1])
		{
		case 'h': usage(argv);
		case 'v':
		  params->verbose = 1;
		  break;
		case 'n':
		  if(i < argc-1 && sscanf(argv[++i],"%d",&v) == 1 && v > 0)
			params->loop = v*2;
		  else
			usage(argv);
          break;
        case 'N':
          if(i < argc-1 && sscanf(argv[++i],"%d",&v) == 1 && v > 0)
            params->posReadCount = v;
          else
            usage(argv);
          break;
		case 'e':
		  if(i < argc-1 && sscanf(argv[++i],"%d",&v) == 1 && v > 0)
			params->errorLimit = v;
		  else
			usage(argv);
		  break;
		case 'u':
		  if(i < argc-1 && sscanf(argv[++i],"%d",&v) == 1 && v > 0)
			params->update = v;
		  else
			usage(argv);
		  break;
		case 'x':
		  if(i < argc-1 && sscanf(argv[++i],"%d",&v) == 1)
			params->x = v;
		  else
			usage(argv);
		  break;
		case 'y':
		  if(i < argc-1 && sscanf(argv[++i],"%d",&v) == 1)
			params->y = v;
		  else
			usage(argv);
		  break;
		case 'z':
		  if(i < argc-1 && sscanf(argv[++i],"%d",&v) == 1)
			params->z = v;
		  else
			usage(argv);
		  break;
		case 'X':
		  if(i < argc-1 && sscanf(argv[++i],"%d",&v) == 1 && v >= 0)
			params->X = v;
		  else
			usage(argv);
		  break;
		case 'Y':
		  if(i < argc-1 && sscanf(argv[++i],"%d",&v) == 1 && v >= 0)
			params->Y = v;
		  else
			usage(argv);
		  break;
		case 'Z':
		  if(i < argc-1 && sscanf(argv[++i],"%d",&v) == 1 && v >= 0)
			params->Z = v;
		  else
			usage(argv);
		  break;
		case 'd':
		  if(i < argc-1 && sscanf(argv[++i],"%d",&v) == 1 && v > 0)
			params->dev = v;
		  else
			usage(argv);
		  break;
		case 'p':
		  if(i < argc-1 && argv[i+1][0] != '-')
			params->port = argv[++i];
		  else
			usage(argv);
		  break;
		case 's':
		  if(i < argc-1 && sscanf(argv[++i],"%d",&v) == 1 && v > 0)
			params->speed = v;
		  else
			usage(argv);
		  break;
		case 'a':
		  if(i < argc-1 && sscanf(argv[++i],"%d",&v) == 1 && v >= 0)
			params->delayA = v;
		  else
			usage(argv);
		  break;
		case 'b':
		  if(i < argc-1 && sscanf(argv[++i],"%d",&v) == 1 && v >= 0)
			params->delayB = v;
		  else
			usage(argv);
		  break;
		default:
		  usage(argv);
		  break;
		}
	  }
	else
	  usage(argv);
	}
}

void printStats(const char axis, const int *positions, const int count)
{
    if(count < 2)
    {
        printf("%c: not enough samples\n", axis);
        return;
    }
    int i, value, min, max = 0;
    double sum = 0.0;
    for(i = 0; i < count; i++)
    {
        value = *(positions+i);
        if(!i || value < min)
            min = value;
        if(!i || value > max)
            max = value;
        sum += value;
    }
    double avg = sum/count;
    printf("%c avg %4.3f min %d max %d", axis, avg, min, max);
    double sqrSum;
    for(i = 0; i < count; i++)
    {
        value = *(positions+i);
        sqrSum = pow(value-avg,2.0);
    }
    printf(" var %5.4f\n",sqrt(sqrSum/(count-1)));
}
