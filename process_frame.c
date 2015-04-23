/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file process_frame.c
 * @brief Contains the actual algorithm and calculations.
 */

/* Definitions specific to this application. Also includes the Oscar main header file. */
#include "template.h"
#include <string.h>
#include <stdlib.h>

#define IMG_SIZE NUM_COLORS*(OSC_CAM_MAX_IMAGE_WIDTH/2)*(OSC_CAM_MAX_IMAGE_HEIGHT/2)

const int nc = OSC_CAM_MAX_IMAGE_WIDTH/2;
const int nr = OSC_CAM_MAX_IMAGE_HEIGHT/2;

const int Border = 6;
const int k = 5;
const int sizeBox = 10;
//const int false = 0;

int TextColor;
int absMaximum;
int avgDxy[3][IMG_SIZE];
int mc[IMG_SIZE];


void ResetProcess();
void AvgDeriv(int);
void CalcDeriv();
void localMax();




void ResetProcess()
{
	//called when "reset" button is pressed
	if(TextColor == CYAN)
		TextColor = MAGENTA;
	else
		TextColor = CYAN;
}


void AvgDeriv(int Index)
{
	//do average in x-direction
	int helpBuf[IMG_SIZE];
	int c, r;
	int Border = 0;
	for(r = nc; r < nr*nc-nc; r+= nc)
		{	/* we skip first and last lines (empty) */
		for(c = Border+1; c < nc-(Border+1); c++) {/* +1 because we have one empty border column */
		/* do pointer arithmetics with respect to center pixel location */
		int* p = &avgDxy[Index][r+c];

		int sx = (*(p-6) + *(p+6)) + ((*(p-5) + *(p+5))<<2) + ((*(p-4) + *(p+4))<<3) +
				((*(p-3) + *(p+3))<<5) + ((*(p-2) + *(p+2))<<6) + ((*(p-1) + *(p+1))<<7) + (*p<<7);
		//now averaged
		helpBuf[r+c] = (sx >> 8);
		}
	}
	//do average in y-direction
	for(r = nc; r < nr*nc-nc; r+= nc)
	{/* we skip first and last lines (empty) */
		for(c = Border+1; c < nc-(Border+1); c++)
		{/* +1 because we have one empty border column */
		/* do pointer arithmetics with respect to center pixel location */
		int* p = &helpBuf[r+c];
		int sy = (*(p-6*nc) + *(p+6*nc)) + ((*(p-5*nc) + *(p+5*nc))<<2) + ((*(p-4*nc) + *(p+4*nc))<<3) +
			((*(p-3*nc) + *(p+3*nc))<<5) + ((*(p-2*nc) + *(p+2*nc))<<6) + ((*(p-1*nc) + *(p+1*nc))<<6) + (*p<<7);
		//now averaged
		avgDxy[Index][r+c] = (sy >> 8);
		}
	}

}


void CalcDeriv()
{
int c, r;

for(r = nc; r < nr*nc-nc; r+= nc)
	{/* we skip the first and last line */
		for(c = 1; c < nc-1; c++)
		{

		/* do pointer arithmetics with respect to center pixel location */
		unsigned char* p = &data.u8TempImage[SENSORIMG][r+c];

		/* implement Sobel filter in x-direction */
		int dx = -(int) *(p-nc-1) + (int) *(p-nc+1)
		-2* (int) *(p-1) + 2* (int) *(p+1)
		-(int) *(p+nc-1) + (int) *(p+nc+1);

		/* implement Sobel filter in y-direction */
		int dy = 	-(int) *(p-nc-1) -2* (int) *(p-nc) - (int) *(p-nc+1)
					+(int) *(p+nc-1) +2* (int) *(p+nc) + (int) *(p+nc+1);

		/* not yet averaged */

		avgDxy[0][r+c] = dx * dy;
		avgDxy[1][r+c] = dy * dy;
		avgDxy[2][r+c] = dx * dy;
		}
	}
}

// too many "for" loops.. needs a more efficient implementation
void localMax()
{
	int c, r;

		for (r = 7 * nc; r < nr * nc - 7 * nc; r += nc) {
			for (c = 7; c < nc - 7; c++) {

				int* p = &mc[c + r];

				// maxima filter
				int localMax = 0;
				for (int i = -6; i < 7; i++)
				{
					for (int j = -6; j < 7; j++)
					{
						if (localMax <= *(p + nc * i + j))
						{
							localMax = *(p + i * nc + j);
						}
					}
				}

				if (localMax == *p && *p > absMaximum)
				{
					DrawBoundingBox(c - sizeBox, r / nc + sizeBox, c + sizeBox,r / nc - sizeBox, 0, GREEN);
				}
			}
		}

}




void ProcessFrame()
{
	uint32 t1, t2;

	//initialize counters
	if(data.ipc.state.nStepCounter == 1) {
		//use for initialization; only done in first step
		memset(data.u8TempImage[THRESHOLD], 0, IMG_SIZE);
		TextColor = CYAN;
	} else {
		memcpy(data.u8TempImage[BACKGROUND], data.u8TempImage[SENSORIMG], IMG_SIZE);

		t1 = OscSupCycGet(); // start time

		// call CalcDeriv
		CalcDeriv();

		// call AvgDeriv
		AvgDeriv(0);
		AvgDeriv(1);
		AvgDeriv(2);

		// corner size
		absMaximum = 0;

		for (int r = 7 * nc; r < nr * nc - 7 * nc; r += nc)
		{
			for (int c = 7; c < nc - 7; c++)
			{
			int i = r + c;
			mc[i] = ((avgDxy[0][i] >> 6) * (avgDxy[1][i] >> 6)
			- (avgDxy[2][i] >> 6) * (avgDxy[2][i] >> 6))
			- ((k* ((avgDxy[0][i] >> 6)	+ (avgDxy[1][i] >> 6))
			* ((avgDxy[0][i] >> 6) + (avgDxy[1][i] >> 6))) >> 7);

			absMaximum = MAX(mc[i], absMaximum);

			}
		}
		absMaximum = absMaximum/100 * data.ipc.state.nThreshold;  // select nThreshold-% greatest values


		// find local Max. and mark them
		localMax();

		t2 = OscSupCycGet();  // end time

		// output to console
		OscLog(INFO, "required = %d us\n", OscSupCycToMicroSecs(t2-t1));
	}
}
