#ifndef BCI_H_  

#define BCI_H_

#include <stdio.h>
#include <iostream>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <strings.h>
#include <time.h>

// for gdf file
#include <biosig.h>
#include <assert.h>
#include <stdlib.h>
#include "biosig-dev.h"

// for manage signal
#include <armadillo>
//#include "sigpack.h"

#define VERBOSE				    100		// printing level  (=10 only incumbent, =20 little output, =50-60 good, =70 verbose, >=100 cplex log)

//hard-wired parameters
//#define 

//data structures  

typedef struct {   
	
	//input data
	HDRTYPE *hdr;
	long n_channels;
	double sample_rate;
	long n_records;
	biosig_data_type *data;

	// parameters 
	char filename[1000];	  			// input file
	int mode;							// mode of execution
										// 		0 - plot a channel
										//		1 - execute
	int s_ch;							//channel to plot
	int exe;							//execution type

} instance;        


#endif   

