/*
% Copyright (C) 2010,2011,2013 Alois Schloegl <alois.schloegl@gmail.org>
% This file is part of the "BioSig for C/C++" repository 
% (biosig4c++) at http://biosig.sf.net/ 

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 3
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

 */

#include <stdlib.h>
#include "mathlink.h"
#include <biosig.h>

#ifdef NDEBUG
#define VERBOSE_LEVEL 0 	// turn off debugging information, but its only used without NDEBUG
#else
extern int VERBOSE_LEVEL; 	// used for debugging, variable is always defined
#endif

void sload(const char *fn, int *SZ, long SZlen) {

	uint16_t numberChannels;
	size_t k=0;
	size_t numberSamples;
	double samplerate;
	double *t;
	char *str = NULL;
	long int sz[2];
	biosig_data_type *data=NULL;
#ifdef __LIBBIOSIG2_H__
	size_t rowcol[2];
#endif

	HDRTYPE *hdr = constructHDR(0,0);

if (VERBOSE_LEVEL > 5)
	fprintf(stdout,"=== start sload ===\n");

/* contains [experiment,series,sweep,trace] numbers for selecting data. */

	while ((k < SZlen) && (k < 5)) {
#ifdef __LIBBIOSIG2_H__
		biosig_set_segment_selection(hdr, k+1, SZ[k]);
#else
		hdr->AS.SegSel[k] = (uint32_t)SZ[k];
#endif
		k++;
	}

	// ********* open file and read header ************
	hdr = sopen(fn, "r", hdr);
	if (serror2(hdr)) {
		destructHDR(hdr);
		MLEvaluate(stdlink,"Message[sload::failed]");
		MLNextPacket(stdlink);
		MLNewPacket(stdlink);
		MLPutSymbol(stdlink,"$Failed");
		return;
	}

#ifdef __LIBBIOSIG2_H__
	numberChannels = biosig_get_number_of_channels(hdr);
	numberSamples = biosig_get_number_of_samples(hdr);
	samplerate = biosig_get_samplerate(hdr);
	biosig_reset_flag(hdr, BIOSIG_FLAG_ROW_BASED_CHANNELS);
#else
	numberChannels = hdr->NS;
	numberSamples = hdr->NRec * hdr->SPR;
	samplerate = hdr->SampleRate;
	hdr->FLAG.ROW_BASED_CHANNELS = 0;
#endif

if (VERBOSE_LEVEL > 5)
	fprintf(stdout,"open filename <%s>NoOfChans=%i\n", fn, numberChannels);

	// ********** read data ********************
	sread(NULL, 0, numberSamples, hdr);
	if (serror2(hdr)) {
		destructHDR(hdr);
		MLEvaluate(stdlink,"Message[sload::failed]");
		MLNextPacket(stdlink);
		MLNewPacket(stdlink);
		MLPutSymbol(stdlink,"$Failed");
		return;
	}

#ifdef __LIBBIOSIG2_H__
	biosig_get_datablock(hdr, &data, &rowcol[0], &rowcol[1]);
	sz[0] = rowcol[1];
	sz[1] = rowcol[0];
#else
	sz[0] = hdr->data.size[1];
	sz[1] = hdr->data.size[0];
	data  = hdr->data.block;
#endif

	MLPutFunction(stdlink, "List", 3);
	// write data matrix
	MLPutRealArray(stdlink, data, sz, NULL, 2);

	// generate and write time axis
	t = (double*)malloc(numberSamples * sizeof(double));
	for (k=0; k < numberSamples;k++) {
		t[k] = (k+1)/samplerate;
	}
	MLPutRealList(stdlink, t, numberSamples);
	free(t);

	// generate and write header information in JSON format
	asprintf_hdr2json(&str, hdr);
	MLPutFunction(stdlink,"ImportString",2);
	  MLPutString(stdlink,str);
	  MLPutString(stdlink,"JSON");
	free(str);

	if (VERBOSE_LEVEL > 5) {
		for (k=0; k<numberChannels; k++)
			fprintf(stdout,"%f ",data[k]);
		fprintf(stdout,"\n\nopen filename <%s>@%p sz=[%zd,%zd]\n", fn, data, sz[1],sz[0]);
	}

	// *********** close file *********************
	destructHDR(hdr);
	return;
}


int main(int argc, char *argv[]) {
   return MLMain(argc, argv);
}

