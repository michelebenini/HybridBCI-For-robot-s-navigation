/*

    Copyright (C) 2007,2008,2009,2010,2013,2015 Alois Schloegl <alois.schloegl@ist.ac.at>
    This file is part of the "BioSig for C/C++" repository 
    (biosig4c++) at http://biosig.sf.net/ 

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 3
    of the License, or (at your option) any later version.

 */

#include "mex.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#ifdef HAVE_CHOLMOD
#  if !defined(__APPLE__)
#    include <suitesparse/cholmod.h>
#  else
#    include <cholmod.h>
#  endif
#endif
#include <biosig-dev.h>
#include <biosig.h>

#ifdef NDEBUG
#define VERBOSE_LEVEL 0 	// turn off debugging information, but its only used without NDEBUG
#else
extern int VERBOSE_LEVEL; 	// used for debugging
#endif


#ifdef tmwtypes_h
  #if (MX_API_VER<=0x07020000)
    typedef int mwSize;
  #endif 
#endif 

#ifndef TRUE
#define TRUE (1)
#endif

#ifdef CHOLMOD_H
//#include "cholmod/matlab/cholmod_matlab.h"
/*
The function sputil_get_sparse and its license was downloaded on Oct 16, 2009 from 
http://www.cise.ufl.edu/research/sparse/cholmod/CHOLMOD/MATLAB/cholmod_matlab.c
http://www.cise.ufl.edu/research/sparse/cholmod/CHOLMOD/MATLAB/License.txt
*/
/*
CHOLMOD/MATLAB Module.
Copyright (C) 2005-2006, Timothy A. Davis
CHOLMOD is also available under other licenses; contact authors for details.
MATLAB(tm) is a Registered Trademark of The MathWorks, Inc.
http://www.cise.ufl.edu/research/sparse

Note that this license is for the CHOLMOD/MATLAB module only.
All CHOLMOD modules are licensed separately.


--------------------------------------------------------------------------------


This Module is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This Module is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this Module; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

/* ========================================================================== */
/* === sputil_get_sparse ==================================================== */
/* ========================================================================== */

/* Create a shallow CHOLMOD copy of a MATLAB sparse matrix.  No memory is
 * allocated.  The resulting matrix A must not be modified.
 */

cholmod_sparse *sputil_get_sparse
(
    const mxArray *Amatlab, /* MATLAB version of the matrix */
    cholmod_sparse *A,	    /* CHOLMOD version of the matrix */
    double *dummy,	    /* a pointer to a valid scalar double */
    mwSize stype		    /* -1: lower, 0: unsymmetric, 1: upper */
)
{
    mwSize *Ap ;
    A->nrow = mxGetM (Amatlab) ;
    A->ncol = mxGetN (Amatlab) ;
    A->p = (mwSize *) mxGetJc (Amatlab) ;
    A->i = (mwSize *) mxGetIr (Amatlab) ;
    Ap = (mwSize*)A->p ;
    A->nzmax = Ap [A->ncol] ;
    A->packed = TRUE ;
    A->sorted = TRUE ;
    A->nz = NULL ;
    A->itype = CHOLMOD_LONG ;       /* was CHOLMOD_INT in v1.6 and earlier */
    A->dtype = CHOLMOD_DOUBLE ;
    A->stype = stype ;

#ifndef MATLAB6p1_OR_EARLIER

    if (mxIsLogical (Amatlab))
    {
	A->x = NULL ;
	A->z = NULL ;
	A->xtype = CHOLMOD_PATTERN ;
    }
    else if (mxIsEmpty (Amatlab))
    {
	/* this is not dereferenced, but the existence (non-NULL) of these
	 * pointers is checked in CHOLMOD */
	A->x = dummy ;
	A->z = dummy ;
	A->xtype = mxIsComplex (Amatlab) ? CHOLMOD_ZOMPLEX : CHOLMOD_REAL ;
    }
    else if (mxIsDouble (Amatlab))
    {
	A->x = mxGetPr (Amatlab) ;
	A->z = mxGetPi (Amatlab) ;
	A->xtype = mxIsComplex (Amatlab) ? CHOLMOD_ZOMPLEX : CHOLMOD_REAL ;
    }
    else
    {
	/* only logical and complex/real double matrices supported */
	//sputil_error (ERROR_INVALID_TYPE, 0) ;        // modified by AS, Oct 2009 
    }

#else

    if (mxIsEmpty (Amatlab))
    {
	/* this is not dereferenced, but the existence (non-NULL) of these
	 * pointers is checked in CHOLMOD */
	A->x = dummy ;
	A->z = dummy ;
	A->xtype = mxIsComplex (Amatlab) ? CHOLMOD_ZOMPLEX : CHOLMOD_REAL ;
    }
    else
    {
	/* in MATLAB 6.1, the matrix is sparse, so it must be double */
	A->x = mxGetPr (Amatlab) ;
	A->z = mxGetPi (Amatlab) ;
	A->xtype = mxIsComplex (Amatlab) ? CHOLMOD_ZOMPLEX : CHOLMOD_REAL ;
    }

#endif 

    return (A) ;
}
/* ========================================================================== */
/* === end of sputil_get_sparse ============================================= */
/* ========================================================================== */
#endif 

#ifdef WITH_PDP 
void sopen_pdp_read(HDRTYPE *hdr);
#endif

//#define VERBOSE_LEVEL  9 
//extern int VERBOSE_LEVEL;
//#define DEBUG

void mexFunction(
    int           nlhs,           /* number of expected outputs */
    mxArray       *plhs[],        /* array of pointers to output arguments */
    int           nrhs,           /* number of inputs */
    const mxArray *prhs[]         /* array of pointers to input arguments */
)

{
	size_t 		k,k1;
	const mxArray	*arg;
	mxArray		*HDR;
	HDRTYPE		*hdr;
	CHANNEL_TYPE*	cp; 
	size_t 		count;
	time_t 		T0;
	char 		*FileName=NULL;  
	int 		status; 
	int		CHAN = 0;
	int		TARGETSEGMENT = 1; 
	double		*ChanList=NULL;
	int		NS = -1;
	char		FlagOverflowDetection = 1, FlagUCAL = 0;
	int		argSweepSel = -1;
	
#ifdef CHOLMOD_H
	cholmod_sparse RR,*rr=NULL;
	double dummy;
#endif 

// ToDO: output single data 
//	mxClassId	FlagMXclass=mxDOUBLE_CLASS;
	

	if (nrhs<1) {
#ifdef mexSOPEN
		mexPrintf("   Usage of mexSOPEN:\n");
		mexPrintf("\tHDR = mexSOPEN(f)\n");
		mexPrintf("   Input:\n\tf\tfilename\n");
		mexPrintf("   Output:\n\tHDR\theader structure\n\n");
#else
		mexPrintf("   Usage of mexSLOAD:\n");
		mexPrintf("\t[s,HDR]=mexSLOAD(f)\n");
		mexPrintf("\t[s,HDR]=mexSLOAD(f,chan)\n\t\tchan must be sorted in ascending order\n");
#ifdef CHOLMOD_H
		mexPrintf("\t[s,HDR]=mexSLOAD(f,ReRef)\n\t\treref is a (sparse) matrix for rerefencing\n");
#endif
		mexPrintf("\t[s,HDR]=mexSLOAD(f,chan,'...')\n");
		mexPrintf("\t[s,HDR]=mexSLOAD(f,chan,'OVERFLOWDETECTION:ON')\n");
		mexPrintf("\t[s,HDR]=mexSLOAD(f,chan,'OVERFLOWDETECTION:OFF')\n");
		mexPrintf("\t[s,HDR]=mexSLOAD(f,chan,'UCAL:ON')\n");
		mexPrintf("\t[s,HDR]=mexSLOAD(f,chan,'UCAL:OFF')\n");
		mexPrintf("\t[s,HDR]=mexSLOAD(f,chan,'OUTPUT:SINGLE')\n");
		mexPrintf("\t[s,HDR]=mexSLOAD(f,chan,'TARGETSEGMENT:<N>')\n");
		mexPrintf("\t[s,HDR]=mexSLOAD(f,chan,'SWEEP',[NE, NG, NS])\n");
		mexPrintf("   Input:\n\tf\tfilename\n");
		mexPrintf("\tchan\tlist of selected channels; 0=all channels [default]\n");
		mexPrintf("\tUCAL\tON: do not calibrate data; default=OFF\n");
//		mexPrintf("\tOUTPUT\tSINGLE: single precision; default='double'\n");
		mexPrintf("\tOVERFLOWDETECTION\tdefault = ON\n\t\tON: values outside dynamic range are not-a-number (NaN)\n");
		mexPrintf("\tTARGETSEGMENT:<N>\n\t\tselect segment <N> in multisegment files (like Nihon-Khoden), default=1\n\t\tIt has no effect for other data formats.\n");
		mexPrintf("\t[NE, NG, NS] are the number of the experiment, the series and the sweep, resp. for sweep selection in HEKA/PatchMaster files. (0 indicates all)\n");
		mexPrintf("\t\t examples: [1,2,3] the 3rd sweep from the 2nd series of experiment 1; [1,3,0] selects all sweeps from experiment=1, series=3. \n\n");
		mexPrintf("   Output:\n\ts\tsignal data, each column is one channel\n");
		mexPrintf("\tHDR\theader structure\n\n");
#endif
		return; 
	}

/*
 	improve checks for input arguments
*/
	/* process input arguments */
	for (k = 0; k < nrhs; k++) {	
		arg = prhs[k];
		if (mxIsEmpty(arg) && (k>0)) {
#ifdef DEBUG		
			mexPrintf("arg[%i] Empty\n",k);
#endif
		}
		else if ((k==0) && mxIsCell(arg) && mxGetNumberOfElements(arg)==1 && mxGetCell(arg,0) && mxIsChar(mxGetCell(arg,0))) {
			FileName = mxArrayToString(mxGetCell(arg,0));
#ifdef DEBUG		
			mexPrintf("arg[%i] IsCell\n",k);
#endif
		}
		else if ((k==0) && mxIsStruct(arg)) {
			FileName = mxArrayToString(mxGetField(prhs[k],0,"FileName"));
#ifdef DEBUG		
			mexPrintf("arg[%i] IsStruct\n",k);
#endif
		}
		else if ((k==1) && mxIsSparse(arg)) {
#ifdef CHOLMOD_H
			rr = sputil_get_sparse(arg,&RR,&dummy,0);
#else
			mexErrMsgTxt("This version of mexSLOAD does not support re-referencing matrix - recompile with -DWITH_CHOLMOD -lcholmod \n");
#endif 
		}
		else if ((k==1) && mxIsNumeric(arg)) {
#ifdef DEBUG		
			mexPrintf("arg[%i] IsNumeric\n",k);
#endif
			ChanList = (double*)mxGetData(prhs[k]);
			NS = mxGetNumberOfElements(prhs[k]);
		}	
		else if (mxIsChar(arg)) {
#ifdef DEBUG		
			mexPrintf("arg[%i]=%s \n",k,mxArrayToString(prhs[k]));
#endif
			if (k==0)			
				FileName = mxArrayToString(prhs[k]);
			else if (!strcmp(mxArrayToString(prhs[k]), "CNT32"))
				; // obsolete - supported for backwards compatibility
			else if (!strcmp(mxArrayToString(prhs[k]), "OVERFLOWDETECTION:ON"))
				FlagOverflowDetection = 1;
			else if (!strcmp(mxArrayToString(prhs[k]), "OVERFLOWDETECTION:OFF"))
				FlagOverflowDetection = 0;
			else if (!strcmp(mxArrayToString(prhs[k]), "UCAL:ON")) 
				FlagUCAL = 1;
			else if (!strcmp(mxArrayToString(prhs[k]), "UCAL:OFF"))
				FlagUCAL = 0;
//			else if (!strcmp(mxArrayToString(prhs[k]),"OUTPUT:SINGLE"))
//				FlagMXclass = mxSINGLE_CLASS;
			else if (!strncmp(mxArrayToString(prhs[k]),"TARGETSEGMENT:",14))
				TARGETSEGMENT = atoi(mxArrayToString(prhs[k])+14);
			else if (!strcasecmp(mxArrayToString(prhs[k]), "SWEEP") && (prhs[k+1] != NULL) && mxIsNumeric(prhs[k+1]))
				argSweepSel = ++k;
		}
		else {
#ifndef mexSOPEN
			mexPrintf("mexSLOAD: argument #%i is invalid.",k+1);	
			mexErrMsgTxt("mexSLOAD fails because of unknown parameter\n");
#else
			mexPrintf("mexSOPEN: argument #%i is invalid.",k+1);	
			mexErrMsgTxt("mexSOPEN fails because of unknown parameter\n");	
#endif
		}
	}

	if (VERBOSE_LEVEL>7) 
		mexPrintf("110: input arguments checked\n");

	hdr = constructHDR(0,0);

#ifdef __LIBBIOSIG2_H__

	unsigned flags = (!!FlagOverflowDetection)*BIOSIG_FLAG_OVERFLOWDETECTION + (!!FlagUCAL)*BIOSIG_FLAG_UCAL;
#ifdef CHOLMOD_H
	flags += (rr!=NULL)*BIOSIG_FLAG_ROW_BASED_CHANNELS;
#else
	biosig_reset_flag(hdr, BIOSIG_FLAG_ROW_BASED_CHANNELS);
#endif
	biosig_set_flag(hdr, flags);

	biosig_set_targetsegment(hdr, TARGETSEGMENT);

	// sweep selection for Heka format
	if (argSweepSel>0) {
		double *SZ     = (double*) mxGetData(prhs[argSweepSel]);
		k = 0;
		while (k < mxGetNumberOfElements(prhs[argSweepSel]) && k < 5) {
			biosig_set_segment_selection(hdr, k+1, (uint32_t)SZ[k]);
			k++;
		}
	}

#else //__LIBBIOSIG2_H__


	hdr->FLAG.OVERFLOWDETECTION = FlagOverflowDetection; 
	hdr->FLAG.UCAL = FlagUCAL;
#ifdef CHOLMOD_H
	hdr->FLAG.ROW_BASED_CHANNELS = (rr!=NULL); 
#else 	
	hdr->FLAG.ROW_BASED_CHANNELS = 0; 
#endif 
	hdr->FLAG.TARGETSEGMENT = TARGETSEGMENT;

	// sweep selection for Heka format 
	if (argSweepSel>0) { 				
		double *SZ     = (double*) mxGetData(prhs[argSweepSel]);
		k = 0;
		while (k < mxGetNumberOfElements(prhs[argSweepSel]) && k < 5) { 
			hdr->AS.SegSel[k] = (uint32_t)SZ[k];
			k++;
		}
	}


#endif // __LIBBIOSIG2_H__ : TODO: below, nothing is converted to level-2 interface, yet


	if (VERBOSE_LEVEL>7) 
		mexPrintf("120: going to sopen\n");

	hdr = sopen(FileName, "r", hdr);
/*
#ifdef WITH_PDP 
	if (hdr->AS.B4C_ERRNUM) {
		hdr->AS.B4C_ERRNUM = 0;
		sopen_pdp_read(hdr);
	}	
#endif
*/
	if (VERBOSE_LEVEL>7) 
		mexPrintf("121: sopen done\n");

	if ((status=serror2(hdr))) {

		const char* fields[]={"TYPE","VERSION","FileName","FLAG","ErrNum","ErrMsg"};
		HDR = mxCreateStructMatrix(1, 1, 6, fields);
#ifdef __LIBBIOSIG2_H__
		mxSetField(HDR,0,"FileName",mxCreateString(biosig_get_filename(hdr)));
		const char *FileTypeString = GetFileTypeString(biosig_get_filetype(hdr));
		mxSetField(HDR,0,"VERSION",mxCreateDoubleScalar(biosig_get_version(hdr)));
#else
		mxSetField(HDR,0,"FileName",mxCreateString(hdr->FileName));
		const char *FileTypeString = GetFileTypeString(hdr->TYPE);
		mxSetField(HDR,0,"VERSION",mxCreateDoubleScalar(hdr->VERSION));
#endif
		mxArray *errnum = mxCreateNumericMatrix(1,1,mxUINT8_CLASS,mxREAL);
		*(uint8_t*)mxGetData(errnum) = (uint8_t)status;
		mxSetField(HDR,0,"ErrNum",errnum);
		
#ifdef HAVE_OCTAVE
		// handle bug in octave: mxCreateString(NULL) causes segmentation fault
		// Octave 3.2.3 causes a seg-fault in mxCreateString(NULL)
		if (FileTypeString) FileTypeString="\0";
#endif
		mxSetField(HDR,0,"TYPE",mxCreateString(FileTypeString));


		char *msg = (char*)malloc(72+23+strlen(FileName)); // 72: max length of constant text, 23: max length of GetFileTypeString()
		if (msg == NULL) 
			mxSetField(HDR,0,"ErrMsg",mxCreateString("Error mexSLOAD: Cannot open file\n"));
		else {	
		    if (status==B4C_CANNOT_OPEN_FILE)
			sprintf(msg,"Error mexSLOAD: file %s not found.\n",FileName);		/* Flawfinder: ignore *** sufficient memory is allocated above */
		    else if (status==B4C_FORMAT_UNKNOWN)
			sprintf(msg,"Error mexSLOAD: Cannot open file %s - format %s not known.\n",FileName,FileTypeString);	/* Flawfinder: ignore *** sufficient memory is allocated above */
		    else if (status==B4C_FORMAT_UNSUPPORTED)
			sprintf(msg,"Error mexSLOAD: Cannot open file %s - format %s not supported [%s].\n", FileName, FileTypeString, hdr->AS.B4C_ERRMSG);	/* Flawfinder: ignore *** sufficient memory is allocated above */
		    else 	
			sprintf(msg,"Error %i mexSLOAD: Cannot open file %s - format %s not supported [%s].\n", status, FileName, FileTypeString, hdr->AS.B4C_ERRMSG); 	/* Flawfinder: ignore *** sufficient memory is allocated above */
			
		    mxSetField(HDR,0,"ErrMsg",mxCreateString(msg));
		    free(msg);
		}    

	if (VERBOSE_LEVEL>7) 
		mexPrintf("737: abort mexSLOAD - sopen failed\n");

		destructHDR(hdr);

	if (VERBOSE_LEVEL>7) 
		mexPrintf("757: abort mexSLOAD - sopen failed\n");

#ifdef mexSOPEN
		plhs[0] = HDR; 
#else
		plhs[0] = mxCreateDoubleMatrix(0,0, mxREAL);
		plhs[1] = HDR; 
#endif 		 
	if (VERBOSE_LEVEL>7) 
		mexPrintf("777: abort mexSLOAD - sopen failed\n");

		return; 
	}

#ifdef CHOLMOD_H
	RerefCHANNEL(hdr,rr,2);
#endif

	if (hdr->FLAG.OVERFLOWDETECTION != FlagOverflowDetection)
		mexPrintf("Warning mexSLOAD: Overflowdetection not supported in file %s\n",hdr->FileName);
	if (hdr->FLAG.UCAL != FlagUCAL)
		mexPrintf("Warning mexSLOAD: Flag UCAL is %i instead of %i (%s)\n",hdr->FLAG.UCAL,FlagUCAL,hdr->FileName);


	if (VERBOSE_LEVEL>7) 
		fprintf(stderr,"[112] SOPEN-R finished NS=%i %i\n",hdr->NS,NS);

//	convert2to4_eventtable(hdr); 
		
#ifdef CHOLMOD_H
	if (hdr->Calib != NULL) {
		NS = hdr->Calib->ncol;
	}
	else 
#endif
	if ((NS<0) || ((NS==1) && (ChanList[0] == 0.0))) { 	// all channels
		for (k=0, NS=0; k<hdr->NS; ++k) {
			if (hdr->CHANNEL[k].OnOff) NS++; 
		}	
	}		
	else {		
		for (k=0; k<hdr->NS; ++k)
			hdr->CHANNEL[k].OnOff = 0; 	// reset
		for (k=0; k<NS; ++k) {
			int ch = (int)ChanList[k];
			if ((ch < 1) || (ch > hdr->NS)) 
				mexPrintf("Invalid channel number CHAN(%i) = %i!\n",k+1,ch); 
			else 	
				hdr->CHANNEL[ch-1].OnOff = 1;  // set
		}		
	}
	
	if (VERBOSE_LEVEL>7) 
		fprintf(stderr,"[113] NS=%i %i\n",hdr->NS,NS);

#ifndef mexSOPEN
	if (hdr->FLAG.ROW_BASED_CHANNELS)
		plhs[0] = mxCreateDoubleMatrix(NS, hdr->NRec*hdr->SPR, mxREAL);
	else
		plhs[0] = mxCreateDoubleMatrix(hdr->NRec*hdr->SPR, NS, mxREAL);

	count = sread(mxGetPr(plhs[0]), 0, hdr->NRec, hdr);
	hdr->NRec = count; 
#endif
	sclose(hdr);
#ifdef CHOLMOD_H
        if (hdr->Calib && hdr->rerefCHANNEL) {
		hdr->NS = hdr->Calib->ncol; 
                free(hdr->CHANNEL);
                hdr->CHANNEL = hdr->rerefCHANNEL;
                hdr->rerefCHANNEL = NULL; 
                hdr->Calib = NULL; 
        }                
#endif 
	if ((status=serror2(hdr))) return;  

	if (VERBOSE_LEVEL>7) 
		fprintf(stderr,"\n[129] SREAD/SCLOSE on %s successful [%i,%i] [%i,%i] %i.\n",hdr->FileName,(int)hdr->data.size[0],(int)hdr->data.size[1],(int)hdr->NRec,(int)count,(int)NS);


//	hdr2ascii(hdr,stderr,4);	

#ifndef mexSOPEN 

	if (nlhs>1) { 
#endif

		char* mexFileName = (char*)mxMalloc(strlen(hdr->FileName)+1); 

		mxArray *tmp, *tmp2, *Patient, *Manufacturer, *ID, *EVENT, *Filter, *Flag, *FileType;
		uint16_t numfields;
		const char *fnames[] = {"TYPE","VERSION","FileName","T0","tzmin","Patient",\
		"HeadLen","NS","SPR","NRec","SampleRate", "FLAG", \
		"EVENT","Label","LeadIdCode","PhysDimCode","PhysDim","Filter",\
		"PhysMax","PhysMin","DigMax","DigMin","Transducer","Cal","Off","GDFTYP","TOffset",\
		"ELEC","Impedance","fZ","AS","Dur","REC","Manufacturer",NULL};

		for (numfields=0; fnames[numfields++] != NULL; );
		HDR = mxCreateStructMatrix(1, 1, --numfields, fnames);

		mxSetField(HDR,0,"TYPE",mxCreateString(GetFileTypeString(hdr->TYPE)));
		mxSetField(HDR,0,"HeadLen",mxCreateDoubleScalar(hdr->HeadLen));
		mxSetField(HDR,0,"VERSION",mxCreateDoubleScalar(hdr->VERSION));
		mxSetField(HDR,0,"NS",mxCreateDoubleScalar(NS));
		mxSetField(HDR,0,"SPR",mxCreateDoubleScalar(hdr->SPR));
		mxSetField(HDR,0,"NRec",mxCreateDoubleScalar(hdr->NRec));
		mxSetField(HDR,0,"SampleRate",mxCreateDoubleScalar(hdr->SampleRate));
		mxSetField(HDR,0,"Dur",mxCreateDoubleScalar(hdr->SPR/hdr->SampleRate));
		mxSetField(HDR,0,"FileName",mxCreateString(hdr->FileName));
                
		mxSetField(HDR,0,"T0",mxCreateDoubleScalar(ldexp(hdr->T0,-32)));
		mxSetField(HDR,0,"tzmin",mxCreateDoubleScalar(hdr->tzmin));

		/* Channel information */ 
#ifdef CHOLMOD_H
/*
        	if (hdr->Calib == NULL) { // is refering to &RR, do not destroy
		        mxArray *Calib = mxCreateDoubleMatrix(hdr->Calib->nrow, hdr->Calib->ncol, mxREAL);

        	}
*/
#endif
		mxArray *LeadIdCode  = mxCreateDoubleMatrix(1,NS, mxREAL);
		mxArray *PhysDimCode = mxCreateDoubleMatrix(1,NS, mxREAL);
		mxArray *GDFTYP      = mxCreateDoubleMatrix(1,NS, mxREAL);
		mxArray *PhysMax     = mxCreateDoubleMatrix(1,NS, mxREAL);
		mxArray *PhysMin     = mxCreateDoubleMatrix(1,NS, mxREAL);
		mxArray *DigMax      = mxCreateDoubleMatrix(1,NS, mxREAL);
		mxArray *DigMin      = mxCreateDoubleMatrix(1,NS, mxREAL);
		mxArray *Cal         = mxCreateDoubleMatrix(1,NS, mxREAL);
		mxArray *Off         = mxCreateDoubleMatrix(1,NS, mxREAL);
		mxArray *Toffset     = mxCreateDoubleMatrix(1,NS, mxREAL);
		mxArray *ELEC_POS    = mxCreateDoubleMatrix(NS,3, mxREAL);
/*
		mxArray *ELEC_Orient = mxCreateDoubleMatrix(NS,3, mxREAL);
		mxArray *ELEC_Area   = mxCreateDoubleMatrix(NS,1, mxREAL);
*/
		mxArray *LowPass     = mxCreateDoubleMatrix(1,NS, mxREAL);
		mxArray *HighPass    = mxCreateDoubleMatrix(1,NS, mxREAL);
		mxArray *Notch       = mxCreateDoubleMatrix(1,NS, mxREAL);
		mxArray *Impedance   = mxCreateDoubleMatrix(1,NS, mxREAL);
		mxArray *fZ          = mxCreateDoubleMatrix(1,NS, mxREAL);
		mxArray *SPR         = mxCreateDoubleMatrix(1,NS, mxREAL);
		mxArray *Label       = mxCreateCellMatrix(NS,1);
		mxArray *Transducer  = mxCreateCellMatrix(NS,1);
		mxArray *PhysDim1    = mxCreateCellMatrix(NS,1);

		for (k=0,k1=0; k1<NS; ++k) 
		if (hdr->CHANNEL[k].OnOff) {
			*(mxGetPr(LeadIdCode)+k1)	= (double)hdr->CHANNEL[k].LeadIdCode;
			*(mxGetPr(PhysDimCode)+k1)	= (double)hdr->CHANNEL[k].PhysDimCode;
			*(mxGetPr(GDFTYP)+k1)		= (double)hdr->CHANNEL[k].GDFTYP;
			*(mxGetPr(PhysMax)+k1)		= (double)hdr->CHANNEL[k].PhysMax;
			*(mxGetPr(PhysMin)+k1)		= (double)hdr->CHANNEL[k].PhysMin;
			*(mxGetPr(DigMax)+k1)		= (double)hdr->CHANNEL[k].DigMax;
			*(mxGetPr(DigMin)+k1)		= (double)hdr->CHANNEL[k].DigMin;
			*(mxGetPr(Toffset)+k1)		= (double)hdr->CHANNEL[k].TOffset;
			*(mxGetPr(Cal)+k1) 		= (double)hdr->CHANNEL[k].Cal;
			*(mxGetPr(Off)+k1) 		= (double)hdr->CHANNEL[k].Off;
			*(mxGetPr(SPR)+k1) 		= (double)hdr->CHANNEL[k].SPR;
			*(mxGetPr(LowPass)+k1) 		= (double)hdr->CHANNEL[k].LowPass;
			*(mxGetPr(HighPass)+k1) 	= (double)hdr->CHANNEL[k].HighPass;
			*(mxGetPr(Notch)+k1) 		= (double)hdr->CHANNEL[k].Notch;
			*(mxGetPr(Impedance)+k1)	= (double)hdr->CHANNEL[k].Impedance;
			*(mxGetPr(fZ)+k1)		= (double)hdr->CHANNEL[k].fZ;
			*(mxGetPr(ELEC_POS)+k1)    	= (double)hdr->CHANNEL[k].XYZ[0];
			*(mxGetPr(ELEC_POS)+k1+NS)	= (double)hdr->CHANNEL[k].XYZ[1];
			*(mxGetPr(ELEC_POS)+k1+NS*2)	= (double)hdr->CHANNEL[k].XYZ[2];
/*
			*(mxGetPr(ELEC_Orient)+k1)	= (double)hdr->CHANNEL[k].Orientation[0];
			*(mxGetPr(ELEC_Orient)+k1+NS)	= (double)hdr->CHANNEL[k].Orientation[1];
			*(mxGetPr(ELEC_Orient)+k1+NS*2)	= (double)hdr->CHANNEL[k].Orientation[2];
			*(mxGetPr(ELEC_Area)+k1)	= (double)hdr->CHANNEL[k].Area;
*/
			mxSetCell(Label,k1,mxCreateString(hdr->CHANNEL[k].Label ? hdr->CHANNEL[k].Label : ""));
			mxSetCell(Transducer,k1,mxCreateString(hdr->CHANNEL[k].Transducer ? hdr->CHANNEL[k].Transducer : ""));
			
			mxSetCell(PhysDim1,k1,mxCreateString(PhysDim3(hdr->CHANNEL[k].PhysDimCode)));
			k1++;
		} 

		mxSetField(HDR,0,"LeadIdCode",LeadIdCode);
		mxSetField(HDR,0,"PhysDimCode",PhysDimCode);
		mxSetField(HDR,0,"GDFTYP",GDFTYP);
		mxSetField(HDR,0,"PhysMax",PhysMax);
		mxSetField(HDR,0,"PhysMin",PhysMin);
		mxSetField(HDR,0,"DigMax",DigMax);
		mxSetField(HDR,0,"DigMin",DigMin);
		mxSetField(HDR,0,"TOffset",Toffset);
		mxSetField(HDR,0,"Cal",Cal);
		mxSetField(HDR,0,"Off",Off);
		mxSetField(HDR,0,"Impedance",Impedance);
		mxSetField(HDR,0,"fZ",fZ);
		mxSetField(HDR,0,"Off",Off);
		mxSetField(HDR,0,"PhysDim",PhysDim1);
		mxSetField(HDR,0,"Transducer",Transducer);
		mxSetField(HDR,0,"Label",Label);

		const char* field[] = {"XYZ",NULL};
		for (numfields=0; field[numfields++] != 0; );
		tmp = mxCreateStructMatrix(1, 1, --numfields, field);
		mxSetField(tmp,0,"XYZ",ELEC_POS);
/*
		mxSetField(tmp,0,"Orientation",ELEC_Orient);
		mxSetField(tmp,0,"Area",ELEC_Area);
*/
		mxSetField(HDR,0,"ELEC",tmp);

		const char* field2[] = {"SPR",NULL};
		for (numfields=0; field2[numfields++] != 0; );
		tmp2 = mxCreateStructMatrix(1, 1, --numfields, field2);
		mxSetField(tmp2,0,"SPR",SPR);
		if (hdr->AS.bci2000!=NULL) {
			mxAddField(tmp2, "BCI2000");
			mxSetField(tmp2,0,"BCI2000",mxCreateString(hdr->AS.bci2000));
		}
		if (hdr->TYPE==Sigma) {	
			mxAddField(tmp2, "H1");
			mxSetField(tmp2,0,"H1",mxCreateString((char*)hdr->AS.Header));
		}	
		mxSetField(HDR,0,"AS",tmp2);
				
		/* FLAG */
		const char* field3[] = {"UCAL","OVERFLOWDETECTION","ROW_BASED_CHANNELS",NULL};
		for (numfields=0; field3[numfields++] != 0; );
		Flag = mxCreateStructMatrix(1, 1, --numfields, field3);
#ifdef MX_API_VER
//#if 1
                // Matlab, Octave 3.6.1 
       		mxSetField(Flag,0,"UCAL",mxCreateLogicalScalar(hdr->FLAG.UCAL));
        	mxSetField(Flag,0,"OVERFLOWDETECTION",mxCreateLogicalScalar(hdr->FLAG.OVERFLOWDETECTION));
        	mxSetField(Flag,0,"ROW_BASED_CHANNELS",mxCreateLogicalScalar(hdr->FLAG.ROW_BASED_CHANNELS));
#else 
                // mxCreateLogicalScalar are not included in Octave 3.0 
	        mxSetField(Flag,0,"UCAL",mxCreateDoubleScalar(hdr->FLAG.UCAL));
       		mxSetField(Flag,0,"OVERFLOWDETECTION",mxCreateDoubleScalar(hdr->FLAG.OVERFLOWDETECTION));
       		mxSetField(Flag,0,"ROW_BASED_CHANNELS",mxCreateDoubleScalar(hdr->FLAG.ROW_BASED_CHANNELS));
#endif
		mxSetField(HDR,0,"FLAG",Flag);

		/* Filter */ 
		const char *filter_fields[] = {"HighPass","LowPass","Notch",NULL};
		for (numfields=0; filter_fields[numfields++] != 0; );
		Filter = mxCreateStructMatrix(1, 1, --numfields, filter_fields);
		mxSetField(Filter,0,"LowPass",LowPass);
		mxSetField(Filter,0,"HighPass",HighPass);
		mxSetField(Filter,0,"Notch",Notch);
		mxSetField(HDR,0,"Filter",Filter);

		/* annotation, marker, event table */
		const char *event_fields[] = {"SampleRate","TYP","POS","DUR","CHN","Desc",NULL};
		
		if (hdr->EVENT.DUR == NULL)
			EVENT = mxCreateStructMatrix(1, 1, 3, event_fields);
		else {	
			EVENT = mxCreateStructMatrix(1, 1, 5, event_fields);
			mxArray *DUR = mxCreateDoubleMatrix(hdr->EVENT.N,1, mxREAL);
			mxArray *CHN = mxCreateDoubleMatrix(hdr->EVENT.N,1, mxREAL);
			for (k=0; k<hdr->EVENT.N; ++k) {
				*(mxGetPr(DUR)+k) = (double)hdr->EVENT.DUR[k];
				*(mxGetPr(CHN)+k) = (double)hdr->EVENT.CHN[k];  // channels use a 1-based index, 0 indicates all channels
			} 
			mxSetField(EVENT,0,"DUR",DUR);
			mxSetField(EVENT,0,"CHN",CHN);
		}

		if (hdr->EVENT.CodeDesc != NULL) {
			mxAddField(EVENT, "CodeDesc");
			mxArray *CodeDesc = mxCreateCellMatrix(hdr->EVENT.LenCodeDesc-1,1);
			for (k=1; k < hdr->EVENT.LenCodeDesc; ++k) {
				mxSetCell(CodeDesc,k-1,mxCreateString(hdr->EVENT.CodeDesc[k]));
			} 
			mxSetField(EVENT,0,"CodeDesc",CodeDesc);
		}	

		mxArray *TYP = mxCreateDoubleMatrix(hdr->EVENT.N,1, mxREAL);
		mxArray *POS = mxCreateDoubleMatrix(hdr->EVENT.N,1, mxREAL);

		for (k=0; k<hdr->EVENT.N; ++k) {
			*(mxGetPr(TYP)+k) = (double)hdr->EVENT.TYP[k];
			*(mxGetPr(POS)+k) = (double)hdr->EVENT.POS[k]+1;   // conversion from 0-based to 1-based indexing 
		} 
		mxSetField(EVENT,0,"TYP",TYP);
		mxSetField(EVENT,0,"POS",POS);

#if (BIOSIG_VERSION >= 10500)
		if (hdr->EVENT.TimeStamp) {
			mxArray *TimeStamp = mxCreateDoubleMatrix(hdr->EVENT.N,1, mxREAL);
			for (k=0; k<hdr->EVENT.N; ++k) {
				*(mxGetPr(TimeStamp)+k) = ldexp(hdr->EVENT.TimeStamp[k],-32);
			} 
			mxAddField(EVENT, "TimeStamp");
			mxSetField(EVENT,0,"TimeStamp",TimeStamp);
		}	
#endif
		mxSetField(EVENT,0,"SampleRate",mxCreateDoubleScalar(hdr->EVENT.SampleRate));
		mxSetField(HDR,0,"EVENT",EVENT);

		/* Record identification */ 
		const char *ID_fields[] = {"Recording","Technician","Hospital","Equipment","IPaddr",NULL};
		for (numfields=0; ID_fields[numfields++] != 0; );
		ID = mxCreateStructMatrix(1, 1, --numfields, ID_fields);
		mxSetField(ID,0,"Recording",mxCreateString(hdr->ID.Recording));
		mxSetField(ID,0,"Technician",mxCreateString(hdr->ID.Technician));
		mxSetField(ID,0,"Hospital",mxCreateString(hdr->ID.Hospital));
		mxSetField(ID,0,"Equipment",mxCreateString((char*)&hdr->ID.Equipment));
		int len = 4; 
		uint8_t IPv6=0;
		for (k=4; k<16; k++) IPv6 |= hdr->IPaddr[k];
		if (IPv6) len=16; 
		mxArray *IPaddr = mxCreateNumericMatrix(1,len,mxUINT8_CLASS,mxREAL);
		memcpy(mxGetData(IPaddr),hdr->IPaddr,len);
		mxSetField(ID,0,"IPaddr",IPaddr); 
		mxSetField(HDR,0,"REC",ID);

		/* Patient Information */ 
		const char *patient_fields[] = {"Sex","Handedness","Id","Name","Weight","Height","Birthday",NULL};
		for (numfields=0; patient_fields[numfields++] != 0; );
		Patient = mxCreateStructMatrix(1, 1, --numfields, patient_fields);
		const char *strarray;
#ifdef __LIBBIOSIG2_H__
		strarray = biosig_get_patient_name(hdr);
		mxSetField(Patient,0,"Name",mxCreateCharMatrixFromStrings(strarray!=NULL, &strarray));

		strarray = biosig_get_patient_id(hdr);
		mxSetField(Patient,0,"Id",mxCreateCharMatrixFromStrings(strarray!=NULL, &strarray));
#else
		strarray = hdr->Patient.Name;
		mxSetField(Patient,0,"Name",mxCreateCharMatrixFromStrings(strarray!=NULL, &strarray));

		strarray = hdr->Patient.Id;
		mxSetField(Patient,0,"Id",mxCreateCharMatrixFromStrings(strarray!=NULL, &strarray));
#endif
		mxSetField(Patient,0,"Handedness",mxCreateDoubleScalar(hdr->Patient.Handedness));

		mxSetField(Patient,0,"Sex",mxCreateDoubleScalar(hdr->Patient.Sex));
		mxSetField(Patient,0,"Weight",mxCreateDoubleScalar((double)hdr->Patient.Weight));
		mxSetField(Patient,0,"Height",mxCreateDoubleScalar((double)hdr->Patient.Height));
		mxSetField(Patient,0,"Birthday",mxCreateDoubleScalar(ldexp(hdr->Patient.Birthday,-32)));

		double d;
		if (hdr->Patient.Weight==0)		d = NAN;	// not-a-number		
		else if (hdr->Patient.Weight==255)	d = INFINITY;	// Overflow
		else					d = (double)hdr->Patient.Weight;
		mxSetField(Patient,0,"Weight",mxCreateDoubleScalar(d));
			
		if (hdr->Patient.Height==0)		d = NAN;	// not-a-number		
		else if (hdr->Patient.Height==255)	d = INFINITY;	// Overflow
		else					d = (double)hdr->Patient.Height;
		mxSetField(Patient,0,"Height",mxCreateDoubleScalar(d));
	
		/* Manufacturer Information */ 
		const char *manufacturer_fields[] = {"Name","Model","Version","SerialNumber",NULL};
		for (numfields=0; manufacturer_fields[numfields++] != 0; );
		Manufacturer = mxCreateStructMatrix(1, 1, --numfields, manufacturer_fields);

#ifdef __LIBBIOSIG2_H__
		strarray = biosig_get_manufacturer_name(hdr);
		mxSetField(Manufacturer,0,"Name",mxCreateCharMatrixFromStrings(strarray!=NULL, &strarray));

		strarray = biosig_get_manufacturer_model(hdr);
		mxSetField(Manufacturer,0,"Model",mxCreateCharMatrixFromStrings(strarray!=NULL, &strarray));

		strarray = biosig_get_manufacturer_version(hdr);
		mxSetField(Manufacturer,0,"Version",mxCreateCharMatrixFromStrings(strarray!=NULL, &strarray));

		strarray = biosig_get_manufacturer_serial_number(hdr);
		mxSetField(Manufacturer,0,"SerialNumber",mxCreateCharMatrixFromStrings(strarray!=NULL, &strarray));

#else
		strarray = hdr->ID.Manufacturer.Name;
		mxSetField(Manufacturer,0,"Name",mxCreateCharMatrixFromStrings(strarray!=NULL, &strarray));

		strarray = hdr->ID.Manufacturer.Model;
		mxSetField(Manufacturer,0,"Model",mxCreateCharMatrixFromStrings(strarray!=NULL, &strarray));

		strarray = hdr->ID.Manufacturer.Version;
		mxSetField(Manufacturer,0,"Version",mxCreateCharMatrixFromStrings(strarray!=NULL, &strarray));

		strarray = hdr->ID.Manufacturer.SerialNumber;
		mxSetField(Manufacturer,0,"SerialNumber",mxCreateCharMatrixFromStrings(strarray!=NULL, &strarray));
#endif
		mxSetField(HDR,0,"Manufacturer",Manufacturer);


	if (VERBOSE_LEVEL>7) 
		fprintf(stdout,"[148] going for SCLOSE\n");

		mxSetField(HDR,0,"Patient",Patient);

#ifndef mexSOPEN
		plhs[1] = HDR; 
	}
#else
	plhs[0] = HDR; 
#endif

	if (VERBOSE_LEVEL>7) fprintf(stdout,"[151] going for SCLOSE\n");
#ifdef CHOLMOD_H
	hdr->Calib = NULL; // is refering to &RR, do not destroy
#endif
	if (VERBOSE_LEVEL>7) fprintf(stdout,"[156] SCLOSE finished\n");
	destructHDR(hdr);
	hdr = NULL; 
	if (VERBOSE_LEVEL>7) fprintf(stdout,"[157] SCLOSE finished\n");
};

