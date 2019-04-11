/*
%
% $Id$
% Copyright (C) 2008,2009 Alois Schloegl <a.schloegl@ieee.org>
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


// swig.i

%module biosig
%{
#include "../biosig.h"
%}


%include <inttypes.i>


typedef int64_t gdf_time; 	/* gdf time is represented in 64 bits */

typedef int64_t nrec_t; 	/* type for number of records */


	/* list of file formats */
enum FileFormat {
	noFile, unknown,
	ABF, ACQ, ACR_NEMA, AIFC, AIFF, AINF, alpha, ARFF, 
	ASCII_IBI, ASCII, AU, ASF, ATES, ATF, AVI, Axona,
	BCI2000, BDF, BESA, BIN, BKR, BLSC, BMP, BNI, BSCS,
	BrainVision, BrainVisionVAmp, BrainVisionMarker, BZ2,
	CDF, CFS, CFWB, CNT, CTF, DICOM, DEMG,
	EBS, EDF, EEG1100, EEProbe, EEProbe2, EEProbeAvr, EGI,
	EGIS, ELF, EMBLA, ePrime, ET_MEG, ETG4000, EVENT, EXIF,
	FAMOS, FEF, FITS, FLAC, GDF, GDF1,
	GIF, GTF, GZIP, HDF, HL7aECG, HEKA, 
	ISHNE, ITX, JPEG, JSON, Lexicor,
	Matlab, MFER, MIDI, MIT, MM, MSI, MSVCLIB, MS_LNK, 
	native, NeuroLoggerHEX, NetCDF, NEURON, NEX1, NIFTI, 
	OGG, OpenXDF,
	PBMA, PBMN, PDF, PDP, Persyst, PGMA, PGMB,
	PLEXON, PNG, PNM, POLY5, PPMA, PPMB, PS,
	RDF, RIFF,
	SASXPT, SCP_ECG, SIGIF, Sigma, SMA, SND, SQLite, 
	SPSS, STATA, SVG, SXI, SYNERGY,
	TIFF, TMS32, TMSiLOG, TRC, UNIPRO, VRML, VTK,
	WAV, WG1, WinEEG, WMF, XML, XPM,
	Z, ZIP, ZIP2, 
};


typedef struct CHANNEL_STRUCT {
	double 		PhysMin;		/* physical minimum */
	double 		PhysMax;		/* physical maximum */
	double 		DigMin;			/* digital minimum */
	double	 	DigMax;			/* digital maximum */
	double		Cal;			/* gain factor */
	double		Off;			/* bias */

	char		OnOff;
	char		Label[MAX_LENGTH_LABEL+1]; 		/* Label of channel */
	uint16_t	LeadIdCode;				/* Lead identification code */
	char 		Transducer[MAX_LENGTH_TRANSDUCER+1];	/* transducer e.g. EEG: Ag-AgCl electrodes */
//	char 		PhysDim[MAX_LENGTH_PHYSDIM+1] ;		/* physical dimension */
			/*PhysDim is now obsolete - use function PhysDim3(PhysDimCode) instead */
	uint16_t	PhysDimCode;		/* code for physical dimension */
	/* char* 	PreFilt;		// pre-filtering */

	float 		TOffset;		/* time delay of sampling */
	float 		LowPass;		/* lowpass filter */
	float 		HighPass;		/* high pass */
	float 		Notch;			/* notch filter */
	float 		XYZ[3];			/* sensor position */
//	float 		Orientation[3];		// sensor direction
//	float 		Area;			// area of sensor (e.g. for MEG)

	union {
        /* context specific channel information */
	float 		Impedance;   		/* Electrode Impedance in Ohm, defined only if PhysDim = _Volt */
	float 		fZ;	   		/* ICG probe frequency, defined only if PhysDim = _Ohm */
	};

	uint16_t 	GDFTYP;			/* data type */
	uint32_t 	SPR;			/* samples per record (block) */
} CHANNEL_TYPE;



/*
	This structure defines the general (fixed) header  
*/
typedef struct {
	float 		VERSION;	/* GDF version number */ 
	const char* 	FileName;
	enum FileFormat TYPE; 		/* type of file format */
	
	struct {
		size_t 			size[2]; /* size {rows, columns} of data block	 */
		biosig_data_type* 	block; 	 /* data block */
	} data;

	uint8_t 	IPaddr[16]; 	/* IP address of recording device (if applicable) */
	double 		SampleRate;	/* Sampling rate */
	int64_t  	NRec;		/* number of records/blocks -1 indicates length is unknown. */	
	gdf_time 	T0; 		/* starttime of recording */
	uint32_t 	HeadLen;	/* length of header in bytes */
	uint32_t 	SPR;		/* samples per block (when different sampling rates are used, this is the LCM(CHANNEL[..].SPR) */
	uint32_t  	LOC[4];		/* location of recording according to RFC1876 */
	uint16_t 	NS;		/* number of channels */
	int16_t 	tzmin; 		/* time zone (minutes of difference to UTC */

#ifdef CHOLMOD_H
	cholmod_sparse  *Calib;                  /* re-referencing matrix */
#endif 	
	CHANNEL_TYPE 	*rerefCHANNEL;  

	/* Patient specific information */
	struct {
		gdf_time 	Birthday; 	/* Birthday of Patient */
		// 		Age;		/* the age is HDR.T0 - HDR.Patient.Birthday, even if T0 and Birthday are not known */ 		
		uint16_t	Headsize[3]; 	/* circumference, nasion-inion, left-right mastoid in millimeter;  */
		char		Name[MAX_LENGTH_NAME+1]; /* because for privacy protection it is by default not supported, support is turned on with FLAG.ANONYMOUS */
//		char*		Name;	/* because for privacy protection it is by default not supported, support is turned on with FLAG.ANONYMOUS */
		char		Id[MAX_LENGTH_PID+1];	/* patient identification, identification code as used in hospital  */
		uint8_t		Weight;		/* weight in kilograms [kg] 0:unkown, 255: overflow  */
		uint8_t		Height;		/* height in centimeter [cm] 0:unkown, 255: overflow  */
		//		BMI;		/* the body-mass index = weight[kg]/height[m]^2 */
		/* Patient classification */
		int	 	Sex;		/* 0:Unknown, 1: Male, 2: Female  */
		int		Handedness;	/* 0:Unknown, 1: Right, 2: Left, 3: Equal */
		int		Smoking;	/* 0:Unknown, 1: NO, 2: YES */
		int		AlcoholAbuse;	/* 0:Unknown, 1: NO, 2: YES */
		int		DrugAbuse;	/* 0:Unknown, 1: NO, 2: YES */
		int		Medication;	/* 0:Unknown, 1: NO, 2: YES */
		struct {
			int 	Visual;		/* 0:Unknown, 1: NO, 2: YES, 3: Corrected */
			int 	Heart;		/* 0:Unknown, 1: NO, 2: YES, 3: Pacemaker */
		} Impairment;
	} Patient; 
	
	struct {
		char		Recording[MAX_LENGTH_RID+1]; 	/* HL7, EDF, GDF, BDF replaces HDR.AS.RID */
		char 		Technician;
		char* 		Hospital; 	
		uint64_t 	Equipment; 	/* identifies this software */
		struct {
			/* see 
				SCP: section1, tag14, 
				MFER: tag23:  "Manufacturer^model^version number^serial number"
				GDF: tag3:  "Manufacturer\0model\0version\0number\0serial number\0"
			*/	
//			char	_field[MAX_LENGTH_MANUF+1];	/* buffer */
			char*	Name;  
			char*	Model;
			char*	Version;
			char*	SerialNumber;
		} Manufacturer;  
	} ID;

	/* position of electrodes; see also HDR.CHANNEL[k].XYZ */
	struct {
		float		REF[3];	/* XYZ position of reference electrode */
		float		GND[3];	/* XYZ position of ground electrode */
	} ELEC;

	/*	EVENTTABLE */
	struct {
		double  	SampleRate;	/* for converting POS and DUR into seconds  */
		uint16_t 	*TYP;	/* defined at ../biosig4matlab/doc/eventcodes.txt */
		uint32_t 	*POS;	/* starting position [in samples] */
		uint32_t 	*DUR;	/* duration [in samples] */
		uint16_t 	*CHN;	/* channel number; 0: all channels  */
#if (BIOSIG_VERSION >= 10500)
		gdf_time        *TimeStamp ATT_ALI;  /* store time stamps */
#endif
		char		**CodeDesc;	/* describtion of "free text"/"user specific" events (encoded with TYP=0..255 */
		uint32_t  	N;	/* number of events */
		uint16_t	LenCodeDesc;	/* length of CodeDesc Table */
	} EVENT; 

	struct {	/* flags */
		char		OVERFLOWDETECTION; 	/* overflow & saturation detection 0: OFF, !=0 ON */
		char		UCAL; 		/* UnCalibration  0: scaling  !=0: NO scaling - raw data return  */
		char		ANONYMOUS; 	/* 1: anonymous mode, no personal names are processed */ 
		char		ROW_BASED_CHANNELS;	/* 0: column-based data [default]; 1: row-based data */ 
		char		TARGETSEGMENT; /* in multi-segment files (like Nihon-Khoden, EEG1100), it is used to select a segment */ 
	} FLAG; 

	CHANNEL_TYPE *CHANNEL;
	  
	struct {	/* File specific data  */
#ifdef ZLIB_H
		gzFile		gzFID;
#endif
#ifdef _BZLIB_H
//		BZFILE*		bzFID;
#endif
		FILE* 		FID;		/* file handle  */
		size_t 		POS;		/* current reading/writing position [in blocks] */
//		int		Des;		/* file descriptor */
		uint8_t		OPEN; 		/* 0: closed, 1:read, 2: write */
		uint8_t		LittleEndian;
		uint8_t		COMPRESSION;   /* 0: no compression 9: best compression */
//		int		DES;		/* descriptor for streams */
	} FILE; 

	/*	internal variables (not public)  */
	struct {
		const char*	B4C_ERRMSG;	/* error message */
//		char 		PID[MAX_LENGTH_PID+1];	/* use HDR.Patient.Id instead */
//		char* 		RID;		/* recording identification */ 
//		uint32_t 	spb;		/* total samples per block */
//		uint32_t 	bpb;  		/* total bytes per block */
//		uint32_t 	bpb8;  		/* total bits per block */
		uint8_t*	Header; 
//		uint8_t*	rawEventData;
//		uint8_t*	rawdata; 	/* raw data block */
//		char		flag_collapsed_rawdata; /*0 if rawdata contain obsolete channels, too. 	*/
//		nrec_t		first;		/* first block loaded in buffer - this is equivalent to hdr->FILE.POS */
//		nrec_t		length;		/* number of block(s) loaded in buffer */
		uint8_t*	auxBUF;		/* auxillary buffer - used for storing EVENT.CodeDesc, MIT FMT infor */
		char*		bci2000;
//		uint32_t	SegSel[5];	/* segment selection in a hirachical data formats, e.g. sweeps in HEKA/PatchMaster format */
		enum B4C_ERROR	B4C_ERRNUM;	/* error code */
//		char		flag_collapsed_rawdata; /* 0 if rawdata contain obsolete channels, too. 	*/
	} AS;
	
	void *aECG;
	
} HDRTYPE;

HDRTYPE* constructHDR(const unsigned NS, const unsigned N_EVENT);
void 	 destructHDR(HDRTYPE* hdr);
HDRTYPE* sopen(const char* FileName, const char* MODE, HDRTYPE* hdr);
int 	sclose(HDRTYPE* hdr);
size_t	sread(biosig_data_type* data, size_t start, size_t length, HDRTYPE* hdr);
size_t  swrite(const biosig_data_type *data, size_t nelem, HDRTYPE* hdr);
int	seof(HDRTYPE* hdr);
void	srewind(HDRTYPE* hdr);
int 	sseek(HDRTYPE* hdr, long int offset, int whence);
long int stell(HDRTYPE* hdr);
int 	serror2(HDRTYPE* hdr);
int	hdr2ascii(HDRTYPE* hdr, FILE *fid, int verbosity);

int RerefCHANNEL(HDRTYPE *hdr, void *ReRef, char rrtype);
const char* GetFileTypeString(enum FileFormat FMT);

uint16_t PhysDimCode(char* PhysDim0);
char* 	PhysDim3(uint16_t PhysDimCode);

/*
HDRTYPE* sopen(char *filename);
%{
	HDRTYPE* sopen(char *filename)
	{
		HDRTYPE *hdr = constructHDR(0,0);
		hdr = sopen(filename, "r", hdr);
		return hdr;
        }
%}


int sclose(HDRTYPE *hdr);
%{
	int sclose(HDRTYPE *hdr)
	{
		sclose(hdr);
		destructHDR(hdr);
		return 0;
        }
%}
*/

void serror();
%{
	void _serror() {
		fprintf(stderr,"Use of SERROR is deprecated - use serror2(HDR) instead"); 	
		serror();
	}	
%}

void hdr2ascii(HDRTYPE* hdr, int verbosity);
%{
	void hdr2ascii(HDRTYPE* hdr, int verbosity)
	{
		hdr2ascii(hdr, stdout, verbosity);
        }
%}
