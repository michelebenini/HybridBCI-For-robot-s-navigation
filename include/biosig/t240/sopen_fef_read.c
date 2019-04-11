/*

    sandbox is used for development and under constraction work
    The functions here are either under construction or experimental. 
    The functions will be either fixed, then they are moved to another place;
    or the functions are discarded. Do not rely on the interface in this function


    $Id$
    Copyright (C) 2009,2012 Alois Schloegl <alois.schloegl@gmail.com>
    This file is part of the "BioSig for C/C++" repository 
    (biosig4c++) at http://biosig.sf.net/ 

    BioSig is free software; you can redistribute it and/or
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


#include <ctype.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include "../biosig-dev.h"

#ifdef WITH_FEF
#include "SessionArchiveSection.h"
#endif


EXTERN_C void sopen_fef_read(HDRTYPE* hdr) {


#ifndef WITH_FEF

	B4C_ERRNUM = B4C_FORMAT_UNSUPPORTED;
	B4C_ERRMSG = "ASN1/FEF currently not supported";

#else 

	fprintf(stdout,"ASN1: Warning FEF/ASN.1 decoding very experimental. Tested only with TaskForceMonitor (TFM). You are warned. \n");

	static asn_TYPE_descriptor_t *pduType = &asn_DEF_SessionArchiveSection;
	SessionArchiveSection_t *SAS = NULL;
	SessionTestSection_t *STS = NULL;
	SessionPhaseSection_t *SPS = NULL; 	/* Decoded structure */
	asn_codec_ctx_t *opt_codec_ctx = 0;
	asn_dec_rval_t rval;
	long ival; 
	double val; 


	if (VERBOSE_LEVEL>7) fprintf(stdout,"ASN1: BER DECODING\n");
	
	size_t pos =32;	
	rval = ber_decode(0, pduType, (void **)&SAS, hdr->AS.Header+32, hdr->HeadLen-32);

	pos += rval.consumed;

	/* backup info for proper freeing of memery */
	hdr->aECG = malloc(sizeof(struct asn1));
	{
		struct asn1 *asn1info = (struct asn1*)hdr->aECG;
		asn1info->pduType = pduType; 
		asn1info->SAS = SAS; 
	}	

/*
	while (pos<hdr->HeadLen) {

fprintf(stdout,"%i/%i\n",pos,hdr->HeadLen);
	
		rval = ber_decode(opt_codec_ctx, pduType, (void **)&SAS, hdr->AS.Header+pos, hdr->HeadLen-32);
		pos += rval.consumed;
	}
*/
	if (VERBOSE_LEVEL>7) fprintf(stdout,"%i/%i\nASN1: BER DECODING DONE\n",(unsigned)pos,hdr->HeadLen);

if (VERBOSE_LEVEL>8) {
//	asn_fprint(stdout, &asn_DEF_SessionArchiveSection, SAS);

//	SAS->s_archive_id
	asn_fprint(stdout, &asn_DEF_FEFString, &SAS->s_archive_id);
	asn_fprint(stdout, &asn_DEF_FEFString, &SAS->s_archive_name);
	asn_fprint(stdout, &asn_DEF_FEFString, SAS->s_archive_comments);

	/**********************************
		Manufacturer information 
	 **********************************/	
	asn_fprint(stdout, &asn_DEF_ManufacturerSpecificSection, SAS->manufacturerspecific);

	/**********************************
		Health Care provider
	 **********************************/	
	asn_fprint(stdout, &asn_DEF_HealthCareProviderSection, SAS->healthcareprovider);
	asn_fprint(stdout, &asn_DEF_PatientDemographicsSection, &SAS->demographics);
}

	/**********************************
		demographic information 
	 **********************************/	
	if (!asn_INTEGER2long(SAS->demographics.sex, &ival))
		hdr->Patient.Sex = ival;

	{
		if (!asn_REAL2double(&SAS->demographics.patientweight->value, &val))
			hdr->Patient.Weight = (uint8_t)val;
		if (!asn_REAL2double(&SAS->demographics.patientheight->value, &val))
			hdr->Patient.Height = (uint8_t)val;

		char *str1 = (char*)SAS->demographics.characternamegroup->givenname.buf;
		char *str2 = (char*)SAS->demographics.characternamegroup->middlename.buf;
		char *str3 = (char*)SAS->demographics.characternamegroup->familyname.buf;
		size_t l1 = strlen(str1); 
		size_t l2 = strlen(str2); 
		size_t l3 = strlen(str3); 
	
		if (l1 <= MAX_LENGTH_NAME) {
			strcpy(hdr->Patient.Name, str1);
		}
		if (l1+l2+1 <= MAX_LENGTH_NAME) {
			hdr->Patient.Name[l1]=' ';
			strcpy(hdr->Patient.Name+1 + l1, str2);		// Flawfinder: ignore
		}
		if (l1+l2+l3+2 <= MAX_LENGTH_NAME) {
			hdr->Patient.Name[l1+l2+1]=' ';
			strcpy(hdr->Patient.Name+2+l1+l2, str3);	// Flawfinder: ignore
		}
	}
	
	/**********************************
		Test Section information 
	 **********************************/	

//	asn_fprint(stdout, &asn_DEF_SessionTestSection, SAS->sessions.list.array[0]);
//		asn_fprint(stdout, &asn_DEF_SessionTestSection, SAS->sessions.list.array[k]);
	if (VERBOSE_LEVEL>7) fprintf(stdout,"Number of TestSections %i\n",SAS->sessions.list.count);

	size_t N = SAS->sessions.list.count; 
	if (N>1)
		biosigERROR(hdr, B4C_FORMAT_UNSUPPORTED, "Error SOPEN(FEF): multiple test sections are not supported, yet");

	STS = SAS->sessions.list.array[0];
//	asn_fprint(stdout, &asn_DEF_SessionTestSection, STS);

	/******* Manufacturer ************/		
//	asn_fprint(stdout, &asn_DEF_MedicalDeviceSystemSection, &STS->medicaldevicesystem);
	strncpy(hdr->ID.Manufacturer._field, (const char*)STS->medicaldevicesystem.systemmodel.manufacturer.buf, MAX_LENGTH_MANUF);
	int LEN = strlen(hdr->ID.Manufacturer._field)+1; 
	hdr->ID.Manufacturer.Name = hdr->ID.Manufacturer._field;

	strncpy(hdr->ID.Manufacturer._field+LEN, (const char*)STS->medicaldevicesystem.systemmodel.model_number.buf, MAX_LENGTH_MANUF-LEN);
	hdr->ID.Manufacturer.Model = hdr->ID.Manufacturer._field+LEN;
	
	/******* Multimedia ************/		

	/******* Session Phase Section SPS ************/		

//	asn_fprint(stdout, &asn_DEF_SessionTestSection, SAS->sessions.list.array[0]);
//	asn_fprint(stdout, &asn_DEF_SessionTestSection, SAS->sessions.list.array[k]);
	N = STS->phases.list.count; 
	if (N>1)
		biosigERROR(hdr, B4C_FORMAT_UNSUPPORTED, "Error SOPEN(FEF): multiple test sections are not supported, yet");

	SPS = STS->phases.list.array[0];
//	asn_fprint(stdout, &asn_DEF_SessionPhaseSection, SPS);
//	asn_fprint(stdout, &asn_DEF_DescriptiveDataSection, &SPS->descriptivedata);

	/**************** Descriptive Data Section DescDS ****************/

//	SPS->descriptivedata->realtimesadescs
//	asn_fprint(stdout, &asn_DEF_SessionTestSection, SAS->sessions.list.array[0]);
	N = SPS->descriptivedata.realtimesadescs->list.count;
	
	hdr->NS = N;
	hdr->CHANNEL = (CHANNEL_TYPE*)calloc(hdr->NS,sizeof(CHANNEL_TYPE));
	typeof(hdr->NS) k; 
	uint64_t FsN=1,FsD=1;
	for (k=0; k<hdr->NS; k++)	{
	
			RealTimeSampleArrayDescriptiveDataSection_t *RTSADDS = SPS->descriptivedata.realtimesadescs->list.array[k];
			if (VERBOSE_LEVEL>8) {
				fprintf(stdout,"[FEF 212] #=%i/%i\n",k,hdr->NS);
				asn_fprint(stdout, &asn_DEF_RealTimeSampleArrayDescriptiveDataSection, RTSADDS);
			}	

			CHANNEL_TYPE *hc = hdr->CHANNEL+k;

		      	strncpy(hc->Label, (const char*)RTSADDS->labelstring->buf, MAX_LENGTH_LABEL);

			if (!asn_INTEGER2long(RTSADDS->unitcode,&ival))
			      	hc->PhysDimCode= (uint16_t)ival;
			else
			      	hc->PhysDimCode= 0;

			if (!hc->PhysDimCode) 
				hc->PhysDimCode = PhysDimCode((const char*)RTSADDS->unitlabelstring->buf);
			
			//******************* samplerate *****************/
			long int n=0, d=0;
			asn_INTEGER2long(&RTSADDS->sampleperiod.denominator, &d);
			asn_INTEGER2long(&RTSADDS->sampleperiod.numerator,&n);
			if (VERBOSE_LEVEL>7) fprintf(stdout,"#%i: %li %li\n",k,n,d);
			if (n && d) {
				FsN = lcm(FsN,n);
				FsD = lcm(FsD,d);
			}
			hc->SPR = d;
			 
			// double Fs = (double)n / d;

			//******************* scaling *****************/
	      		hc->PhysMax   = +100;
		      	hc->PhysMin   = -100;
		      	hc->DigMax    = +2047;
		      	hc->DigMin    = -2048;
			if (asn_REAL2double(&RTSADDS->scaleandrangespec.lowerabsolutevalue,&hc->PhysMin)
			 || asn_REAL2double(&RTSADDS->scaleandrangespec.upperabsolutevalue,&hc->PhysMax)
			 || asn_REAL2double(&RTSADDS->scaleandrangespec.lowervaluescaled,&hc->DigMin)
			 || asn_REAL2double(&RTSADDS->scaleandrangespec.uppervaluescaled,&hc->DigMax) )
				fprintf(stderr,"Warning (FEF): scaling factors in channel %i not available\n",k+1);

			hc->Cal = (hc->PhysMax-hc->PhysMin)/(hc->DigMax-hc->DigMin);
			hc->Off =  hc->PhysMin-hc->Cal*hc->DigMin;
			
			//******************* bits *****************/
			long int bits=-1,dt=-1;
			asn_INTEGER2long(&RTSADDS->saspecification.storagesize,&bits);
			asn_INTEGER2long(&RTSADDS->saspecification.storagedatatype,&dt);
			hc->GDFTYP = 0;
			if (dt==2) 	// ieee 754
			{
				if (bits==32)
					hc->GDFTYP = 16; // float32
				else if (bits==64)
					hc->GDFTYP = 17; // float64
				else if (bits==128)
					hc->GDFTYP = 18; // float128
			}
			else if (dt==1) // signed 
			{
				if (bits==8) 		hc->GDFTYP = 1; // 
				else if (bits==16)	hc->GDFTYP = 3; //
				else if (bits==24)	hc->GDFTYP = 255+24; //
				else if (bits==32)	hc->GDFTYP = 5; //
				else if (bits==64)	hc->GDFTYP = 7; //
			}		
			else if (dt==0) // unsigned 
			{
				if (bits==8)		hc->GDFTYP = 2; // 
				else if (bits==16)	hc->GDFTYP = 4; //
				else if (bits==24)	hc->GDFTYP = 511+24; //
				else if (bits==32)	hc->GDFTYP = 6; //
				else if (bits==64)	hc->GDFTYP = 8; //
			}
			long int arraysize=0; 
			asn_INTEGER2long(&RTSADDS->saspecification.storagesize,&arraysize);
			if (!arraysize) hc->SPR = arraysize; 
			// else 
			/* Number of samples of this signal in a subblock of SAMDB. 
			   Arraysize is actually redundant information in other cases 
			   than distribution sample array. Therefore, if arraysize = 0, 
			   the number of samples is calculated as <subblocklength> 
			   divided by <sampleperiod> of the signal.
			*/

			//******************* filters *****************/
			if (RTSADDS->sasignalfrequency) {
				double d;
				asn_REAL2double(&RTSADDS->sasignalfrequency->lowedgefreq, &d);
				hc->HighPass = d;
				asn_REAL2double(&RTSADDS->sasignalfrequency->highedgefreq, &d);
				hc->LowPass = d;
			} else 	
			{
			      	hc->HighPass  = -1;
			      	hc->LowPass   = INFINITY;
			}

			// TODO: MetricCalEntry, MetricCalType,
			
			// TODO: 				
		      	hc->LeadIdCode= 0;
	      		strcpy(hc->Transducer, "");
		      	hc->bi 	      = 0;
	      		hc->OnOff     = 1;
	      		hc->Notch     = -1;
		      	hc->Impedance = INFINITY;
		      	hc->XYZ[0] 	= 0.0;
	      		hc->XYZ[1] 	= 0.0;
		      	hc->XYZ[2] 	= 0.0;
	}
	size_t d = gcd(FsD,FsN);
	if (VERBOSE_LEVEL>7) fprintf(stdout,"Fs=%i/%i\n",(unsigned)FsN,(unsigned)FsD);
	hdr->SampleRate = ((double)(FsD/d))/((double)(FsN/d));
	hdr->SPR = FsD;

	/************** Measured Data Section MeasDS ***********************/
	N = SPS->measureddata.list.count;
	if (VERBOSE_LEVEL>7) fprintf(stdout,"Number of MeasuredData %i\n",(unsigned)N);
//	asn_fprint(stdout, &asn_DEF_SessionPhaseSection, SPS);
	
	for (k=0; k<N; k++) 
	{
		long int nrec,spr,n,d; 

		MeasuredDataSection_t *MDS = SPS->measureddata.list.array[k];
//		asn_fprint(stdout, &asn_DEF_RealTimeSampleArrayMeasuredDataSection, RTSAMDS);

		unsigned n2; 
		size_t listlen = MDS->realtimesas->list.count; 
		hdr->SPR = 1; 
		for (n2=0; n2 <listlen ; n2++) {
			CHANNEL_TYPE *hc = hdr->CHANNEL+n2;
			RealTimeSampleArrayMeasuredDataSection_t *RTSAMDS = MDS->realtimesas->list.array[n2];
//			if (VERBOSE_LEVEL>7)
//				asn_fprint(stdout, &asn_DEF_RealTimeSampleArrayMeasuredDataSection, MDS->realtimesas->list.array[n2]);

			asn_INTEGER2long(&RTSAMDS->numberofsubblocks,&nrec);
			asn_INTEGER2long(&RTSAMDS->subblocklength.numerator,&n);
			asn_INTEGER2long(&RTSAMDS->subblocklength.denominator,&d);
			asn_INTEGER2long(&RTSAMDS->subblocksize,&spr);

			// &RTSAMDS->metriclist //
			if (VERBOSE_LEVEL>7) 
				fprintf(stdout,"%i/%i blk: %i/%i #subblocks:%i subblocklength:%i/%i subblocksize:%i size:%i \n",(int)n2,(int)listlen,(int)k,(int)N,(int)nrec,(int)n,(int)d,(int)spr,(int)RTSAMDS->data.size);
			//RTSAMDS->data->buf
			//RTSAMDS->data.size
			
			hc->bufptr = RTSAMDS->data.buf; 
			hc->SPR = (RTSAMDS->data.size<<3)/GDFTYP_BITS[hc->GDFTYP]; 
			if (hdr->SPR < hc->SPR) hdr->SPR = hc->SPR;
		}
	}

	for (k=0; k<hdr->NS; k++) {
		CHANNEL_TYPE *hc = hdr->CHANNEL+k;
		double d = ((double)hdr->SPR)/hc->SPR; 
		hc->OnOff = (d==floor(d)) && hc->SPR>8;  // do not show non-signal data. TODO: use this data to decode event information
		hc->bi  = 0; 
		hc->bi8 = 0; 

	}

	hdr->NRec = 1; 
	hdr->AS.first = 0;
	hdr->AS.length = 1;
 	 
 	 
	/****** Notes Section information SNS **********************************/	
	if (VERBOSE_LEVEL>7)
	for (k=0; k<SAS->notes->list.count; k++) {
		asn_fprint(stdout, &asn_DEF_SessionNotesSection, SAS->notes->list.array[k++]);
	}
	
	/******* post checks ************/		

/*
	if(errno) {
		fprintf(stdout,"Error FEF: %s\n",strerror(errno));
		// Error message is already printed 
//		exit(-1);
	}
*/

	/* Check ASN.1 constraints */
	char errbuf[128];
	size_t errlen = sizeof(errbuf);
	if(asn_check_constraints(pduType, SAS,
		errbuf, &errlen)) {
		fprintf(stdout, "SOPEN_FEF_READ: ASN.1 constraint "
			"check failed: %s\n", errbuf);
	}

//	asn_fprint(stdout, pduType, structure);
//	ASN_STRUCT_FREE(*pduType, SAS);

#endif 
	
}

EXTERN_C void sclose_fef_read(HDRTYPE* hdr) {

	if (VERBOSE_LEVEL>7)
		fprintf(stdout,"sclose_FEF_read\n");
#ifdef WITH_FEF
	
	if (hdr->aECG) {
		struct asn1 *asn1info = (struct asn1*)hdr->aECG;	
		ASN_STRUCT_FREE((*(asn_TYPE_descriptor_t *)(asn1info->pduType)), ((SessionArchiveSection_t*)asn1info->SAS));
		free(hdr->aECG);
		hdr->aECG = NULL; 
	}	
#endif 
}
