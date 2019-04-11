/*

    $Id: test_scp_decode.c,v 1.2 2007-08-31 13:18:15 schloegl Exp $
    Copyright (C) 2007 Eugenio Cervesato
    Modified by Alois Schloegl

    This function is part of the "BioSig for C/C++" repository
    (biosig4c++) at http://biosig.sf.net/


    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 */
#if defined(_MSC_VER) && (_MSC_VER < 1600)
#define __LITTLE_ENDIAN=1
#define __BIG_ENDIAN=0
USEUNIT("t210/scp-decode.cpp");
#include "patient.h"
#include <alloc.h>
#endif

#include "../biosig.h"
#include "../t210/structures.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int scp_decode(HDRTYPE* hdr, pointer_section *info_sections, DATA_DECODE &info_decoding, DATA_RECORD &info_recording, DATA_INFO &info_textual, bool &add_filter);

//#include "patient.h"
int main(int argc, char* argv[])
{
	HDRTYPE *hdr ;
        pointer_section *section;
        DATA_DECODE decode;
        DATA_RECORD record;
        DATA_INFO textual;
        bool add_filter=true;
        FILE *in;

        hdr = (HDRTYPE *)malloc(sizeof(HDRTYPE));
        if( (section = (pointer_section *)malloc(sizeof(pointer_section)*12)) ==NULL)
        {
               return 2;
        }
        hdr->FileName=argv[1];
        hdr->FILE.OPEN=0;
        hdr->FILE.COMPRESSION=0;
        scp_decode(hdr, section, decode, record, textual, add_filter);

        for (int i=0;i<decode.flag_Res.number_samples;i+=2500)
        for (int k=0;k<8;k+=3)
	{
        	int x=decode.Reconstructed[i+k*5000];         // data returned is in microVolt -> DATA CONSINSTENT
        	fprintf(stdout,"%i %i %i\n",i,k,x);
	}
       	return(0);
}

