/*

    Copyright (C) 2009,2015,2016 Alois Schloegl <alois.schloegl@gmail.com>
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


#include "biosig-network.h"

#include <ctype.h>
#include <errno.h>
#ifndef _WIN32
# include <pwd.h>
#endif
#include <sys/stat.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>



int main (int argc, char *argv[]) {

	int s=0; 
    	register int sd;
	struct stat info;
	int state; 
	
	sd = bscs_connect(argv[1]); 

   	if(sd<0) {
      		perror("cannot connect ");
      		exit(1);
   	}

	fprintf(stdout,"client 122 %i %s <%s>\n",errno,strerror(errno),getenv("HOME"));
	
	char path2keys[1024];
	char *str;
#ifdef _WIN32
	str = strncpy(path2keys,getenv("HOME"),1023);
#elif 0
	// TODO: getpwuid can cause a memory leak in glibc2.15 and earlier 
	struct passwd *p = getpwuid(geteuid());	
	str = strncpy(path2keys,p->pw_dir,1023);
	//path2keys = strdup(p->pw_dir); 
	if (VERBOSE_LEVEL>7)  fprintf(stdout,"Name:%s\nPw:%s\nuid/gui: %i/%i\nreal name: %s\n$HOME:%s\nSHELL:%s\n",p->pw_name,p->pw_passwd,p->pw_uid,p->pw_gid,p->pw_gecos,p->pw_dir,p->pw_shell);
#endif
	str += strlen(path2keys);
	str[0] = FILESEP;
	str = (char*)memcpy(str+1,".biosig",7)+7;
	str[0] = FILESEP;
	str[1] = 0;
	size_t path2keysLength = strlen(path2keys); 
	stat(path2keys, &info);
	if (!(S_ISDIR(info.st_mode)))
		mkdir(path2keys,S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH); // read/write/search permissions for owner and group, and with read/search permissions for others.

	SERVER_STATE = STATE_INIT; 
	
	HDRTYPE *hdr=constructHDR(0,0); 
	uint64_t ID = 0; 

//	bscs_send_msg(sd,"Hello Server");
	fprintf(stdout,"client 125 %i %s\n",errno,strerror(errno));

	char clbuf[1000]; 
	char *cmd;
	while (1) {
		fprintf(stdout,"prompt$:");		
	     	cmd = fgets(clbuf,1000,stdin); 

		if (!strncasecmp(clbuf,"exit",4) || !strncasecmp(clbuf,"bye",3)) {
			fprintf(stdout,"\n%p>\n",cmd);
			bscs_close(sd);
			close(sd);
			break; 
		}		 			
//		fprintf(stdout,"cmd=<%s> %i\n",cmd,strncasecmp(cmd,"close",5));
		
		if (cmd[0]=='!') 
			system(cmd+1); 
		else if (!strncasecmp(cmd,"ow+c",4)) {
			ID = 0; 
			s=bscs_open(sd, &ID);
			fprintf(stdout,"ow ID=%016"PRIx64" s=%i %08x\n",ID,s,s);
			s=bscs_close(sd);
			fprintf(stdout,"c ID=%016"PRIx64" s=%i %08x\n",ID,s,s);
		}
		else if (!strncasecmp(cmd,"or+c",4)) {
			ID = htole64(0x233ab6dfc96f664fLL);
			s=bscs_open(sd, &ID);
			fprintf(stdout,"or ID=%016"PRIx64" s=%i %08x\n",ID,s,s);
//		   	hdr->TYPE = unknown; 
//			getfiletype(hdr);
			bscs_requ_hdr(sd,hdr);
			bscs_requ_dat(sd,0,hdr->NRec,hdr);
//			bscs_requ_evt(sd,hdr);
			s=bscs_close(sd);
			fprintf(stdout,"c ID=%016"PRIx64" s=%i %08x\n",ID,s,s);
		}
		else if (!strncasecmp(cmd,"openr",5)) {
			ID = htole64(0x233ab6dfc96f664fLL);

			ID = htole64(0x233ab6dfc96f664fLL);
			fprintf(stdout,"or ID=%016"PRIx64" s=%i %08x\n",ID,s,s);
			s=bscs_open(sd, &ID);
		}
		else if (!strncasecmp(cmd,"openw",5)) {
			char *fn = cmd+5;
			char *ix; 
		     	while (isspace(fn[0])) fn++; 
		     	ix = fn; 
		     	while (!isspace(ix[0])) ix++; 
		     	ix[0]=0;
		     	
		     	hdr = sopen(fn,"r",hdr);

			ID = 0; 
			s=bscs_open(sd, &ID);
			fprintf(stdout,"openw: ID=%016"PRIx64" s=%i %08x\n",ID,s,s);
		}
		else if (!strncasecmp(cmd,"sendhdr",7)) {
			s= bscs_send_hdr(sd, hdr);
		}
		else if (!strncasecmp(cmd,"senddat",7)) {
			s= bscs_send_dat(sd, hdr->AS.rawdata, hdr->AS.bpb*hdr->AS.length);
			fprintf(stdout,"sent dat %i\n",s);
		}
		else if (!strncasecmp(cmd,"sendevt",7)) {
			if (hdr->TYPE != GDF) hdrEVT2rawEVT(hdr); 
			if (hdr->EVENT.N>0) s= bscs_send_evt(sd, hdr);
		}
		
		else if (!strncasecmp(cmd,"hdr",3)) {
			bscs_requ_hdr(sd,hdr);
		}
		else if (!strncasecmp(cmd,"show1",5)) {
			hdr2ascii(hdr,stdout,1);
		}
		else if (!strncasecmp(cmd,"show2",5)) {
			hdr2ascii(hdr,stdout,2);
		}
		else if (!strncasecmp(cmd,"show3",5)) {
			hdr2ascii(hdr,stdout,3);
		}
		else if (!strncasecmp(cmd,"show4",5)) {
			hdr2ascii(hdr,stdout,4);
		}
		else if (!strncasecmp(cmd,"dat",3)) {
			bscs_requ_dat(sd,0,hdr->NRec,hdr);
		}
		else if (!strncasecmp(cmd,"evt",3)) {
			bscs_requ_evt(sd,hdr);
		}
		else if (!strncasecmp(cmd,"ow",2)) {
			ID = 0; 
			s=bscs_open(sd, &ID);
			fprintf(stdout,"ow ID=%016"PRIx64" s=%i %08x\n",ID,s,s);
		}
		else if (!strncasecmp(cmd,"close",5)) {
			s=bscs_close(sd);
			fprintf(stdout,"oc ID=%016"PRIx64" s=%i %08x\n",ID,s,s);
		}
		else if (!strncasecmp(cmd,"requ ",5)) {
			char *fn = cmd+5;
			char *ix; 
		     	while (isspace(fn[0])) fn++; 
		     	ix = fn; 
		     	while (!isspace(ix[0])) ix++; 
		     	ix[0]=0;
		     	cat64(fn,&ID);
			bscs_open(sd, &ID);
fprintf(stdout,"11 %i\n",hdr->EVENT.N);
			bscs_requ_hdr(sd,hdr);
		   	hdr->TYPE = unknown; 
fprintf(stdout,"12 %i\n",hdr->EVENT.N);
			getfiletype(hdr);
fprintf(stdout,"13 %s %i\n",GetFileTypeString(hdr->TYPE),hdr->EVENT.N);
			bscs_requ_hdr(sd,hdr);
fprintf(stdout,"14a %i %i %i\n",0,(int)hdr->NRec,hdr->EVENT.N);
			bscs_requ_dat(sd,0,hdr->NRec,hdr);
fprintf(stdout,"14b %i\n",hdr->EVENT.N);
//			bscs_requ_evt(sd,hdr);
fprintf(stdout,"14c %i\n",hdr->EVENT.N);
		   	if (hdr->TYPE == GDF) {
fprintf(stdout,"15\n");
				;//hdr2ascii(hdr,stdout,3);
fprintf(stdout,"16 %i\n",hdr->EVENT.N);
			}	
			bscs_close(sd);
		}
		else if (!strncasecmp(cmd,"send ",5)) {
			char *fn = cmd+5;
			char *ix; 
		     	while (isspace(fn[0])) fn++; 
		     	ix = fn; 
		     	while (!isspace(ix[0])) ix++; 
		     	ix[0]=0;
		     	
		     	hdr = sopen(fn,"r",NULL);
//			if (hdr->TYPE!=GDF) {
			if (serror2(hdr)) {
				stat(fn, &info);
		     		FILE *fid = fopen(fn,"r"); 
		     		if (fid==NULL)
			     		fprintf(stdout,"file %s not found\n",fn);
		     		else {	

			     		uint8_t *buf = (uint8_t*) malloc(info.st_size);
					ssize_t count = fread(buf, 1, info.st_size, fid); 
			     		fclose(fid); 

					ID = 0; 
					if ((state = bscs_open(sd,&ID))) // write-open
						fprintf(stdout,"BSCS_OPEN failed: state=%08x\n",state);
					else
					{
				     		/* write key file */ 
						char *tmp = strrchr(fn,FILESEP);
						if (tmp==NULL) tmp = fn; 
						strcpy(path2keys+path2keysLength,tmp);
			
						fid = fopen(path2keys,"w");	// TODO: prevent overwritting
						fprintf(fid,"key4biosig: host=%s ID=%016"PRIx64" ",argv[1],ID);
						fclose(fid);

						fprintf(stdout,"open_w ID=%016"PRIx64"\n",ID);
						hdr->HeadLen = count; 
						bscs_send_dat(sd, buf, count);
						bscs_close(sd);
					}	
					free(buf); 
		     		}
		     	}	
		     	else {
		     	
				sread_raw(0,hdr->NRec,hdr,1,NULL,0); 	// collapse rawdata (remove obsolete channels)
				size_t bpb = bpb8_collapsed_rawdata(hdr)>>3;
				if (serror2(hdr)) {
					sclose(hdr);
					exit(-1);
				}	
		
				ID = 0; 
				bscs_open(sd,&ID); // write-open
		     		/* write key file */ 

				char *tmp = strrchr(fn,FILESEP);
				if (tmp==NULL) tmp = fn; 
				strcpy(path2keys+path2keysLength,tmp);

				FILE *fid = fopen(path2keys,"w");	// TODO: prevent overwritting
				if (fid==NULL) 
					fprintf(stdout,"error: %i %s\n",errno, strerror(errno));		     		

				fprintf(fid,"bscs://%s/%016"PRIx64"",argv[1],ID);
				fclose(fid);

				fprintf(stdout,"open_w  ID=%016"PRIx64" len=%"PRIiPTR"\n",ID,hdr->AS.length);
				s= bscs_send_hdr(sd, hdr);
				fprintf(stdout,"sent hdr %i %i\n",s,hdr->AS.bpb);

				s = bscs_send_dat(sd, hdr->AS.rawdata, hdr->AS.length*bpb);
				fprintf(stdout,"sent dat %i %"PRIiPTR"\n",s,bpb);
					
				if (hdr->TYPE != GDF) hdrEVT2rawEVT(hdr); 
				if (hdr->EVENT.N>0) s= bscs_send_evt(sd, hdr);
				fprintf(stdout,"sent evt %i\n",s);
				s = bscs_close(sd);
				fprintf(stdout,"closed %08x\n",s);
			     			
			     	sclose(hdr);
			}     	 
		}			
	}	

    /*
     * We can simply use close() to terminate the
     * connection, since we're done .
     */
//    bscs_disconnect(sd);
	return(0);
}
