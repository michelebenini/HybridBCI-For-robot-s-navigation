#include "bci.h" 

void parse_command_line(int argc, char** argv, instance *inst);
void free_memory(instance inst);
void read_input(instance *inst);

int opt(instance inst);

int main(int argc, char **argv) 
{ 
	      
	if( VERBOSE >= 2 ) 
	{ 
		for(int a = 0; a < argc; a++) 
			printf("%s ", argv[a]); 
		printf("\n"); 
	}

	instance inst;

	parse_command_line(argc,argv, &inst); 
	read_input(&inst);
	opt(inst);
	free_memory(inst);

	return 0; 
}         

void read_input(instance *inst)
{
	inst->hdr = constructHDR(0, 0);
    inst->hdr->TYPE = GDF;
	inst->hdr = sopen(inst->filename,"r",inst->hdr);
	inst->n_channels = (long)biosig_get_number_of_channels(inst->hdr);
    inst->sample_rate = inst->hdr->SampleRate;
	inst->n_records = (long)biosig_get_number_of_records(inst->hdr);
	inst->data = biosig_get_data(inst->hdr,' ');
	
	if( VERBOSE > 90)
	{ 
		std::cout << "File read : " << inst->filename << std::endl;	
		std::cout << "Channels : " << inst->n_channels << std::endl;
		std::cout << "Samples rate : " << inst->sample_rate << std::endl;
		std::cout << "Records : " << inst->n_records << std::endl;
	}
	sclose(inst->hdr);
}

void parse_command_line(int argc, char** argv, instance *inst)
{ 
	
	if( VERBOSE >= 1000 ) printf(" running %s with %d parameters \n", argv[0], argc-1); 
		
	// default   
	strcpy(inst->filename, "data/B0101T.gdf");
	inst->n_channels = 0;
	inst->sample_rate = 0;
	inst->n_records = 0;
	inst->data = NULL;
	inst->mode = -1;
	inst->s_ch = -1;
	inst->exe = -1;

    int help = 0; 
    if( argc < 1 ) help = 100;	
	
	for( int i = 1; i < argc; i++ ) 
	{ 
		if( strcmp(argv[i],"-file") == 0 && argc > i+1 ) { strcpy(inst->filename,argv[++i]); continue; } 			// input file
		if( strcmp(argv[i],"-f") == 0 && argc > i+1 ) { strcpy(inst->filename,argv[++i]); continue; } 				// input file
		if( strcmp(argv[i],"-show") == 0 && argc > i+1 ) { inst->s_ch = atoi(argv[++i]); inst->mode = 0; continue; }// show channel
		if( strcmp(argv[i],"-execute") == 0 && argc > i+1 ) { inst->exe = atoi(argv[++i]); inst->mode = 1;continue;}// execute 
		if( strcmp(argv[i],"-help") == 0 ) { help = 2; continue; } 													// help
		if( strcmp(argv[i],"--help") == 0 ) { help = 2; continue; } 												// help
		help = i;
    }      

	if( help || (VERBOSE >= 10) )		// print current parameters
	{
		printf("\nAvailable parameters (vers. 0.1-2019) --------------------------------------------------------\n");
		printf("-file %s\n", inst->filename); 
		printf("-show %d\n",inst->s_ch);
		printf("-execute %d\n",inst->exe);
		printf("\nenter -help or --help for help\n");
		printf("----------------------------------------------------------------------------------------------\n\n");
	}        
	
	if( help )exit(1);

}    

void free_memory(instance inst){
	// memory deallocation
	destructHDR(inst.hdr);
}





