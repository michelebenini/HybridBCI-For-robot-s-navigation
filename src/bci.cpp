#include "bci.h"    

void create_script(instance inst, int i);
void show(instance inst);
void show_channel(instance inst, int num);
void execute(instance inst);
void preprocessing(instance inst);
void gdf_to_json(instance inst);
void remove_char(char *str,char *c);

int opt(instance inst)
{ 
    if(VERBOSE > 90){
        std::cout << "\nExecution mode : " << inst.mode << std::endl;
    }
    switch (inst.mode)
    {
        case 0:
            show_channel(inst, inst.s_ch);
            break;
        case 1:
            execute(inst);
            break;
        default:
            std::cout << "Wrong Command!!" << std::endl;
            break;
    }
    return 0; 
} 
void create_script(instance inst, int i){
    long double ratio = (long double)1/inst.sample_rate;
    FILE *dt;
    const char *ch = biosig_channel_get_label(biosig_get_channel(inst.hdr, i));
    char name_c[100];
    char c[] = " :\\\t\n";
    strcpy(name_c,ch);
    remove_char(name_c, c);
    char dir[] = "script/";
    if(VERBOSE > 60){
        std::cout << std::endl << "Channel in use : " << name_c << std::endl;
        std::cout << "Directory of destination : " << dir << std::endl;
    }
    char filename[100];
    filename[0] ='\0';
    strcat(filename, dir);
    strcat(filename,"data/");
    strcat(filename,name_c);
    strcat(filename,".plt");
    
    
    if(VERBOSE > 60){
        std::cout << "File write : " << filename << std::endl;
    }
    dt = fopen(filename, "w");

    for(long int j = 0; j < inst.n_records; j++ ){
        fprintf(dt,"%lf %lf\n",(double)(ratio*j), (double)inst.data[j*inst.n_channels+i]);  // get the value from the struture
    }
    fclose(dt);
    
    FILE *s;
    char scriptname[100];
    scriptname[0] ='\0';
    strcat(scriptname, dir);
    strcat(scriptname, "script_");
    strcat(scriptname, name_c);
    strcat(scriptname, ".p");

    s = fopen(scriptname, "w");
    if(VERBOSE > 60){
        std::cout << "Script write : " << scriptname << std::endl;
    }

    fprintf(s,"set autoscale\n");
    fprintf(s,"set style line 1\\\n\tlinecolor rgb '#0060ad'\n");
    fprintf(s,"set term wxt title '%s'\n",name_c);
    fprintf(s, "plot '%s' with lines linestyle 1",filename );
        
    fclose(s);
}
void show(instance inst){
    for(int i = 0; i < inst.n_channels; i++){
        show_channel(inst, i);
    }
}
void show_channel(instance inst, int num){
    if(num < 0 || num >= inst.n_channels){
        printf("\nChannel selected doesn't exist!!\nThis file has: %ld channels!!\nInsert a number from 0 to %ld!!\n",inst.n_channels,inst.n_channels-1);
        exit(0);
    }
    create_script(inst, num);
    FILE *gp = popen("gnuplot -p","w");

    const char *ch = biosig_channel_get_label(biosig_get_channel(inst.hdr, num));
    char name_c[100];
    char c[] = " :\\\t\n";
        
    strcpy(name_c,ch);
    remove_char(name_c, c);

    char dir[] = "script/";
    char scriptname[100];
    scriptname[0] ='\0';
    strcat(scriptname, dir);
    strcat(scriptname, "script_");
    strcat(scriptname, name_c);
    strcat(scriptname, ".p");


    fprintf(gp, "load '%s'\n",scriptname);
    
    fflush(NULL);
    fprintf(gp, "exit\n");
    fclose(gp);
}
void execute(instance inst){
    preprocessing(inst);
    //feature_extraction();
    //dimension_reduction();
    //learning_algorthm();
}
void preprocessing(instance inst){
    printf("AsAAA %ld", inst.n_channels);
}
void gdf_to_json(instance inst){
	char *jsonstr = NULL;
    char str[100];

    strcpy(str,inst.filename);
    
    char separator1 = '.';
    char *token1 = strtok(str, &separator1);

    char separator2 = '/';
    char *token2 = strtok(token1, &separator2);
    token2 = strtok(NULL, &separator2);
    
    char filename[100];
    filename[0] = '\0';

    
    strcat(filename, "data/json/");
    strcat(filename, token2);
    strcat(filename, ".json");
    
    if(VERBOSE > 10){
        std::cout << "File token named : " << token2 << std::endl;
        std::cout << "File json named : " << filename << std::endl;
    }

		
    asprintf_hdr2json(&jsonstr, inst.hdr);
    if(VERBOSE > 90){
        std::cout << "Json string : " << std::endl << jsonstr << std::endl;
    }
    FILE *fjson = fopen(filename, "w");
    fprintf(fjson,"%s",jsonstr);
    fclose(fjson);

}
void remove_char(char *str,char *c){
    int len_str = strlen(str);
    int len_c = strlen(c);
    int shift = 0;
    int flag = 0;
    
    for(int i = 0; i + shift < len_str; i++){
        flag = 0;
        for(int j = 0; j < len_c && flag == 0 ; j++ ){
            if(str[i+shift] == c[j]){
                flag = 1;
                shift++;
                i--;
            }
        }
        if(flag == 0)
            str[i] = str[i+shift];
        if( i + shift + 1 == len_str)
            str[i+1] = '\0';
    }

}