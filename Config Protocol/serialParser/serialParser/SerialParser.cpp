#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define FILENAME "in.txt"
#define MAX 5000
#define NAME_MAX 20
#define PARAMS_IN_CONFIG 3
#define PARAMS_PER_SENS 7

#define ADC 0
#define I2C 1

typedef struct{
	int command;
} I2C_command;

typedef struct{
	float a;
	float b;
	float c;
} Transfer_funct;

typedef struct{
	char name[NAME_MAX];
	int type;
	int address;
	Transfer_funct t;
	int settle_time;
	I2C_command *read;
	I2C_command *init;
} Sensor;

typedef struct{
	int time;
	bool overwrite;
	Sensor *sensors;
} Config;

float transfer(Transfer_funct t, int x){
	return ( x*x*t.a + x*t.b + t.c);
}

Config config;
FILE *fd;
char temp1[100];
char *line;
char buf[MAX];
char err_log[MAX];

int main(int argc, char* argv[]){

	fd = fopen(FILENAME, "r");
	int num_s, i, ret, commands_in_init, commands_in_read;
	Sensor *s;


	while (fgets(temp1, MAX, fd) != NULL)
		strcat(buf, temp1);

	line = strtok(buf, "\n");
	if (line == NULL){
		sprintf(err_log, "No line retrieved, Expected configuration line\n");
		return -1;
	}
	if ((ret = sscanf(line, "%u,%u,%u", &config.time, &config.overwrite, &num_s)) != PARAMS_IN_CONFIG)
	{
		sprintf(err_log, "On configuration line, expected %D parameters, received %d\n", PARAMS_IN_CONFIG, ret);
		return -1;
	}
	config.sensors = (Sensor*)malloc(num_s*sizeof(Sensor));
	for (i = 0; i < num_s; i++){
		line = strtok(NULL, "\n");
		if (line == NULL){
			sprintf(err_log, "No line retrieved, Expected sensor description #%d\n",i);
			return -1;
		}

		s = &config.sensors[i];
		if ((ret = sscanf(line, "%s ,%u,%u,%f,%f,%f,%u", 
			s->name, 
			&s->type, 
			&s->address, 
			&s->t.a,
			&s->t.b,
			&s->t.c,
			&s->settle_time
			)) != PARAMS_PER_SENS)
		{
			sprintf(err_log, "On sensor #%d line, expected %d parameters, received %d\n", i,PARAMS_PER_SENS ,ret);
			return -1;
		}

		if (s->type == I2C)
		{
			num_s = 3;
			//strtok_r
			//s->init
		}
	}




	fclose(fd);
	return 0;
}