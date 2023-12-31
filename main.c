#include "ACMSim.h"
#include "controller.h"
#include "inverter.h"
#include "measure.h"

static write_header_to_file(FILE* fw);
static write_data_to_file(FILE* fw);


int main()
{
	printf("Number of simulation: %d\n\n", NUMBER_OF_SIM);

	/* 系统初始化 */
	SMachine_init();
	SmMeasure_init();
	controller_init();

	FILE* fw;
	fw = fopen("pmsm_sim.dat", "w");
	write_header_to_file(fw);

	/*主循环*/
	clock_t begin,end;
	begin = clock();
	int dfe=0;	// 控制 控制器动作周期
	/* 电机仿真 */
	for(int _ = 0; _ < NUMBER_OF_SIM; ++_){
		/* 设置负载和转速 */
		if(CTRL.timebase > 10){
			ACSM.rpm_cmd = 200;
		}else if (CTRL.timebase > 5){
			ACSM.Tload = 5;
		}else{
			ACSM.rpm_cmd = 50;
			ACSM.Tload = 1;
		}
		/* 电机仿真 */
		if(machine_simulation()){
			printf("Break the loop.\n");
			break;
		}
		/* 控制器动作周期*/
		if(++dfe == DOWN_FREQ_EXE){
			dfe = 0;
			CTRL.timebase += TS;
			measurement();
			write_data_to_file(fw);
			control(ACSM.rpm_cmd);	// 转/min
		}
		inverter();
	}
	fclose(fw);
	end = clock();
	printf("The simulation in C costs %g sec.\n", (double)(end - begin)/CLOCKS_PER_SEC);
	system("python ./ACSMPlot.py");

	return (0);
}

static write_header_to_file(FILE* fw){
	fprintf(fw, "x0,x1,x2,x3,uMs_cmd,uTs_cmd,iMs_cmd,iMs,iTs_cmd,iTs\n");
}

static write_data_to_file(FILE* fw){
	static int j = 0;
	if(++j == 10){
		j=0;
		fprintf(fw, "%g,%g,%g,%g,%g,%g,%g,%g,%g,%g\n",
				ACSM.x[0],ACSM.x[1],ACSM.x[2],ACSM.x[3],
				CTRL.uDs_cmd, CTRL.uQs_cmd, CTRL.iDs_cmd, CTRL.iDs, CTRL.iQs_cmd, CTRL.iQs);
	}
}

