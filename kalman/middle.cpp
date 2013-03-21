#include "middle.h"

unsigned long get_current_millis() {
	struct timeval tv; gettimeofday(&tv,NULL); 
	return (long)((tv.tv_sec - tv_start.tv_sec)*1000 + (long)(tv.tv_usec - tv_start.tv_usec)/1000);
}

/*Fuegt dem Scheduler eine neue Aufgabe hinzu*/
void schedule_add(unsigned long plus, enum sched_tasks current_task, short param) {	
	struct sched_task new_task;
	new_task.tv_msec	= plus + get_current_millis();
	new_task.task 		= current_task;
	new_task.param 		= param;
	int i;
	for (i = 0; i < MAX_TASKS; i++) {
		if (scheduler[i].task == NOTHING) {
			scheduler[i] = new_task;
			break;
		} else if (i == MAX_TASKS-1) {
			if(WARNINGS){std::perror("SCHEDULE_ADD: Buffer voll, Task konnte nicht eingefuegt werden.");}
		}
	}
}

void send_ext_ctrl() {
	#if EXT_CTRL > 0
	if (desktop_build) return;
	/*Sendet die aktuell vorgeschlagenen Werte fuer roll und pitch an den Copter*/
	mavlink_message_t ex_msg;
	uint8_t ex_buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_msg_huch_ext_ctrl_pack(0, MAV_COMP_ID_IMU, &ex_msg, TARGET_SYSTEM, TARGET_COMPONENT, 0, roll, pitch, yaw, thrust);
	uint16_t ex_len = mavlink_msg_to_send_buffer(ex_buf, &ex_msg);
	if (write(tty_fd, ex_buf, ex_len) != ex_len) {if(WARNINGS){std::perror("LOOP: Es konnten keine Daten fuer die externe Kontrolle gesendet werden.");}}
	#endif	
}

void request_data_stream(short stream_id, short mav_speed, unsigned char mav_mode) {
	mavlink_message_t smsg;
	uint8_t sbuf[MAVLINK_MAX_PACKET_LEN];
	mavlink_msg_request_data_stream_pack(0, MAV_COMP_ID_IMU, &smsg, 1, 0, stream_id, mav_speed, mav_mode);
	uint16_t len = mavlink_msg_to_send_buffer(sbuf, &smsg);
	if (write(tty_fd, sbuf, len) != len) {std::perror("LOOP: Es konnte kein Datenstream abbonniert werden."); exit(1);}
}

/*Stoppt die aktiven Datenstreams und beendet das Programm, wenn SIGINT gesendet wird*/
void signal_callback_handler(int signum) {
	request_data_stream(MAV_DATA_STREAM_ALL,0,0);
	std::fclose(fd_112);
	std::fclose(fd_113);
	std::fclose(fd_114);
	std::fclose(fd_115);
	std::fclose(fd_data);
	exit(signum);
}

int main (int argc, char* argv[]) {
	max_pitch = MAX_PITCH; 
	max_roll = MAX_ROLL;
	max_thrust = MAX_THRUST;
	breakpoint = 0;
	desktop_build = 0;
	var_s = VARIANCE_S;
	var_i = VARIANCE_I;
	var_b = VARIANCE_B;
	float hs_p = HOLD_STILL_ROLL_KP;
	float hs_i = HOLD_STILL_ROLL_TN;
	float hs_d = HOLD_STILL_ROLL_TV;
	float htm_p = ANC_ROLL_KP;
	float htm_i = ANC_ROLL_TN;
	float htm_d = ANC_ROLL_TV;
	for(int i=1;i<argc;i++) {
		if (!strcmp("--pitch",argv[i]) || !strcmp("--p",argv[i])) {
			if ((i+1) < argc) {
				if (!strcmp("0",argv[i+1])) {
					max_pitch = 0;
					i++;
				} else {
					max_pitch = atoi(argv[i+1]);
					if (max_pitch == 0) {
						std::cout << argv[i+1] << " is not a valid value for --pitch. Using default value (" << MAX_PITCH << ").\n";
						max_pitch = MAX_PITCH;
					} else {
						i++;
					}
				}
			} else {
				std::cout << "Warning: --pitch needs an additional value. Using default value (" << MAX_PITCH << ").\n"; 
			}
		} else if (!strcmp("--roll",argv[i]) || !strcmp("--r",argv[i])) {
			if ((i+1) < argc) {
				if (!strcmp("0",argv[i+1])) {
					max_roll = 0;
					i++;
				} else {
					max_roll = atoi(argv[i+1]);
					if (max_roll == 0) {
						std::cout << argv[i+1] << " is not a valid value for --roll. Using default value (" << MAX_ROLL << ").\n";
						max_roll = MAX_ROLL;
					} else {
						i++;
					}
				}
			} else {
				std::cout << "Warning: --roll needs an additional value. Using default value (" << MAX_ROLL << ").\n"; 
			}
		} else if(!strcmp("--hsp",argv[i])) {
			if ((i+1) < argc) {
				if (!strcmp("0",argv[i+1])) {
					hs_p = 0;
					i++;
				} else {
					hs_p = atof(argv[i+1]);
					if (hs_p == 0) {
						std::cout << argv[i+1] << " is not a valid value for --hsp. Using default value (" << HOLD_STILL_ROLL_KP << ").\n";
						hs_p = HOLD_STILL_ROLL_KP;
					} else {
						i++;
					}
				}
			} else {
				std::cout << "Warning: --hsp needs an additional value. Using default value (" << HOLD_STILL_ROLL_KP << ").\n"; 
			}
		}  else if(!strcmp("--hsi",argv[i])) {
			if ((i+1) < argc) {
				if (!strcmp("0",argv[i+1])) {
					hs_i = 0;
					i++;
				} else {
					hs_i = atof(argv[i+1]);
					if (hs_i == 0) {
						std::cout << argv[i+1] << " is not a valid value for --hsi. Using default value (" << HOLD_STILL_ROLL_TN << ").\n";
						hs_i = HOLD_STILL_ROLL_TN;
					} else {
						i++;
					}
				}
			} else {
				std::cout << "Warning: --hsi needs an additional value. Using default value (" << HOLD_STILL_ROLL_TN << ").\n"; 
			}
		} else if(!strcmp("--hsd",argv[i])) {
			if ((i+1) < argc) {
				if (!strcmp("0",argv[i+1])) {
					hs_d = 0;
					i++;
				} else {
					hs_d = atof(argv[i+1]);
					if (hs_d == 0) {
						std::cout << argv[i+1] << " is not a valid value for --hsd. Using default value (" << HOLD_STILL_ROLL_TV << ").\n";
						hs_d = HOLD_STILL_ROLL_TV;
					} else {
						i++;
					}
				}
			} else {
				std::cout << "Warning: --hsd needs an additional value. Using default value (" << HOLD_STILL_ROLL_TV << ").\n"; 
			}
		} else if(!strcmp("--ancp",argv[i])) {
			if ((i+1) < argc) {
				if (!strcmp("0",argv[i+1])) {
					htm_p = 0;
					i++;
				} else {
					htm_p = atof(argv[i+1]);
					if (htm_p == 0) {
						std::cout << argv[i+1] << " is not a valid value for --ancp. Using default value (" << ANC_ROLL_KP << ").\n";
						htm_p = ANC_ROLL_KP;
					} else {
						i++;
					}
				}
			} else {
				std::cout << "Warning: --ancp needs an additional value. Using default value (" << ANC_ROLL_KP << ").\n"; 
			}
		} else if(!strcmp("--anci",argv[i])) {
			if ((i+1) < argc) {
				if (!strcmp("0",argv[i+1])) {
					htm_i = 0;
					i++;
				} else {
					htm_i = atof(argv[i+1]);
					if (htm_i == 0) {
						std::cout << argv[i+1] << " is not a valid value for --anci. Using default value (" << ANC_ROLL_TN << ").\n";
						htm_i = ANC_ROLL_TN;
					} else {
						i++;
					}
				}
			} else {
				std::cout << "Warning: --anci needs an additional value. Using default value (" << ANC_ROLL_TN << ").\n"; 
			}
		} else if(!strcmp("--ancd",argv[i])) {
			if ((i+1) < argc) {
				if (!strcmp("0",argv[i+1])) {
					htm_d = 0;
					i++;
				} else {
					htm_p = atof(argv[i+1]);
					if (htm_d == 0) {
						std::cout << argv[i+1] << " is not a valid value for --ancd. Using default value (" << ANC_ROLL_TV << ").\n";
						htm_d = ANC_ROLL_TV;
					} else {
						i++;
					}
				}
			} else {
				std::cout << "Warning: --ancd needs an additional value. Using default value (" << ANC_ROLL_TV << ").\n"; 
			}
		} else if(!strcmp("--vars",argv[i])) {
			if ((i+1) < argc) {
				if (!strcmp("0",argv[i+1])) {
					var_s = 0;
					i++;
				} else {
					var_s = atof(argv[i+1]);
					if (var_s == 0) {
						std::cout << argv[i+1] << " is not a valid value for --vars. Using default value (" << VARIANCE_S << ").\n";
						var_s = VARIANCE_S;
					} else {
						i++;
					}
				}
			} else {
				std::cout << "Warning: --vars needs an additional value. Using default value (" << VARIANCE_S << ").\n"; 
			}
		} else if(!strcmp("--vari",argv[i])) {
			if ((i+1) < argc) {
				if (!strcmp("0",argv[i+1])) {
					var_i = 0;
					i++;
				} else {
					var_i = atof(argv[i+1]);
					if (var_i == 0) {
						std::cout << argv[i+1] << " is not a valid value for --vari. Using default value (" << VARIANCE_I << ").\n";
						var_i = VARIANCE_I;
					} else {
						i++;
					}
				}
			} else {
				std::cout << "Warning: --vari needs an additional value. Using default value (" << VARIANCE_I << ").\n"; 
			}
		} else if(!strcmp("--varb",argv[i])) {
			if ((i+1) < argc) {
				if (!strcmp("0",argv[i+1])) {
					var_b = 0;
					i++;
				} else {
					var_b = atof(argv[i+1]);
					if (var_b == 0) {
						std::cout << argv[i+1] << " is not a valid value for --varb. Using default value (" << VARIANCE_B << ").\n";
						var_b = VARIANCE_B;
					} else {
						i++;
					}
				}
			} else {
				std::cout << "Warning: --varb needs an additional value. Using default value (" << VARIANCE_B << ").\n"; 
			}
		} else if(!strcmp("--b",argv[i])) {
			if ((i+1) < argc) {
				if (!strcmp("0",argv[i+1])) {
					breakpoint = 0;
					i++;
				} else {
					breakpoint = atof(argv[i+1]);
					if (breakpoint == 0) {
						std::cout << argv[i+1] << " is not a valid value for --b. Using default value (" << (int)breakpoint << ").\n";
					} else {
						i++;
					}
				}
			} else {
				std::cout << "Warning: --b needs an additional value. Using default value (" << (int)breakpoint << ").\n"; 
			}
		}  else if(!strcmp("--db",argv[i])) {
				desktop_build = 1;
				std::cout << "Warning: Desktop-Build!\n";
		} else {
			if (strcmp("--?",argv[i]) && strcmp("--help",argv[i]) && strcmp("--h",argv[i])) std::cout << "Unknown Input " << argv[i] << "\n";
			std::cout << "Valid inputs are: \n --pitch VAL \t Sets maximum pitch to (short)VAL \n --roll VAL \t Sets maximum roll to (short)VAL \n --hsp VAL \t Sets P-value of the hs-controller to (float)VAL \n --hsi VAL \t Sets I-value of the hs-controller to (float)VAL \n --hsd VAL \t Sets D-value of the hs-controller to (float)VAL \n --ancp VAL \t Sets P-value of the anc-controller to (float)VAL \n --anci VAL \t Sets I-value of the anc-controller to (float)VAL\n --ancd VAL \t Sets D-value of the anc-controller to (float)VAL \n --b VAL \t Sets a breakpoint (0 = None, 1 = HS, 2 = ANC, 3 = ANC with yaw)\n --db \t\t Triggers the desktop-mode (no ext_ctrl-messages and mode switch set to ext_ctrl)\n";
			exit(1);
		}
	}
	pid_roll.set(hs_p,hs_i,hs_d);
	pid_pitch.set(hs_p,hs_i,hs_d);
	pid_roll_anc.set(htm_p,htm_i,htm_d);
	pid_pitch_anc.set(htm_p,htm_i,htm_d);
	
	#if LOG == 1
	/*Config dump*/
	char tmp_dir[32];
	std::fstream f;
	f.open("log_dir", std::ios::in);
	f.getline(tmp_dir, sizeof(tmp_dir));
	f.close();
	int tmp = atoi(tmp_dir);
	tmp++;
	f.open("log_dir", std::ios::out);
	f << (int)tmp;
	f.close();
	strcpy(log_dir,"log/");
	strcat(log_dir,tmp_dir);
	strcpy(tmp_dir,log_dir);
	if (mkdir(log_dir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) < 0) {std::perror("Verzeichnis fuer Log-Dateien konnte nicht angelegt werden."); exit(1);}
	f.open(strcat(tmp_dir,"/config"), std::ios::out);
	f << "DATA_STREAM_SPEED: " << DATA_STREAM_SPEED << "\nSE_COUNT: " << SE_COUNT << "\nSRF_SPEED: " << SRF_SPEED << "\nENABLED_SENSORS: " << ENABLED_SENSORS << "\nMAX_DISTANCE: " << MAX_DISTANCE << "\nSE_MIN_DISTANCE: " << SE_MIN_DISTANCE << "\nSE_MIN: " << SE_MIN << "\nSE_MIN_DIFF: " << SE_MIN_DIFF << "\nMAX_ROLL_ANGLE: " << MAX_ROLL_ANGLE << "\nMAX_PITCH_ANGLE: " << MAX_PITCH_ANGLE << "\nSE_DATA_BUFFER_SIZE: " << SE_DATA_BUFFER_SIZE << "\nFILTER_MODE: " << FILTER_MODE << "\nSTD_FACTOR: " << STD_FACTOR << "\nMIN_STD: " << MIN_STD << "\nMAX_ROLL: " << max_roll << "\nMAX_PITCH: " << max_pitch << "\nCONST_YAW: " << CONST_YAW << "\nIMAX: " << IMAX << "\nHOLD_STILL_KP: " << hs_p << "\nHOLD_STILL_TN: " << hs_i << "\nHOLD_STILL_TV: " << hs_d << "\nSTILL_FAK: " << STILL_FAK_ROLL << "\nCHECK_STILL_SPEED: " << CHECK_STILL_SPEED << "\nCHECK_STILL_COUNT: " << CHECK_STILL_COUNT << "\nVARIANCE_S: " << var_s << "\nVARIANCE_I" << var_i; 
	f.close();
	#endif
	setup();
	while(1) loop();
	return 0;
}

/*Initialisierung*/
void setup() {
	/*Öffnen der seriellen Schnitstelle TTY_DEVICE*/
	tty_fd = open(TTY_DEVICE, O_RDWR);
	if (tty_fd == -1) {std::perror("SETUP: " TTY_DEVICE " kann nicht geoeffnet werden."); exit(1);}
	
	/*Konfiguration der seriellen Schnittstelle*/	
	if (tcgetattr(tty_fd, &attr) != 0) {std::perror("SETUP: tcgetattr() fehlgeschlagen."); exit(1);}
	/*input   modes*/ attr.c_iflag = 0;
	/*output  modes*/ attr.c_oflag = OPOST | ONLCR;	
	/*control modes*/ attr.c_cflag = TTY_DEVICE_SPEED | CS8 | CRTSCTS | CLOCAL | CREAD;
	/*local   modes*/ attr.c_lflag = 0;
	if (tcsetattr(tty_fd, TCSAFLUSH, &attr) != 0){	std::perror("SETUP: tcsetattr() fehlgeschlagen"); exit(1);}
	first_heartbeat = 0;	

#if SENSOR_MODE == REAL_VAL
	srf_fd = open(SRF_DEVICE, O_RDWR);
	if (srf_fd == -1) {std::perror("SETUP: " SRF_DEVICE "kann nicht geoeffnet werden."); exit(1);}
#elif SENSOR_MODE == FAKE
	srf_fd = 0;
#endif
	/*Einstellen der anfaenglichen Messgeschwindigkeiten, Erzeugen der Sensoren*/
	for (int i = 0; i < SE_COUNT; i++) {
		srf_speed[i] = SRF_SPEED;
		unsigned char align = i;
		/*Korrektur zur Winkelberechnung*/
		if (i == 1) align = 2;
		if (i == 2) align = 1; 
		srf.push_back(SRF((SE0_ADDRESS + i),srf_fd,align,var_s,var_i,0));
		nvalue[i] = 0;
		nvalue2[i] = 0;
	}
	
	/*Maxbotix*/
	srf.push_back(SRF(0,0,0,var_s,var_i,0));

	/*signal handler für SIGINT (Strg+C) registrieren*/
	signal(SIGINT, signal_callback_handler);
	
	/*Initialisierung des Schedulers*/
	gettimeofday(&tv_start,NULL);
	for (int i = 0; i < MAX_TASKS; i++) 	scheduler[i].task = NOTHING;
	/*Erzeugen des Log-Verzeichnis und Log-Dateien*/
	#if LOG > 0
	/*time_t current_timestamp = time(0);
	tm *current_time;
	current_time = localtime(&current_timestamp);
	sprintf(log_dir,"log/%d_%d_%d %d:%d:%d",(current_time->tm_year+1900),(current_time->tm_mon+1),(current_time->tm_mday),(current_time->tm_hour),(current_time->tm_min),(current_time->tm_sec));*/
	char tmp_dir[32];
	strcpy(tmp_dir,log_dir);
	fd_112 = std::fopen(strcat(tmp_dir,"/112"), "a"); std::fprintf(fd_112,"#TS\tdata\tmean\n"); strcpy(tmp_dir,log_dir);
	fd_113 = std::fopen(strcat(tmp_dir,"/113"), "a"); std::fprintf(fd_113,"#TS\tdata\tmean\n"); strcpy(tmp_dir,log_dir);
	fd_114 = std::fopen(strcat(tmp_dir,"/114"), "a"); std::fprintf(fd_114,"#TS\tdata\tmean\n"); strcpy(tmp_dir,log_dir);
	fd_115 = std::fopen(strcat(tmp_dir,"/115"), "a"); std::fprintf(fd_115,"#TS\tdata\tmean\n"); strcpy(tmp_dir,log_dir);
	fd_max = std::fopen(strcat(tmp_dir,"/max"), "a"); std::fprintf(fd_max,"#TS\tdata\tmean\tbaro\n"); strcpy(tmp_dir,log_dir);
	fd_acc = std::fopen(strcat(tmp_dir,"/acc"), "a"); std::fprintf(fd_acc,"#TS\txacc\tyacc\tzacc\n"); strcpy(tmp_dir,log_dir);
	fd_data= std::fopen(strcat(tmp_dir,"/data"),"a"); std::fprintf(fd_data,"#TS\troll\tpitch\tyaw\theading\tstate\troll_rad\tpitch_rad\n"); strcpy(tmp_dir,log_dir);
	#endif

	current_pitch_rad = 0;
	current_roll_rad = 0;
	current_heading = 0;	
	state = IDLE;
	roll = 0; pitch = 0; yaw = 0;
	still = 0;
	first_heading = -1;
	for(int i = 0; i < 360; i++) env[i] = 0;
	xacc = 0; yacc = 0;
	
	init_state = 0;
	
	rotation_angle = 6;
	yaw_sign = 1;
	rotation_angle_sign = 1;
	hs_state = 0;
	align_se = 3;
	
	char target_ip[15];
	//if (desktop_build)	strcpy(target_ip, "192.168.0.180");
	//else			strcpy(target_ip, "192.168.2.170");
	strcpy(target_ip, "192.168.2.170");
	memset(&gcAddr, 0, sizeof(gcAddr));
	gcAddr.sin_family = AF_INET;
	gcAddr.sin_addr.s_addr = inet_addr(target_ip);
	gcAddr.sin_port = htons(14550);
	
	/*Einkommentieren, damit regelmäßig Daten auf das Terminal geschrieben werden*/
	schedule_add(0,SHOW_ME,0);
}

/*Mainloop*/
void loop() {
	/*Auswertung des anstehenden Aufgaben im Scheduler*/
	int i; unsigned char new_value = 0; unsigned char new_heading = 0; 
	for (i = 0; i < MAX_TASKS; i++) {
		if (scheduler[i].task == NOTHING || get_current_millis() < scheduler[i].tv_msec) {continue;}
		switch (scheduler[i].task) {
			case NOTHING: break;
			case MEASURE:
				/*Fordert den als Parameter uebergebenen Sensor auf eine Messung zu starten und bestimmt den Lese-Zeitpunkt*/			
				srf[scheduler[i].param - SE0_ADDRESS].measure();
				schedule_add((long)srf_speed[scheduler[i].param - SE0_ADDRESS], READ_MEASURE, scheduler[i].param);
				scheduler[i].task = NOTHING;
				break;
			case READ_MEASURE: {
				unsigned short index = scheduler[i].param - SE0_ADDRESS;
				/*Liest die Messdaten des als Parameter uebergebenen Sensors aus und veranlasst erneute Messung*/
				#if FILTER_MODE == KALMAN
				float acc = 0;
				if 	(index == 0) {acc = yacc;	yacc = 0;}
				else if	(index == 1) {acc = yacc_m;	yacc_m = 0;}
				else if	(index == 2) {acc = xacc_m;	xacc_m = 0;}
				else if	(index == 3) {acc = xacc;	xacc = 0;}
				srf[index].read_it(get_current_millis(),acc);
				#else
				srf[index].read_it(get_current_millis());
				#endif
				schedule_add((long)srf[index].get_delay(), MEASURE, scheduler[i].param);
				if(state == HOLD_STILL || state == MEASURE_ENVIRONMENT || state == GET_ANCHOR) nvalue[index] = 1;
				new_value = scheduler[i].param;
				
				if (SE_CORRECTION & 2) {
					/*Korrektur der Messwerte bei Schieflage des Copters bei glaubhaften Winkeln*/
					if ((index == 0 || index == 1) && (abs(current_roll_rad) < MAX_ROLL_ANGLE))		srf[index].set_mean((unsigned short)(srf[index].get_mean()*cos(current_roll_rad)));
					else if ((index == 2 || index == 3) && (abs(current_pitch_rad) < MAX_PITCH_ANGLE))	srf[index].set_mean((unsigned short)(srf[index].get_mean()*cos(current_roll_rad)));
				}
				if (SE_CORRECTION & 1) {
					/*Korrektur der Messwerte, wenn sich ein Sensor im Nahbereich (< SE_MIN_DISTANCE) befindet*/
					static unsigned char 	rm_state[SE_COUNT];
					static short		rm_min[SE_COUNT];
					if (state != IDLE && state != INIT && state != DELAY) {
						if (rm_state[index] == 0 && srf[index].get_mean() < SE_MIN_DISTANCE) {
							rm_state[index] = 1;
							if (index == 0 || index == 2)	rm_min[index] = srf[index+1].get_mean() - SE_MIN_DIFF;
							else				rm_min[index] = srf[index-1].get_mean() - SE_MIN_DIFF;
							srf[index].set_mean(SE_MIN);
						} else if (rm_state[index] == 1) {
							if ((index == 0 || index == 2) && srf[index+1].get_mean() < rm_min[index]) {
								rm_state[index] = 0;
							} else if ((index == 1 || index == 3) && srf[index-1].get_mean() < rm_min[index]) {
								rm_state[index] = 0;
							} else {
								srf[index].set_mean(SE_MIN);
							}
						}
					}
					if (rm_state[0] == 1 && rm_state[1] == 1) {rm_state[0] = 0; rm_state[1] = 0;}
					if (rm_state[2] == 1 && rm_state[3] == 1) {rm_state[2] = 0; rm_state[3] = 0;}
				}
				
				/*Speichert den gelesenen Messwert in der entsprechenden Log-Datei*/
				#if LOG > 0
				std::FILE *fd;
				if (scheduler[i].param == SE0_ADDRESS) 	fd = fd_112;
				else if (scheduler[i].param == SE1_ADDRESS) 	fd = fd_113;
				else if (scheduler[i].param == SE2_ADDRESS) 	fd = fd_114;
				else						fd = fd_115;
				std::fprintf(fd,"%lu\t%u\t%u\n", srf[scheduler[i].param - SE0_ADDRESS].get_msec(), srf[scheduler[i].param - SE0_ADDRESS].get_data(), srf[scheduler[i].param - SE0_ADDRESS].get_mean());
				#endif
				scheduler[i].task = NOTHING;
				break;
			}
			case SAVE_LOG:
				/*Sichert die bisher geloggten Daten*/
				#if LOG > 0
				std::fclose(fd_112);
				std::fclose(fd_113);
				std::fclose(fd_114);
				std::fclose(fd_115);
				std::fclose(fd_max);
				std::fclose(fd_acc);
				std::fclose(fd_data);
				char tmp_dir[32]; strcpy(tmp_dir,log_dir);
				fd_112 = std::fopen(strcat(tmp_dir,"/112"), "a"); strcpy(tmp_dir,log_dir);
				fd_113 = std::fopen(strcat(tmp_dir,"/113"), "a"); strcpy(tmp_dir,log_dir);
				fd_114 = std::fopen(strcat(tmp_dir,"/114"), "a"); strcpy(tmp_dir,log_dir);
				fd_115 = std::fopen(strcat(tmp_dir,"/115"), "a"); strcpy(tmp_dir,log_dir);
				fd_max = std::fopen(strcat(tmp_dir,"/max"), "a"); strcpy(tmp_dir,log_dir);
				fd_acc = std::fopen(strcat(tmp_dir,"/acc"), "a"); strcpy(tmp_dir,log_dir);
				fd_data= std::fopen(strcat(tmp_dir,"/data"),"a"); strcpy(tmp_dir,log_dir);
				schedule_add(LOG_SPEED,SAVE_LOG,0);
				#endif
				scheduler[i].task = NOTHING;
				break;   
			case CHECK_STILL:
				/*Prueft auf Stillstand des Copters*/
				if (still && state == HOLD_STILL) {
					if (scheduler[i].param < CHECK_STILL_COUNT) {
						schedule_add(CHECK_STILL_SPEED,CHECK_STILL,scheduler[i].param+1);
					} else {
						state = MEASURE_ENVIRONMENT;
						still = 0;
						schedule_add(CHECK_STILL_SPEED,CHECK_STILL, 0);
					}
				} else {
					schedule_add(CHECK_STILL_SPEED,CHECK_STILL,0);
				}
				scheduler[i].task = NOTHING;
				break;
			case CHANGE_STATE:
				/*Ändert den aktuellen Status zum als Parameter übergebenen Status*/
				state = (states)scheduler[i].param;
				if ((states)scheduler[i].param == HOLD_STILL) hs_state = 0;
				scheduler[i].task = NOTHING;
				break;
			case SHOW_ME: {
				/*Schreibt Messwerte und Neigungswinkel auf das Terminal*/
				char st[16];
				switch (state) {
					case INIT: 			std::sprintf(st, "INIT"); 		break;
					case MEASURE_ENVIRONMENT: 	std::sprintf(st, "ME"); 		break;
					case IDLE: 			std::sprintf(st, "IDLE"); 		break;
					case HOLD_STILL: 		std::sprintf(st, "HOLD_STILL"); 	break;
					case GET_ALIGNMENT: 		std::sprintf(st, "GET_ALIGNMENT"); 	break;
					case GET_ANCHOR: 		std::sprintf(st, "GET_ANCHOR"); 	break;
					case DELAY:			std::sprintf(st, "DELAY");		break;
					default: 			std::sprintf(st, "???"); 		break;
				}
					
				if (roll > 9 || roll < 0) 	std::printf("roll: %d\tpitch: %d\tyaw: %d\tthrust: %d\theading: %d\tdhead: %d\tSE0: %d\tSE1: %d\tSE2: %d\tSE3:%d\tMax:%d\tstate: %s\n", roll, pitch, yaw, thrust, current_heading, desired_heading, srf[0].get_mean(), srf[1].get_mean(), srf[2].get_mean(), srf[3].get_mean(),srf[4].get_mean(),st);
				else 				std::printf("roll: %d\t\tpitch: %d\tyaw: %d\tthrust: %d\theading: %d\tdhead. %d\tSE0: %d\tSE1: %d\tSE2: %d\tSE3:%d\tMax:%d\tstate: %s\n", roll, pitch, yaw, thrust, current_heading, desired_heading, srf[0].get_mean(), srf[1].get_mean(), srf[2].get_mean(), srf[3].get_mean(),srf[4].get_mean(),st);
				schedule_add(50,SHOW_ME,0);
				scheduler[i].task = NOTHING;
				
				/*Mavlinkkommunikation mit QGroundcontrol (basierend auf Based on http://qgroundcontrol.org/dev/mavlink_linux_integration_tutorial)*/
				mavlink_message_t msg;
				uint16_t len;
				uint8_t buf[MAVLINK_MAX_PACKET_LEN];
				
				static int i; i++;
				if (i == 10) {
					i = 0;
					mavlink_msg_heartbeat_pack(0, 0, &msg, 0, 0, 0, 0, 0);
					len = mavlink_msg_to_send_buffer(buf, &msg);
					sendto(s, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
				}
				mavlink_msg_debug_vect_pack(0,0,&msg,"SE0",0,xacc,srf[0].get_mean(),srf[0].get_data());
				len = mavlink_msg_to_send_buffer(buf, &msg);
				sendto(s, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
				mavlink_msg_debug_vect_pack(0,0,&msg,"MAX",0,srf[4].get_baro(),srf[4].get_mean(),srf[4].get_data());
				len = mavlink_msg_to_send_buffer(buf, &msg);
				sendto(s, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
				/*mavlink_msg_debug_vect_pack(0,0,&msg,"SE0",0,xacc,yacc,zacc);
				len = mavlink_msg_to_send_buffer(buf, &msg);
				sendto(s, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));*/
				break;
			}
			default: if(WARNINGS){std::perror("LOOP: Unbekannte Aufgabe im Scheduler.");} break; 
		}
	}

	/*Auswertung der Daten, die an der seriellen Schnittstelle anliegen.*/
	struct timeval tv; tv.tv_sec = 0; tv.tv_usec = 0;
	/*Prueft, ob Daten an der seriellen Schnittstelle anliegen*/
	FD_ZERO(&tty_fdset);
	FD_SET(tty_fd, &tty_fdset);
	if (select(tty_fd+1, &tty_fdset, NULL, NULL, &tv) == -1) {std::perror("LOOP: select() fehlgeschlagen"); exit(1);}

	/*Liest ein einzelnes Byte von der seriellen Schnittstelle und prueft, ob ein Mavlinkpaket vervollstaendigt wurde*/
	if (FD_ISSET(tty_fd, &tty_fdset)) {
		static mavlink_message_t msg;
		static mavlink_status_t status;
		char c[1];
		if (read(tty_fd,c,1) == -1) {if(WARNINGS){std::perror("LOOP: Fehler beim Lesen aus " TTY_DEVICE);}}
		#if DEBUG_LEVEL > 2
		printf("%#x\n",c[0]);
		#endif
		if (mavlink_parse_char(MAVLINK_COMM_0,(uint8_t) *c, &msg, &status)) {
			#if DEBUG_LEVEL > 1
			printf("%u Mavlinkpaket empfangen: %d\n", get_current_millis(), msg.msgid);
			#endif
			 //printf("%lu Mavlinkpaket empfangen: %d\n", get_current_millis(), msg.msgid);
			
			switch (msg.msgid) {
				case MAVLINK_MSG_ID_HEARTBEAT:
					/*Beim Empfang des ersten Heartbeats wird zunaechst ein eventuell vorhandener Datenstream gekuendigt und ein neuer Datenstream abonnemiert*/
					mavlink_heartbeat_t hb;
					mavlink_msg_heartbeat_decode(&msg,&hb);
					if (!first_heartbeat) {
						first_heartbeat = 1;
						request_data_stream(MAV_DATA_STREAM_ALL,0,0);
					}
					//if (state == IDLE) {
					if (state == IDLE && (hb.custom_mode == 13 /*EXT_CTRL*/ || desktop_build)) {
						std::cout << "State changed from IDLE to INIT.\n";
						state = INIT;
						schedule_add(2000,CHANGE_STATE,(int)HOLD_STILL);
						request_data_stream(MAV_DATA_STREAM_EXTRA2/*Attitude & Ranger*/,DATA_STREAM_SPEED,1);
						request_data_stream(MAV_DATA_STREAM_RAW_SENSORS,DATA_STREAM_SPEED,1);
						//request_data_stream(MAV_DATA_STREAM_RAW_CONTROLLER,5,1);
						init_state = 1;
					}
					break;
				case MAVLINK_MSG_ID_VFR_HUD:
					/*Speichert bei Empfang von HUD-Daten die aktuelle Ausrichtung des Copters*/
					mavlink_vfr_hud_t hud_data;
					mavlink_msg_vfr_hud_decode(&msg, &hud_data);
					old_heading = current_heading;
					current_heading = hud_data.heading;
					new_heading = 1;
					break;
				case MAVLINK_MSG_ID_ATTITUDE:
					mavlink_attitude_t adata;
					mavlink_msg_attitude_decode(&msg,&adata);
					old_heading = current_heading;
					current_heading = ((short)lround(DEG(adata.yaw))+360)%360;
					current_pitch_rad = adata.pitch;
					current_roll_rad = adata.roll;
					new_heading = 1;
					/*FIXME*/
					/*static long first_tv;
					if (first_tv == 0) first_tv= get_current_millis();
					static short counter;
					counter++;
					std::cout << "TEST: " << (float)counter << " " << get_current_millis()-first_tv << "\n";*/
					break;
				case MAVLINK_MSG_ID_RAW_IMU:
					/*Speichert die Beschleunigungen auf der x- und y-Achse*/
					mavlink_raw_imu_t raw_imu;
					mavlink_msg_raw_imu_decode(&msg,&raw_imu);
					
					/*Additiv, da IMU-Werte schneller als Ultraschallwerte gelesen werden*/
					xacc += raw_imu.xacc;
					xacc_m -= raw_imu.xacc;
					yacc += raw_imu.yacc;
					yacc_m -= raw_imu.yacc;
					zacc += raw_imu.zacc + 1000;
					#if LOG > 0
					std::fprintf(fd_acc,"%lu\t%d\t%d\t%d\n", get_current_millis(), raw_imu.xacc, raw_imu.yacc, raw_imu.zacc);
					#endif
					
					break;
				case MAVLINK_MSG_ID_RC_CHANNELS_SCALED:
					mavlink_rc_channels_scaled_t rc;
					mavlink_msg_rc_channels_scaled_decode(&msg,&rc);
					
					/*Additiv, da IMU-Werte schneller als Ultraschallwerte gelesen werden*/
					//std::cout << " 3: " << rc.chan3_scaled << " 5: " << rc.chan5_scaled << " 6: " << rc.chan6_scaled << " 7: " << rc.chan7_scaled << " 8: " << rc.chan8_scaled << "\n";
					break;
				case MAVLINK_MSG_ID_HUCH_RANGER:
					if (1) {
						mavlink_huch_ranger_t hmsg;
						mavlink_msg_huch_ranger_decode(&msg,&hmsg);
						int baro_tmp = (hmsg.ranger3 << 16) + hmsg.ranger2;
						static unsigned char first_baro;
						static int first_baro_val;
						if (first_baro && srf[SE_COUNT].get_mean() < 200) {
							first_baro = 0;
							//std::cout << "fb0\n";
						}
						if (!first_baro && hmsg.ranger1 > 300) {
							first_baro_val = baro_tmp-300;
							first_baro = 1;
							//std::cout << "fb1" << " fbv: " << first_baro_val << "\n";
						}
						srf[SE_COUNT].read_it_extern(get_current_millis(),hmsg.ranger1,(baro_tmp - first_baro_val),zacc);
						thrust = between<short>(pid_thrust.get(CONST_HEIGHT - srf[SE_COUNT].get_mean(),srf[SE_COUNT].get_msec_diff()),0,max_thrust);
						zacc = 0;
						send_ext_ctrl();
						#if LOG > 0
						std::fprintf(fd_max,"%lu\t%u\t%u\t%d\n", srf[SE_COUNT].get_msec(), srf[SE_COUNT].get_data(), srf[SE_COUNT].get_mean(),(baro_tmp-first_baro_val));
						#endif
					}
					break;
				default: break;
			}
		}
	}		

	/*Berechnung der vorgeschlagenen Werte fuer roll, pitch und yaw an Hand der neuen Messwerte*/
	if (state != IDLE && state != INIT && !new_value && !new_heading) {/*Wenn keine neuen Daten vorhanden sind -> Abarbeitung überspringen*/return;}
	switch (state) {	
		case IDLE: /*Wartet auf Aktivierung der externen Kontrolle*/
			break;
		case INIT:
			std::cout << "INIT\n";
			/*Initialisiert Abarbeitung, wenn externe Kontrolle zum ersten Mal aktiviert wurde*/
			schedule_add(5000,CHECK_STILL,0);
			#if LOG > 0
			schedule_add(LOG_SPEED,SAVE_LOG,0);
			//schedule_add(SLAM_SPEED,SAVE_SLAM,0);
			#endif
			for (int i = 0; i < SE_COUNT; i++) {
				if ((int)pow(2,i) & ENABLED_SENSORS) {
					schedule_add(0, MEASURE, SE0_ADDRESS + i);
				}
			}
			slam_init(current_heading,get_current_millis());
			state = DELAY;
			break;
		case HOLD_STILL:
			if(1) {
				static short set_point[SE_COUNT];
				if (hs_state == 0) {
					hs_state = 1;
					set_point[0] = (srf[0].get_mean() + srf[1].get_mean())/2;
					set_point[1] = set_point[0];
					set_point[2] = (srf[2].get_mean() + srf[3].get_mean())/2;
					set_point[3] = set_point[2];
					//for (int i = 0; i < SE_COUNT; i++) {
					//	set_point[i] = srf[i].get_mean();
					//	std::cout << "Set_Point " << i << ":" << set_point[i] << "\n";
					//}
					/*for (int i = 0; i < SE_COUNT; i++) {
						if (set_point[i] < 100 || set_point[i] > 0) {
							if (i == 0 || i == 2) {
								set_point[i+1] -= (100 - set_point[i]);
							} else {
								set_point[i-1] -= (100 - set_point[i]);
							}
							set_point[i] = 100;
						}
					}*/
					for (int i = 0; i < SE_COUNT; i++) {
						std::cout << "Set_Point " << i << ":" << set_point[i] << "\n";
					}
				}
				/*Veranlasst den Copter auf Grundlage der Änderungen der Messwerte still zu stehen*/
				if (!new_value) break;
				if (nvalue[0] == 1 && nvalue[1] == 1) {
					if (srf[0].get_mean() == SE_MIN)	roll = between<short>(pid_roll.get(0 - srf[1].get_mean() + set_point[1],srf[1].get_msec_diff()),-max_roll,max_roll);
					else if (srf[1].get_mean() == SE_MIN) 	roll = between<short>(pid_roll.get(srf[0].get_mean() - set_point[0],srf[0].get_msec_diff()),-max_roll,max_roll);
					else 					roll = between<short>(pid_roll.get(((srf[0].get_mean() - set_point[0]) - (srf[1].get_mean() - set_point[1]))/2,(srf[0].get_msec_diff() + srf[1].get_msec_diff())/2),-max_roll,max_roll);
					nvalue[0] = 0; nvalue[1] = 0;
				}
				if (nvalue[2] == 1 && nvalue[3] == 1) {
					if (srf[2].get_mean() == SE_MIN)	pitch = between<short>(pid_pitch.get(0 - srf[3].get_mean() + set_point[3],srf[3].get_msec_diff()),-max_pitch,max_pitch);
					else if (srf[3].get_mean() == SE_MIN) 	pitch = between<short>(pid_pitch.get(srf[2].get_mean() - set_point[2],srf[2].get_msec_diff()),-max_pitch,max_pitch);
					else 					pitch = between<short>(pid_pitch.get(((srf[2].get_mean() - set_point[2]) - (srf[3].get_mean() - set_point[3]))/2,(srf[2].get_msec_diff() + srf[3].get_msec_diff())/2),-max_pitch,max_pitch);
					nvalue[2] = 0; nvalue[3] = 0;
				}
				yaw = 0;
				send_ext_ctrl();
				/*Prüfung auf Stillstand*/
				if (abs(roll) <= STILL_FAK_ROLL*max_roll/100 && abs(pitch) <= STILL_FAK_PITCH*max_pitch/100) {
					//still = 1; 
					still = 0; /*FIXME*/
				} else {
					still = 0;
				}
			}
		break;
		case MEASURE_ENVIRONMENT:
			/*Vermisst die Umgebung durch eine Drehung um 360/SE_COUNT Grad*/
			if (breakpoint == 1) {state = HOLD_STILL; hs_state= 0; break;}
			if (abs(xacc) > HOLD_STILL_MAX_XACC || abs(yacc) > HOLD_STILL_MAX_YACC) {/*Wurde der Copter zu stark bewegt: Abbruch*/state = HOLD_STILL; hs_state = 0; std::cout << "State changed to HOLD_STILL\n"; break;}
			if (first_heading == -1) {
				/*Speichert die anfaengliche Ausrichtung und veranlasst den Copter sich zu drehen*/
				std::cout << "State changed to MEASURE_ENVIRONMENT\n";
				first_heading = current_heading;
				roll = 0; pitch = 0; rotation = 0; yaw = rotation_angle_sign*CONST_YAW;
				send_ext_ctrl();
			}
			/*Berechnung der aktuellen Drehung*/
			if (new_heading) {
				if (abs(current_heading - old_heading) > 180/SE_COUNT)	rotation += current_heading - old_heading - sign(current_heading - old_heading)*360;
				else 							rotation += current_heading - old_heading;
				for(int i = 0; i < SE_COUNT; i++) {
					if(nvalue[i] == 1) {
						env[(current_heading + srf[i].get_alignment()*(360/SE_COUNT))%360] = srf[i].get_mean();
						nvalue[i] = 0;
					}
				}
				//if (abs(rotation) >= 360/SE_COUNT) {
				if (abs(rotation) >= rotation_angle) {
					/*Copter hat sich ausreichend gedreht, Abbruch der Vermessung*/
					//state = HEAD_TO_MIDDLE;
					state = GET_ALIGNMENT;
					roll = 0; pitch = 0; yaw = 0;
					send_ext_ctrl();
					rotation_angle_sign = sign(rotation);
				}
			}
			break;
		case GET_ALIGNMENT: {
			short min_v = 0;
			for (int i = 0; i < rotation_angle; i++) {
				if (min_v < env[(first_heading + rotation_angle_sign*i + 360 + srf[align_se].get_alignment()*(360/SE_COUNT))%360]) {
					min_v = env[(first_heading + rotation_angle_sign*i + 360 + srf[align_se].get_alignment()*(360/SE_COUNT))%360];
					desired_heading = (first_heading + rotation_angle_sign*i + 360 + srf[align_se].get_alignment()*(360/SE_COUNT))%360;
				}
				/*for (int j = 0; j < SE_COUNT; j++) {
					if (min_v[j] < env[(first_heading + rotation_angle_sign*i + j*360/SE_COUNT + 360)%360]) {
						min_v[j] = env[(first_heading + rotation_angle_sign*i + j*360/SE_COUNT + 360)%360];
						min_h[j] = (first_heading + rotation_angle_sign*i + 360)%360;
					}
				}*/
			}
			//desired_heading = round((float)(min_h[0]+min_h[1]+min_h[2]+min_h[3])/4);
			/*Bestimmung der künftigen Drehrichtung*/
			for (int k = 0; k < rotation_angle/2; k++) {
				/*Wenn Minimum in der ersten Hälfte der Drehung gefunden wurde, Wechsel der Drehrichtung bzw. Abbruch*/
				if (desired_heading == (first_heading+rotation_angle_sign*k)%360) {
					rotation_angle_sign *= -1;
					if (init_state == 2) init_state = 3;
					break;
				}
			}
			/*Cleanup*/
			first_heading = -1; yaw = 0; pid_roll.reset(); pid_pitch.reset();
			for(int i = 0; i < 360; i++) env[i] = 0;
			
			if (init_state == 1) 	init_state = 2;
			if (init_state == 3) {
				state = GET_ANCHOR;
				pid_yaw.reset();
				pid_yaw.set(HTM_YAW_KP,HTM_YAW_TN,HTM_YAW_TV);
				pid_yaw.set_target(0);
				roll = 0; pitch = 0; yaw = 0;
				std::cout << "State changed to GET_ANCHOR\n";
				tv_old_heading = get_current_millis();
			} else {
				state = HOLD_STILL;
				hs_state = 0;
			}
			break;
		}
		case GET_ANCHOR:
			/*Copter versucht sich stabil auf ANCHOR_DISTANCE und desired_heading zu stellen*/
			if (new_heading) {
				short heading_diff;
				/*Berechnung der Differenz zwischen aktueller Ausrichtung und gewünschter Ausrichtung*/
				if (abs(desired_heading - current_heading) > 180) {
					heading_diff = desired_heading - current_heading - sign(desired_heading - current_heading)*360;
				} else {
					heading_diff = desired_heading - current_heading;
				}
				yaw = pid_yaw.get(heading_diff,get_current_millis() - tv_old_heading);
				tv_old_heading = get_current_millis();
			}
			if (new_value) {
				roll = 0; pitch = 0;
				if (nvalue[align_se] == 1) {
					if (align_se == 0)	roll   = between<short>(pid_roll_anc.get(srf[align_se].get_mean(), srf[align_se].get_msec_diff()), -max_roll,  max_roll);
					else if (align_se == 1)	roll   = between<short>(pid_roll_anc.get((-1)*srf[align_se].get_mean(), srf[align_se].get_msec_diff()), -max_roll,  max_roll);
					else if (align_se == 2)	pitch  = between<short>(pid_pitch_anc.get(srf[align_se].get_mean(), srf[align_se].get_msec_diff()), -max_pitch,  max_pitch);
					else if (align_se == 3)	pitch  = between<short>(pid_pitch_anc.get((-1)*srf[align_se].get_mean(), srf[align_se].get_msec_diff()), -max_pitch,  max_pitch);
					nvalue[align_se] = 0;
				}
				if 	(srf[0].get_mean() < 30) 	roll = 1000;
				else if (srf[1].get_mean() < 30)	roll = -1000;
				else if (srf[2].get_mean() < 30)	pitch = 1000;
				else if (srf[3].get_mean() < 30)	pitch = -1000;
				if (abs(srf[0].get_mean() - ANCHOR_DISTANCE) < 5) {
					for (int i = 0; i < SE_COUNT; i++) nvalue[i] = 0;
					/*state = MOVE_BACKWARD*/
					/*TODO Neuausrichtung des Copters?*/
				}
			}
			if (breakpoint == 3) 		{roll = 0; pitch = 0; send_ext_ctrl();}
			else if (breakpoint != 2) 	{send_ext_ctrl();}
			break;
		case DELAY: 	break;
		case HTM_DELAY:	break;
		default:	break;
	}
	#if LOG > 0
	std::fprintf(fd_data,"%lu\t%d\t%d\t%d\t%d\t%d\t%f\t%f\n", (unsigned long)get_current_millis(), roll, pitch, yaw, current_heading, state, current_roll_rad, current_pitch_rad);
	#endif	
}