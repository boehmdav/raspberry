#include "middle.h"

unsigned int get_current_millis() {
	struct timeval tv; gettimeofday(&tv,NULL); 
	return ((tv.tv_sec - tv_start.tv_sec)*1000 + (tv.tv_usec - tv_start.tv_usec)/1000);
}

/*Fuegt dem Scheduler eine neue Aufgabe hinzu*/
void schedule_add(unsigned int plus, enum sched_tasks current_task, short param) {	
	struct sched_task new_task;
	new_task.tv_msec	= plus + get_current_millis();
	new_task.task 		= current_task;
	new_task.param 		= param;
	int i;
	for (i = 0; i < MAX_TASKS; i++) {
		if (scheduler[i].task == NOTHING) {
			scheduler[i] = new_task;
			#if DEBUG_LEVEL > 2
			printf("%u SCHEDULE: Neuer Task hinzugefuegt: %u %d %d\n", get_current_millis(), new_task.tv_msec, new_task.task, new_task.param);
			#endif
			break;
		} else if (i == MAX_TASKS-1) {
			if(WARNINGS){perror("SCHEDULE_ADD: Buffer voll, Task konnte nicht eingefuegt werden.");}
		}
	}
}

void send_ext_ctrl() {
	#if EXT_CTRL > 0
	/*Sendet die aktuell vorgeschlagenen Werte fuer roll und pitch an den Copter*/
	mavlink_message_t ex_msg;
	uint8_t ex_buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_msg_huch_ext_ctrl_pack(0, MAV_COMP_ID_IMU, &ex_msg, TARGET_SYSTEM, TARGET_COMPONENT, 0, roll, pitch, yaw, 0);
	uint16_t ex_len = mavlink_msg_to_send_buffer(ex_buf, &ex_msg);
	if (write(tty_fd, ex_buf, ex_len) != ex_len) {if(WARNINGS){perror("LOOP: Es konnten keine Daten fuer die externe Kontrolle gesendet werden.");}}
	#endif	
}

void request_data_stream(short stream_id, short mav_speed, unsigned char mav_mode) {
	//std::cout << "ID:" << stream_id << " Speed:" << mav_speed << " Mode:" << (int)mav_mode << "\n";
	mavlink_message_t smsg;
	uint8_t sbuf[MAVLINK_MAX_PACKET_LEN];
	mavlink_msg_request_data_stream_pack(0, MAV_COMP_ID_IMU, &smsg, 1, 0, stream_id, mav_speed, mav_mode);
	uint16_t len = mavlink_msg_to_send_buffer(sbuf, &smsg);
	if (write(tty_fd, sbuf, len) != len) {perror("LOOP: Es konnte kein Datenstream abbonniert werden."); exit(1);}
}

/*Stoppt die aktiven Datenstreams und beendet das Programm, wenn SIGINT gesendet wird*/
void signal_callback_handler(int signum) {
   request_data_stream(MAV_DATA_STREAM_ALL,0,0);
   exit(signum);
}

int main () {
	setup();
	while(1) loop();
	return 0;
}

/*Initialisierung*/
void setup() {
	/*Oeffnen der seriellen Schnitstelle TTY_DEVICE*/
	tty_fd = open(TTY_DEVICE, O_RDWR);
	if (tty_fd == -1) {perror("SETUP: " TTY_DEVICE " kann nicht geoeffnet werden."); exit(1);}
	
	/*Konfiguration der seriellen Schnittstelle*/	
	if (tcgetattr(tty_fd, &attr) != 0) {perror("SETUP: tcgetattr() fehlgeschlagen."); exit(1);}
	/*input   modes*/ attr.c_iflag = 0;
	/*output  modes*/ attr.c_oflag = OPOST | ONLCR;	
	/*control modes*/ attr.c_cflag = TTY_DEVICE_SPEED | CS8 | CRTSCTS | CLOCAL | CREAD;
	/*local   modes*/ attr.c_lflag = 0;
	if (tcsetattr(tty_fd, TCSAFLUSH, &attr) != 0){perror("SETUP: tcsetattr() fehlgeschlagen"); exit(1);}
	first_heartbeat = 0;	

	srf_fd = open(SRF_DEVICE, O_RDWR);
	if (srf_fd == -1) {perror("SETUP: " SRF_DEVICE "kann nicht geoeffnet werden."); exit(1);}
	
	/*Einstellen der anfaenglichen Messgeschwindigkeiten, Erzeugen der Sensoren*/
	for (int i = 0; i < SE_COUNT; i++) {
		srf_speed[i] = SRF_SPEED;
		srf.push_back(SRF((SE0_ADDRESS + i),srf_fd));
		nvalue[i] = 0;
		nvalue2[i] = 0;
	}

	/*signal handler für SIGINT (Strg+C) registrieren*/
	signal(SIGINT, signal_callback_handler);
	
	/*Initialisierung des Schedulers*/
	gettimeofday(&tv_start,NULL);
	for (int i = 0; i < MAX_TASKS; i++) 	scheduler[i].task = NOTHING;

	/*Erzeugen des Log-Verzeichnis und Log-Dateien*/
	#if LOG > 0
	time_t current_timestamp = time(0);
	tm *current_time;
	current_time = localtime(&current_timestamp);
	sprintf(log_dir,"log/%d_%d_%d %d:%d:%d",(current_time->tm_year+1900),(current_time->tm_mon+1),(current_time->tm_mday),(current_time->tm_hour),(current_time->tm_min),(current_time->tm_sec));
	if (mkdir(log_dir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) < 0) {perror("Verzeichnis fuer Log-Dateien konnte nicht angelegt werden."); exit(1);}
	char tmp_dir[32]; strcpy(tmp_dir,log_dir);
	fd_112 = fopen(strcat(tmp_dir,"/112"), "a"); fprintf(fd_112,"#TS\tdata\tmean\n"); strcpy(tmp_dir,log_dir);
	fd_113 = fopen(strcat(tmp_dir,"/113"), "a"); fprintf(fd_113,"#TS\tdata\tmean\n"); strcpy(tmp_dir,log_dir);
	fd_114 = fopen(strcat(tmp_dir,"/114"), "a"); fprintf(fd_114,"#TS\tdata\tmean\n"); strcpy(tmp_dir,log_dir);
	fd_115 = fopen(strcat(tmp_dir,"/115"), "a"); fprintf(fd_115,"#TS\tdata\tmean\n"); strcpy(tmp_dir,log_dir);
	fd_data= fopen(strcat(tmp_dir,"/data"),"a"); fprintf(fd_data,"#TS\troll\tpitch\tyaw\theading\tstate\txacc\tyacc\n"); strcpy(tmp_dir,log_dir);
	#endif

	current_heading = 0;	
	state = IDLE;
	roll = 0; pitch = 0; yaw = 0;
	still = 0;
	first_heading = -1;
	for(int i = 0; i < 360; i++) env[i] = 0;
	xacc = 0; yacc = 0;
	
	init_state = 0;
	
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
				schedule_add(srf_speed[scheduler[i].param - SE0_ADDRESS], READ_MEASURE, scheduler[i].param);
				scheduler[i].task = NOTHING;
				break;
			case READ_MEASURE:
				/*Liest die Messdaten des als Parameter uebergebenen Sensors aus und veranlasst erneute Messung*/
				srf[scheduler[i].param - SE0_ADDRESS].read_it(get_current_millis());
				schedule_add(0, MEASURE, scheduler[i].param);
				if(state == HOLD_STILL || state == MEASURE_ENVIRONMENT) {
					nvalue[scheduler[i].param - SE0_ADDRESS] = 1;
				}
				/*if (state == HOLD_STILL || state == HEAD_TO_MIDDLE || state = HTM_DELAY) {
					nvalue2[scheduler[i].param - SE0_ADDRESS] = 1;
				}*/
				new_value = scheduler[i].param;
				
				/*if (state == HOLD_STILL || state ==  HEAD_TO_MIDDLE || state == HTM_DELAY) {
					if (nvalue2[0] == 1 && nvalue2[1] == 1 && nvalue2[2] == 1 && nvalue2[3] == 1) {
						slam_insert_v((srf[0].get_cm_per_s() + srf[1].get_cm_per_s())/2,(srf[2].get_cm_per_s() + srf[3].get_cm_per_s())/2,current_heading,get_current_millis());
						nvalue2[0] = 0; nvalue2[1] = 0; nvalue2[2] = 0; nvalue2[3] = 0;
					}
				}*/
				/*Speichert den gelesenen Messwert in der entsprechenden Log-Datei*/
				#if LOG > 0
				FILE *fd;
				if (scheduler[i].param == SE0_ADDRESS) 	fd = fd_112;
				else if (scheduler[i].param == SE1_ADDRESS) 	fd = fd_113;
				else if (scheduler[i].param == SE2_ADDRESS) 	fd = fd_114;
				else						fd = fd_115;
				fprintf(fd,"%u\t%u\t%u\n", srf[scheduler[i].param - SE0_ADDRESS].get_msec(), srf[scheduler[i].param - SE0_ADDRESS].get_data(), srf[scheduler[i].param - SE0_ADDRESS].get_mean());
				#endif
				scheduler[i].task = NOTHING;
				break;
			case SAVE_LOG:
				/*Sichert die bisher geloggten Daten*/
				#if LOG > 0
				fclose(fd_112);
				fclose(fd_113);
				fclose(fd_114);
				fclose(fd_115);
				fclose(fd_data);
				char tmp_dir[32]; strcpy(tmp_dir,log_dir);
				fd_112 = fopen(strcat(tmp_dir,"/112"), "a"); strcpy(tmp_dir,log_dir);
				fd_113 = fopen(strcat(tmp_dir,"/113"), "a"); strcpy(tmp_dir,log_dir);
				fd_114 = fopen(strcat(tmp_dir,"/114"), "a"); strcpy(tmp_dir,log_dir);
				fd_115 = fopen(strcat(tmp_dir,"/115"), "a"); strcpy(tmp_dir,log_dir);
				fd_data= fopen(strcat(tmp_dir,"/data"),"a"); strcpy(tmp_dir,log_dir);
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
				scheduler[i].task = NOTHING;
				break;
			case SHOW_ME:
				/*Schreibt Messwerte und Neigungswinkel auf das Terminal*/
				if (roll > 9 || roll < 0) 	printf("roll: %d\tpitch: %d \tSE0: %d\tSE1: %d\tSE2: %d\tSE3:%d\tstate:%d\n", roll, pitch, srf[0].get_mean(), srf[1].get_mean(), srf[2].get_mean(), srf[3].get_mean(),(short)state);
				else 				printf("roll: %d\t\tpitch: %d \tSE0: %d\tSE1: %d\tSE2: %d\tSE3:%d\tstate%d\n", roll, pitch, srf[0].get_mean(), srf[1].get_mean(), srf[2].get_mean(), srf[3].get_mean(),(short)state	);
				schedule_add(100,SHOW_ME,0);
				scheduler[i].task = NOTHING;
				break;
			default: if(WARNINGS){perror("LOOP: Unbekannte Aufgabe im Scheduler.");} break; 
		}
	}

	/*Auswertung der Daten, die an der seriellen Schnittstelle anliegen.*/
	struct timeval tv; tv.tv_sec = 0; tv.tv_usec = 0;
	/*Prueft, ob Daten an der seriellen Schnittstelle anliegen*/
	FD_ZERO(&tty_fdset);
	FD_SET(tty_fd, &tty_fdset);
	if (select(tty_fd+1, &tty_fdset, NULL, NULL, &tv) == -1) {perror("LOOP: select() fehlgeschlagen"); exit(1);}

	/*Liest ein einzelnes Byte von der seriellen Schnittstelle und prueft, ob ein Mavlinkpaket vervollstaendigt wurde*/
	if (FD_ISSET(tty_fd, &tty_fdset)) {
		static mavlink_message_t msg;
		static mavlink_status_t status;
		char c[1];
		if (read(tty_fd,c,1) == -1) {if(WARNINGS){perror("LOOP: Fehler beim Lesen aus " TTY_DEVICE);}}
		#if DEBUG_LEVEL > 2
		printf("%#x\n",c[0]);
		#endif
		if (mavlink_parse_char(MAVLINK_COMM_0,(uint8_t) *c, &msg, &status)) {
			#if DEBUG_LEVEL > 1
			printf("%u Mavlinkpaket empfangen: %d\n", get_current_millis(), msg.msgid);			
			#endif			
			//printf("%u Mavlinkpaket empfangen: %d\n", get_current_millis(), msg.msgid);
			
			switch (msg.msgid) {
				case MAVLINK_MSG_ID_HEARTBEAT:
					/*Beim Empfang des ersten Heartbeats wird zunaechst ein eventuell vorhandener Datenstream gekuendigt und ein neuer Datenstream abonnemiert*/
					if (!first_heartbeat) {
						first_heartbeat = 1;
						request_data_stream(MAV_DATA_STREAM_ALL,0,0);
						request_data_stream(MAV_DATA_STREAM_RC_CHANNELS,DATA_STREAM_SPEED,1);
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
				case MAVLINK_MSG_ID_RAW_IMU:
					/*Speichert die Beschleunigungen auf der x- und y-Achse*/
					mavlink_raw_imu_t raw_imu;
					mavlink_msg_raw_imu_decode(&msg,&raw_imu);
					xacc = raw_imu.xacc;
					yacc = raw_imu.yacc;
					//slam_insert_acc(xacc,yacc,current_heading,get_current_millis());
					//std::cout << "xacc: " << xacc << " yacc: " << yacc << "\n";
					break;
				case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
					/*Prüft, ob eine Umstellung auf externe Kontrolle erfolgt ist*/
					if (state != IDLE) {break;}
					mavlink_rc_channels_raw_t rc;
					mavlink_msg_rc_channels_raw_decode(&msg,&rc);
					//std::cout << "RC_CH5_Raw: " << rc.chan5_raw << "\n";
					/*TODO Entfernen*/init_state = 1; rc.chan5_raw = MODE_SWITCH_RANGE_UP-1;
					if (init_state && rc.chan5_raw < MODE_SWITCH_RANGE_UP && rc.chan5_raw > MODE_SWITCH_RANGE_DOWN) {
						std::cout << "State changed from IDLE to INIT.\n";
						schedule_add(0,CHANGE_STATE,(int)INIT);
						schedule_add(2000,CHANGE_STATE,(int)HOLD_STILL);
						request_data_stream(MAV_DATA_STREAM_RC_CHANNELS,0,0);
						request_data_stream(MAV_DATA_STREAM_EXTRA2/*VFR_HUD*/,DATA_STREAM_SPEED,1);
						request_data_stream(MAV_DATA_STREAM_RAW_SENSORS,DATA_STREAM_SPEED,1);
					}
					if(rc.chan5_raw > MODE_SWITCH_RANGE_UP || rc.chan5_raw < MODE_SWITCH_RANGE_DOWN) {
						init_state = 1;
					}
				default: break;
			}
		}
	}		

	/*Berechnung der vorgeschlagenen Werte fuer roll, pitch und yaw an Hand der neuen Messwerte*/
	if (!new_value && !new_heading) {/*Wenn keine neuen Daten vorhanden sind -> Abarbeitung überspringen*/return;}
	switch (state) {
		case IDLE: /*Wartet auf Aktivierung der externen Kontrolle*/ break;
		case INIT:
			/*Initialisiert Abarbeitung, wenn externe Kontrolle zum ersten Mal aktiviert wurde*/
			schedule_add(5000,CHECK_STILL,0);
			#if LOG > 0
			schedule_add(LOG_SPEED,SAVE_LOG,0);
			//schedule_add(SLAM_SPEED,SAVE_SLAM,0);
			#endif
			for (int i = 0; i < SE_COUNT; i++) schedule_add(0, MEASURE, SE0_ADDRESS + i);
			slam_init(current_heading,get_current_millis());
			state = DELAY;
			break;
		case HOLD_STILL:
			/*Veranlasst den Copter auf Grundlage der Änderungen der Messwerte still zu stehen*/
			if (new_heading) break;
			if (nvalue[0] == 1 && nvalue[1] == 1) {
				roll  = between<short>(pid_roll.get((srf[0].get_mean_diff()+srf[1].get_mean_diff())/2, (srf[0].get_msec_diff() + srf[1].get_msec_diff())/2), -MAX_ROLL,  MAX_ROLL);
				nvalue[0] = 0; nvalue[1] = 0;
			}
			if (nvalue[2] == 1 && nvalue[3] == 1) {
				pitch  = between<short>(pid_pitch.get((srf[2].get_mean_diff()+srf[3].get_mean_diff())/2, (srf[2].get_msec_diff() + srf[3].get_msec_diff())/2), -MAX_PITCH,  MAX_PITCH);
				nvalue[2] = 0; nvalue[3] = 0;
			}
			yaw = 0;
			send_ext_ctrl();	
			/*Prüfung auf Stillstand*/
			if (abs(roll) <= STILL_FAK_ROLL*MAX_DISTANCE/100 && abs(pitch) <= STILL_FAK_PITCH*MAX_DISTANCE/100) {
				still = 1;
			} else {
				still = 0;
			}
		break;
		case MEASURE_ENVIRONMENT:
			/*Vermisst die Umgebung durch eine Drehung um 360/SE_COUNT Grad*/
			/*TODO ENTFERNEN*/ //state = HOLD_STILL; break;
			if (abs(xacc) > HOLD_STILL_MAX_XACC || abs(yacc) > HOLD_STILL_MAX_YACC) {/*Wurde der Copter zu stark bewegt: Abbruch*/state = HOLD_STILL; std::cout << "State changed to HEAD_TO_MIDDLE\n"; break;}
			if (first_heading == -1) {
				/*Speichert die anfaengliche Ausrichtung und veranlasst den Copter sich zu drehen*/
				std::cout << "State changed to MEASURE_ENVIRONMENT\n";
				//TODO: Entfernen slam_refresh_pos(get_current_millis());
				first_heading = current_heading;
				roll = 0; pitch = 0; rotation = 0; yaw = CONST_YAW;
				send_ext_ctrl();
			}
			if (new_value) slam_insert_measurement(srf[new_value-SE0_ADDRESS].get_mean(),current_heading + (360/SE_COUNT)*(new_value-SE0_ADDRESS));
			/*Berechnung der aktuellen Drehung*/
			if (new_heading) {
				if (abs(current_heading - old_heading) > 180/SE_COUNT)	rotation += current_heading - old_heading - sign(current_heading - old_heading)*360;
				else 							rotation += current_heading - old_heading;
				for(int i = 0; i < SE_COUNT; i++) {
					if(nvalue[i] == 1) env[(current_heading + i*(360/SE_COUNT))%360] = srf[i].get_mean();
					nvalue[i] = 0;
				}
				if (abs(rotation) >= 360/SE_COUNT) {
					/*Copter hat sich ausreichend gedreht, Abbruch der Vermessung*/
					state = HEAD_TO_MIDDLE;
					roll = 0; pitch = 0; yaw = 0;
					send_ext_ctrl();
				}
			}
			break;
		case HEAD_TO_MIDDLE:
			/*Berechnet an Hand der vermessenen Umgebung vorgeschlagene Werte um den Mittelpunkt zu erreichen*/
			if(1) {
				std::cout << "State changed to HEAD_TO_MIDDLE\n";
				printf("First_heading:%d\n",first_heading);
				/*for (int i = 0; i < 90; i++) {
					printf("%d\t%d\t%d\t%d\t%d\n", i, env[(first_heading + i + 360/SE_COUNT)%360],env[(first_heading + i + 90 + 360/SE_COUNT)%360], env[(first_heading + i + 180 + 360/SE_COUNT)%360],env[(first_heading + i + 270 + 360/SE_COUNT)%360]);
				}*/
				short used_headings = 360/SE_COUNT;
				for (int i = 0; i < 360/SE_COUNT; i++) {
					if (env[(first_heading + i + 360/SE_COUNT)%360] == 0 || env[(first_heading + i + 180 + 360/SE_COUNT)%360] == 0) {used_headings--; continue;}
					short tmp_roll  = env[(first_heading + i + 360/SE_COUNT)%360] - env[(first_heading + i + 180 + 360/SE_COUNT)%360];
					short tmp_pitch = env[(first_heading + i + 90 + 360/SE_COUNT)%360] - env[(first_heading + i + 270 + 360/SE_COUNT)%360];
					roll +=  (short)(sign(tmp_roll) *HTMR(abs(tmp_roll)) *sin(i*PI/180));
					pitch += (short)(sign(tmp_pitch)*HTMP(abs(tmp_pitch))*cos(i*PI/180));
				}
				if (used_headings == 0)	{roll = 0; pitch = 0;}
				else 			{roll = roll/used_headings; pitch = pitch/used_headings; rotation = 0;}
				first_heading = -1; yaw = 0; pid_roll.reset(); pid_pitch.reset();
				state = HTM_DELAY;
				schedule_add(HTM_DELAY_TIME, CHANGE_STATE, HOLD_STILL);
				send_ext_ctrl();
				slam_refresh_pos(get_current_millis());
				slam_draw_image("slam.ppm");
				std::cout << "State changed to HOLD_STILL\n";
				//printf("%d\t%d\n",roll,pitch);
			}
		break;
		case DELAY: break;
		case HTM_DELAY: break;
		default: break;
	}
	#if LOG > 0
	fprintf(fd_data,"%u\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", get_current_millis(), roll, pitch, yaw, current_heading, state, xacc, yacc);
	#endif	
}