#include "slam.h"

std::vector< std::vector<unsigned char> > env_no;
std::vector< std::vector<unsigned char> > env_nw;
std::vector< std::vector<unsigned char> > env_so;
std::vector< std::vector<unsigned char> > env_sw;

std::vector< std::vector<unsigned char> > env_no_tmp;
std::vector< std::vector<unsigned char> > env_nw_tmp;
std::vector< std::vector<unsigned char> > env_so_tmp;
std::vector< std::vector<unsigned char> > env_sw_tmp;

struct position slam_pos;
struct position old_pos;

unsigned short null_heading;

float slam_xr, slam_yr;
float slam_xv, slam_yv;
unsigned int old_tv_msec_acc;
unsigned int acc_tv_msec;

template <typename T>
T max(T a, T b) {
	if (a > b) return a;
	return b;
}


void slam_insert (struct position pos, unsigned char val) {
	if (pos.x >= 0 && pos.y >= 0) {
		while ((unsigned int) pos.x >= env_no.size()) {
			static std::vector<unsigned char> tmp;
			env_no.push_back(tmp);
		}
		while ((unsigned int) pos.y >= env_no[pos.x].size()) {
			env_no[pos.x].push_back(ENV_ITEM_UNDEF);
		}
		env_no[pos.x][pos.y] = val;
	} else if (pos.x >= 0 /*&& y < 0*/) {
		pos.y = -pos.y-1;
		while ((unsigned int) pos.x >= env_so.size()) {
			static std::vector<unsigned char> tmp;
			env_so.push_back(tmp);
		}
		while ((unsigned int) pos.y >= env_so[pos.x].size()) {
			env_so[pos.x].push_back(ENV_ITEM_UNDEF);
		}
		env_so[pos.x][pos.y] = val;
	} else if (/*x < 0*/ pos.y >= 0) {
		pos.x = -pos.x-1;
		while ((unsigned int) pos.x >= env_nw.size()) {
			static std::vector<unsigned char> tmp;
			env_nw.push_back(tmp);
		}
		while ((unsigned int) pos.y >= env_nw[pos.x].size()) {
			env_nw[pos.x].push_back(ENV_ITEM_UNDEF);
		}
		env_nw[pos.x][pos.y] = val;
	} else {
		pos.x = -pos.x-1; pos.y = -pos.y-1;
		while ((unsigned int) pos.x >= env_sw.size()) {
			static std::vector<unsigned char> tmp;
			env_sw.push_back(tmp);
		}
		while ((unsigned int) pos.y >= env_sw[pos.x].size()) {
			env_sw[pos.x].push_back(ENV_ITEM_UNDEF);
		}
		env_sw[pos.x][pos.y] = val;
	}
	//std::cout << "Insert ("<< pos.x << "," << pos.y << "):" << (int)slam_get(pos) << "size:"  << env_no.size() << " " << env_nw.size() << " " << env_so.size() << " " << env_sw.size() << "\n";
}

void slam_insert_tmp (struct position pos, unsigned char val) {
	if (pos.x >= 0 && pos.y >= 0) {
		while ((unsigned int) pos.x >= env_no_tmp.size()) {
			static std::vector<unsigned char> tmp;
			env_no_tmp.push_back(tmp);
		}
		while ((unsigned int) pos.y >= env_no_tmp[pos.x].size()) {
			env_no_tmp[pos.x].push_back(ENV_ITEM_UNDEF);
		}
		env_no_tmp[pos.x][pos.y] = val;
	} else if (pos.x >= 0 /*&& y < 0*/) {
		pos.y = -pos.y-1;
		while ((unsigned int) pos.x >= env_so_tmp.size()) {
			static std::vector<unsigned char> tmp;
			env_so_tmp.push_back(tmp);
		}
		while ((unsigned int) pos.y >= env_so_tmp[pos.x].size()) {
			env_so_tmp[pos.x].push_back(ENV_ITEM_UNDEF);
		}
		env_so_tmp[pos.x][pos.y] = val;
	} else if (/*x < 0*/ pos.y >= 0) {
		pos.x = -pos.x-1;
		while ((unsigned int) pos.x >= env_nw_tmp.size()) {
			static std::vector<unsigned char> tmp;
			env_nw_tmp.push_back(tmp);
		}
		while ((unsigned int) pos.y >= env_nw_tmp[pos.x].size()) {
			env_nw_tmp[pos.x].push_back(ENV_ITEM_UNDEF);
		}
		env_nw_tmp[pos.x][pos.y] = val;
	} else {
		pos.x = -pos.x-1; pos.y = -pos.y-1;
		while ((unsigned int) pos.x >= env_sw_tmp.size()) {
			static std::vector<unsigned char> tmp;
			env_sw_tmp.push_back(tmp);
		}
		while ((unsigned int) pos.y >= env_sw_tmp[pos.x].size()) {
			env_sw_tmp[pos.x].push_back(ENV_ITEM_UNDEF);
		}
		env_sw_tmp[pos.x][pos.y] = val;
	}
}

unsigned char slam_get(struct position pos) {
	if (pos.x >= 0 && pos.y >= 0) {
		if ((unsigned int)pos.x < env_no.size() && (unsigned int)pos.y < env_no[pos.x].size()) {
			return env_no[pos.x][pos.y];
		} else {
			return ENV_ITEM_UNDEF;
		}
	} else if (pos.x >= 0 /*&& y < 0*/) {
		pos.y = -pos.y-1;
		if ((unsigned int)pos.x < env_so.size() && (unsigned int)pos.y < env_so[pos.x].size()) {
			return env_so[pos.x][pos.y];
		} else {
			return ENV_ITEM_UNDEF;
		}
	} else if (/*x < 0*/ pos.y >= 0) {
		pos.x = -pos.x-1;
		if ((unsigned int)pos.x < env_nw.size() && (unsigned int)pos.y < env_nw[pos.x].size()) {
			return env_nw[pos.x][pos.y];
		} else {
			return ENV_ITEM_UNDEF;
		}
	} else {
		pos.x = -pos.x-1; pos.y = -pos.y-1;
		if ((unsigned int)pos.x < env_sw.size() && (unsigned int)pos.y < env_sw[pos.x].size()) {
			return env_sw[pos.x][pos.y];
		} else {
			return ENV_ITEM_UNDEF;
		}
	}
}

unsigned char slam_get_tmp(struct position pos) {
	if (pos.x >= 0 && pos.y >= 0) {
		if ((unsigned int)pos.x < env_no_tmp.size() && (unsigned int)pos.y < env_no_tmp[pos.x].size()) {
			return env_no_tmp[pos.x][pos.y];
		} else {
			return ENV_ITEM_UNDEF;
		}
	} else if (pos.x >= 0 /*&& y < 0*/) {
		pos.y = -pos.y-1;
		if ((unsigned int)pos.x < env_so_tmp.size() && (unsigned int)pos.y < env_so_tmp[pos.x].size()) {
			return env_so_tmp[pos.x][pos.y];
		} else {
			return ENV_ITEM_UNDEF;
		}
	} else if (/*x < 0*/ pos.y >= 0) {
		pos.x = -pos.x-1;
		if ((unsigned int)pos.x < env_nw_tmp.size() && (unsigned int)pos.y < env_nw_tmp[pos.x].size()) {
			return env_nw_tmp[pos.x][pos.y];
		} else {
			return ENV_ITEM_UNDEF;
		}
	} else {
		pos.x = -pos.x-1; pos.y = -pos.y-1;
		if ((unsigned int)pos.x < env_sw_tmp.size() && (unsigned int)pos.y < env_sw_tmp[pos.x].size()) {
			return env_sw_tmp[pos.x][pos.y];
		} else {
			return ENV_ITEM_UNDEF;
		}
	}
}

unsigned char slam_get(int x, int y) {
	struct position pos;
	pos.x = x; pos.y = y;
	return slam_get(pos);
}

unsigned char slam_get_tmp(int x, int y) {
	struct position pos;
	pos.x = x; pos.y = y;
	return slam_get_tmp(pos);
}


struct position line_value_x(float m, float n, int x) {
	struct position ret_val;
	ret_val.x = x;
	ret_val.y = (int) lroundf(m*x+n);
	return ret_val;
}

struct position line_value_y(float m, float n, int y) {
	struct position ret_val;
	ret_val.y = y;
	ret_val.x = (int) lroundf(m*y+n);
	return ret_val;
}

void draw_line(struct position start_pos, struct position target_pos, unsigned char start_val, unsigned char path_val, unsigned char target_val) {
	struct position tmp_pos; tmp_pos.x = start_pos.x; tmp_pos.y = start_pos.y;
	if (start_pos.x != target_pos.x) {
		float m = (float)(target_pos.y - start_pos.y)/(float)(target_pos.x - start_pos.x);
		float n = (float)start_pos.y - m*(float)start_pos.x;
		if (start_pos.x < target_pos.x) {
			while(tmp_pos.x < target_pos.x) {
				slam_insert(line_value_x(m,n,tmp_pos.x),path_val);
				tmp_pos.x++;
			}
		} else {
			while(tmp_pos.x > target_pos.x) {
				slam_insert(line_value_x(m,n,tmp_pos.x),path_val);
				tmp_pos.x--;
			}
		}
	}
	if (start_pos.y != target_pos.y) {
		float m = (float)(target_pos.x - start_pos.x)/(float)(target_pos.y - start_pos.y);
		float n = (float)start_pos.x - m*(float)start_pos.y;
		if (start_pos.y < target_pos.y) {
			while(tmp_pos.y < target_pos.y) {
				slam_insert(line_value_y(m,n,tmp_pos.y),path_val);
				tmp_pos.y++;
			}
		} else {
			while(tmp_pos.y > target_pos.y) {
				slam_insert(line_value_y(m,n,tmp_pos.y),path_val);
				tmp_pos.y--;
			}
		}
	}
	slam_insert(start_pos,start_val);
	slam_insert(target_pos,target_val);
}

void draw_line_tmp(struct position start_pos, struct position target_pos, unsigned char start_val, unsigned char path_val, unsigned char target_val) {
	struct position tmp_pos; tmp_pos.x = start_pos.x; tmp_pos.y = start_pos.y;
	if (start_pos.x != target_pos.x) {
		float m = (float)(target_pos.y - start_pos.y)/(float)(target_pos.x - start_pos.x);
		float n = (float)start_pos.y - m*(float)start_pos.x;
		if (start_pos.x < target_pos.x) {
			while(tmp_pos.x < target_pos.x) {
				slam_insert_tmp(line_value_x(m,n,tmp_pos.x),path_val);
				tmp_pos.x++;
			}
		} else {
			while(tmp_pos.x > target_pos.x) {
				slam_insert_tmp(line_value_x(m,n,tmp_pos.x),path_val);
				tmp_pos.x--;
			}
		}
	}
	if (start_pos.y != target_pos.y) {
		float m = (float)(target_pos.x - start_pos.x)/(float)(target_pos.y - start_pos.y);
		float n = (float)start_pos.x - m*(float)start_pos.y;
		if (start_pos.y < target_pos.y) {
			while(tmp_pos.y < target_pos.y) {
				slam_insert_tmp(line_value_y(m,n,tmp_pos.y),path_val);
				tmp_pos.y++;
			}
		} else {
			while(tmp_pos.y > target_pos.y) {
				slam_insert_tmp(line_value_y(m,n,tmp_pos.y),path_val);
				tmp_pos.y--;
			}
		}
	}
	slam_insert_tmp(start_pos,start_val);
	slam_insert_tmp(target_pos,target_val);
}


void slam_insert_measurement(unsigned short measurement, unsigned short heading) {
	short tmp_heading = (heading - null_heading+360)%360;
	struct position tmp_pos;
#if SLAM_RESOLUTION > 1
	measurement = measurement/SLAM_RESOLUTION;
#endif
	tmp_pos.x = slam_pos.x + (measurement+COPTER_RADIUS)*sin(RAD(tmp_heading));
	tmp_pos.y = slam_pos.y + (measurement+COPTER_RADIUS)*cos(RAD(tmp_heading));
	//std::cout << heading << " " << tmp_heading << " " << sin(RAD(tmp_heading)) << " " << tmp_pos.x << " " << tmp_pos.y <<"\n";
	draw_line_tmp(slam_pos,tmp_pos,ENV_ITEM_UNSOLID,ENV_ITEM_UNSOLID,ENV_ITEM_SOLID);
}

unsigned int max_y_n() {
	unsigned int ret_val = 0;
	for (unsigned int i = 0; i < env_no.size(); i++) {
		if (ret_val < env_no[i].size()) ret_val = env_no[i].size();
	}
	for (unsigned int i = 0; i < env_nw.size(); i++) {
		if (ret_val < env_nw[i].size()) ret_val = env_nw[i].size();
	}
	return ret_val;
}

unsigned int max_y_s() {
	unsigned int ret_val = 0;
	for (unsigned int i = 0; i < env_so.size(); i++) {
		if (ret_val < env_so[i].size()) ret_val = env_so[i].size();
	}
	for (unsigned int i = 0; i < env_sw.size(); i++) {
		if (ret_val < env_sw[i].size()) ret_val = env_sw[i].size();
	}
	return ret_val;
}

unsigned int max_y_n_tmp() {
	unsigned int ret_val = 0;
	for (unsigned int i = 0; i < env_no_tmp.size(); i++) {
		if (ret_val < env_no_tmp[i].size()) ret_val = env_no_tmp[i].size();
	}
	for (unsigned int i = 0; i < env_nw_tmp.size(); i++) {
		if (ret_val < env_nw_tmp[i].size()) ret_val = env_nw_tmp[i].size();
	}
	return ret_val;
}

unsigned int max_y_s_tmp() {
	unsigned int ret_val = 0;
	for (unsigned int i = 0; i < env_so_tmp.size(); i++) {
		if (ret_val < env_so_tmp[i].size()) ret_val = env_so_tmp[i].size();
	}
	for (unsigned int i = 0; i < env_sw_tmp.size(); i++) {
		if (ret_val < env_sw_tmp[i].size()) ret_val = env_sw_tmp[i].size();
	}
	return ret_val;
}

/*void slam_insert_acc(int xa, int ya, unsigned short heading, unsigned int tv_msec) {
	float tmp_msec = (tv_msec - old_tv_msec_acc)/1000;
	old_tv_msec_acc = tv_msec;
	short tmp_heading = (heading - null_heading+360)%360;
	
	slam_xv += ((float)ya*sin(RAD(tmp_heading)) + (float)xa*cos(RAD(tmp_heading)))*tmp_msec;
	slam_yv += ((float)xa*sin(RAD(tmp_heading)) + (float)ya*cos(RAD(tmp_heading)))*tmp_msec;
	slam_xr += slam_xv*tmp_msec; slam_yr += slam_yv*tmp_msec;
	std::cout << "xa:" << xa << " ya:" << ya << " h:" << heading << " tmp_h:" << tmp_heading << " xv:" << slam_xv << " yv:" << slam_yv << " xr:" << slam_xr << " yr:" << slam_yr <<"\n";
}*/

void slam_insert_v(float vx/*cm/s*/, float vy/*cm/s*/, unsigned short heading, unsigned int tv_msec) {
	short tmp_heading = (heading - null_heading+360)%360;
	float tmp_msec = (float)(tv_msec - old_tv_msec_acc)/1000;	/*Sekunde*/
	old_tv_msec_acc = tv_msec;
	
	//slam_xr += (vy*sin(RAD(tmp_heading)) + vx*cos(RAD(tmp_heading)))*tmp_msec; /*Zentimeter*/
	//slam_yr += (vx*sin(RAD(tmp_heading)) + vy*cos(RAD(tmp_heading)))*tmp_msec; /*Zentimeter*/
	slam_xr += vx*tmp_msec; slam_yr += vy*tmp_msec;
	std::cout << "vx:" << vx << "\tvy:" << vy << "\th:" << heading << "\ttmp_h:" << tmp_heading << "\ttmp_msec:" << tmp_msec << "\txr:" << slam_xr << "\tyr:" << slam_yr <<"\n";
	//std::cout << vy*sin(RAD(tmp_heading)) + vx*cos(RAD(tmp_heading)) << " " << tv_msec << " " << tmp_msec << "\n";
	
}

void slam_refresh_pos(unsigned short tv_msec) {
	struct position approx_pos_acc; approx_pos_acc.x = 0; approx_pos_acc.y = 0;
	std::cout << "REFRESH: " << env_nw.size() << "\n";
	if (!(env_no.size() == 0 && env_nw.size() == 0 && env_sw.size() == 0 && env_so.size() == 0)) {
		approx_pos_acc.x = lroundf(slam_xr) + slam_pos.x;
		approx_pos_acc.y = lroundf(slam_yr) + slam_pos.y;
		std::cout << "SRP1\n";
		#if SLAM_CORR == 1
		std::cout << "SRP2\n";
		std::cout << "apa.x:" << approx_pos_acc.x << " apa.y:" << approx_pos_acc.y << "\n"; 
		
		int Nxd = (max(env_no_tmp.size() + env_so_tmp.size(),env_nw_tmp.size() + env_sw_tmp.size()))*2-1;
		int Nyd = (max_y_n_tmp() + max_y_s_tmp())*2-1;
		
		fftwf_complex *A, *C, *D;
		fftwf_plan p1, p2, p3;
		
		A = (fftwf_complex*) fftwf_malloc(sizeof(fftwf_complex) * (Nxd) * (Nyd));
		C = (fftwf_complex*) fftwf_malloc(sizeof(fftwf_complex) * (Nxd) * (Nyd));
		D = (fftwf_complex*) fftwf_malloc(sizeof(fftwf_complex) * (Nxd) * (Nyd));

		p1 = fftwf_plan_dft_2d(Nxd, Nyd, A, C, FFTW_FORWARD, FFTW_ESTIMATE);
		p2 = fftwf_plan_dft_2d(Nxd, Nyd, A, D, FFTW_FORWARD, FFTW_ESTIMATE);
		p3 = fftwf_plan_dft_2d(Nxd, Nyd, C, A, FFTW_BACKWARD, FFTW_ESTIMATE);	
		
		/*A mit temporärem Grid füllen und via Zero-Padding die Größe verdoppeln*/
		/*Dabei werden nur die Informationen "Hindernis vorhanden" (1) und "kein Hindernis vorhanden" (0) genutzt*/
		for (int i = 0; i < Nxd; i++) {
			if ((i > Nxd/2)) {
				for (int j = 0; j < Nyd; j++) {
					A[i*Nyd +j][0] = 0;
					A[i*Nyd +j][1] = 0;
				}
			} else {
				for (int j = Nyd/2; j < Nyd; j++) {
					A[i*Nyd +j][0] = 0;
					A[i*Nyd +j][1] = 0;
				}
				for (int j = 0; j < Nyd/2; j++) {
					unsigned char tmp = slam_get_tmp(i-(Nxd/4),j-(Nyd/4));
					if (tmp == ENV_ITEM_PATH) tmp = 0;
					else if (tmp == ENV_ITEM_APPROX) tmp = 1;
					else if (tmp == ENV_ITEM_SOLID) tmp = 1;
					else if (tmp == ENV_ITEM_UNSOLID) tmp = 0;
					A[i*Nyd +j][0] = tmp;
					A[i*Nyd +j][1] = 0;
				}
			}
		}
		
		//Einkommentieren um den Inhalt von A in der Datei "A1" zu speichern
		/*FILE *fd_ppm2 = fopen("A1", "w");
		for (int i = 0; i <Nxd; i++) {
			for (int j = 0; j < Nyd; j++) {
				fprintf(fd_ppm2,"%d ",(int)(A[i*Nyd+j][0]));
			}
			fprintf(fd_ppm2,"\n");
		}*/
		
		//FFT durchführen und in Matrix C speichern
		fftwf_execute(p1);
		
		/*A wie oben mit Originaldaten füllen*/
		for (int i = 0; i < Nxd; i++) {
			if ((i > Nxd/2)) {
				for (int j = 0; j < Nyd; j++) {
					A[i*Nyd +j][0] = 0;
					A[i*Nyd +j][1] = 0;
				}
			} else {
				for (int j = 0; j < Nyd/2; j++) {
					unsigned char tmp = slam_get(i+approx_pos_acc.x-(Nxd/4),j+approx_pos_acc.y-(Nyd/4));
					if (tmp == ENV_ITEM_PATH) tmp = 0;
					else if (tmp == ENV_ITEM_APPROX) tmp = 1;
					else if (tmp == ENV_ITEM_SOLID) tmp = 1;
					else if (tmp == ENV_ITEM_UNSOLID) tmp = 0;
					if (tmp == 1)
					//std::cout << "i: " << i << " j: " << j << " tmp:" << (int)tmp << "\n";
					A[i*Nyd +j][0] = tmp;
					A[i*Nyd +j][1] = 0;
				}
				for (int j = Nyd/2; j < Nyd; j++) {
					A[i*Nyd +j][0] = 0;
					A[i*Nyd +j][1] = 0;
				}
			}
		}
		
		//Einkommentieren um den Inhalt von A in der Datei "A1" zu speichern
		/*FILE *fd_ppm = fopen("A2", "w"); 
		for (int i = 0; i <Nxd; i++) {
			for (int j = 0; j < Nyd; j++) {
				fprintf(fd_ppm,"%d ",(int)(A[i*Nyd+j][0]));
			}
			fprintf(fd_ppm,"\n");
		}*/
		
		//FFT durchführen und in D speichern
		fftwf_execute(p2);
		
		//Punktweise Multiplikation der Spektren C und D (in-place)
		for (int i = 0; i < Nxd; i++) {
				for (int j = 0; j < Nyd; j++) {
					float tmp = C[i*(Nyd) + j][0]*D[i*(Nyd)+ j][0] + C[i*(Nyd) + j][1]*D[i*(Nyd)+ j][1];
					C[i*(Nyd)+j][1] = C[i*(Nyd) + j][1]*D[i*(Nyd)+ j][0] - C[i*(Nyd) + j][0]*D[i*(Nyd)+ j][1];
					C[i*(Nyd)+j][0] = tmp;
				}
		}
		
		fftwf_free(D);
		//IFFT durchführen und in A speichern
		fftwf_execute(p3);
		fftwf_free(C);
		
		float max = 0;
		struct position max_pos; max_pos.x = 0; max_pos.y = 0;
		
		/*Maximum in der Korrelationsmatrix finden und daraus Position errechnen*/
		for (int i = 0; i < Nxd; i++) {
			for (int j = 0; j < Nyd; j++) {
				//std::cout << i << " " << j << " " << Nxd << " " << Nyd << "\n";
				//std::cout << A[i*(Nyd)+j][0] << " ";
				if (A[i*(Nxd)+j][0] > max) {
					max = A[i*(Nyd)+j][0];
					if (i <= (Nxd/2 + 1))	max_pos.x = i;
					else 			max_pos.x = i - Nxd;
					if (j <= (Nyd/2 + 1))	max_pos.y = j;
					else 			max_pos.y = j - Nyd;
					//std::cout << "max_pos " << i << " " << j << "Nxd,Nyd " << Nxd << " " << Nyd << "\n";
				}
			}
			//std::cout << "\n";
		}
		
		approx_pos_acc.x -= max_pos.x; approx_pos_acc.y -= max_pos.y;
		
		std::cout << "apa.x:" << approx_pos_acc.x << " apa.y:" << approx_pos_acc.y << "\n";
		#endif
	}
	acc_tv_msec = tv_msec; slam_xv = 0; slam_yv = 0; slam_xr = 0; slam_yr = 0;
	//std::cout << "env_no_tmp.size() " << env_no_tmp.size() << "\n";
	for(unsigned int i = 0; i < env_no_tmp.size(); i++) {
		//std::cout << "env_no_tmp[" << i << "].size() " << env_no_tmp[i].size() << "\n";
		for(unsigned int j = 0; j < env_no_tmp[i].size(); j++) {
			if (env_no_tmp[i][j] != ENV_ITEM_UNDEF) {
				struct position tmp; tmp.x = i + approx_pos_acc.x; tmp.y = j + approx_pos_acc.y;
				slam_insert(tmp,env_no_tmp[i][j]);
				//std::cout << "NO:" << (int)env_no_tmp[i][j] << " written to (" << tmp.x << "," << tmp.y << "). \n";
			}
		}
		env_no_tmp[i].resize(0);
	}
	env_no_tmp.resize(0);
	//std::cout << "env_nw_tmp.size() " << env_nw_tmp.size() << "\n";
	for(unsigned int i = 0; i < env_nw_tmp.size(); i++) {
		//std::cout << "env_nw_tmp[" << i << "].size() " << env_nw_tmp[i].size() << "\n";
		for(unsigned int j = 0; j < env_nw_tmp[i].size(); j++) {
			if (env_nw_tmp[i][j] != ENV_ITEM_UNDEF) {
				struct position tmp; tmp.x = approx_pos_acc.x - i; tmp.y = j + approx_pos_acc.y;
				slam_insert(tmp,env_nw_tmp[i][j]);
				//std::cout << "NW:" << (int)env_nw_tmp[i][j] << " written to (" << tmp.x << "," << tmp.y << "). \n";
			}
		}
		env_nw_tmp[i].resize(0);
	}
	env_nw_tmp.resize(0);
	//std::cout << "env_so_tmp.size() " << env_so_tmp.size() << "\n";
	for(unsigned int i = 0; i < env_so_tmp.size(); i++) {
		//std::cout << "env_so_tmp[" << i << "].size() " << env_so_tmp[i].size() << "\n";
		for(unsigned int j = 0; j < env_so_tmp[i].size(); j++) {
			if (env_so_tmp[i][j] != ENV_ITEM_UNDEF) {
				struct position tmp; tmp.x = i + approx_pos_acc.x; tmp.y = approx_pos_acc.y - j;
				slam_insert(tmp,env_so_tmp[i][j]);
				//std::cout << "SO:" << (int)env_so_tmp[i][j] << " written to (" << tmp.x << "," << tmp.y << "). \n";
			}
		}
		env_so_tmp[i].resize(0);
	}
	env_so_tmp.resize(0);
	//std::cout << "env_sw_tmp.size() " << env_sw_tmp.size() << "\n";
	for(unsigned int i = 0; i < env_sw_tmp.size(); i++) {
		//std::cout << "env_sw_tmp[" << i << "].size() " << env_sw_tmp[i].size() << "\n";
		for(unsigned int j = 0; j < env_sw_tmp[i].size(); j++) {
			if (env_sw_tmp[i][j] != ENV_ITEM_UNDEF) {
				struct position tmp; tmp.x = approx_pos_acc.x - i; tmp.y = approx_pos_acc.y - j;
				slam_insert(tmp,env_sw_tmp[i][j]);
				//std::cout << "SW:" << (int)env_sw_tmp[i][j] << " written to (" << tmp.x << "," << tmp.y << "). \n";
			}
		}
		env_sw_tmp[i].resize(0);
	}
	env_sw_tmp.resize(0);
	
	draw_line(slam_pos,approx_pos_acc,ENV_ITEM_PATH,ENV_ITEM_PATH,ENV_ITEM_PATH);
	
	slam_pos.x = approx_pos_acc.x;
	slam_pos.y = approx_pos_acc.y;
}

void slam_draw_image(const char* file) {
	unsigned int max_w = max_y_s();
	unsigned int max_o = max_y_n();
	FILE *fd_ppm = fopen(file, "w"); fprintf(fd_ppm,"P3\n%d %d\n255\n", max_w+max_o, (max<unsigned int>(env_no.size(),env_nw.size())+max<unsigned int>(env_so.size(),env_sw.size())));
	//std::cout << (-1)*((int)max<unsigned int>(env_no.size(),env_nw.size())) << " " << (int)max<unsigned int>(env_so.size(),env_sw.size()) << "\n";
	//std::cout << (-1)*((int)max_w) << " " << (int)max_o << "\n";
	//std::cout << "draw_image\n";
	for (int i = (-1)*((int)max<unsigned int>(env_so.size(),env_sw.size())); i < (int)max<unsigned int>(env_no.size(),env_nw.size()); i++) {
		for (int j = (-1)*((int)max_w); j < (int)max_o; j++) {
			//std::cout << i << " " << j << " " << (int)get(i,j) << " ";
			switch (slam_get(i,j)) {
				case ENV_ITEM_APPROX: 	fprintf(fd_ppm,"%s ",COLOR_ITEM_APPROX);	break;
				case ENV_ITEM_PATH: 	fprintf(fd_ppm,"%s ",COLOR_ITEM_PATH);		break;
				case ENV_ITEM_UNDEF: 	fprintf(fd_ppm,"%s ",COLOR_ITEM_UNDEF);	break;
				case ENV_ITEM_SOLID: 	fprintf(fd_ppm,"%s ",COLOR_ITEM_SOLID);	break;
				case ENV_ITEM_UNSOLID: 	fprintf(fd_ppm,"%s ",COLOR_ITEM_UNSOLID);	break;
				default: 		fprintf(fd_ppm,"%s ",COLOR_ITEM_ERROR);	break;
			}
		}
		//std::cout << "\n";
		fprintf(fd_ppm,"\n");
	}
}

void slam_init(unsigned short heading, unsigned int tv_msec) {
	slam_pos.x = 0;
	slam_pos.y = 0;
	null_heading = heading;
	slam_xr = 0; slam_yr = 0;
	acc_tv_msec = tv_msec;
	old_tv_msec_acc = 0;
}