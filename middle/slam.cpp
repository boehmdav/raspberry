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

float slam_xacc, slam_yacc;
unsigned int old_tv_msec_acc;
unsigned int acc_tv_msec;

template <typename T>
T max(T a, T b) {
	if (a > b) return a;
	return b;
}

/*TODO Wertigkeiten der einzelnen Felder beachten (um Infos nicht zu überschreiben)?*/
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

unsigned char slam_get(int x, int y) {
	struct position pos;
	pos.x = x; pos.y = y;
	return slam_get(pos);
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
	//std::cout << null_heading << " " << heading << " " << tmp_heading << " " << sin(RAD(tmp_heading)) << "\n";
	tmp_pos.x = slam_pos.x + (measurement+COPTER_RADIUS)*sin(RAD(tmp_heading));
	tmp_pos.y = slam_pos.y + (measurement+COPTER_RADIUS)*cos(RAD(tmp_heading));
	draw_line_tmp(slam_pos,tmp_pos,ENV_ITEM_UNSOLID,ENV_ITEM_UNSOLID,ENV_ITEM_SOLID);
}

unsigned int max_y_o() {
	unsigned int ret_val = 0;
	for (unsigned int i = 0; i < env_no.size(); i++) {
		if (ret_val < env_no[i].size()) ret_val = env_no[i].size();
	}
	for (unsigned int i = 0; i < env_nw.size(); i++) {
		if (ret_val < env_nw[i].size()) ret_val = env_nw[i].size();
	}
	return ret_val;
}

unsigned int max_y_w() {
	unsigned int ret_val = 0;
	for (unsigned int i = 0; i < env_so.size(); i++) {
		if (ret_val < env_so[i].size()) ret_val = env_so[i].size();
	}
	for (unsigned int i = 0; i < env_sw.size(); i++) {
		if (ret_val < env_sw[i].size()) ret_val = env_sw[i].size();
	}
	return ret_val;
}

void slam_insert_acc(int xa, int ya, unsigned short heading, unsigned int tv_msec) {
	unsigned int tmp_msec = tv_msec - old_tv_msec_acc;
	old_tv_msec_acc = tv_msec;
	short tmp_heading = (heading - null_heading+360)%360;
	slam_xacc +=((float)ya*sin(RAD(tmp_heading)) + (float)xa*cos(RAD(tmp_heading)))*pow(((float)tmp_msec/1000),2);
	slam_yacc +=((float)xa*sin(RAD(tmp_heading)) + (float)ya*cos(RAD(tmp_heading)))*pow((float)tmp_msec/1000,2);
	std::cout << "xa:" << xa << " ya:" << ya << " h:" << heading << " tmp_h:" << tmp_heading << " xacc:" << slam_xacc << " yacc:" << slam_yacc << "\n";
}

void slam_refresh_pos(unsigned short tv_msec) {
	struct position approx_pos_acc;
	approx_pos_acc.x = lroundf(slam_xacc) + slam_pos.x;
	approx_pos_acc.y = lroundf(slam_yacc) + slam_pos.y;
	
	std::cout << "apa.x:" << approx_pos_acc.x << " apa.y:" << approx_pos_acc.y << "\n"; 
	
	acc_tv_msec = tv_msec; slam_xacc = 0; slam_yacc = 0;
	
	//approx_pos_acc via Daten in env_tmp anpassen

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
				struct position tmp; tmp.x = i + approx_pos_acc.x; tmp.y = j + approx_pos_acc.y;
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
				struct position tmp; tmp.x = i + approx_pos_acc.x; tmp.y = j + approx_pos_acc.y;
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
				struct position tmp; tmp.x = i + approx_pos_acc.x; tmp.y = j + approx_pos_acc.y;
				slam_insert(tmp,env_sw_tmp[i][j]);
				//std::cout << "SW:" << (int)env_sw_tmp[i][j] << " written to (" << tmp.x << "," << tmp.y << "). \n";
			}
		}
		env_sw_tmp[i].resize(0);
	}
	env_sw_tmp.resize(0);
	
	/*for(unsigned int i = 0; i < env_no.size(); i++) {
		for(unsigned int j = 0; j < env_no[i].size(); j++) {
			std::cout << (int)env_no[i][j] << " ";
		}
		std::cout << "\n";
	}
	std::cout << env_no.size() << "\n";
	for(unsigned int i = 0; i < env_no.size(); i++) {
		std::cout << env_no[i].size() << " ";
	}
	std::cout << "\n";*/
	
	draw_line(slam_pos,approx_pos_acc,ENV_ITEM_PATH,ENV_ITEM_PATH,ENV_ITEM_PATH);
	
	slam_pos.x = approx_pos_acc.x;
	slam_pos.y = approx_pos_acc.y;
	
	
}

void slam_draw_image(const char* file) {
	unsigned int max_w = max_y_w();
	unsigned int max_o = max_y_o();
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
	slam_xacc = 0; slam_yacc = 0;
	acc_tv_msec = tv_msec;
	old_tv_msec_acc = 0;
}
