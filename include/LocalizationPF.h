
#pragma once

#include "ParticleFilter.h"
#include "leica.h"
#include "include/myfun.h"
#include "include/mycv.h"
#include "include/LocalizationPF.h"
#include "include/Lidar2d.h"
#include "include/OmniCamera.h"
#include "include/GPS.h"

#include <direct.h>
#include <signal.h>


/**********************************************************/
//	PFによる位置推定用のクラス
/**********************************************************/
//	遷移モデルのみ実装
//	計測モデルは各プログラムごとに実装
class LocalizationPF :public ParticleFilter<Position<>>
{
public:

	/**********************************************************/
	//  パラメータのファイル出力
	/**********************************************************/
	void outputParameter(std::string ofpath) {
		std::string filename = "C://Users/Robot/Documents/Visual Studio 2013/Projects/Localization.ver2/include/parameter.h";
		//std::string filename = "C://Users/robot/Desktop/Localization.ver2_2/Localization.ver2/include/parameter.h";
		//std::string filename = "C://Users/Nozomu Ohashi/Documents/Visual Studio 2013/Projects/Localization/Localization.ver2/include/parameter.h";
		std::ifstream ifs(filename);
		if (ifs.fail()) {
			readError(filename);
		}
		std::ofstream ofs1(ofpath + "parameter.h");
		std::string str;
		while (std::getline(ifs, str)) {
			ofs1 << str << std::endl;
		}
	}

	/*  コンストラクタ  */
	LocalizationPF() :exist_true_position(EXIST_TRUE_POSITION)
	{
		trial_type = TRIAL_TYPE;
		switch (trial_type)
		{
		case TRIAL_SIMULTANEOUS:
		case TRIAL_PEARSON:
		case TRIAL_PEARSON_NONSTAT:
		case TRIAL_NON_TIMESEQUENCE:
		case TRIAL_NON_TIMESEQUENCE_SIMUL:
		case TRIAL_SUYAMA_STAT:
		case TRIAL_SUYAMA_NONSTAT:
			sensor_num = 4;
			break;
		case TRIAL_3SENSORS_SIMULATNEOUS:
		case TRIAL_3SENSORS_PEARSON:
		case TRIAL_3SENSORS_PEARSON_NONSTAT:
		case TRIAL_3SENSORS_SUYAMA_NONSTAT:
		case TRIAL_3SENSORS_LRF_GPS:
			sensor_num = 3;
			break;
		default:
			std::cout << "NO TRIAL_TYPE: " << trial_type << ", " << __FILE__ << " LINE: " << __LINE__ << std::endl;
			exit(0);
			break;
		}

		localization_only_true_position = LOCALIZATION_ONLY_TRUE_POSITION;


		lid2_l.resize(LIKELIHOOD_THREAD);
		lid2_u.resize(LIKELIHOOD_THREAD);
		omni.resize(LIKELIHOOD_THREAD);
		gpgga.resize(LIKELIHOOD_THREAD);

		

		///* thread確保 */
		//lid2_l.resize(LIKELIHOOD_THREAD);
		//lid2_u.resize(LIKELIHOOD_THREAD);
		//omni.resize(LIKELIHOOD_THREAD);
		//gpgga.resize(LIKELIHOOD_THREAD);
		likelihood_threads.resize(LIKELIHOOD_THREAD);

		/*  自己位置推定の初期位置  */
		//	GL座標系からLC座標系に変換
		gl2lc.setOriginal(MAP_ORG_LAT, MAP_ORG_LON, MAP_ORG_ELE, MAP_ORG_HEAD);
		//ini_position = gl2lc.getPosition(INI_POS_LAT, INI_POS_LON, INI_POS_ELE, INI_POS_HEAD);


		/*  パーティクルフィルタのパラメータ  */
		sample_size = SAMPLE_SIZE;
		ini_sample_radius.set(INI_SAMPLE_RADIUS_X, INI_SAMPLE_RADIUS_Y, INI_SAMPLE_RADIUS_R);
		trans_par_sys_var.set(TRANS_PAR_SYS_VAR_X, TRANS_PAR_SYS_VAR_Y, TRANS_PAR_SYS_VAR_R);

		/*  オドメトリ誤差分散  */
		//odometry_system_noise.set(ODOMETRY_SYSTEM_NOISE_R, ODOMETRY_SYSTEM_NOISE_THETA);

		/*  オドメトリデータの初期値  */
		odometry1 = new Position<>(0.0, 0.0, 0.0);






	};

	/*  デストラクタ  */
	//	ポインタメンバ変数の領域解放"delete"を忘れずに
	~LocalizationPF()
	{
		delete odometry1, odometry2;
		delete true_position;
		map_img.release();
		map_img_color.release();
		map_img_clone.release();

		lid2_l.clear();
		lid2_u.clear();
		omni.clear();
		gpgga.clear();

		lid2_l.shrink_to_fit();
		lid2_u.shrink_to_fit();
		omni.shrink_to_fit();
		gpgga.shrink_to_fit();

		particle_video.release();
		particle_large_video.release();
		weighted_stat_particle_video.release();
		measurement_data_video.release();

		allClear();

	};

	void allClear() {

		estimated_position.clear();		//	重み付き平均による推定位置
		result_time.clear();
		all_particles.clear();
		all_particles_after_resampling.clear();
		all_stat_particles.clear();
		all_lid2_l_likelihood.clear();
		all_lid2_u_likelihood.clear();
		all_omni_likelihood.clear();
		all_gpgga_likelihood.clear();
		all_fusion_likelihood.clear();
		all_stat_lid2_l_likelihood.clear();
		all_stat_lid2_u_likelihood.clear();
		all_stat_omni_likelihood.clear();
		all_stat_gpgga_likelihood.clear();
		all_omni_img_sim_pos.clear();
		all_omni_img_sim.clear();
		all_stock_tidx.clear();
		diff_time_ini_now.clear();
		all_error.clear();
		error_time.clear();
		all_omni_img_sim.clear();
		all_omni_img_sim_pos.clear();
		all_meas_gps_pos.clear();
		all_th.clear();

		estimated_position.shrink_to_fit();		//	重み付き平均による推定位置
		result_time.shrink_to_fit();
		all_particles.shrink_to_fit();
		all_particles_after_resampling.shrink_to_fit();
		all_stat_particles.shrink_to_fit();
		all_lid2_l_likelihood.shrink_to_fit();
		all_lid2_u_likelihood.shrink_to_fit();
		all_omni_likelihood.shrink_to_fit();
		all_gpgga_likelihood.shrink_to_fit();
		all_fusion_likelihood.shrink_to_fit();
		all_stat_lid2_l_likelihood.shrink_to_fit();
		all_stat_lid2_u_likelihood.shrink_to_fit();
		all_stat_omni_likelihood.shrink_to_fit();
		all_stat_gpgga_likelihood.shrink_to_fit();
		all_omni_img_sim_pos.shrink_to_fit();
		all_omni_img_sim.shrink_to_fit();
		all_stock_tidx.shrink_to_fit();
		diff_time_ini_now.shrink_to_fit();
		all_error.shrink_to_fit();
		error_time.shrink_to_fit();
		all_omni_img_sim.shrink_to_fit();
		all_omni_img_sim_pos.shrink_to_fit();
		all_meas_gps_pos.shrink_to_fit();
		all_th.shrink_to_fit();

		esti_time = ini_time;
		read_meas_step = ini_step;
		tidx = ini_tidx;
		no = ini_no;
		finish = false;
		fin_movie_creator_ = false;
		read_all_measurement_ = false;
		now_step = 0;
		movie_step = 0;

		*odometry1 = Position<>(0.0, 0.0, 0.0);
		stat_particles.clear();
		stat_lid2_l_likelihood.clear();
		stat_lid2_u_likelihood.clear();
		stat_omni_likelihood.clear();
		stat_gpgga_likelihood.clear();
		lid2_l_likelihood.clear();
		lid2_u_likelihood.clear();
		omni_likelihood.clear();
		gpgga_likelihood.clear();
		fusion_likelihood.clear();

		particle_video.release();
		particle_large_video.release();
		weighted_stat_particle_video.release();
		measurement_data_video.release();
		add_measurement_movie_step = 0;
		fin_add_measurement_movie_ = false;

		use_.clear();							// 	各ステップで選択されたセンサ
		similarity_table.clear();	//	センサから得られる確率分布間の類似度
		similar_table_.clear();		//	センサから得られる確率分布間を類似・非類似で閾値処理

		all_meas_lrf_l.clear();
		all_meas_lrf_u.clear();
		all_meas_keypoints.clear();
		all_meas_desriptor.clear();
		all_meas_gps.clear();
		all_meas_odometry.clear();
		all_meas_time.clear();


		error_time.clear();
	}

	void initOutput(std::string ofpath)
	{
		/* Make Folder */
		_mkdir(ofpath.c_str());
		std::string ofpath_data = ofpath + "Data/";
		_mkdir(ofpath_data.c_str());
		std::string ofpath_data_gitignore = ofpath + "Data/gitignore/";
		_mkdir(ofpath_data_gitignore.c_str());
		std::string ofpath_image = ofpath + "image/";
		_mkdir(ofpath_image.c_str());
		std::string ofpath_movie = ofpath + "movie/";
		_mkdir(ofpath_movie.c_str());

		///*  推定位置出力ファイルを初期化  */
		//initOutputEstiPosition(ofpath);

		///*  推定誤差の出力ファイルを初期化  */
		//initOutputEstiError(ofpath);

		//initEliminateSensorForAnalysis();

		outputParameter(ofpath);

		initLastInpugFileNumber(ofpath);

		/* 計測動画初期化 */
		initMeasurementDataVideo(ofpath);

		/*  パーティクル動画の初期化  */
		initParticleVideo(ofpath);
		initParticleLargeVideo(ofpath);

		initWeightedStatParVideo(ofpath);

		//initUseSensor();
		//initEliminateSensorForAnalysis();
		//initUseType();
		//initSimilarityTable();
		//initSimilarTable_();
	}

	/**********************************************************/
	//  ファイルの読み込み
	/**********************************************************/

	void initialize() {

		//*odometry1 = Position<>(0.0, 0.0, 0.0);

		//for (auto& tmp : lid2_l)
		//{
		//	tmp.clearMeasurement();
		//}
		//for (auto& tmp : lid2_u)
		//{
		//	tmp.clearMeasurement();
		//}
		//for (auto& tmp : omni)
		//{
		//	tmp.clearMeasurement();
		//}
		//for (auto& tmp : gpgga)
		//{
		//	tmp.clearMeasurement();
		//}
		stat_particles.clear();
		stat_lid2_l_likelihood.clear();
		stat_lid2_u_likelihood.clear();
		stat_omni_likelihood.clear();
		stat_gpgga_likelihood.clear();
		lid2_l_likelihood.clear();
		lid2_u_likelihood.clear();
		omni_likelihood.clear();
		gpgga_likelihood.clear();
		fusion_likelihood.clear();

	}


public:

	void readGridMap(std::string filename)
	{
		/*  障害物地図の読み込み  */
		map_img = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
		map_img_color = cv::imread(filename, CV_LOAD_IMAGE_COLOR);	//	障害物地図をカラーで保存
		if (map_img.empty() || map_img_color.empty())	readError(filename);
	}

	/*  障害物地図の読み込み  */
	void readGridMapEach(std::string ifpath)
	{
		/*  障害物地図の読み込み  */
		std::string filename;
		switch (trial_type)
		{
		case TRIAL_SIMULTANEOUS:
		case TRIAL_PEARSON:
		case TRIAL_PEARSON_NONSTAT:
		case TRIAL_NON_TIMESEQUENCE:
		case TRIAL_NON_TIMESEQUENCE_SIMUL:
		case TRIAL_SUYAMA_STAT:
		case TRIAL_SUYAMA_NONSTAT:
			filename = ifpath + "lrf_lower/gridmap.bmp";
			map_img_lower = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
			if (map_img_lower.empty()) {
				readError(filename);
			}
			filename = ifpath + "lrf_upper/gridmap.bmp";
			map_img_upper = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
			if (map_img_upper.empty()) {
				readError(filename);
			}
			break;
		case TRIAL_3SENSORS_SIMULATNEOUS:
		case TRIAL_3SENSORS_PEARSON:
		case TRIAL_3SENSORS_PEARSON_NONSTAT:
		case TRIAL_3SENSORS_SUYAMA_NONSTAT:
		case TRIAL_3SENSORS_LRF_GPS:
			filename = ifpath + "lrf_lower/gridmap.bmp";
			map_img_lower = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
			if (map_img_lower.empty()) {
				readError(filename);
			}
			break;
		default:
			std::cout << "NO TRIAL_TYPE: " << trial_type << ", " << __FILE__ << " LINE: " << __LINE__ << std::endl;
			exit(0);
			break;
		}
	}

	/*  オドメトリデータの読み込み  */
	bool readOdometry(int no, const int step)
	{
		/*  領域の確保  */
		if (!odometry2)	odometry2 = new Position<>;

		/*  オドメトリデータの読み込み  */
		std::string filename = IFPATH_MEAS + "odometry/odometry_no" + std::to_string(no) + "_" + std::to_string(step) + "th.csv";
		std::ifstream ifs_odo(filename);
		if (ifs_odo.fail())	return false;	//	読み込みに失敗した場合，falseを返す
		ifs_odo >> *odometry2;

		odometry2->r *= M_PI / 1800.0;

		return true;
	}


	/*  計測時刻の読み込み  */
	bool readTime(int no, const int step)
	{
		std::string filename = IFPATH_MEAS + "time/time_no" + std::to_string(no) + "_" + std::to_string(step) + "th.csv";
		std::ifstream ifs(filename);
		if (ifs.fail())	return false;

		MyTime tmp;
		ifs >> tmp;

		esti_time = tmp;
		return true;
	}



	/*  真の位置を読み込み  */
	void readTruePosition(int no)
	{
		/* leica 読み込み */
		gl2lc.setOriginal(MAP_ORG_LAT, MAP_ORG_LON, MAP_ORG_ELE, MAP_ORG_HEAD);
		Coor<> leica_coor = gl2lc.getCoor(LEICA_ORG_LAT, LEICA_ORG_LON, LEICA_ORG_ELE);
		leica::Param<> param_leica(leica_coor);
		param_leica.setHorizontalErrorDeg(LEICA_HORIZONTAL_ERROR);
		std::vector<leica::Data<>> dataset_leica;
		{
			std::string filename = IFPATH_MEAS + "true_position/true_position.csv";
			std::ifstream ifs(filename);
			if (ifs.fail())	readError(filename);
			leica::readDeg(ifs, param_leica, dataset_leica);
		}

		/*  領域の確保  */
		if (!true_position)	true_position = new std::vector<Position<>>;

		for (int i = 0; i < dataset_leica.size(); i++) {
			int tmp = i + 1;
			while (tmp < dataset_leica.size() && leica::distance(dataset_leica[i], dataset_leica[tmp]) < 3000) {
				tmp++;
			}
			if (tmp >= dataset_leica.size()) {
				tmp = i;
			}
			Position<> pos = dataset_leica[i].position(dataset_leica[tmp]);
			if (tmp >= dataset_leica.size() - 1) {
				pos.r = true_position->back().r;
			}
			true_position->push_back(pos);
			true_time.push_back(dataset_leica[i].time);
		}

		/* 初期位置がmap内になるようにする */
		while (true) {
			int tmp = tidx + 1;
			while (leica::distance(dataset_leica[tidx], dataset_leica[tmp]) < 3000) {
				tmp++;
			}
			ini_position = dataset_leica[tidx].position(dataset_leica[tmp]);
			cv::Point pixel = ToPixel(ini_position, map_img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			if (pixel.x < 0 || map_img.cols < pixel.x ||
				pixel.y < 0 || map_img.rows < pixel.y) {
				std::cout << "Initial Position is out of Map!" << std::endl;
				exit(0);
			}
			if (!onObject_(ini_position)) {
				break;
			}
			tidx++;
		}
		/* 計測ステップも進める */
		while (true) {
			std::cout << read_meas_step << std::endl;
			readTime(no, read_meas_step + 1);

			if (true_time[tidx] < esti_time) {
				read_meas_step++;
				bool o_ = LocalizationPF::readOdometry(read_meas_no, read_meas_step);
				assert(o_ == true);
				*odometry1 = *odometry2;

				break;
			}
			read_meas_step++;
		}

		while (esti_time > true_time[tidx]) {
			tidx++;
		}

		int idx;
		if (tidx<dataset_leica.size()-1){
			idx = tidx + 1;
		}
		else{
			idx = tidx;
		}
		ini_position = dataset_leica[tidx].position(dataset_leica[idx]);


		ini_time = esti_time;
		ini_step = read_meas_step;
		ini_tidx = tidx;
		ini_no = no;

		std::cout << "Skip steps: 1-" << read_meas_step + 1 << std::endl;
		std::cout << "Initial position: " << ini_position << std::endl;
	}

	Coor<> getMapClickedCoor(std::string window_name, const cv::Mat& map){
		float scale = VISUALIZE_MAP_SCALE;
		cv::Mat mat;
		cv::resize(map, mat, cv::Size(), scale, scale);
		cv::flip(mat, mat, 0);
		cv::imshow(window_name, mat);
		int map_res = MAP_RES / scale;
		MouseEventImage mei;
		mei.setMouseEvent(window_name);
		mei.waitLeftButton();
		cv::Point p(mei.getX(), mei.getY());
		p.y = mat.rows - p.y;
		Coor<> pos = ToPosition(p, mat.cols, mat.rows, map_res, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		return pos;
	}

	void readEnvironment() {

		std::string filename;
		switch (trial_type)
		{
		case TRIAL_SIMULTANEOUS:
		case TRIAL_PEARSON:
		case TRIAL_PEARSON_NONSTAT:
		case TRIAL_NON_TIMESEQUENCE:
		case TRIAL_NON_TIMESEQUENCE_SIMUL:
		case TRIAL_SUYAMA_STAT:
		case TRIAL_SUYAMA_NONSTAT:

			/*  障害物地図を読み込み  */
			switch (LOCALIZATION_AREA)
			{
			case LOCALIZATION_PARKING:
				LocalizationPF::readGridMap(IFPATH_ENV + "lrf_upper/gridmap.bmp");
				break;
			case LOCALIZATION_SQUARE:
			case LOCALIZATION_B2:
			case LOCALIZATION_AROUND_8GO:
				LocalizationPF::readGridMap(IFPATH_ENV + "lrf_lower/gridmap.bmp");
				break;
			default:
				std::cout << "LOCALIZATION_AREA: " << LOCALIZATION_AREA << ", " << __FILE__ << " LINE: " << __LINE__ << std::endl;
				exit(0);
				break;
			}

			LocalizationPF::readGridMapEach(IFPATH_ENV);

			/*  真の位置情報の読み込み  */
			if (exist_true_position){
				LocalizationPF::readTruePosition(no);
			}
			else{
				while (true){
					// 真の位置がない場合、初期位置を指定
					std::string window_name = "map";
					cv::namedWindow(window_name);
					cv::Mat mat = map_img_color.clone();
					std::cout << "clic left button for initial: ";
					Coor<> coor1 = getMapClickedCoor(window_name, mat);
					std::cout << coor1 << std::endl;
					// クリックポイントの描画
					cv::Point p1_check = ToPixel(coor1, mat, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
					cv::circle(mat, p1_check, IMAGE_TRUEPOSITION_RADIUS, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS, 8, 0);
					std::cout << "clic left button for second point: ";
					Coor<> coor2 = getMapClickedCoor(window_name, mat);
					std::cout << coor2 << std::endl;

					/* 初期姿勢の計算・出力 */
					mat = map_img_color.clone();
					ini_position = Position<>(coor1.x, coor1.y, std::atan2(coor2.y - coor1.y, coor2.x - coor1.x));
					cv::Point ini_p = ToPixel(ini_position, mat, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
					drawPosition(ini_position, mat, cv::Scalar(0, 255, 0), mat.cols, mat.rows, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y, cv::Point(0, 0), TRUE_POSITION);
					cv::resize(mat, mat, cv::Size(0, 0), VISUALIZE_MAP_SCALE, VISUALIZE_MAP_SCALE);
					cv::flip(mat, mat, 0);
					cv::imshow(window_name, mat);
					cv::waitKey(20);
					std::cout << "Are you sure(y/n)?: ";
					char c;
					std::cin >> c;
					if (c == 'y')
						break;
				}

				//ini_position = pos1;
			}


			/*  センサデータの読み込み  */

			filename = IFPATH_ENV + "map.csv";

			lid2_l[0].readPointCloud(IFPATH_ENV + "lrf_lower/pointcloud.csv");
			lid2_l[0].setEnvICP();
			lid2_u[0].readPointCloud(IFPATH_ENV + "lrf_upper/pointcloud.csv");
			lid2_u[0].setEnvICP();
			omni[0].readImg(IFPATH + "Environment/img.bmp");

			//omni[0].readEnvImgAvi(IFPATH_ENV_OMNI + "omni/img.avi");
			switch (OMNI_FEATURE)
			{
			case OMNI_FEATURE_SIFT:
				omni[0].readEnvKeypoint(IFPATH_ENV_OMNI + "omni/sift/");
				omni[0].readEnvDescriptor(IFPATH_ENV_OMNI + "omni/sift/");
					omni[0].readEnvImgPosition(IFPATH_ENV_OMNI + "omni/sift/img_pos.csv");
				assert(omni[0].env_img_position.size() == omni[0].env_keypoints.size());
				assert(omni[0].env_img_position.size() == omni[0].env_descriptors.size());
				std::cout << omni[0].env_img_position.size() << std::endl;
				std::cout << omni[0].env_keypoints.size() << std::endl;
				if (USE_BOF){
					omni[0].readEnvCentroid(IFPATH_ENV_OMNI + "omni/sift/centroid.csv");
					omni[0].readEnvHistgram(IFPATH_ENV_OMNI + "omni/sift/histogram.csv");
				}
				break;
			case OMNI_FEATURE_SURF:
				omni[0].readEnvKeypoint(IFPATH_ENV_OMNI + "omni/surf/");
				omni[0].readEnvDescriptor(IFPATH_ENV_OMNI + "omni/surf/");
					omni[0].readEnvImgPosition(IFPATH_ENV_OMNI + "omni/surf/img_pos.csv");
				if (USE_BOF){
					omni[0].readEnvCentroid(IFPATH_ENV_OMNI + "omni/surf/centroid.csv");
					omni[0].readEnvHistgram(IFPATH_ENV_OMNI + "omni/surf/histgram.csv");
				}
				break;
			default:
				std::cout << "OMNI_FEATURE: " << OMNI_FEATURE << std::endl;
				break;
			}

			//omni_tmp[th].readEnvDescriptor(ENVIRONMENT_DATE_OMNI + "omni/surf/");
			gpgga[0].setMapOrgGL(MAP_ORG_LAT, MAP_ORG_LON, MAP_ORG_ELE);


			break;
		case TRIAL_3SENSORS_SIMULATNEOUS:
		case TRIAL_3SENSORS_PEARSON:
		case TRIAL_3SENSORS_PEARSON_NONSTAT:
		case TRIAL_3SENSORS_SUYAMA_NONSTAT:
		case TRIAL_3SENSORS_LRF_GPS:

			LocalizationPF::readGridMap(IFPATH_ENV + "lrf_lower/gridmap.bmp");
			LocalizationPF::readGridMapEach(IFPATH_ENV);

			/*  真の位置情報の読み込み  */
			if (exist_true_position){
				LocalizationPF::readTruePosition(no);
			}
			else{
				while (true){
					// 真の位置がない場合、初期位置を指定
					std::string window_name = "map";
					cv::namedWindow(window_name);
					cv::Mat mat = map_img_color.clone();
					std::cout << "clic left button for initial: ";
					Coor<> coor1 = getMapClickedCoor(window_name, mat);
					std::cout << coor1 << std::endl;
					// クリックポイントの描画
					cv::Point p1_check = ToPixel(coor1, mat, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
					cv::circle(mat, p1_check, IMAGE_TRUEPOSITION_RADIUS, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS, 8, 0);
					std::cout << "clic left button for second point: ";
					Coor<> coor2 = getMapClickedCoor(window_name, mat);
					std::cout << coor2 << std::endl;

					/* 初期姿勢の計算・出力 */
					mat = map_img_color.clone();
					ini_position = Position<>(coor1.x, coor1.y, std::atan2(coor2.y - coor1.y, coor2.x - coor1.x));
					cv::Point ini_p = ToPixel(ini_position, mat, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
					drawPosition(ini_position, mat, cv::Scalar(0, 255, 0), mat.cols, mat.rows, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y, cv::Point(0, 0), TRUE_POSITION);
					cv::resize(mat, mat, cv::Size(0, 0), VISUALIZE_MAP_SCALE, VISUALIZE_MAP_SCALE);
					cv::flip(mat, mat, 0);
					cv::imshow(window_name, mat);
					cv::waitKey(20);
					std::cout << "Are you sure(y/n)?: ";
					char c;
					std::cin >> c;
					if (c == 'y')
						break;
				}

				//ini_position = pos1;
			}


			/*  センサデータの読み込み  */

			filename = IFPATH_ENV + "map.csv";

			lid2_l[0].readPointCloud(IFPATH_ENV + "lrf_lower/pointcloud.csv");
			lid2_l[0].setEnvICP();
			omni[0].readImg(IFPATH + "Environment/img.bmp");

			//omni[0].readEnvImgAvi(IFPATH_ENV_OMNI + "omni/img.avi");
			switch (OMNI_FEATURE)
			{
			case OMNI_FEATURE_SIFT:
				omni[0].readEnvKeypoint(IFPATH_ENV_OMNI + "omni/sift/");
				omni[0].readEnvDescriptor(IFPATH_ENV_OMNI + "omni/sift/");
				omni[0].readEnvImgPosition(IFPATH_ENV_OMNI + "omni/sift/img_pos.csv");
				assert(omni[0].env_img_position.size() == omni[0].env_keypoints.size());
				assert(omni[0].env_img_position.size() == omni[0].env_descriptors.size());
				std::cout << omni[0].env_img_position.size() << std::endl;
				std::cout << omni[0].env_keypoints.size() << std::endl;
				if (USE_BOF){
					omni[0].readEnvCentroid(IFPATH_ENV_OMNI + "omni/sift/centroid.csv");
					omni[0].readEnvHistgram(IFPATH_ENV_OMNI + "omni/sift/histogram.csv");
				}
				break;
			case OMNI_FEATURE_SURF:
				omni[0].readEnvKeypoint(IFPATH_ENV_OMNI + "omni/surf/");
				omni[0].readEnvDescriptor(IFPATH_ENV_OMNI + "omni/surf/");
				omni[0].readEnvImgPosition(IFPATH_ENV_OMNI + "omni/surf/img_pos.csv");
				if (USE_BOF){
					omni[0].readEnvCentroid(IFPATH_ENV_OMNI + "omni/surf/centroid.csv");
					omni[0].readEnvHistgram(IFPATH_ENV_OMNI + "omni/surf/histgram.csv");
				}
				break;
			default:
				std::cout << "OMNI_FEATURE: " << OMNI_FEATURE << std::endl;
				break;
			}

			//omni_tmp[th].readEnvDescriptor(ENVIRONMENT_DATE_OMNI + "omni/surf/");
			gpgga[0].setMapOrgGL(MAP_ORG_LAT, MAP_ORG_LON, MAP_ORG_ELE);

			break;
		default:
			std::cout << "NO TRIAL_TYPE: " << trial_type << ", " << __FILE__ << " LINE: " << __LINE__ << std::endl;
			exit(0);
			break;
		}



	}


	void setEnvironmentDetach() {

		//omni[0].setEnvImgFromAvi();
		//omni[0].setEnvDescriptor(OMNI_FEATURE);
		//omni[0].clearEnvImg();
		//omni.front().setEnvDescriptor(ENV_DESCRITOR_THREAD);

		GlobalToLocal& gl2lc_tmp = gl2lc;

		//for (int th = 0; th < LIKELIHOOD_THREAD; th++)
		//{
		//	Lidar2d& lid2_l_tmp = lid2_l[th];
		//	Lidar2d& lid2_u_tmp = lid2_u[th];
		//	OmniCamera& omni_tmp = omni[th];
		//	Gps& gpgga_tmp = gpgga[th];
		//	likelihood_threads[th] = std::thread([th, &lid2_l_tmp, &lid2_u_tmp, &omni_tmp, &gpgga_tmp, gl2lc_tmp] {
		//		lid2_l_tmp = lid2_l_tmp.front();
		//		lid2_l_tmp.setEnvICP();
		//		lid2_u_tmp = lid2_u_tmp.front();
		//		lid2_u_tmp.setEnvICP();
		//		omni_tmp = omni_tmp.front();
		//		gpgga_tmp = gpgga_tmp.front();
		//	});
		//}

		for (int th = 1; th < LIKELIHOOD_THREAD; th++)
		{
			lid2_l[th] = lid2_l[0];
			lid2_l[th].setEnvICP();
			switch (trial_type)
			{
			case TRIAL_SIMULTANEOUS:
			case TRIAL_PEARSON:
			case TRIAL_PEARSON_NONSTAT:
			case TRIAL_NON_TIMESEQUENCE:
			case TRIAL_NON_TIMESEQUENCE_SIMUL:
			case TRIAL_SUYAMA_STAT:
			case TRIAL_SUYAMA_NONSTAT:
				lid2_u[th] = lid2_u[0];
				lid2_u[th].setEnvICP();
				break;
			case TRIAL_3SENSORS_SIMULATNEOUS:
			case TRIAL_3SENSORS_PEARSON:
			case TRIAL_3SENSORS_PEARSON_NONSTAT:
			case TRIAL_3SENSORS_SUYAMA_NONSTAT:
			case TRIAL_3SENSORS_LRF_GPS:
				break;
			default:
				std::cout << "NO TRIAL_TYPE: " << trial_type << ", " << __FILE__ << " LINE: " << __LINE__ << std::endl;
				break;
			}
			omni[th] = omni[0];
			gpgga[th] = gpgga[0];
		}

		fin_read_env = true;
		std::cout << "Complete Environment" << std::endl;

	}
	void setEnvironment() {

		/* 読み込み */
		readEnvironment();

		set_environment_thread = std::thread(&LocalizationPF::setEnvironmentDetach, this);

		set_environment_thread.detach();


	}

	/* 信頼度マップ */
	void setReliabilityMap() {
		{
			std::string filename = IFPATH_RMAP + "reliability_map_gps.csv";
			std::ifstream ifs(filename);
			if (ifs.fail()) {
				readError(filename);
			}
			ifs >> cvReadData<float>(reliabity_map_gps);
		}
		{
			std::string filename = IFPATH_RMAP + "reliability_map_omni.csv";
			std::ifstream ifs(filename);
			if (ifs.fail()) {
				readError(filename);
			}
			ifs >> cvReadData<float>(reliabity_map_omni);
		}
		{
			std::string filename = IFPATH_RMAP + "reliability_map_lrf_l.csv";
			std::ifstream ifs(filename);
			if (ifs.fail()) {
				readError(filename);
			}
			ifs >> cvReadData<float>(reliabity_map_lrf_l);
		}
		{
			std::string filename = IFPATH_RMAP + "reliability_map_lrf_u.csv";
			std::ifstream ifs(filename);
			if (ifs.fail()) {
				readError(filename);
			}
			ifs >> cvReadData<float>(reliabity_map_lrf_u);
		}
	}


	/**********************************************************/
	//  途中スタート用
	/**********************************************************/

	/*  パーティクルの読み込み  */
	void readParticle(std::string ofpath)
	{
		/*  読み込み  */
		std::string filename = ofpath + "Data/particle_state_last.csv";
		std::ifstream ifs_par(filename);
		if (ifs_par.fail()) readError(filename);

		std::string str;
		while (std::getline(ifs_par, str))
		{
			std::istringstream istr(str);

			Particle<Position<>>* par = new Particle<Position<>>;
			Position<>* position = new Position<>;

			istr >> *position;
			par->setState(position);
			getParticles().push_back(par);
		}

	}

	/*  動画の読み込み  */
	void readParticleVideo()
	{
		//std::string filename = OFPATH + "Movie/particle.avi";
		//
		//cv::VideoCapture vcr(filename);
		//if (!vcr.isOpened()) readError(filename);


		///*  ビデオ関係  */
		//// ファイルをオープンし，ビデオライタを初期化
		//// filename - 出力ファイル名
		//// fourcc - コーデック
		//// fps - 1 秒あたりのフレーム数
		//// frameSize - ビデオフレームのサイズ
		//// isColor - ビデオストリームがカラーか，グレースケールかを指定
		//particle_video.open(filename, CV_FOURCC('X', 'V', 'I', 'D'), PARTICLE_FPS, cv::Size(1500, 1500));
		//if (!particle_video.isOpened())	writeError(filename);

		//while (1)
		//{
		//	cv::Mat mat;

		//	vcr >> mat;

		//	if (mat.empty()) break;

		//	particle_video << mat;

		//}
	}

	/*  推定位置の読み込み  */
	void readEstPos(std::string ofpath)
	{
		/*  読み込み  */
		std::string filename = ofpath + "Data/estimated_position.csv";
		std::ifstream ifs(filename);
		if (ifs.fail()) readError(filename);

		/*  1行スキップ  */
		std::string str;
		std::getline(ifs, str);

		ifs >> estimated_position;
	}


	/*  オドメトリデータの更新  */
	void swapOdometry()
	{
		*odometry1 = *odometry2;
	}

	/*  読み込みクラス  */
	/**********************************************************/
	//	出力ファイルの初期化

	/**********************************************************/

	/*  推定位置の出力ファイルを初期化  */
	void initOutputEstiPosition(std::string OFPATH)
	{
		{
			std::string filename = OFPATH + "Data/estimated_position.csv";
			std::ofstream ofs_est(filename, std::ios_base::out);
			if (ofs_est.fail())	writeError(filename);
			ofs_est << "time,s,x,y,rad" << std::endl;
			ofs_est.close();
		}
		//{
		//	std::string filename = "./output/Data/estimated_position.csv";
		//	std::ofstream ofs_est(filename, std::ios_base::out);
		//	if (ofs_est.fail())	writeError(filename);
		//	ofs_est << "time,x,y,rad" << std::endl;
		//	ofs_est.close();
		//}
	}

	/*  推定誤差の出力ファイルを初期化  */
	void initOutputEstiError(std::string OFPATH)
	{
		{
			std::string filename = OFPATH + "Data/error.csv";
			std::ofstream ofs_err(filename, std::ios_base::out);
			if (ofs_err.fail()) writeError(filename);
			ofs_err << "time,s,x,y,rad,abs" << std::endl;
			ofs_err.close();
		}
		//{
		//	std::string filename = "./output/Data/error.csv";
		//	std::ofstream ofs_err(filename, std::ios_base::out);
		//	if (ofs_err.fail()) writeError(filename);
		//	ofs_err << "time,x,y,rad" << std::endl;
		//	ofs_err.close();
		//}

	}

	void initLastInpugFileNumber(std::string OFPATH) {

		std::string filename = OFPATH + "Data/last_input_file_number.csv";
		std::ofstream ofs(filename, std::ios_base::out);
		if (ofs.fail()) {
			writeError(filename);
		}
		ofs << 0 << std::endl;
		ofs.close();
	}


	/*  パーティクル動画の初期化  */
	void initParticleVideo(std::string ofpath)
	{
		/*  ビデオ関係  */
		// ファイルをオープンし，ビデオライタを初期化
		// filename - 出力ファイル名
		// fourcc - コーデック
		// fps - 1 秒あたりのフレーム数
		// frameSize - ビデオフレームのサイズ
		// isColor - ビデオストリームがカラーか，グレースケールかを指定

		std::string filename = ofpath + "Movie/particle.avi";
		//std::string filename = "./output/Movie/particle.avi";
		Coor<> rect(CUT_MAP_RADIUS_X*6.0, CUT_MAP_RADIUS_Y*4.0);
		cv::Size rect_pix = ToPixelSize(rect, map_img_clone, MAP_RES);
		rect_pix.width *= MOVIE_SCALE_W;
		rect_pix.height *= MOVIE_SCALE_H;
		particle_video.open(filename, CV_FOURCC('X', 'V', 'I', 'D'), PARTICLE_FPS, rect_pix);
		if (!particle_video.isOpened())	writeError(filename);
	}

	void initParticleLargeVideo(std::string ofpath)
	{
		/*  ビデオ関係  */
		// ファイルをオープンし，ビデオライタを初期化
		// filename - 出力ファイル名
		// fourcc - コーデック
		// fps - 1 秒あたりのフレーム数
		// frameSize - ビデオフレームのサイズ
		// isColor - ビデオストリームがカラーか，グレースケールかを指定

		std::string filename = ofpath + "Movie/particle_large.avi";
		//std::string filename = "./output/Movie/particle.avi";
		Coor<> rect(CUT_MAP_RADIUS_X*6.0, CUT_MAP_RADIUS_Y*4.0);
		cv::Size rect_pix = ToPixelSize(rect, map_img_color, MAP_RES);
		rect_pix.width *= MOVIE_SCALE_W;
		rect_pix.height *= MOVIE_SCALE_H;
		particle_large_video.open(filename, CV_FOURCC('X', 'V', 'I', 'D'), PARTICLE_FPS, rect_pix);
		if (!particle_large_video.isOpened())	writeError(filename);
	}

	void initWeightedStatParVideo(std::string ofpath)
	{
		/*  ビデオ関係  */
		// ファイルをオープンし，ビデオライタを初期化
		// filename - 出力ファイル名
		// fourcc - コーデック
		// fps - 1 秒あたりのフレーム数
		// frameSize - ビデオフレームのサイズ
		// isColor - ビデオストリームがカラーか，グレースケールかを指定

		std::string filename = ofpath + "Movie/weighted_stat_particle.avi";
		//std::string filename = "./output/Movie/weighted_stat_particle.avi";
		Coor<> rect(CUT_MAP_RADIUS_X*6.0, CUT_MAP_RADIUS_Y*4.0);
		cv::Size rect_pix = ToPixelSize(rect, map_img_clone, MAP_RES);
		rect_pix.width *= MOVIE_SCALE_W;
		rect_pix.height *= MOVIE_SCALE_H;
		weighted_stat_particle_video.open(filename, CV_FOURCC('X', 'V', 'I', 'D'), PARTICLE_FPS, rect_pix);
		if (!weighted_stat_particle_video.isOpened())	writeError(filename);
	}

	void initMeasurementDataVideo(std::string ofpath)
	{
		/*  ビデオ関係  */
		// ファイルをオープンし，ビデオライタを初期化
		// filename - 出力ファイル名
		// fourcc - コーデック
		// fps - 1 秒あたりのフレーム数
		// frameSize - ビデオフレームのサイズ
		// isColor - ビデオストリームがカラーか，グレースケールかを指定

		std::string filename = ofpath + "Movie/measurement_data.avi";
		int fps = MEASUREMENT_DATA_VIDEO_FPS;
		Coor<> rect(CUT_MAP_RADIUS_X*4.0, CUT_MAP_RADIUS_Y*4.0);
		cv::Size rect_pix = ToPixelSize(rect, map_img_clone, MAP_RES);
		rect_pix.width *= MOVIE_SCALE_W;
		rect_pix.height *= MOVIE_SCALE_H;
		measurement_data_video.open(filename, MEASUREMENT_DATA_VIDEO_FOURCC, fps, rect_pix);
		if (!measurement_data_video.isOpened())	writeError(filename);
	}


	/**********************************************************/
	//  パーティクルフィルタ
	/**********************************************************/

	void Transition()
	{
		/*  状態遷移に付加する雑音の準備  */
		std::random_device rnd;     // 非決定的な乱数生成器を生成
		std::mt19937_64 mt(rnd());     //  メルセンヌ・ツイスタの64ビット版、引数は初期シード値

		//	パーティクルフィルタシステムノイズ
		Position<> par_mean(0.0, 0.0, 0.0);
		std::normal_distribution<double> par_sys_noise_x(par_mean.x, trans_par_sys_var.x);
		std::normal_distribution<double> par_sys_noise_y(par_mean.y, trans_par_sys_var.y);
		std::normal_distribution<double> par_sys_noise_r(par_mean.r, trans_par_sys_var.r);

		/*  ロボットの移動量をRobot座標系からLocal座標系に変換  */

		Polar<> delta;	//	移動半径と変化角度はRobot座標系とLocal座標系で共通

		//	前のステップからの移動量半径を算出
		delta.r = *odometry2 | *odometry1;
		delta.theta = odometry2->r - odometry1->r;

		//	オドメトリ
		Polar<> odo_mean(0.0, 0.0);	//  平均は全て0
		if (delta.theta < M_PI / 16){
			odometry_system_noise.set(std::abs(delta.r) + 0.0001, std::abs(delta.theta) + 0.0001);
			//odometry_system_noise.set(std::abs(delta.r / 2.0) + 0.0001, std::abs(delta.theta) + 0.0001);
		}
		else{
			odometry_system_noise.set(std::abs(delta.r / 2.0) + 0.0001, std::abs(M_PI / 16) + 0.0001);
		}

		std::normal_distribution<double> noise_r(odo_mean.r, odometry_system_noise.r);				//	ガウス雑音を生成
		std::normal_distribution<double> noise_theta(odo_mean.theta, odometry_system_noise.theta);	//	ガウス雑音を生成


		switch (trial_type)
		{
		case TRIAL_SIMULTANEOUS:
		case TRIAL_PEARSON:
		case TRIAL_PEARSON_NONSTAT:
		case TRIAL_SUYAMA_STAT:
		case TRIAL_SUYAMA_NONSTAT:
		case TRIAL_3SENSORS_SIMULATNEOUS:
		case TRIAL_3SENSORS_PEARSON:
		case TRIAL_3SENSORS_PEARSON_NONSTAT:
		case TRIAL_3SENSORS_SUYAMA_NONSTAT:
		case TRIAL_3SENSORS_LRF_GPS:
			/*  パーティクルの遷移  */
			for (const auto &par : getParticles())
			{
				Position<> par_tmp = *par->getState();
				Polar<> delta_tmp = delta;

				//	オドメトリのノイズ付加
				Polar<> noise(noise_r(mt), noise_theta(mt));
				delta_tmp += noise;
				//	パーティクルの遷移
				par_tmp.x += delta_tmp.r*std::cos(par_tmp.r);
				par_tmp.y += delta_tmp.r*std::sin(par_tmp.r);
				par_tmp.r += delta_tmp.theta;

				//	パーティクルフィルタのシステムノイズ付加
				Position<> par_noise(par_sys_noise_x(mt), par_sys_noise_y(mt), par_sys_noise_r(mt));
				par_tmp += par_noise;

				*par->getState() = par_tmp;

			}

			break;
		case TRIAL_NON_TIMESEQUENCE:
		case TRIAL_NON_TIMESEQUENCE_SIMUL:
			/*  パーティクルの遷移  */
			for (const auto &par : getParticles())
			{
				Position<> par_tmp = *par->getState();
				Polar<> delta_tmp = delta;

				//	オドメトリのノイズ付加
				Polar<> noise(noise_r(mt), noise_theta(mt));
				delta_tmp += noise;
				//	パーティクルの遷移
				par_tmp.x += delta_tmp.r*std::cos(par_tmp.r);
				par_tmp.y += delta_tmp.r*std::sin(par_tmp.r);
				par_tmp.r += delta_tmp.theta;

				*par->getState() = par_tmp;

			}

			break;
		default:
			std::cout << "Error 'TRIAL': " << TRIAL_TYPE << "FILE: " << __FILE__ << " LINE: " << __LINE__ << std::endl;
			exit(0);
			break;
		}



		std::cout << "Complete 'Transition' " << std::endl;
	};

	/*  計測データの読み込み  */
	//std::vector<cv::KeyPoint> setKeypoints(cv::Mat img) {
	//	std::vector<cv::KeyPoint> keypoints;
	//	/*  Operatorの塗りつぶし  */
	//	cv::Point center(img.cols / 1.9, img.rows / 2);
	//	cv::Size radius(img.rows / 3, img.rows / 3);
	//	double start_angle = -105;	//	扇系の中心角度
	//	double angle = 35;	//	角度
	//	cv::ellipse(img, center, radius, start_angle, 0, angle, cv::Scalar(0, 0, 0), -1, CV_AA);
	//	/* 外枠の塗りつぶし */
	//	cv::circle(img, center, 510, cv::Scalar(0, 0, 0), 150);
	//	//　等間隔の画像に切り出し
	//	cv::Rect roi_rect(ROI_ORG_X, ROI_ORG_Y, ROI_SIZE_X, ROI_SIZE_Y); // x,y,w,h
	//	img = img(roi_rect);
	//	//SURF
	//	cv::SurfFeatureDetector detector(1000);
	//	detector.detect(img, keypoints);
	//	return keypoints;
	//}

	//cv::Mat setDescriptor(cv::Mat img, std::vector<cv::KeyPoint> keypoints) {
	//	cv::SurfDescriptorExtractor extractor;
	//	//画像の特徴点における特徴量を抽出
	//	cv::Mat descriptors;
	//	extractor.compute(img, keypoints, descriptors);
	//	return descriptors;
	//}


	//
	void setMeasurement() {
		std::cout << 1 << std::endl;
		while (true) {
			int min = std::min({ all_meas_odometry.size(), all_meas_time.size(), all_meas_lrf_l.size(), all_meas_lrf_u.size(), all_meas_keypoints.size(), all_meas_desriptor.size(), all_meas_gps.size() });
			if (now_step < min) {
				break;
			}
			if (read_all_measurement_) {
				finish = true;
				return;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(MOVIE_CREATER_SLEEP_MILLISECONDS));
		}

		if (odometry2 == nullptr) {
			odometry2 = new Position<>;
		}
		*odometry2 = all_meas_odometry[now_step];
		esti_time = all_meas_time[now_step];

		for (int i = 0; i < LIKELIHOOD_THREAD; i++) {

			lid2_l[i].setMeasScan(all_meas_lrf_l[now_step]);
			switch (trial_type)
			{
			case TRIAL_SIMULTANEOUS:
			case TRIAL_PEARSON:
			case TRIAL_PEARSON_NONSTAT:
			case TRIAL_NON_TIMESEQUENCE:
			case TRIAL_NON_TIMESEQUENCE_SIMUL:
			case TRIAL_SUYAMA_STAT:
			case TRIAL_SUYAMA_NONSTAT:
				lid2_u[i].setMeasScan(all_meas_lrf_u[now_step]);
				break;
			case TRIAL_3SENSORS_SIMULATNEOUS:
			case TRIAL_3SENSORS_PEARSON:
			case TRIAL_3SENSORS_PEARSON_NONSTAT:
			case TRIAL_3SENSORS_SUYAMA_NONSTAT:
			case TRIAL_3SENSORS_LRF_GPS:
			default:
				std::cout << "NO TRIAL_TYPE: " << trial_type << ", " << __FILE__ << " LINE: " << __LINE__ << std::endl;
				break;
			}
			omni[i].setKeypoints(all_meas_keypoints[now_step]);
			omni[i].setDescriptors(all_meas_desriptor[now_step]);
			gpgga[i].setMeasSignal(all_meas_gps[now_step]);
		}
		all_meas_gps_pos.push_back(gpgga[0].mean);

		std::cout << "Complete setMeasurement: " << now_step << std::endl;

		now_step++;

	}

	void readMeasurement1(){


		clock_t lap1 = clock();
		std::string filename;
		filename = IFPATH_MEAS + "lrf_lower/lrf_no" + std::to_string(read_meas_no) + "_" + std::to_string(read_meas_step) + "th.csv";
		std::vector<Polar<>> scan_l;
		std::ifstream ifs_lrf_l(filename);
		if (ifs_lrf_l.fail()) {
			std::cout << filename << " dose not exist" << std::endl;
			read_all_measurement_ = true;
		}
		ifs_lrf_l >> scan_l;
		all_meas_lrf_l.push_back(scan_l);

		clock_t lap2 = clock();
		filename = IFPATH_MEAS + "lrf_upper/lrf_no" + std::to_string(read_meas_no) + "_" + std::to_string(read_meas_step) + "th.csv";
		std::vector<Polar<>> scan_u;
		std::ifstream ifs_lrf_u(filename);
		if (ifs_lrf_u.fail()) {
			std::cout << filename << " dose not exist" << std::endl;
			read_all_measurement_ = true;
		}
		ifs_lrf_u >> scan_u;
		all_meas_lrf_u.push_back(scan_u);

		//filename = IFPATH_MEAS + "img/img_no" + std::to_string(read_meas_no) + "_" + std::to_string(read_meas_step) + "th.bmp";
		//cv::Mat img;
		//img = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
		//if (img.empty()) {
		//	std::cout << filename << " dose not exist" << std::endl;
		//	read_all_measurement_=true;
		//}
		////all_meas_img.push_back(img);

		switch (OMNI_FEATURE)
		{
		case OMNI_FEATURE_SIFT:{
			clock_t lap3 = clock();
			filename = IFPATH_MEAS + "sift/keypoint/keypoint_no" + std::to_string(read_meas_no) + "_" + std::to_string(read_meas_step) + "th.yml";
			std::vector<cv::KeyPoint> keys;
			cv::FileStorage fs(filename, cv::FileStorage::READ);
			cv::FileNode fn = fs["keypoints"];
			cv::read(fn, keys);
			if (keys.empty()){
				std::cout << filename << " dose not exist" << std::endl;
				read_all_measurement_ = true;
			}
			all_meas_keypoints.push_back(keys);

			//std::vector<cv::KeyPoint> keys = setKeypoints(img);
			//if (keys.empty()) {
			//	std::cout << filename << " dose not exist" << std::endl;
			//	read_all_measurement_=true;
			//}
			//all_meas_keypoints.push_back(keys);

			clock_t lap4 = clock();

			//filename = IFPATH_MEAS + "sift/descriptor/desc_no" + std::to_string(read_meas_no) + "_" + std::to_string(read_meas_step) + "th.csv";
			//std::ifstream ifs(filename);
			//if (ifs.fail()){
			//	read_all_measurement_ = true;
			//}
			//clock_t lap4_5 = clock();
			//std::vector<std::vector<float>> v;
			//{
			//	v.reserve(1500);
			//	std::string str;
			//	while (std::getline(ifs, str))
			//	{
			//		std::vector<float> tmp = Split<float>(str, ",");
			//		v.push_back(tmp);
			//	}
			//}
			//clock_t lap4_7 = clock();
			//cv::Mat descriptor;
			//for (int i = 0; i < v.size(); i++){
			//	cv::Mat mat(v[i], true);
			//	mat = mat.t();
			//	descriptor.push_back(mat);
			//}
			filename = IFPATH_MEAS + "sift/descriptor/desc_no" + std::to_string(read_meas_no) + "_" + std::to_string(read_meas_step) + "th.bmp";
			cv::Mat_<float> descriptor = cv::imread(filename, CV_LOAD_IMAGE_ANYCOLOR);
			all_meas_desriptor.push_back(descriptor);

			//cv::Mat desc = setDescriptor(img, keys);
			//if (desc.empty()) {
			//	std::cout << filename << " dose not exist" << std::endl;
			//	read_all_measurement_=true;
			//}
			//all_meas_desriptor.push_back(desc);
			break;
		}
		case OMNI_FEATURE_SURF:{
			clock_t lap3 = clock();
			filename = IFPATH_MEAS + "surf/keypoint/keypoint_no" + std::to_string(read_meas_no) + "_" + std::to_string(read_meas_step) + "th.yml";
			std::vector<cv::KeyPoint> keys;
			cv::FileStorage fs(filename, cv::FileStorage::READ);
			cv::FileNode fn = fs["keypoints"];
			cv::read(fn, keys);
			if (keys.empty()){
				std::cout << filename << " dose not exist" << std::endl;
				read_all_measurement_ = true;
			}
			all_meas_keypoints.push_back(keys);

			//std::vector<cv::KeyPoint> keys = setKeypoints(img);
			//if (keys.empty()) {
			//	std::cout << filename << " dose not exist" << std::endl;
			//	read_all_measurement_=true;
			//}
			//all_meas_keypoints.push_back(keys);

			clock_t lap4 = clock();

			filename = IFPATH_MEAS + "surf/descriptor/desc_no" + std::to_string(read_meas_no) + "_" + std::to_string(read_meas_step) + "th.csv";
			std::ifstream ifs(filename);
			if (ifs.fail()){
				read_all_measurement_ = true;
			}
			clock_t lap4_5 = clock();
			std::vector<std::vector<float>> v;
			{
				v.reserve(1500);
				std::string str;
				while (std::getline(ifs, str))
				{
					std::vector<float> tmp = Split<float>(str, ",");
					v.push_back(tmp);
				}
			}
			clock_t lap4_7 = clock();
			cv::Mat descriptor;
			for (int i = 0; i < v.size(); i++){
				cv::Mat mat(v[i], true);
				mat = mat.t();
				descriptor.push_back(mat);
			}
			all_meas_desriptor.push_back(descriptor);

			//cv::Mat desc = setDescriptor(img, keys);
			//if (desc.empty()) {
			//	std::cout << filename << " dose not exist" << std::endl;
			//	read_all_measurement_=true;
			//}
			//all_meas_desriptor.push_back(desc);
			break;
		}
		default:
			std::cout << "OMNI_FEATURE: " << OMNI_FEATURE << std::endl;
			exit(0);
			break;
		}

		clock_t lap5 = clock();
		filename = IFPATH_MEAS + "gps/gps_no" + std::to_string(read_meas_no) + "_" + std::to_string(read_meas_step) + "th.txt";
		std::ifstream ifs_gps(filename);
		if (ifs_gps.fail()) {
			std::cout << filename << " dose not exist" << std::endl;
			read_all_measurement_ = true;
		}
		std::string str;
		ifs_gps >> str;
		all_meas_gps.push_back(str);

		clock_t lap6 = clock();
		filename = IFPATH_MEAS + "odometry/odometry_no" + std::to_string(read_meas_no) + "_" + std::to_string(read_meas_step) + "th.csv";
		std::ifstream ifs_odo(filename);
		if (ifs_odo.fail()) {
			std::cout << filename << " dose not exist" << std::endl;
			read_all_measurement_ = true;
		}
		Position<> odo;
		ifs_odo >> odo;
		odo.r *= M_PI / 1800.0;
		all_meas_odometry.push_back(odo);

		clock_t lap7 = clock();
		filename = IFPATH_MEAS + "time/time_no" + std::to_string(read_meas_no) + "_" + std::to_string(read_meas_step) + "th.csv";
		std::ifstream ifs_time(filename);
		if (ifs_time.fail()) {
			std::cout << filename << " dose not exist" << std::endl;
			read_all_measurement_ = true;
		}
		MyTime time;
		ifs_time >> time;
		all_meas_time.push_back(time);

		if (read_all_measurement_) {
			finish = true;
			return;
		}



		std::cout << "ReadMeasurement: " << read_meas_step << std::endl;
		//std::cout << lap1 << "," << lap2 << "," << lap3 << "," << lap4 << "," << lap4_5 << "," << lap4_7 << "," << lap5 << "," << lap6 << "," << lap7 << std::endl;

		read_meas_step += SKIP_STEPS;


		if (odometry2 == nullptr) {
			odometry2 = new Position<>;
		}
		*odometry2 = all_meas_odometry.front();
		esti_time = all_meas_time.front();

		switch (trial_type)
		{
		case TRIAL_SIMULTANEOUS:
		case TRIAL_PEARSON:
		case TRIAL_PEARSON_NONSTAT:
		case TRIAL_NON_TIMESEQUENCE:
		case TRIAL_NON_TIMESEQUENCE_SIMUL:
		case TRIAL_SUYAMA_STAT:
		case TRIAL_SUYAMA_NONSTAT:
			for (int i = 0; i < LIKELIHOOD_THREAD; i++) {

				lid2_l[i].setMeasScan(all_meas_lrf_l.front());
				lid2_u[i].setMeasScan(all_meas_lrf_u.front());
				omni[i].setKeypoints(all_meas_keypoints.front());
				omni[i].setDescriptors(all_meas_desriptor.front());
				gpgga[i].setMeasSignal(all_meas_gps.front());
			}
			all_meas_gps_pos.push_back(gpgga[0].mean);
			break;
		case TRIAL_3SENSORS_SIMULATNEOUS:
		case TRIAL_3SENSORS_PEARSON:
		case TRIAL_3SENSORS_PEARSON_NONSTAT:
		case TRIAL_3SENSORS_SUYAMA_NONSTAT:
		case TRIAL_3SENSORS_LRF_GPS:
			for (int i = 0; i < LIKELIHOOD_THREAD; i++) {

				lid2_l[i].setMeasScan(all_meas_lrf_l.front());
				omni[i].setKeypoints(all_meas_keypoints.front());
				omni[i].setDescriptors(all_meas_desriptor.front());
				gpgga[i].setMeasSignal(all_meas_gps.front());
			}
			all_meas_gps_pos.push_back(gpgga[0].mean);
			break;
		default:
			break;
		}


		std::cout << "Complete setMeasurement: " << now_step << std::endl;

		now_step++;

	}

	/* Movie Creater*/

	// 位置を描画
	void drawPosition(const Position<>& pos, cv::Mat& map, cv::Scalar color, int map_cols, int map_rows, int map_res, double map_img_org_x, double map_img_org_y, cv::Point upleft_pix, PositionType position_type){

		// 描画パラメータ
		int radius, thickness, arrow_length;
		switch (position_type)
		{
		case PARTICLE:
			radius = IMAGE_PARTICLE_RADIUS;
			thickness = IMAGE_PARTICLE_ARROW_THICKNESS;
			arrow_length = IMAGE_PARTICLE_ARROW_LENGTH;
			break;
		case ESTIMATED_POSITION:
			radius = IMAGE_ESTIPOSITION_RADIUS;
			thickness = IMAGE_ESTI_ARROW_THICKNESS;
			arrow_length = IMAGE_ESTI_ARROW_LENGTH;
			break;
		case TRUE_POSITION:
			radius = IMAGE_TRUEPOSITION_RADIUS;
			thickness = IMAGE_TPOS_ARROW_THICKNESS;
			arrow_length = IMAGE_TPOS_ARROW_LENGTH;
			break;
		default:
			break;
		}

		// 描画
		cv::Point pos_pix = ToPixel(pos, map_cols, map_rows, map_res, map_img_org_x, map_img_org_y);
		pos_pix -= upleft_pix;
		if (pos_pix.x > 0 && pos_pix.x < map_cols &&
			pos_pix.y>0 && pos_pix.y < map_rows){
			cv::circle(map, pos_pix, radius, color, thickness, 8, 0);
			// 矢印描画
			double l = arrow_length*map_res;
			Coor<> coor2;
			coor2.x = l*std::cos(pos.r) + pos.x;
			coor2.y = l*std::sin(pos.r) + pos.y;
			cv::Point p2 = ToPixel(coor2, map_cols, map_rows, map_res, map_img_org_x, map_img_org_y);	//	パーティクルピクセル
			p2 -= upleft_pix;
			cv::line(map, pos_pix, p2, color, thickness);
		}

	}

	/*  画像の切り出し  */
	// 切り出しの左上を返還
	cv::Point rectMap(const cv::Mat& map_in, cv::Mat& map_out, const Position<>& center, int cut_map_radius_x, int cut_map_radius_y, int map_res, float map_img_org_x, float map_img_org_y){
		Coor<> upleft(center.x - cut_map_radius_x, center.y - cut_map_radius_y);
		//Coor<> upleft(estimated_position.back().x - CUT_MAP_RADIUS_X, estimated_position.back().y - CUT_MAP_RADIUS_Y);
		Coor<> rect(cut_map_radius_x*2.0, cut_map_radius_y*2.0);
		cv::Point upleft_pix = ToPixel(upleft, map_in.cols, map_in.rows, map_res, map_img_org_x, map_img_org_y);	//
		cv::Size rect_pix = ToPixelSize(rect, map_in, map_res);


		if (upleft_pix.x < 0)	upleft_pix.x = 0;
		if (upleft_pix.y < 0)	upleft_pix.y = 0;

		if (upleft_pix.x >= map_in.cols - rect_pix.width)	upleft_pix.x = map_in.cols - rect_pix.width;
		if (upleft_pix.y >= map_in.rows - rect_pix.height)	upleft_pix.y = map_in.rows - rect_pix.height;

		cv::Mat mat = cv::Mat(map_in, cv::Rect(upleft_pix, rect_pix));

		map_out = mat;

		return upleft_pix;
	}

	// 真の位置を補完するかどうか
	Position<> getTruePosition(int step){
		Position<> now_tpos;
		if (true_time[all_stock_tidx[step]] == esti_time || all_stock_tidx[step] - 1 < 0) {
			now_tpos = true_position->at(all_stock_tidx[step]);
		}
		else {
#if MODE_LINIER_INTERPOLATION
			//std::cout << true_time[all_stock_tidx[step]] << "," << esti_time << std::endl;
			Position<> diff_tpos = true_position->at(all_stock_tidx[step]) - true_position->at(all_stock_tidx[step] - 1);
			double diff_ttime = MyTime::diff(true_time[all_stock_tidx[step] - 1], true_time[all_stock_tidx[step]]);
			double diff_etime = MyTime::diff(true_time[all_stock_tidx[step] - 1], esti_time);
			now_tpos = true_position->at(all_stock_tidx[step] - 1) + diff_tpos / diff_ttime*diff_etime;
			now_tpos.r = true_position->at(all_stock_tidx[step] - 1).r;
#endif
		}
		return now_tpos;
	}


	//
	void visualizeScan(const std::vector<Polar<>>& scan, cv::Mat& map, const Position<> esti_pos, int step) {

		std::vector<Polar<>> scan_role;
		for (const auto& s : scan) {
			if (s.r == 1)	continue;
			Polar<> p = s;
			p.theta += esti_pos.r;
			scan_role.push_back(p);
		}

		std::vector<Coor<>> scan_xy = polToCoor(scan_role);

		std::vector<Coor<>> scan_from_robot;
		for (const auto& s : scan_xy) {
			Coor<> coor;
			coor.x = s.x + esti_pos.x;
			coor.y = s.y + esti_pos.y;
			scan_from_robot.push_back(coor);
		}


		for (const auto&s : scan_from_robot) {
			cv::Point pixel = ToPixel(s, map.cols, map.rows, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//
			cv::circle(map, pixel, MEASUREMENT_DATA_VIDEO_SCAN_RADIUS, MEASUREMENT_DATA_VIDEO_SCAN_COLOR, -1);
		}

		/* 推定位置の出力*/
		drawPosition(esti_pos, map, IMAGE_ESTIPOSITION_COLOR, map.cols, map.rows, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y, cv::Point(0, 0), ESTIMATED_POSITION);

		/* 真の位置のみ描画 */
		if (exist_true_position){
			Position<> now_tpos = getTruePosition(step);
			drawPosition(now_tpos, map, IMAGE_TRUEPOSITION_COLOR, map.cols, map.rows, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y, cv::Point(0, 0), TRUE_POSITION);
		}

	}

	//
	void visualizeOmniImg(const std::vector<Position<>>& img_pos, cv::Mat& map, const std::vector<double> img_sim, const Position<> esti_pos, int step) {

		for (int i = 0; i < img_pos.size(); i++) {
			cv::Point pixel = ToPixel(img_pos[i], map.cols, map.rows, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//
			cv::circle(map, pixel, MEASUREMENT_DATA_VIDEO_IMG_POS_RADIUS, MEASUREMENT_DATA_VIDEO_IMG_POS_COLOR, -1);
			cv::putText(map, std::to_string(img_sim[i]), MEASUREMENT_DATA_VIDEO_IMG_SIM_TEXT_POINT, MEASUREMENT_DATA_VIDEO_IMG_SIM_TEXT_FONT, MEASUREMENT_DATA_VIDEO_IMG_SIM_TEXT_SCALE, MEASUREMENT_DATA_VIDEO_IMG_SIM_TEXT_COLOR, MEASUREMENT_DATA_VIDEO_IMG_SIM_TEXT_THIN, CV_AA);
		}

		cv::Point esti_pixel = ToPixel(esti_pos, map.cols, map.rows, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		cv::circle(map, esti_pixel, IMAGE_ESTIPOSITION_RADIUS, IMAGE_ESTIPOSITION_COLOR, IMAGE_ESTI_ARROW_THICKNESS, 8, 0);

		/* 推定位置の出力*/
		drawPosition(esti_pos, map, IMAGE_ESTIPOSITION_COLOR, map.cols, map.rows, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y, cv::Point(0, 0), ESTIMATED_POSITION);

		/* 真の位置のみ描画 */
		if (exist_true_position){
			Position<> now_tpos = getTruePosition(step);
			drawPosition(now_tpos, map, IMAGE_TRUEPOSITION_COLOR, map.cols, map.rows, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y, cv::Point(0, 0), TRUE_POSITION);
		}
	}

	//
	void visualizeGPS(const Position<> gps, cv::Mat& map, const Position<> esti_pos, int step) {
		cv::Point pix_gps = ToPixel(gps, map, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//
		cv::circle(map, pix_gps, MEASUREMENT_DATA_VIDEO_GPS_RADIUS, MEASUREMENT_DATA_VIDEO_GPS_COLOR, -1);

		/* 推定位置の出力*/
		drawPosition(esti_pos, map, IMAGE_ESTIPOSITION_COLOR, map.cols, map.rows, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y, cv::Point(0, 0), ESTIMATED_POSITION);

		/* 真の位置のみ描画 */
		if (exist_true_position){
			Position<> now_tpos = getTruePosition(step);
			drawPosition(now_tpos, map, IMAGE_TRUEPOSITION_COLOR, map.cols, map.rows, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y, cv::Point(0, 0), TRUE_POSITION);
		}
	}


	/*  重み付きパーティクルを障害物地図上に表示  */
	void visualizeParticleWeight(cv::Mat& map, const std::vector<Position<>>& particle, const std::vector<double> &likelihood, int step, int cols, int rows, double map_res, cv::Point upleft_pix, bool visualize_weighted_mean)
	{
		/* パーティクル描画 */
		//cv::cvtColor(map, map, CV_BGR2HSV); // RGB→HSVに変換
		//std::vector<int> up_idx = sortUpIdx(likelihood);		//	パーティクルを昇順にソート
		////cv::cvtColor(map, map, cv::COLOR_RGB2HSV);	// RGB→HSVに変換
		//for (const auto &idx : up_idx)
		//{
		//	if (likelihood[idx] != 0.0) {
		//		cv::Scalar color;
		//		double min = *std::min_element(likelihood.begin(), likelihood.end());
		//		double max = *std::max_element(likelihood.begin(), likelihood.end());
		//		color = cv::Scalar(150, 255 - (int)(likelihood[idx] / max*255.0 + 0.5), 255);
		//		int intensity;
		//		if (max != min){
		//			intensity = (int)((likelihood[idx] - min) / (max - min)*255.0 + 0.5);
		//		}
		//		else if (max == 1.0){
		//			intensity = 255;
		//		}
		//		else{
		//			intensity = 128;
		//		}
		//		if (intensity < 0) intensity = 0;
		//		color = cv::Scalar(180, intensity, 255);
		//		drawPosition(particle[idx], map, color, cols, rows, map_res, MAP_IMG_ORG_X, MAP_IMG_ORG_Y, upleft_pix, PARTICLE);
		//	}
		//}
		//cv::cvtColor(map, map, CV_HSV2BGR); // HSV→RGBに変換
		cv::cvtColor(map, map, CV_BGR2HSV); // RGB→HSVに変換
		std::vector<int> up_idx = sortUpIdx(likelihood);		//	パーティクルを昇順にソート
		//cv::cvtColor(map, map, cv::COLOR_RGB2HSV);	// RGB→HSVに変換
		for (const auto &idx : up_idx)
		{
			if (likelihood[idx] != 0.0) {
				cv::Scalar color;
				double max = 10.0/(double)likelihood.size();
				int s = (int)(likelihood[idx] / max*255.0 + 0.5);
				if (s > 255)s = 255;
				color = cv::Scalar(150, s, 255);
				drawPosition(particle[idx], map, color, cols, rows, map_res, MAP_IMG_ORG_X, MAP_IMG_ORG_Y, upleft_pix, PARTICLE);
			}
		}
		cv::cvtColor(map, map, CV_HSV2BGR); // HSV→RGBに変換


		/* 重み付き平均の出力*/
		if (visualize_weighted_mean)
		{
			Position<> esti_pos(0.0, 0.0, 0.0);
			std::vector<double> likelihood_tmp = likelihood;
			if (std::count(likelihood_tmp.begin(), likelihood_tmp.end(), 1.0) == likelihood_tmp.size()){
				likelihood_tmp = std::vector<double>(likelihood_tmp.size(), 1.0 / likelihood_tmp.size());
			}
			for (int i = 0; i < particle.size(); i++){
				esti_pos += particle[i] * likelihood_tmp[i];
			}
			drawPosition(esti_pos, map, IMAGE_ESTIPOSITION_COLOR, cols, rows, map_res, MAP_IMG_ORG_X, MAP_IMG_ORG_Y, upleft_pix, ESTIMATED_POSITION);
		}

		/* 真の位置のみ描画 */
		if (exist_true_position){
			Position<> now_tpos = getTruePosition(step);
			drawPosition(now_tpos, map, IMAGE_TRUEPOSITION_COLOR, cols, rows, map_res, MAP_IMG_ORG_X, MAP_IMG_ORG_Y, upleft_pix, TRUE_POSITION);
		}
	}


	/*  パーティクル動画にフレームを追加  */
	void addMeasurementVideo() {

		cv::Mat img_lrf_l, img_lrf_u, img_omni, img_gps;
		img_lrf_l = map_img_lower.clone();
		img_lrf_u = map_img_upper.clone();
		img_omni = map_img_color.clone();
		img_gps = map_img_color.clone();

		// 描画
		visualizeScan(all_meas_lrf_l.front(), img_lrf_l, estimated_position[movie_step], movie_step);
		if (sensor_num == 3){
			img_lrf_u = cv::Mat::zeros(cv::Size(map_img_color.cols, map_img_color.rows), CV_8UC3);
		}
		else{
			visualizeScan(all_meas_lrf_l.front(), img_lrf_u, estimated_position[movie_step], movie_step);
		}
		visualizeOmniImg(all_omni_img_sim_pos.front(), img_omni, all_omni_img_sim.front(), estimated_position[movie_step], movie_step);
		visualizeGPS(all_meas_gps_pos.front(), img_gps, estimated_position[movie_step], movie_step);


		// カット
		Position<> center = estimated_position[movie_step];
		rectMap(img_lrf_l, img_lrf_l, center, CUT_MAP_RADIUS_X, CUT_MAP_RADIUS_Y, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		rectMap(img_lrf_u, img_lrf_u, center, CUT_MAP_RADIUS_X, CUT_MAP_RADIUS_Y, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		rectMap(img_omni, img_omni, center, CUT_MAP_RADIUS_X, CUT_MAP_RADIUS_Y, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		rectMap(img_gps, img_gps, center, CUT_MAP_RADIUS_X, CUT_MAP_RADIUS_Y, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);

		// 上下反転
		cv::flip(img_lrf_l, img_lrf_l, 0);
		cv::flip(img_lrf_u, img_lrf_u, 0);
		cv::flip(img_omni, img_omni, 0);
		cv::flip(img_gps, img_gps, 0);


		cv::putText(img_lrf_l, "l_l" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		cv::putText(img_lrf_u, "l_u" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		cv::putText(img_omni, "c" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		cv::putText(img_gps, "g" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);

		int w = img_lrf_l.cols;
		int h = img_lrf_l.rows;

		cv::Mat img(cv::Size(w*2.0, h*2.0), CV_8UC3, cv::Scalar(0, 0, 0));

		assert(img_lrf_l.channels() == img.channels());
		assert(img_lrf_u.channels() == img.channels());
		assert(img_omni.channels() == img.channels());
		assert(img_gps.channels() == img.channels());

		img_lrf_l.copyTo(img(cv::Rect(cv::Point(0, 0), cv::Size(w, h))));
		img_lrf_u.copyTo(img(cv::Rect(cv::Point(w, 0), cv::Size(w, h))));
		img_omni.copyTo(img(cv::Rect(cv::Point(0, h), cv::Size(w, h))));
		img_gps.copyTo(img(cv::Rect(cv::Point(w, h), cv::Size(w, h))));

		cv::resize(img, img, cv::Size(), MOVIE_SCALE_W, MOVIE_SCALE_H);

		measurement_data_video << img;
		measurement_data_img = img;

	}
	void addFlameWeightedParImg(bool stat_)
	{
		std::vector<Position<>> particle;
		std::vector<double> likelihood_lid2_l, likelihood_lid2_u, likelihood_omni, likelihood_gps, likelihood_fusion, likelihood_all;
		if (stat_){
			particle = all_stat_particles[movie_step];
			likelihood_lid2_l = all_stat_lid2_l_likelihood[movie_step];
			likelihood_lid2_u = all_stat_lid2_u_likelihood[movie_step];
			likelihood_omni = all_stat_omni_likelihood[movie_step];
			likelihood_gps = all_stat_gpgga_likelihood[movie_step];
			likelihood_fusion = std::vector<double>(STAT_SAMPLE_SIZE, 0.0);
		}
		else{
			particle = all_particles[movie_step];
			likelihood_lid2_l = all_lid2_l_likelihood[movie_step];
			likelihood_lid2_u = all_lid2_u_likelihood[movie_step];
			likelihood_omni = all_omni_likelihood[movie_step];
			likelihood_gps = all_gpgga_likelihood[movie_step];
			likelihood_fusion = all_fusion_likelihood[movie_step];
		}
		likelihood_all = std::vector<double>(particle.size(), 1.0);

		// 描画と切り出し
		cv::Mat img_lrf_l, img_lrf_u, img_omni, img_gps, img_fusion, img_all;
		img_lrf_l = map_img_color.clone();
		img_lrf_u = map_img_color.clone();
		img_omni = map_img_color.clone();
		img_gps = map_img_color.clone();
		img_fusion = map_img_color.clone();
		img_all = map_img_color.clone();
		Position<> center;

		/* 切り出し中心 */
		if (exist_true_position){
			center = getTruePosition(movie_step);
		}
		else{
			center = estimated_position[movie_step];
		}


		std::thread thread_lrf_l, thread_lrf_u, thread_omni, thread_gpgga, thread_fusion, thread_all;
		//thread_lrf_l = std::thread([&]{
			visualizeParticleWeight(img_lrf_l, particle, likelihood_lid2_l, movie_step, img_lrf_l.cols, img_lrf_l.rows, MAP_RES, cv::Point(0, 0), false);
			rectMap(img_lrf_l, img_lrf_l, center, CUT_MAP_RADIUS_X, CUT_MAP_RADIUS_Y, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		//});
		//thread_lrf_l.join();
		//thread_lrf_u = std::thread([&]{
			if (sensor_num == 4){
				visualizeParticleWeight(img_lrf_u, particle, likelihood_lid2_u, movie_step, img_lrf_u.cols, img_lrf_u.rows, MAP_RES, cv::Point(0, 0), false);
			}
			else if (sensor_num == 3){
				img_lrf_u = cv::Mat::zeros(cv::Size(map_img_color.cols, map_img_color.rows), CV_8UC3);
			}
			rectMap(img_lrf_u, img_lrf_u, center, CUT_MAP_RADIUS_X, CUT_MAP_RADIUS_Y, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		//});
		//thread_lrf_u.join();
		//thread_omni = std::thread([&]{
			visualizeParticleWeight(img_omni, particle, likelihood_omni, movie_step, img_omni.cols, img_omni.rows, MAP_RES, cv::Point(0, 0), false);
			rectMap(img_omni, img_omni, center, CUT_MAP_RADIUS_X, CUT_MAP_RADIUS_Y, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		//});
		//thread_omni.join();
		//thread_gpgga = std::thread([&]{
			visualizeParticleWeight(img_gps, particle, likelihood_gps, movie_step, img_gps.cols, img_gps.rows, MAP_RES, cv::Point(0, 0), false);
			rectMap(img_gps, img_gps, center, CUT_MAP_RADIUS_X, CUT_MAP_RADIUS_Y, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		//});
		//thread_gpgga.join();
		//thread_fusion = std::thread([&]{
			visualizeParticleWeight(img_fusion, particle, likelihood_fusion, movie_step, img_fusion.cols, img_fusion.rows, MAP_RES, cv::Point(0, 0), false);
			drawPosition(estimated_position[movie_step], img_fusion, IMAGE_ESTIPOSITION_COLOR, img_fusion.cols, img_fusion.rows, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y, cv::Point(0, 0), ESTIMATED_POSITION);
			rectMap(img_fusion, img_fusion, center, CUT_MAP_RADIUS_X, CUT_MAP_RADIUS_Y, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		//});
		//thread_fusion.join();
		//thread_all = std::thread([&]{
			visualizeParticleWeight(img_all, particle, likelihood_all, movie_step, img_all.cols, img_all.rows, MAP_RES, cv::Point(0, 0), false);
			rectMap(img_all, img_all, center, CUT_MAP_RADIUS_X, CUT_MAP_RADIUS_Y, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		//});

		//thread_all.join();

		// 上下反転
		cv::flip(img_lrf_l, img_lrf_l, 0);
		cv::flip(img_lrf_u, img_lrf_u, 0);
		cv::flip(img_omni, img_omni, 0);
		cv::flip(img_gps, img_gps, 0);
		cv::flip(img_fusion, img_fusion, 0);
		cv::flip(img_all, img_all, 0);


		/* 時刻表記 */
		if (sensor_num == 4){
			if (use_sim_[movie_step][0] == true){
				cv::putText(img_lrf_l, "l_l" + result_time[movie_step].str() + " Use", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 255, 0), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			}
			else{
				cv::putText(img_lrf_l, "l_l" + result_time[movie_step].str() + " Eli", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 0, 255), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			}
			if (use_sim_[movie_step][1] == true){
				cv::putText(img_lrf_u, "l_u" + result_time[movie_step].str() + " Use", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 255, 0), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			}
			else{
				cv::putText(img_lrf_u, "l_u" + result_time[movie_step].str() + " Eli", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 0, 255), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			}
			if (use_sim_[movie_step][2] == true){
				cv::putText(img_omni, "c" + result_time[movie_step].str() + " Use", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 255, 0), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			}
			else{
				cv::putText(img_omni, "c" + result_time[movie_step].str() + " Eli", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 0, 255), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			}
			if (use_sim_[movie_step][3] == true){
				cv::putText(img_gps, "g" + result_time[movie_step].str() + " Use", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 255, 0), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			}
			else{
				cv::putText(img_gps, "g" + result_time[movie_step].str() + " Eli", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 0, 255), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			}
		}
		else if (sensor_num == 3){
			if (use_sim_[movie_step][0] == true){
				cv::putText(img_lrf_l, "l_l" + result_time[movie_step].str() + " Use", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 255, 0), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			}
			else{
				cv::putText(img_lrf_l, "l_l" + result_time[movie_step].str() + " Eli", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 0, 255), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			}
			if (use_sim_[movie_step][1] == true){
				cv::putText(img_omni, "c" + result_time[movie_step].str() + " Use", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 255, 0), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			}
			else{
				cv::putText(img_omni, "c" + result_time[movie_step].str() + " Eli", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 0, 255), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			}
			if (use_sim_[movie_step][2] == true){
				cv::putText(img_gps, "g" + result_time[movie_step].str() + " Use", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 255, 0), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			}
			else{
				cv::putText(img_gps, "g" + result_time[movie_step].str() + " Eli", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 0, 255), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			}
		}

		if (std::find(use_sim_[movie_step].begin(), use_sim_[movie_step].end(), true) == use_sim_[movie_step].end()) {
			cv::putText(img_fusion, "Fu" + result_time[movie_step].str() + " Use All", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		}
		else{
			cv::putText(img_fusion, "Fu" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		}
		cv::putText(img_all, "All" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);

		int w = img_lrf_l.cols;
		int h = img_lrf_l.rows;

		cv::Mat img(cv::Size(w*3.0, h*2.0), CV_8UC3, cv::Scalar(0, 0, 0));

		img_lrf_l.copyTo(img(cv::Rect(cv::Point(0, 0), cv::Size(w, h))));
		img_lrf_u.copyTo(img(cv::Rect(cv::Point(w, 0), cv::Size(w, h))));
		img_omni.copyTo(img(cv::Rect(cv::Point(0, h), cv::Size(w, h))));
		img_gps.copyTo(img(cv::Rect(cv::Point(w, h), cv::Size(w, h))));
		img_fusion.copyTo(img(cv::Rect(cv::Point(2 * w, 0), cv::Size(w, h))));
		img_all.copyTo(img(cv::Rect(cv::Point(2 * w, h), cv::Size(w, h))));

		cv::resize(img, img, cv::Size(), MOVIE_SCALE_W, MOVIE_SCALE_H);


		//writeWeightedStatParAllImg(img);

		if (stat_){
			weighted_stat_particle_video << img;
			weighted_stat_particle_img = img;
		}
		else{
			particle_video << img;
			particle_img = img;
		}



	}
	void addFlameParticleLargeVideo()
	{
		std::vector<Position<>> particle;
		std::vector<double> likelihood_lid2_l, likelihood_lid2_u, likelihood_omni, likelihood_gps, likelihood_fusion, likelihood_all;
		particle = all_particles[movie_step];
		likelihood_lid2_l = all_lid2_l_likelihood[movie_step];
		likelihood_lid2_u = all_lid2_u_likelihood[movie_step];
		likelihood_omni = all_omni_likelihood[movie_step];
		likelihood_gps = all_gpgga_likelihood[movie_step];
		likelihood_fusion = all_fusion_likelihood[movie_step];
		likelihood_all = std::vector<double>(particle.size(), 1.0);

		/* 切り出し中心 */
		Position<> center;
		if (exist_true_position){
			center = getTruePosition(movie_step);
		}
		else{
			center = estimated_position[movie_step];
		}

		// 描画と切り出し
		cv::Mat mat = map_img_color.clone();
		cv::Point upleft_pix = rectMap(mat, mat, center, CUT_LARGE_MAP_RADIUS_X, CUT_LARGE_MAP_RADIUS_Y, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		double scale = CUT_MAP_RADIUS_X / CUT_LARGE_MAP_RADIUS_X;
		cv::resize(mat, mat, cv::Size(), scale, scale);
		upleft_pix.x *= scale;
		upleft_pix.y *= scale;
		int cols = map_img_color.cols*scale;
		int rows = map_img_color.rows*scale;
		float map_res = MAP_RES / scale;
		int circle_pix = 1000 / map_res;
		cv::circle(mat, cv::Point(mat.cols / 2.0, mat.rows / 2.0), circle_pix, cv::Scalar(0, 0, 0), 1, 8, 0);

		cv::Mat img_lrf_l, img_lrf_u, img_omni, img_gps, img_fusion, img_all;
		img_lrf_l = mat.clone();
		img_lrf_u = mat.clone();
		img_omni = mat.clone();
		img_gps = mat.clone();
		img_fusion = mat.clone();
		img_all = mat.clone();

		std::thread thread_lrf_l, thread_lrf_u, thread_omni, thread_gpgga, thread_fusion, thread_all;
		thread_lrf_l = std::thread([&]{
			visualizeParticleWeight(img_lrf_l, particle, likelihood_lid2_l, movie_step, cols, rows, map_res, upleft_pix, false);
		});
		thread_lrf_u = std::thread([&]{
			if (sensor_num == 4){
				visualizeParticleWeight(img_lrf_u, particle, likelihood_lid2_u, movie_step, cols, rows, map_res, upleft_pix, false);
			}
			else if (sensor_num == 3){
				img_lrf_u = cv::Mat::zeros(cv::Size(img_lrf_u.cols, img_lrf_u.rows), CV_8UC3);
			}
		});
		thread_omni = std::thread([&]{
			visualizeParticleWeight(img_omni, particle, likelihood_omni, movie_step, cols, rows, map_res, upleft_pix, false);
		});
		thread_gpgga = std::thread([&]{
			visualizeParticleWeight(img_gps, particle, likelihood_gps, movie_step, cols, rows, map_res, upleft_pix, false);
		});
		thread_fusion = std::thread([&]{
			visualizeParticleWeight(img_fusion, particle, likelihood_fusion, movie_step, cols, rows, map_res, upleft_pix, false);
			drawPosition(estimated_position[movie_step], img_fusion, IMAGE_ESTIPOSITION_COLOR, cols, rows, map_res, MAP_IMG_ORG_X, MAP_IMG_ORG_Y, upleft_pix, ESTIMATED_POSITION);
		});
		thread_all = std::thread([&]{
			visualizeParticleWeight(img_all, particle, likelihood_all, movie_step, cols, rows, map_res, upleft_pix, false);
		});

		thread_lrf_l.join();
		thread_lrf_u.join();
		thread_omni.join();
		thread_gpgga.join();
		thread_fusion.join();
		thread_all.join();

		// 上下反転
		cv::flip(img_lrf_l, img_lrf_l, 0);
		cv::flip(img_lrf_u, img_lrf_u, 0);
		cv::flip(img_omni, img_omni, 0);
		cv::flip(img_gps, img_gps, 0);
		cv::flip(img_fusion, img_fusion, 0);
		cv::flip(img_all, img_all, 0);


		/* 時刻表記 */
		if (sensor_num == 4){
			if (use_sim_[movie_step][0] == true){
				cv::putText(img_lrf_l, "l_l" + result_time[movie_step].str() + " Use", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 255, 0), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			}
			else{
				cv::putText(img_lrf_l, "l_l" + result_time[movie_step].str() + " Eli", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 0, 255), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			}
			if (use_sim_[movie_step][1] == true){
				cv::putText(img_lrf_u, "l_u" + result_time[movie_step].str() + " Use", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 255, 0), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			}
			else{
				cv::putText(img_lrf_u, "l_u" + result_time[movie_step].str() + " Eli", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 0, 255), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			}
			if (use_sim_[movie_step][2] == true){
				cv::putText(img_omni, "c" + result_time[movie_step].str() + " Use", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 255, 0), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			}
			else{
				cv::putText(img_omni, "c" + result_time[movie_step].str() + " Eli", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 0, 255), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			}
			if (use_sim_[movie_step][3] == true){
				cv::putText(img_gps, "g" + result_time[movie_step].str() + " Use", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 255, 0), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			}
			else{
				cv::putText(img_gps, "g" + result_time[movie_step].str() + " Eli", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 0, 255), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			}
		}
		else if (sensor_num == 3){
			if (use_sim_[movie_step][0] == true){
				cv::putText(img_lrf_l, "l_l" + result_time[movie_step].str() + " Use", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 255, 0), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			}
			else{
				cv::putText(img_lrf_l, "l_l" + result_time[movie_step].str() + " Eli", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 0, 255), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			}
			if (use_sim_[movie_step][1] == true){
				cv::putText(img_omni, "c" + result_time[movie_step].str() + " Use", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 255, 0), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			}
			else{
				cv::putText(img_omni, "c" + result_time[movie_step].str() + " Eli", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 0, 255), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			}
			if (use_sim_[movie_step][2] == true){
				cv::putText(img_gps, "g" + result_time[movie_step].str() + " Use", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 255, 0), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			}
			else{
				cv::putText(img_gps, "g" + result_time[movie_step].str() + " Eli", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 0, 255), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			}
		}
		if (std::find(use_sim_[movie_step].begin(), use_sim_[movie_step].end(), true) == use_sim_[movie_step].end()) {
			cv::putText(img_fusion, "Fu" + result_time[movie_step].str() + " Use All", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		}
		else{
			cv::putText(img_fusion, "Fu" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		}
		cv::putText(img_all, "All" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);

		int w = img_lrf_l.cols;
		int h = img_lrf_l.rows;

		cv::Mat img(cv::Size(w*3.0, h*2.0), CV_8UC3, cv::Scalar(0, 0, 0));

		img_lrf_l.copyTo(img(cv::Rect(cv::Point(0, 0), cv::Size(w, h))));
		img_lrf_u.copyTo(img(cv::Rect(cv::Point(w, 0), cv::Size(w, h))));
		img_omni.copyTo(img(cv::Rect(cv::Point(0, h), cv::Size(w, h))));
		img_gps.copyTo(img(cv::Rect(cv::Point(w, h), cv::Size(w, h))));
		img_fusion.copyTo(img(cv::Rect(cv::Point(2 * w, 0), cv::Size(w, h))));
		img_all.copyTo(img(cv::Rect(cv::Point(2 * w, h), cv::Size(w, h))));

		cv::resize(img, img, cv::Size(), MOVIE_SCALE_W, MOVIE_SCALE_H);


		//writeWeightedStatParAllImg(img);

		particle_large_video << img;
		particle_large_img = img;


	}


	//
	inline int minAllMeasurementData() {
		int min;
		switch (trial_type)
		{
		case TRIAL_SIMULTANEOUS:
		case TRIAL_PEARSON:
		case TRIAL_PEARSON_NONSTAT:
		case TRIAL_NON_TIMESEQUENCE:
		case TRIAL_NON_TIMESEQUENCE_SIMUL:
		case TRIAL_SUYAMA_STAT:
		case TRIAL_SUYAMA_NONSTAT:
			min = std::min({
				all_meas_odometry.size(), all_meas_time.size(), all_meas_lrf_l.size(), all_meas_lrf_u.size(),
				all_meas_keypoints.size(), all_meas_desriptor.size(), all_meas_gps.size(), all_meas_gps_pos.size(), estimated_position.size(),
				all_omni_img_sim_pos.size(), all_omni_img_sim.size(),
				all_stock_tidx.size(), diff_time_ini_now.size(),
			});
			break;
		case TRIAL_3SENSORS_SIMULATNEOUS:
		case TRIAL_3SENSORS_PEARSON:
		case TRIAL_3SENSORS_PEARSON_NONSTAT:
		case TRIAL_3SENSORS_SUYAMA_NONSTAT:
		case TRIAL_3SENSORS_LRF_GPS:
			min = std::min({
				all_meas_odometry.size(), all_meas_time.size(), all_meas_lrf_l.size(),
				all_meas_keypoints.size(), all_meas_desriptor.size(), all_meas_gps.size(), all_meas_gps_pos.size(), estimated_position.size(),
				all_omni_img_sim_pos.size(), all_omni_img_sim.size(),
				all_stock_tidx.size(), diff_time_ini_now.size(),
			});
			break;
		default:
			break;
		}

		return min;

	}

	//
	inline int minAllParticleLikelihood() {
		int min;
		switch (trial_type)
		{
		case TRIAL_SIMULTANEOUS:
		case TRIAL_PEARSON:
		case TRIAL_PEARSON_NONSTAT:
		case TRIAL_NON_TIMESEQUENCE:
		case TRIAL_NON_TIMESEQUENCE_SIMUL:
		case TRIAL_SUYAMA_STAT:
		case TRIAL_SUYAMA_NONSTAT:
			min = std::min({
				estimated_position.size(),
				all_particles.size(), all_lid2_l_likelihood.size(), all_lid2_u_likelihood.size(), all_omni_likelihood.size(),
				all_gpgga_likelihood.size(), all_fusion_likelihood.size(),
				all_stock_tidx.size(), diff_time_ini_now.size(),
			});
			break;
		case TRIAL_3SENSORS_SIMULATNEOUS:
		case TRIAL_3SENSORS_PEARSON:
		case TRIAL_3SENSORS_PEARSON_NONSTAT:
		case TRIAL_3SENSORS_SUYAMA_NONSTAT:
		case TRIAL_3SENSORS_LRF_GPS:
			min = std::min({
				estimated_position.size(),
				all_particles.size(), all_lid2_l_likelihood.size(), all_omni_likelihood.size(),
				all_gpgga_likelihood.size(), all_fusion_likelihood.size(),
				all_stock_tidx.size(), diff_time_ini_now.size(),
			});
			break;
		default:
			break;
		}

		return min;
	}


	//
	inline int minAllDataSize() {
		int min;
		switch (trial_type)
		{
		case TRIAL_SIMULTANEOUS:
		case TRIAL_PEARSON:
		case TRIAL_PEARSON_NONSTAT:
		case TRIAL_NON_TIMESEQUENCE:
		case TRIAL_NON_TIMESEQUENCE_SIMUL:
		case TRIAL_SUYAMA_STAT:
		case TRIAL_SUYAMA_NONSTAT:
			min = std::min({
				all_meas_odometry.size(), all_meas_time.size(), all_meas_lrf_l.size(), all_meas_lrf_u.size(),
				all_meas_keypoints.size(), all_meas_desriptor.size(), all_meas_gps.size(), all_meas_gps_pos.size(), estimated_position.size(),
				all_particles.size(), all_stat_particles.size(), all_lid2_l_likelihood.size(), all_lid2_u_likelihood.size(), all_omni_likelihood.size(),
				all_gpgga_likelihood.size(), all_fusion_likelihood.size(), all_stat_lid2_l_likelihood.size(), all_stat_lid2_u_likelihood.size(),
				all_stat_omni_likelihood.size(), all_stat_gpgga_likelihood.size(),
				all_omni_img_sim_pos.size(), all_omni_img_sim.size(),
				all_stock_tidx.size(), diff_time_ini_now.size(),
			});
			break;
		case TRIAL_3SENSORS_SIMULATNEOUS:
		case TRIAL_3SENSORS_PEARSON:
		case TRIAL_3SENSORS_PEARSON_NONSTAT:
		case TRIAL_3SENSORS_SUYAMA_NONSTAT:
		case TRIAL_3SENSORS_LRF_GPS:
			min = std::min({
				all_meas_odometry.size(), all_meas_time.size(), all_meas_lrf_l.size(),
				all_meas_keypoints.size(), all_meas_desriptor.size(), all_meas_gps.size(), all_meas_gps_pos.size(), estimated_position.size(),
				all_particles.size(), all_stat_particles.size(), all_lid2_l_likelihood.size(), all_omni_likelihood.size(),
				all_gpgga_likelihood.size(), all_fusion_likelihood.size(), all_stat_lid2_l_likelihood.size(),
				all_stat_omni_likelihood.size(), all_stat_gpgga_likelihood.size(),
				all_omni_img_sim_pos.size(), all_omni_img_sim.size(),
				all_stock_tidx.size(), diff_time_ini_now.size(),
			});
			break;
		default:
			break;
		}


		return min;
	}


	void debug(){
		if (!MODE_TEST){
			return;
		}
		omni[0].debug(no, now_step);
		cv::Mat img = particle_img;
		cv::resize(img, img, cv::Size(), 0.5, 0.5);
		cv::imshow("particle", img);
		cv::Mat measure_img = measurement_data_img;
		cv::resize(measure_img, measure_img, cv::Size(), 0.5, 0.5);
		cv::imshow("measure_img", measure_img);
		cv::Mat weighted_img;
		if (weighted_stat_particle_img.empty()){
			weighted_img = cv::Mat::zeros(300, 300, CV_8UC1);
		}
		else{
			weighted_img = weighted_stat_particle_img;
		}
		cv::resize(weighted_img, weighted_img, cv::Size(), 0.5, 0.5);
		cv::imshow("weighted_par", weighted_img);
		cv::waitKey();
	}



public:

	/*  パーティクルを初期化  */
	//	sample_size数のパーティクルを生成
	//	ini_positionの半径sample_range/2の範囲にパーティクルを散布
	void initPF()
	{
		getParticles().clear();

		Position<>	lowerbound = ini_position - ini_sample_radius;
		Position<>	upperbound = ini_position + ini_sample_radius;

		std::cout << lowerbound << std::endl;
		std::cout << upperbound << std::endl;

		std::random_device rnd;     // 非決定的な乱数生成器を生成
		std::mt19937_64 mt(rnd());     //  メルセンヌ・ツイスタの64ビット版、引数は初期シード値
		std::uniform_real_distribution<double> noise_x(lowerbound.x, upperbound.x);	//パーティクルの散布範囲
		std::uniform_real_distribution<double> noise_y(lowerbound.y, upperbound.y);	//パーティクルの散布範囲
		std::uniform_real_distribution<double> noise_r(lowerbound.r, upperbound.r);	//パーティクルの散布範囲

		for (int si = 0; si < sample_size; si++)
		{
			Particle<Position<>>* par = new Particle<Position<>>;	//	パーティクルの領域確保
			Position<>* position = new Position<>;	//	Stateの領域確保

			while (1)
			{
				position->set(noise_x(mt), noise_y(mt), noise_r(mt));	//	指定された範囲にパーティクルを散布
				//	存在可能領域に散布された場合，break
				cv::Point pixel = ToPixel(*position, map_img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
				if ((int)map_img.at<unsigned char>(pixel) != 0)	break;
			}

			par->setState(position);
			getParticles().push_back(par);	//	パーティクルを追加
		}

		getParticles().shrink_to_fit();	//	領域を最適化

		std::cout << "Complete 'Initialization' " << std::endl;
	};

	void sampling(Position<> center) {

		getParticles().clear();

		Position<>	lowerbound = center - ini_sample_radius;
		Position<>	upperbound = center + ini_sample_radius;

		std::random_device rnd;     // 非決定的な乱数生成器を生成
		std::mt19937_64 mt(rnd());     //  メルセンヌ・ツイスタの64ビット版、引数は初期シード値
		std::uniform_real_distribution<double> noise_x(lowerbound.x, upperbound.x);	//パーティクルの散布範囲
		std::uniform_real_distribution<double> noise_y(lowerbound.y, upperbound.y);	//パーティクルの散布範囲
		std::uniform_real_distribution<double> noise_r(lowerbound.r, upperbound.r);	//パーティクルの散布範囲

		for (int si = 0; si < sample_size; si++)
		{
			Particle<Position<>>* par = new Particle<Position<>>;	//	パーティクルの領域確保
			Position<>* position = new Position<>;	//	Stateの領域確保

			while (1)
			{
				position->set(noise_x(mt), noise_y(mt), noise_r(mt));	//	指定された範囲にパーティクルを散布
				//	存在可能領域に散布された場合，break
				cv::Point pixel = ToPixel(*position, map_img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
				if ((int)map_img.at<unsigned char>(pixel) == 255)	break;
				//break;
			}

			par->setState(position);
			getParticles().push_back(par);	//	パーティクルを追加
		}

		getParticles().shrink_to_fit();	//	領域を最適化

		std::cout << "Complete 'Sampling' " << std::endl;

	}


/*	void sampling(Position<> center) {

		getParticles().clear();

		std::random_device rnd;     // 非決定的な乱数生成器を生成
		std::mt19937_64 mt(rnd());     //  メルセンヌ・ツイスタの64ビット版、引数は初期シード値
		std::uniform_real_distribution<double> noise_range(0, INI_SAMPLE_RADIUS_RANGE);
		std::uniform_real_distribution<double> noise_theta(-M_PI, M_PI);
		std::uniform_real_distribution<double> noise_r(-INI_SAMPLE_RADIUS_R, INI_SAMPLE_RADIUS_R);	//パーティクルの散布範囲

		for (int si = 0; si < sample_size; si++)
		{
			Particle<Position<>>* par = new Particle<Position<>>;	//	パーティクルの領域確保
			Position<>* position = new Position<>;	//	Stateの領域確保

			while (1)
			{
				double range = noise_range(mt);
				double theta = noise_theta(mt);
				double r = noise_r(mt);

				double x = range*std::cos(theta) + center.x;
				double y = range*std::sin(theta) + center.y;
				double rr = r + center.r;
				
				position->set(x, y, rr);	//	指定された範囲にパーティクルを散布
				//	存在可能領域に散布された場合，break
				cv::Point pixel = ToPixel(*position, map_img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
				if ((int)map_img.at<unsigned char>(pixel) != 0)	break;
			}

			par->setState(position);
			getParticles().push_back(par);	//	パーティクルを追加
		}

		getParticles().shrink_to_fit();	//	領域を最適化

		std::cout << "Complete 'Sampling' " << std::endl;

	}*/



	/*  推定位置を算出  */
	//	それぞれのパラメータの重み付き平均値
	void calcEPos()
	{
		Position<> epos(0.0, 0.0, 0.0);

		for (const auto& par : getParticles())
		{
			epos += *(par->getState())*par->getWeight();
		}

		estimated_position.push_back(epos);
		std::cout << "Complete calcEpos" << std::endl;
	}
	void calcError() {

		while (esti_time > true_time[tidx]) {
			tidx++;
		}

		std::cout << "esti_time: " << esti_time << ", true_time: " << true_time[tidx] << std::endl;
		if (esti_time != true_time[tidx]){
			std::cout << "Skip calcError" << std::endl;
			return;
		}
		Position<> error = true_position->at(tidx) - estimated_position.back();
		all_error.push_back(error);
		error_time.push_back(true_time[tidx]);

		std::cout << "Time: " << true_time[tidx] << ", Error: " << error << std::endl;

		if (tidx + 1 >= true_position->size()) {
			finish = true;
		}
		std::cout << "Complete calcError" << std::endl;
	}

	Position<> maxParameters(std::vector<Particle<Position<>>*> particles) {
		Position<> max = *particles.front()->getState();
		for (int i = 1; i < particles.size(); i++) {
			if (max.x < particles[i]->getState()->x) {
				max.x = particles[i]->getState()->x;
			}
			if (max.y < particles[i]->getState()->y) {
				max.y = particles[i]->getState()->y;
			}
			if (max.r < particles[i]->getState()->r) {
				max.r = particles[i]->getState()->r;
			}
		}
		return max;
	}

	Position<> minParameters(std::vector<Particle<Position<>>*> particles) {
		Position<> min = *particles.front()->getState();
		for (int i = 1; i < particles.size(); i++) {
			if (min.x > particles[i]->getState()->x) {
				min.x = particles[i]->getState()->x;
			}
			if (min.y > particles[i]->getState()->y) {
				min.y = particles[i]->getState()->y;
			}
			if (min.r > particles[i]->getState()->r) {
				min.r = particles[i]->getState()->r;
			}
		}
		return min;
	}


	/*  パーティクルが障害物map上で存在不可能領域にあるときtrue  */
	template <typename T> bool onObject_(T tmp)
	{
		cv::Point pixel = ToPixel(tmp, map_img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	障害物地図上のパーティクル存在するピクセル

		if (pixel.x < 0 || pixel.x >= map_img.cols ||
			pixel.y < 0 || pixel.y >= map_img.rows ||
			(int)map_img.at<unsigned char>(pixel) != 255)	//	存在不可能領域の条件式
		{
			return true;
		}
		return false;
	}


	/**********************************************************/
	//	Grid Map 上に描画
	/**********************************************************/

protected:

	/*  重みなしパーティクルを障害物地図上に表示  */
	virtual void visualizeParticle()
	{
		cv::cvtColor(map_img_clone, map_img_clone, CV_BGR2HSV);
		for (const auto &particle : getParticles())
		{
			cv::Point particle_pixel = ToPixel(*particle->getState(), map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
			if (particle_pixel.x<0 || particle_pixel.x>map_img.cols ||
				particle_pixel.y<0 || particle_pixel.y>map_img.rows ||
				map_img.at<unsigned char>(particle_pixel) == 0)
			{
				continue;
			}
			cv::Scalar color = cv::Scalar(180, (int)(0.015 / 0.02*255.0 + 0.5), 255);
			cv::circle(map_img_clone, particle_pixel, 2, color, -1, 8, 0);	//	パーティクルの描画
		}
		cv::cvtColor(map_img_clone, map_img_clone, CV_HSV2BGR); // 

	}


	/*  軌跡を描画  */
	void visualizeEstiRoute()
	{
		/* 色設定 */
		cv::Scalar color;
		switch (trial_type)
		{
		case TRIAL_SIMULTANEOUS:
		case TRIAL_3SENSORS_SIMULATNEOUS:
			color = cv::Scalar(255, 0, 255);
			break;
		case TRIAL_PEARSON:
		case TRIAL_PEARSON_NONSTAT:
		case TRIAL_NON_TIMESEQUENCE:
		case TRIAL_NON_TIMESEQUENCE_SIMUL:
		case TRIAL_SUYAMA_STAT:
		case TRIAL_SUYAMA_NONSTAT:
		case TRIAL_3SENSORS_PEARSON:
		case TRIAL_3SENSORS_PEARSON_NONSTAT:
		case TRIAL_3SENSORS_SUYAMA_NONSTAT:
		case TRIAL_3SENSORS_LRF_GPS:
			color = cv::Scalar(255, 0, 0);
			break;
		default:
			break;
		}




		/*  推定位置の軌跡を描画  */
		cv::Point estimated_pixel1 = ToPixel(estimated_position.front(), map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		cv::Point estimated_pixel2;
		for (int si = 0; si < estimated_position.size() - 1; si++)
		{
			estimated_pixel2 = ToPixel(estimated_position[si + 1], map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			cv::line(map_img_clone, estimated_pixel1, estimated_pixel2, cv::Scalar(255, 0, 0), 2);
			estimated_pixel1 = estimated_pixel2;
		}

	};

	//void visualizeTrueRoute()
	//{
	//	/*  真の位置の軌跡を描画  */
	//	cv::Point true_pixel1 = ToPixel(true_position->front(), map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
	//	cv::Point true_pixel2;
	//	for (int si = 0; si < true_position->size() - 1; si++)
	//	//for (int si = 0; si < true_position->size() - 20; si++)
	//		{
	//		true_pixel2 = ToPixel(true_position->at(si + 1), map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
	//		cv::circle(map_img_clone, true_pixel2, 3, cv::Scalar(0, 255, 0), -1, 8, 0);	//	パーティクルの描画
	//		//cv::line(map_img_clone, true_pixel1, true_pixel2, cv::Scalar(0, 255, 0), 2);
	//		true_pixel1 = true_pixel2;
	//	}
	//}

	/*  円を描画  */
	void visualizeCercle()
	{
		//for (int i = 0; i < estimated_position.size(); i = i + IMG_CIRCLE_STEP)
		//{
		//	cv::Point epix = ToPixel(estimated_position[i], map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		//	cv::circle(map_img_clone, epix, 5, cv::Scalar(0, 255, 0), -1);
		//}
	}

	/*  真の軌跡を描画  */
	void visualizeTrueRoute()
	{
		/*  真の位置の軌跡を描画  */
		cv::Point true_pixel1 = ToPixel(true_position->front(), map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		cv::Point true_pixel2;
		for (int si = TRUE_IDX_INI - 1; si < tidx; si++)
			//for (int si = 0; si < true_position->size()-1; si++)
		{
			true_pixel2 = ToPixel(true_position->at(si + 1), map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			cv::circle(map_img_clone, true_pixel2, 3, cv::Scalar(0, 255, 0), -1, 8, 0);	//	パーティクルの描画
			//cv::line(map_img_clone, true_pixel1, true_pixel2, cv::Scalar(0, 255, 0), 2);
			true_pixel1 = true_pixel2;
		}
	}

	/* 矢印の描画 */
	void cvArrow(cv::Mat img, cv::Point pt1, cv::Point pt2, cv::Scalar color, int thickness = 1, int lineType = 8, int shift = 0) {
		cv::line(img, pt1, pt2, color, thickness, lineType, shift);
		//float vx = (float)(pt2.x - pt1.x);
		//float vy = (float)(pt2.y - pt1.y);
		//float v = sqrt(vx*vx + vy*vy);
		//float ux = vx / v;
		//float uy = vy / v;
		////矢印の幅の部分
		//float w = 5, h = 10;
		//cv::Point ptl, ptr;
		//ptl.x = (int)((float)pt2.x - uy*w - ux*h);
		//ptl.y = (int)((float)pt2.y + ux*w - uy*h);
		//ptr.x = (int)((float)pt2.x + uy*w - ux*h);
		//ptr.y = (int)((float)pt2.y - ux*w - uy*h);
		////矢印の先端を描画する
		//cv::line(img, pt2, ptl, color, thickness, lineType, shift);
		//cv::line(img, pt2, ptr, color, thickness, lineType, shift);
	}

	/**********************************************************/
	//	ファイル出力
	/**********************************************************/

public:

	void writeParState(std::string OFPATH)
	{
		std::string filename = OFPATH + "Data/particle_state_last.csv";
		std::ofstream ofs_par(filename, std::ios_base::out);
		if (ofs_par.fail()) {
			writeError(filename);
		}
		for (const auto& par : getParticles())
		{
			ofs_par << *par->getState() << std::endl;
		}

		ofs_par.close();

		filename = OFPATH + "Data/size_of_esti_position.csv";
		std::ofstream ofs_step(filename, std::ios_base::out);
		ofs_step << std::to_string(estimated_position.size()) << std::endl;
		ofs_step.close();
	}

	void writeLastParState(std::string OFPATH) {
		std::string filename = OFPATH + "Data/particle_state_last.csv";
		std::ofstream ofs_par(filename, std::ios_base::out);
		if (ofs_par.fail()) {
			writeError(filename);
		}
		for (const auto& par : getParticles())
		{
			ofs_par << *par->getState() << std::endl;
		}

		ofs_par.close();

		filename = OFPATH + "Data/size_of_esti_position.csv";
		std::ofstream ofs_step(filename, std::ios_base::out);
		ofs_step << std::to_string(estimated_position.size()) << std::endl;
		ofs_step.close();
	}


	void writeSizeOfEstiPosition(std::string OFPATH) {
		std::string filename = OFPATH + "Data/size_of_esti_position.csv";
		std::ofstream ofs_step(filename, std::ios_base::out);
		ofs_step << std::to_string(estimated_position.size()) << std::endl;
		ofs_step.close();
	}

	/*  ロボットの軌跡を出力  */
	void writeRoute(std::string OFPATH)
	{
		map_img_clone = map_img_color.clone();	//	描画用のグリッドマップを初期化
		visualizeEstiRoute();	//	ロボットの通った軌跡を描画
		//visualizeCercle();
		visualizeTrueRoute();	//	ロボットの通った真の軌跡を描画
		cv::flip(map_img_clone, map_img_clone, 0);	//	上下反転

		/*  ファイル出力  */
		std::string filename = OFPATH + "Image/route.bmp";
		cv::imwrite(filename, map_img_clone);
		cv::imwrite("./output/route.bmp", map_img_clone);
	}

	/* 読み込みファイルナンバーの出力 */
	void writeLastInpugFileNumber(std::string OFPATH) {
		{
			std::string filename = OFPATH + "Data/last_input_file_number.csv";
			std::ofstream ofs(filename, std::ios_base::out);
			if (ofs.fail()) {
				writeError(filename);
			}
			ofs << "no,step" << std::endl;
			ofs << read_meas_no << "," << read_meas_step << std::endl;
			ofs.close();
		}
	}

	/*  推定位置をファイル出力  */
	void writeOutputEstiPosition(std::string OFPATH)
	{
		{
			std::string filename = OFPATH + "Data/estimated_position.csv";
			std::ofstream ofs_est(filename, std::ios_base::app);
			if (ofs_est.fail()) {
				writeError(filename);
			}
			ofs_est << esti_time << "," << MyTime::diff(ini_time, esti_time) << "," << estimated_position.back() << std::endl;
			ofs_est.close();
		}
		//{
		//	std::string filename = "./output/Data/estimated_position.csv";
		//	std::ofstream ofs_est(filename, std::ios_base::app);
		//	ofs_est << esti_time << "," << estimated_position.back() << "," << std::endl;
		//	ofs_est.close();
		//}
	}
	void writeOutputEstiPositionAll(std::string ofpath)
	{
		std::string filename = ofpath + "Data/estimated_position.csv";
		std::ofstream ofs_est(filename, std::ios_base::out);
		if (ofs_est.fail()) {
			writeError(filename);
		}
		if (estimated_position.size() != result_time.size() || estimated_position.size() != diff_time_ini_now.size()) {
			std::cout << "Size of 'result_time', 'diff_time_ini_now' and 'estimated_position' are not same!" << std::endl;
			exit(0);
		}
		ofs_est << "step,time,s,x,y,rad" << std::endl;
		for (int i = 0; i < estimated_position.size(); i++) {
			ofs_est << i << "," << result_time[i] << "," << diff_time_ini_now[i] << "," << estimated_position[i] << std::endl;
		}
		ofs_est.close();
		std::cout << "Complete Output Esti Position" << std::endl;
	}

	/*  */
	void writeUseSensor(std::string ofpath)
	{
		std::string filename = ofpath + "Data/use_sensor.csv";
		std::ofstream ofs(filename, std::ios_base::app);
		ofs << esti_time << "," << MyTime::diff(ini_time, esti_time) << "," << use_.back() << std::endl;
		ofs.close();
	}
	void writeUseSensorAll(std::string ofpath)
	{
		std::string filename = ofpath + "Data/use_sensor.csv";
		std::ofstream ofs(filename, std::ios_base::out);
		if (use_.size() != result_time.size() || use_.size() != diff_time_ini_now.size()) {
			std::cout << "Size of 'result_time', 'diff_time_ini_now' and 'use_' are not same!" << std::endl;
			exit(0);
		}
		ofs << "step,time,s,lrf_l,lrf_u,cmr,gps" << std::endl;
		for (int i = 0; i < use_.size(); i++) {
			ofs << i << "," << result_time[i] << "," << diff_time_ini_now[i] << "," << use_[i] << std::endl;
		}
		ofs.close();
		std::cout << "Complete Output Use Sensor" << std::endl;

		{
			std::string filename = ofpath + "Data/use_sensor_sim_.csv";
			std::ofstream ofs(filename, std::ios_base::out);
			if (use_.size() != result_time.size() || use_.size() != diff_time_ini_now.size()) {
				std::cout << "Size of 'result_time', 'diff_time_ini_now' and 'use_' are not same!" << std::endl;
				exit(0);
			}
			ofs << "step,time,s,lrf_l,lrf_u,cmr,gps" << std::endl;
			for (int i = 0; i < use_.size(); i++) {
				ofs << i << "," << result_time[i] << "," << diff_time_ini_now[i] << "," << use_sim_[i] << std::endl;
			}
			ofs.close();
			std::cout << "Complete Output Use Sensor" << std::endl;
		}

	}


	void writeEliminateSensorForAnalysis(std::string ofpath)
	{
		std::string filename = ofpath + "Data/eliminate_sensor_for_analysis.csv";
		std::ofstream ofs(filename, std::ios_base::app);
		ofs << esti_time << "," << MyTime::diff(ini_time, esti_time) << ",";
		for (int i = 0; i < sensor_num; i++) {
			if (use_.back()[i] == false) {
				ofs << std::to_string(sensor_num - i);
			}
			else {
				ofs << "-1";
			}
			if (i != sensor_num - 1) {
				ofs << ",";
			}
		}
		ofs << std::endl;
		ofs.close();
	}
	void writeEliminateSensorForAnalysisAll(std::string ofpath)
	{
		std::string filename = ofpath + "Data/eliminate_sensor_for_analysis.csv";
		std::ofstream ofs(filename, std::ios_base::out);
		if (ofs.fail()) {
			writeError(filename);
		}
		if (use_.size() != result_time.size() || use_.size() != diff_time_ini_now.size()) {
			std::cout << "Size of 'result_time', 'diff_time_ini_now' and 'use_' are not same!" << std::endl;
			exit(0);
		}

		switch (trial_type)
		{
		case TRIAL_SIMULTANEOUS:
		case TRIAL_PEARSON:
		case TRIAL_PEARSON_NONSTAT:
		case TRIAL_NON_TIMESEQUENCE:
		case TRIAL_NON_TIMESEQUENCE_SIMUL:
		case TRIAL_SUYAMA_STAT:
		case TRIAL_SUYAMA_NONSTAT:
			ofs << "step,time,s,lrf_l,lrf_u,cmr,gps" << std::endl;
			break;
		case TRIAL_3SENSORS_SIMULATNEOUS:
		case TRIAL_3SENSORS_PEARSON:
		case TRIAL_3SENSORS_PEARSON_NONSTAT:
		case TRIAL_3SENSORS_SUYAMA_NONSTAT:
		case TRIAL_3SENSORS_LRF_GPS:
			ofs << "step,time,s,lrf_l,cmr,gps" << std::endl;
			break;
		default:
			break;
		}

		for (int i = 0; i < use_.size(); i++) {

			ofs << i << "," << result_time[i] << "," << diff_time_ini_now[i] << ",";
			for (int j = 0; j < sensor_num; j++) {
				if (use_[i][j] == false) {
					ofs << std::to_string(sensor_num - j);
				}
				else {
					ofs << "-1";
				}
				if (j != sensor_num - 1) {
					ofs << ",";
				}
			}
			ofs << std::endl;
		}
		ofs.close();
		std::cout << "Complete writeEliminateSensorForAnalysisAll" << std::endl;

	}



	void writeSimilarityTable(std::string ofpath)
	{
		std::string filename = ofpath + "Data/gitignore/similarity_table/similarity_table_" + std::to_string(estimated_position.size()) + "_" + esti_time.filenameStr() + ".csv";
		std::ofstream ofs(filename, std::ios_base::out);
		if (ofs.fail()) writeError(filename);
		ofs << esti_time << std::endl;
		ofs << similarity_table.back();
		ofs << std::endl;
		ofs.close();
	}
	void writeSimilarityTableAll(std::string ofpath)
	{
		std::string filename = ofpath + "Data/gitignore/similarity_table.csv";
		std::ofstream ofs(filename, std::ios_base::out);
		if (ofs.fail()) writeError(filename);
		if (similarity_table.size() != result_time.size() || similarity_table.size() != diff_time_ini_now.size()) {
			std::cout << "Size of 'result_time', 'diff_time_ini_now' and 'similarity_table' are not same!" << std::endl;
			exit(0);
		}

		switch (trial_type)
		{
		case TRIAL_SIMULTANEOUS:
		case TRIAL_PEARSON:
		case TRIAL_PEARSON_NONSTAT:
		case TRIAL_NON_TIMESEQUENCE:
		case TRIAL_NON_TIMESEQUENCE_SIMUL:
		case TRIAL_SUYAMA_STAT:
		case TRIAL_SUYAMA_NONSTAT:
			ofs << "step,time,s,lrf_l,lrf_u,cmr,gps" << std::endl;
			break;
		case TRIAL_3SENSORS_SIMULATNEOUS:
		case TRIAL_3SENSORS_PEARSON:
		case TRIAL_3SENSORS_PEARSON_NONSTAT:
		case TRIAL_3SENSORS_SUYAMA_NONSTAT:
		case TRIAL_3SENSORS_LRF_GPS:
			ofs << "step,time,s,lrf_l,cmr,gps" << std::endl;
			break;
		default:
			break;
		}

		for (int i = 0; i < similarity_table.size(); i++) {
			for (int j = 0; j < similarity_table[i].size(); j++) {
				ofs << i << "," << result_time[i] << "," << diff_time_ini_now[i] << "," << similarity_table[i][j] << std::endl;
			}
		}
		ofs.close();
		std::cout << "Complete writeSimilarityTableAll" << std::endl;
	}

	void writeSimilarTable_(std::string ofpath)
	{
		std::string filename = ofpath + "Data/gitignore/similar_table_/similar_table_" + std::to_string(estimated_position.size()) + +"_" + esti_time.filenameStr() + ".csv";
		std::ofstream ofs(filename, std::ios_base::out);
		if (ofs.fail()) writeError(filename);
		ofs << similar_table_.back();
		ofs << std::endl;
		ofs.close();
	}
	void writeSimilarTableAll_(std::string ofpath)
	{
		std::string filename = ofpath + "Data/gitignore/similar_table_.csv";
		std::ofstream ofs(filename, std::ios_base::out);
		if (ofs.fail()) writeError(filename);
		if (similar_table_.size() != result_time.size() || similar_table_.size() != diff_time_ini_now.size()) {
			std::cout << "Size of 'result_time', 'diff_time_ini_now' and 'similar_table_' are not same!" << std::endl;
			exit(0);
		}

		switch (trial_type)
		{
		case TRIAL_SIMULTANEOUS:
		case TRIAL_PEARSON:
		case TRIAL_PEARSON_NONSTAT:
		case TRIAL_NON_TIMESEQUENCE:
		case TRIAL_NON_TIMESEQUENCE_SIMUL:
		case TRIAL_SUYAMA_STAT:
		case TRIAL_SUYAMA_NONSTAT:
			ofs << "step,time,s,lrf_l,lrf_u,cmr,gps" << std::endl;
			break;
		case TRIAL_3SENSORS_SIMULATNEOUS:
		case TRIAL_3SENSORS_PEARSON:
		case TRIAL_3SENSORS_PEARSON_NONSTAT:
		case TRIAL_3SENSORS_SUYAMA_NONSTAT:
		case TRIAL_3SENSORS_LRF_GPS:
			ofs << "step,time,s,lrf_l,cmr,gps" << std::endl;
			break;
		default:
			break;
		}

		for (int i = 0; i < similar_table_.size(); i++) {
			for (int j = 0; j < similar_table_[i].size(); j++) {
				ofs << i << "," << result_time[i] << "," << diff_time_ini_now[i] << "," << similar_table_[i][j] << std::endl;
			}
		}
		ofs.close();
		std::cout << "Complete writeSimilarTableAll_" << std::endl;
	}

	void writeOutputParticleAll(std::string ofpath) {
		std::string filename = ofpath + "Data/gitignore/particle.csv";
		std::ofstream ofs(filename, std::ios_base::out);
		if (ofs.fail()) {
			writeError(filename);
		}
		if (all_particles.size() != result_time.size() || all_particles.size() != diff_time_ini_now.size()) {
			std::cout << "Size of 'result_time', 'diff_time_ini_now' and 'all_particles' are not same!" << std::endl;
			exit(0);
		}
		else if (all_particles.size() != all_fusion_likelihood.size()) {
			std::cout << "Size of 'all_particles' and 'fusion_likelihood' are not same!" << std::endl;
			exit(0);
		}
		else if (all_particles.size() != all_lid2_l_likelihood.size()) {
			std::cout << "Size of 'all_particles' and 'lid2_l_likelihood' are not same!" << std::endl;
			exit(0);
		}
		else if (all_particles.size() != all_omni_likelihood.size()) {
			std::cout << "Size of 'all_particles' and 'omni_likelihood' are not same!" << std::endl;
			exit(0);
		}
		else if (all_particles.size() != all_gpgga_likelihood.size()) {
			std::cout << "Size of 'all_particles' and 'gpgga_likelihood' are not same!" << std::endl;
			exit(0);
		}

		switch (trial_type)
		{
		case TRIAL_SIMULTANEOUS:
		case TRIAL_PEARSON:
		case TRIAL_PEARSON_NONSTAT:
		case TRIAL_NON_TIMESEQUENCE:
		case TRIAL_NON_TIMESEQUENCE_SIMUL:
		case TRIAL_SUYAMA_STAT:
		case TRIAL_SUYAMA_NONSTAT:
			ofs << "step,time,s,x,y,rad,fusion,lrf_l,lrf_u,cmr,gps" << std::endl;
			for (int i = 0; i < all_particles.size(); i++) {
				for (int j = 0; j < all_particles[i].size(); j++) {
					ofs << i << "," << result_time[i] << "," << diff_time_ini_now[i] << "," << all_particles[i][j] << "," << all_fusion_likelihood[i][j] << "," << all_lid2_l_likelihood[i][j] << "," << all_lid2_u_likelihood[i][j] << "," << all_omni_likelihood[i][j] << "," << all_gpgga_likelihood[i][j] << std::endl;
				}
			}
			break;
		case TRIAL_3SENSORS_SIMULATNEOUS:
		case TRIAL_3SENSORS_PEARSON:
		case TRIAL_3SENSORS_PEARSON_NONSTAT:
		case TRIAL_3SENSORS_SUYAMA_NONSTAT:
		case TRIAL_3SENSORS_LRF_GPS:
			ofs << "step,time,s,x,y,rad,fusion,lrf_l,cmr,gps" << std::endl;
			for (int i = 0; i < all_particles.size(); i++) {
				for (int j = 0; j < all_particles[i].size(); j++) {
					ofs << i << "," << result_time[i] << "," << diff_time_ini_now[i] << "," << all_particles[i][j] << "," << all_fusion_likelihood[i][j] << "," << all_lid2_l_likelihood[i][j] << "," << all_omni_likelihood[i][j] << "," << all_gpgga_likelihood[i][j] << std::endl;
				}
			}
			break;
		default:
			break;
		}

		ofs.close();
		std::cout << "Complete writeOutputParticleAll" << std::endl;
	}
	void writeOutputParticleAllAfterResampling(std::string ofpath) {
		std::string filename = ofpath + "Data/gitignore/particle_after_resampling.csv";
		std::ofstream ofs(filename, std::ios_base::out);
		if (ofs.fail()) {
			writeError(filename);
		}
		if (all_particles_after_resampling.size() != result_time.size() || all_particles_after_resampling.size() != diff_time_ini_now.size()) {
			std::cout << "Size of 'result_time', 'diff_time_ini_now' and 'all_particles_after_resampling' are not same!" << std::endl;
			exit(0);
		}

		ofs << "step,time,s,x,y,rad" << std::endl;
		for (int i = 0; i < all_particles_after_resampling.size(); i++) {
			for (int j = 0; j < all_particles_after_resampling[i].size(); j++) {
				ofs << i << "," << result_time[i] << "," << diff_time_ini_now[i] << "," << all_particles_after_resampling[i][j] << std::endl;
			}
		}
		ofs.close();
		std::cout << "Complete writeOutputParticleAllAfterResampling" << std::endl;
	}

	void writeOutputStatParticleAll(std::string ofpath) {
		std::string filename = ofpath + "Data/gitignore/stat_particle.csv";
		std::ofstream ofs(filename, std::ios_base::out);
		if (ofs.fail()) {
			writeError(filename);
		}
		if (all_stat_particles.size() != result_time.size() || all_stat_particles.size() != diff_time_ini_now.size()) {
			std::cout << "Size of 'result_time', 'diff_time_ini_now' and 'all_stat_particles' are not same!" << std::endl;
			exit(0);
		}
		else if (all_stat_particles.size() != all_stat_lid2_l_likelihood.size()) {
			std::cout << "Size of 'all_stat_particles' and 'lid2_l_likelihood' are not same!" << std::endl;
			exit(0);
		}
		else if (all_stat_particles.size() != all_stat_omni_likelihood.size()) {
			std::cout << "Size of 'all_stat_particles' and 'omni_likelihood' are not same!" << std::endl;
			exit(0);
		}
		else if (all_stat_particles.size() != all_stat_gpgga_likelihood.size()) {
			std::cout << "Size of 'all_stat_particles' and 'gpgga_likelihood' are not same!" << std::endl;
			exit(0);
		}

		switch (trial_type)
		{
		case TRIAL_SIMULTANEOUS:
		case TRIAL_PEARSON:
		case TRIAL_PEARSON_NONSTAT:
		case TRIAL_NON_TIMESEQUENCE:
		case TRIAL_NON_TIMESEQUENCE_SIMUL:
		case TRIAL_SUYAMA_STAT:
		case TRIAL_SUYAMA_NONSTAT:
			ofs << "step,time,s,x,y,rad,fusion,lrf_l,lrf_u,cmr,gps" << std::endl;
			for (int i = 0; i < all_stat_particles.size(); i++) {
				for (int j = 0; j < all_stat_particles[i].size(); j++) {
					ofs << i << "," << result_time[i] << "," << diff_time_ini_now[i] << "," << all_stat_particles[i][j] << "," << all_stat_lid2_l_likelihood[i][j] << "," << all_stat_lid2_u_likelihood[i][j] << "," << all_stat_omni_likelihood[i][j] << "," << all_stat_gpgga_likelihood[i][j] << std::endl;
				}
			}
			break;
		case TRIAL_3SENSORS_SIMULATNEOUS:
		case TRIAL_3SENSORS_PEARSON:
		case TRIAL_3SENSORS_PEARSON_NONSTAT:
		case TRIAL_3SENSORS_SUYAMA_NONSTAT:
		case TRIAL_3SENSORS_LRF_GPS:
			ofs << "step,time,s,x,y,rad,lrf_l,cmr,gps" << std::endl;
			for (int i = 0; i < all_stat_particles.size(); i++) {
				for (int j = 0; j < all_stat_particles[i].size(); j++) {
					ofs << i << "," << result_time[i] << "," << diff_time_ini_now[i] << "," << all_stat_particles[i][j] << "," << all_stat_lid2_l_likelihood[i][j] << "," << all_stat_omni_likelihood[i][j] << "," << all_stat_gpgga_likelihood[i][j] << std::endl;
				}
			}
			break;
		default:
			break;
		}

		ofs.close();
		std::cout << "Complete writeOutputParticleAll" << std::endl;
	}



	/*  推定誤差の出力  */
	void writeOutputEstiError(std::string OFPATH)
	{
		while (esti_time > true_time[tidx]) {
			tidx++;
		}

		std::cout << "esti_time: " << esti_time << ", true_time: " << true_time[tidx] << std::endl;
		if (esti_time != true_time[tidx])	return;

		Position<> error = true_position->at(tidx) - estimated_position.back();
		all_error.push_back(error);
		error_time.push_back(true_time[tidx]);

		if (tidx + 1 >= true_position->size()) {
			finish = true;
		}

	}
	void writeOutputEstiErrorAll(std::string OFPATH)
	{
		std::string filename = OFPATH + "Data/error.csv";
		std::ofstream ofs_est(filename, std::ios_base::out);
		if (ofs_est.fail()) {
			writeError(filename);
		}
		if (all_error.size() != error_time.size()) {
			std::cout << "Size of 'error_time' and 'all_error' are not same!" << std::endl;
			exit(0);
		}

		Position<> error_sum(0.0, 0.0, 0.0);
		int sum_num = 0;

		ofs_est << "time,x,y,rad,abs" << std::endl;
		for (int i = 0; i < all_error.size(); i++) {
			error_sum += all_error[i];
			sum_num++;
			if (i == all_error.size() || error_time[i] != error_time[i + 1]){
				double abs = std::sqrt(std::pow(error_sum.x, 2) + std::pow(error_sum.y, 2)) / sum_num;
				ofs_est << error_time[i] << "," << error_sum / sum_num << "," << abs << std::endl;
				error_sum = Position<>(0.0, 0.0, 0.0);
				sum_num = 0;
			}
		}
		ofs_est.close();
		std::cout << "Complete writeOutputEstiErrorAll" << std::endl;

	}

	void writeOutputThAll(std::string ofpath){
		std::string filename = ofpath + "Data/similarity_th.csv";
		std::ofstream ofs(filename, std::ios_base::out);
		if (ofs.fail()) {
			writeError(filename);
		}
		if (all_th.size() != result_time.size()) {
			std::cout << "Size of 'result_time' and 'all_th' are not same!" << std::endl;
			exit(0);
		}

		for (int i = 0; i < all_th.size(); i++){
			ofs << all_th[i] << std::endl;
		}
		std::cout << "Complete writeOutputThAll" << std::endl;


	}

	void createMovie()
	{
		//std::thread thread_meas=std::thread(&LocalizationPF::addMeasurementVideo1,this);
		//std::thread thread_par = std::thread(&LocalizationPF::addFlameParticleVideo1,this);
		//std::thread thread_stat_par = std::thread(&LocalizationPF::addFlameWeightedStatParImg1,this);
		//thread_meas.join();
		//thread_par.join();
		//thread_stat_par.join();

		std::thread thread_meas;
		std::thread thread_par;
		std::thread thread_par_large;
		std::thread thread_stat_par;

		switch (trial_type)
		{
		case TRIAL_SIMULTANEOUS:
		case TRIAL_PEARSON_NONSTAT:
		case TRIAL_NON_TIMESEQUENCE:
		case TRIAL_NON_TIMESEQUENCE_SIMUL:
		case TRIAL_SUYAMA_NONSTAT:
		case TRIAL_3SENSORS_SIMULATNEOUS:
		case TRIAL_3SENSORS_PEARSON_NONSTAT:
		case TRIAL_3SENSORS_SUYAMA_NONSTAT:
		case TRIAL_3SENSORS_LRF_GPS:
			thread_meas = std::thread(&LocalizationPF::addMeasurementVideo, this);
			thread_meas.join();
			//std::cout << 1 << std::endl;
			thread_par = std::thread([&]{
				addFlameWeightedParImg(false);
			});
			thread_par.join();
			//std::cout << 1 << std::endl;
			thread_par_large = std::thread(&LocalizationPF::addFlameParticleLargeVideo, this);
			thread_par_large.join();
			//std::cout << 1 << std::endl;
			break;
		case TRIAL_PEARSON:
		case TRIAL_SUYAMA_STAT:
		case TRIAL_3SENSORS_PEARSON:
			thread_meas = std::thread(&LocalizationPF::addMeasurementVideo, this);
			thread_par = std::thread([&]{
				addFlameWeightedParImg(false);
			});
			thread_par_large = std::thread(&LocalizationPF::addFlameParticleLargeVideo, this);
			thread_stat_par = std::thread([&]{
				addFlameWeightedParImg(true);
			});
			thread_meas.join();
			thread_par.join();
			thread_stat_par.join();
			thread_par_large.join();
			break;

			std::cout << "Error 'TRIAL': " << TRIAL_TYPE << "FILE: " << __FILE__ << " LINE: " << __LINE__ << std::endl;
			exit(0);
			break;
		}


		//addMeasurementVideo2();
		//addFlameParticleVideo2();
		//addFlameWeightedStatParImg2();

		movie_step++;

		std::cout << "Complete createMovie" << std::endl;
	}


	/*  環境情報の読み込み  */

	void readLast()
	{
		//readParticle();

		//readNowStep();

		//readEstPos();

		//readUseType();

		//readOdometry(now_step);
		//swapOdometry();

		//readParticleVideo();

	}

	/*  開始ステップ番号の読み込み  */
	void readNowStep(std::string ofpath)
	{
		/*  読み込み  */
		std::string filename = ofpath + "Data/last_step.csv";
		std::ifstream ifs(filename);
		if (ifs.fail()) readError(filename);

		ifs >> now_step;

	}

	//void readUseType()
	//{
	//	/*  読み込み  */
	//	std::string filename = OFPATH_SIMUL + "Data/use_type.csv";
	//	std::ifstream ifs(filename);
	//	if (ifs.fail()) readError(filename);
	//	/*  1行スキップ  */
	//	std::string str;
	//	std::getline(ifs, str);
	//	while (std::getline(ifs, str))
	//	{
	//		std::istringstream istr(str);
	//		int tmp;
	//		istr >> tmp;
	//		use_type.push_back(tmp);
	//	}
	//}




	/**********************************************************/
	//	出力ファイルの初期化
	/**********************************************************/


	/**********************************************************/
	//  パーティクルフィルタ
	/**********************************************************/


	/*  パーティクルの尤度算出  */
	void prepareForCalcLikelihood(){
		/*  Omni Camera  */
		if (USE_BOF){
			if (estimated_position.empty())	{
				omni[0].calcSimilarityBetweenImages(ini_position, OMNI_FEATURE);
				omni[0].setMeasBofSift(ini_position);
			}
			else{
				omni[0].calcSimilarityBetweenImages(estimated_position.back(), OMNI_FEATURE);
				omni[0].setMeasBofSift(estimated_position.back());
			}
		}
		else{
			if (estimated_position.empty())	{
				omni[0].calcSimilarityBetweenImages(ini_position, OMNI_FEATURE);
			}
			else{
				omni[0].calcSimilarityBetweenImages(estimated_position.back(), OMNI_FEATURE);
			}
		}

		std::cout << omni[0].env_histgram.rows << std::endl;
		std::cout << omni[0].imgForCalcLikeAll_positioin.size() << std::endl;

		for (int i = 1; i < LIKELIHOOD_THREAD; i++) {
			omni[i] = omni[0];
		}
		all_omni_img_sim_pos.push_back(omni[0].imgForCalcLikeAll_positioin);
		all_omni_img_sim.push_back(omni[0].imgForCalcAll_dmatch_size);

	}

	void calcParticleLikelihood(){

		std::vector<Particle<Position<>>*>& get_particles = getParticles();
		std::vector<double>& lid2_l_likelihood_tmp = lid2_l_likelihood;
		std::vector<double>& lid2_u_likelihood_tmp = lid2_u_likelihood;
		std::vector<double>& omni_likelihood_tmp = omni_likelihood;
		std::vector<double>& gpgga_likelihood_tmp = gpgga_likelihood;
		int no_tmp = no;
		Position<> ini_position_tmp = ini_position;
		std::vector<Position<>> estimated_position_tmp = estimated_position;
		cv::Mat& map_img_tmp = map_img;


		switch (trial_type)
		{
		case TRIAL_SIMULTANEOUS:
		case TRIAL_PEARSON:
		case TRIAL_PEARSON_NONSTAT:
		case TRIAL_NON_TIMESEQUENCE:
		case TRIAL_NON_TIMESEQUENCE_SIMUL:
		case TRIAL_SUYAMA_STAT:
			// resize
			lid2_l_likelihood.resize(getParticles().size());
			lid2_u_likelihood.resize(getParticles().size());
			omni_likelihood.resize(getParticles().size());
			gpgga_likelihood.resize(getParticles().size());
			fusion_likelihood.resize(getParticles().size());


			for (int th = 0; th < LIKELIHOOD_THREAD; th++)
			{
				Lidar2d& lid2_l_tmp = lid2_l[th];
				Lidar2d& lid2_u_tmp = lid2_u[th];
				OmniCamera& omni_tmp = omni[th];
				Gps& gpgga_tmp = gpgga[th];
				likelihood_threads[th] = std::thread([th, &lid2_l_tmp, &lid2_u_tmp, &omni_tmp, &gpgga_tmp, &lid2_l_likelihood_tmp, &lid2_u_likelihood_tmp, &omni_likelihood_tmp, &gpgga_likelihood_tmp, no_tmp, ini_position_tmp, estimated_position_tmp, get_particles, map_img_tmp] {

					/*  Lidar2D  */
					lid2_l_tmp.setMeasICP();
					lid2_u_tmp.setMeasICP();

					/*  Omni Camera  */

					//if (estimated_position_tmp.empty())	omni_tmp.setMeasBofSift1(ini_position_tmp);
					//else
					//{
					//	omni_tmp.setMeasBofSift1(estimated_position_tmp.back());
					//}
					//omni_tmp.calcSimilarityBetweenImages(get_particles);


					/*  GPS  */
					gpgga_tmp.setMeasND();

					for (int pi = th; pi < SAMPLE_SIZE; pi += LIKELIHOOD_THREAD)
					{
						//cv::Point pixel = ToPixel(*get_particles[pi]->getState(), map_img_tmp, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	障害物地図上のパーティクル存在するピクセル
						//if (pixel.x < 0 || pixel.x >= map_img_tmp.cols ||
						//	pixel.y < 0 || pixel.y >= map_img_tmp.rows ||
						//	(int)map_img_tmp.at<unsigned char>(pixel) != 255)	//	存在不可能領域の条件式
						//{
						//	lid2_l_likelihood_tmp[pi] = 0.0;
						//	lid2_u_likelihood_tmp[pi] = 0.0;
						//	omni_likelihood_tmp[pi] = 0.0;
						//	gpgga_likelihood_tmp[pi] = 0.0;
						//	continue;
						//}
						/*  尤度算出  */
						assert(get_particles.size() == lid2_l_likelihood_tmp.size());
						assert(get_particles.size() == lid2_u_likelihood_tmp.size());
						assert(get_particles.size() == omni_likelihood_tmp.size());
						assert(get_particles.size() == gpgga_likelihood_tmp.size());

						//clock_t lap0 = clock();
						double lid2_l_w = lid2_l_tmp.getLikelihoodICP(*get_particles[pi]->getState(), ICP_PAIR_NUM_TH_L);
						//clock_t lap1 = clock();
						double lid2_u_w = lid2_u_tmp.getLikelihoodICP(*get_particles[pi]->getState(), ICP_PAIR_NUM_TH_U);
						//clock_t lap2 = clock();
						double omni_w, omni_w_rotate;
						if (USE_BOF){
							omni_w = omni_tmp.getLikelihoodBofSift(*get_particles[pi]->getState());
							//omni_w_rotate = omni_tmp.getLikelihoodRotate(*get_particles[pi]->getState(), OMNI_FEATURE);
						}
						else{
							omni_w = omni_tmp.getLikelihood(*get_particles[pi]->getState(), OMNI_FEATURE);
						}
						//clock_t lap3 = clock();
						double gps_w = gpgga_tmp.getLikelihoodND(*get_particles[pi]->getState());
						//clock_t lap4 = clock();
						lid2_l_likelihood_tmp[pi] = lid2_l_w;
						lid2_u_likelihood_tmp[pi] = lid2_u_w;
						omni_likelihood_tmp[pi] = omni_w;
						gpgga_likelihood_tmp[pi] = gps_w;
						//std::cout << (int)(lap1 - lap0) << "," << (int)(lap2 - lap1) << "," << (int)(lap3 - lap2) << "," << (int)(lap4 - lap3)  << std::endl;
					}

				});
			}

			for (int i = 0; i < LIKELIHOOD_THREAD; i++)
			{
				likelihood_threads[i].join();
			}


			/* 正規化 */
			if (ADD_BIAS){
				Normalize(lid2_l_likelihood);
				Normalize(lid2_u_likelihood);
				Normalize(omni_likelihood);
				Normalize(gpgga_likelihood);
				addBias(lid2_l_likelihood);
				addBias(lid2_u_likelihood);
				addBias(omni_likelihood);
				addBias(gpgga_likelihood);
				Normalize(lid2_l_likelihood);
				Normalize(lid2_u_likelihood);
				Normalize(omni_likelihood);
				Normalize(gpgga_likelihood);
			}
			else{
				Normalize(lid2_l_likelihood);
				Normalize(lid2_u_likelihood);
				Normalize(omni_likelihood);
				Normalize(gpgga_likelihood);
			}
			break;

		case TRIAL_SUYAMA_NONSTAT:
		case TRIAL_3SENSORS_SIMULATNEOUS:
		case TRIAL_3SENSORS_PEARSON:
		case TRIAL_3SENSORS_PEARSON_NONSTAT:
		case TRIAL_3SENSORS_SUYAMA_NONSTAT:
		case TRIAL_3SENSORS_LRF_GPS:
			// resize
			lid2_l_likelihood.resize(getParticles().size());
			omni_likelihood.resize(getParticles().size());
			gpgga_likelihood.resize(getParticles().size());
			fusion_likelihood.resize(getParticles().size());


			for (int th = 0; th < LIKELIHOOD_THREAD; th++)
			{
				Lidar2d& lid2_l_tmp = lid2_l[th];
				OmniCamera& omni_tmp = omni[th];
				Gps& gpgga_tmp = gpgga[th];
				likelihood_threads[th] = std::thread([th, &lid2_l_tmp, &omni_tmp, &gpgga_tmp, &lid2_l_likelihood_tmp, &omni_likelihood_tmp, &gpgga_likelihood_tmp, no_tmp, ini_position_tmp, estimated_position_tmp, get_particles, map_img_tmp] {

					/*  Lidar2D  */
					lid2_l_tmp.setMeasICP();

					/*  Omni Camera  */

					//if (estimated_position_tmp.empty())	omni_tmp.setMeasBofSift1(ini_position_tmp);
					//else
					//{
					//	omni_tmp.setMeasBofSift1(estimated_position_tmp.back());
					//}
					//omni_tmp.calcSimilarityBetweenImages(get_particles);


					/*  GPS  */
					gpgga_tmp.setMeasND();

					for (int pi = th; pi < SAMPLE_SIZE; pi += LIKELIHOOD_THREAD)
					{
						//cv::Point pixel = ToPixel(*get_particles[pi]->getState(), map_img_tmp, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	障害物地図上のパーティクル存在するピクセル
						//if (pixel.x < 0 || pixel.x >= map_img_tmp.cols ||
						//	pixel.y < 0 || pixel.y >= map_img_tmp.rows ||
						//	(int)map_img_tmp.at<unsigned char>(pixel) != 255)	//	存在不可能領域の条件式
						//{
						//	lid2_l_likelihood_tmp[pi] = 0.0;
						//	omni_likelihood_tmp[pi] = 0.0;
						//	gpgga_likelihood_tmp[pi] = 0.0;
						//	continue;
						//}
						/*  尤度算出  */
						assert(get_particles.size() == lid2_l_likelihood_tmp.size());
						assert(get_particles.size() == omni_likelihood_tmp.size());
						assert(get_particles.size() == gpgga_likelihood_tmp.size());

						//clock_t lap0 = clock();
						double lid2_l_w = lid2_l_tmp.getLikelihoodICP(*get_particles[pi]->getState(), ICP_PAIR_NUM_TH_L);
						//clock_t lap1 = clock();
						//clock_t lap2 = clock();
						double omni_w;
						if (USE_BOF){
							omni_w = omni_tmp.getLikelihoodBofSift(*get_particles[pi]->getState());
						}
						else{
							omni_w = omni_tmp.getLikelihood(*get_particles[pi]->getState(), OMNI_FEATURE);
						}
						//clock_t lap3 = clock();
						double gps_w = gpgga_tmp.getLikelihoodND(*get_particles[pi]->getState());
						//clock_t lap4 = clock();
						lid2_l_likelihood_tmp[pi] = lid2_l_w;
						omni_likelihood_tmp[pi] = omni_w;
						gpgga_likelihood_tmp[pi] = gps_w;
						//std::cout << (int)(lap1 - lap0) << "," << (int)(lap2 - lap1) << "," << (int)(lap3 - lap2) << "," << (int)(lap4 - lap3)  << std::endl;
					}

				});
			}

			for (int i = 0; i < LIKELIHOOD_THREAD; i++)
			{
				likelihood_threads[i].join();
			}


			/* 正規化 */
			if (ADD_BIAS){
				Normalize(lid2_l_likelihood);
				Normalize(omni_likelihood);
				Normalize(gpgga_likelihood);
				addBias(lid2_l_likelihood);
				addBias(omni_likelihood);
				addBias(gpgga_likelihood);
				Normalize(lid2_l_likelihood);
				Normalize(omni_likelihood);
				Normalize(gpgga_likelihood);
			}
			else{
				Normalize(lid2_l_likelihood);
				Normalize(omni_likelihood);
				Normalize(gpgga_likelihood);
			}

			break;
		default:
			std::cout << "NO TRIAL_TYPE: " << trial_type << ", " << __FILE__ << " LINE: " << __LINE__ << std::endl;
			exit(0);
			break;
		}

		std::cout << "Finish calcLikelihood" << std::endl;
	}

	void generateStatParticle(){
		/*  センサ統合用パーティクル生成  */
		{
			Position<> min, max;
			Position<> particle_min = minParameters(getParticles());
			Position<> particle_max = maxParameters(getParticles());
			if (estimated_position.empty())
			{
				min = ini_position - stat_sample_radius;
				max = ini_position + stat_sample_radius;
				min.r = particle_min.r - stat_sample_radius.r;
				max.r = particle_max.r + stat_sample_radius.r;
			}
			else
			{
				min = estimated_position.back() - stat_sample_radius;
				max = estimated_position.back() + stat_sample_radius;
				min.r = particle_min.r - stat_sample_radius.r;
				max.r = particle_max.r + stat_sample_radius.r;
			}
			std::random_device rnd;     // 非決定的な乱数生成器を生成
			std::mt19937_64 mt(rnd());     //  メルセンヌ・ツイスタの64ビット版、引数は初期シード値
			std::uniform_real_distribution<double> noise_x(min.x, max.x);	//パーティクルの散布範囲
			std::uniform_real_distribution<double> noise_y(min.y, max.y);	//パーティクルの散布範囲
			std::uniform_real_distribution<double> noise_r(min.r, max.r);	//パーティクルの散布範囲

			/* 白領域内のみでSTAT_SAMPLE_SIZE個 */
			//for (int i = 0; i < STAT_SAMPLE_SIZE; i++)
			//{
			//	while (true)
			//	{
			//		Position<> tmp(noise_x(mt), noise_y(mt), noise_r(mt));
			//		//if (!onObject_(tmp))
			//		//{
			//		stat_particles.push_back(tmp);
			//		break;
			//		//}
			//	}
			//}

			/* 白・黒合わせてSTAT_SAMPLE_SIZE個で黒領域のサンプルはskip */
			for (int i = 0; i < STAT_SAMPLE_SIZE; i++)
			{
				Position<> tmp(noise_x(mt), noise_y(mt), noise_r(mt));
				if (!onObject_(tmp))
				{
					stat_particles.push_back(tmp);
				}
			}

		}
		std::cout << "Finish generateLikelihoodStat" << std::endl;

	}

	void calcStatParticleLikelihood(){

		// resize
		stat_lid2_l_likelihood.resize(stat_particles.size());
		stat_lid2_u_likelihood.resize(stat_particles.size());
		stat_omni_likelihood.resize(stat_particles.size());
		stat_gpgga_likelihood.resize(stat_particles.size());


		/**********************************************************/
		//	尤度算出の準備
		/**********************************************************/

		std::vector<double>& stat_lid2_l_likelihood_tmp = stat_lid2_l_likelihood;
		std::vector<double>& stat_lid2_u_likelihood_tmp = stat_lid2_u_likelihood;
		std::vector<double>& stat_omni_likelihood_tmp = stat_omni_likelihood;
		std::vector<double>& stat_gpgga_likelihood_tmp = stat_gpgga_likelihood;
		std::vector<Position<>>& stat_particles_tmp = stat_particles;
		int no_tmp = no;
		Position<>& ini_position_tmp = ini_position;
		std::vector<Position<>>& estimated_position_tmp = estimated_position;
		cv::Mat map_img_tmp = map_img;

		clock_t lap1 = clock();


		switch (trial_type)
		{
		case TRIAL_SIMULTANEOUS:
		case TRIAL_PEARSON:
		case TRIAL_PEARSON_NONSTAT:
		case TRIAL_NON_TIMESEQUENCE:
		case TRIAL_NON_TIMESEQUENCE_SIMUL:
		case TRIAL_SUYAMA_STAT:
			for (int th = 0; th < LIKELIHOOD_THREAD; th++)
			{
				Lidar2d& lid2_l_tmp = lid2_l[th];
				Lidar2d& lid2_u_tmp = lid2_u[th];
				OmniCamera& omni_tmp = omni[th];
				Gps& gpgga_tmp = gpgga[th];
				likelihood_threads[th] = std::thread([th, &lid2_l_tmp, &lid2_u_tmp, &omni_tmp, &gpgga_tmp, &stat_lid2_l_likelihood_tmp, &stat_lid2_u_likelihood_tmp, &stat_omni_likelihood_tmp, &stat_gpgga_likelihood_tmp, no_tmp, ini_position_tmp, estimated_position_tmp, stat_particles_tmp, map_img_tmp] {
					/*  Lidar2D  */
					lid2_l_tmp.setMeasICP();
					lid2_u_tmp.setMeasICP();


					//omni_tmp.calcSimilarityBetweenImages(stat_particles_tmp);


					/*  GPS  */
					gpgga_tmp.setMeasND();


					for (int pi = th; pi < stat_particles_tmp.size(); pi += LIKELIHOOD_THREAD)
					{
						//cv::Point pixel = ToPixel(stat_particles_tmp[pi], map_img_tmp, MAP_qRES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	障害物地図上のパーティクル存在するピクセル
						//if (pixel.x < 0 || pixel.x >= map_img_tmp.cols ||
						//	pixel.y < 0 || pixel.y >= map_img_tmp.rows ||
						//	(int)map_img_tmp.at<unsigned char>(pixel) != 255)	//	存在不可能領域の条件式
						//{
						//	stat_lid2_l_likelihood_tmp[pi] = 0.0;
						//	stat_lid2_u_likelihood_tmp[pi] = 0.0;
						//	stat_omni_likelihood_tmp[pi] = 0.0;
						//	stat_gpgga_likelihood_tmp[pi] = 0.0;
						//	continue;
						//}
						/*  尤度算出  */
						double lid2_l_w = lid2_l_tmp.getLikelihoodICP(stat_particles_tmp[pi], ICP_PAIR_NUM_TH_L);
						double lid2_u_w = lid2_u_tmp.getLikelihoodICP(stat_particles_tmp[pi], ICP_PAIR_NUM_TH_U);
						double omni_w;
						if (USE_BOF){
							omni_w = omni_tmp.getLikelihoodBofSift(stat_particles_tmp[pi]);
						}
						else{
							omni_w = omni_tmp.getLikelihood(stat_particles_tmp[pi], OMNI_FEATURE);
						}
						double gps_w = gpgga_tmp.getLikelihoodND(stat_particles_tmp[pi]);
						stat_lid2_l_likelihood_tmp[pi] = lid2_l_w;
						stat_lid2_u_likelihood_tmp[pi] = lid2_u_w;
						stat_omni_likelihood_tmp[pi] = omni_w;
						stat_gpgga_likelihood_tmp[pi] = gps_w;
					}
				});
			}
			for (int i = 0; i < LIKELIHOOD_THREAD; i++){
				likelihood_threads[i].join();
			}

			/**********************************************************/
			//	正規化
			/**********************************************************/
			if (ADD_BIAS){
				Normalize(stat_lid2_l_likelihood);
				Normalize(stat_lid2_u_likelihood);
				Normalize(stat_omni_likelihood);
				Normalize(stat_gpgga_likelihood);
				addBias(stat_lid2_l_likelihood);
				addBias(stat_lid2_u_likelihood);
				addBias(stat_omni_likelihood);
				addBias(stat_gpgga_likelihood);
				Normalize(stat_lid2_l_likelihood);
				Normalize(stat_lid2_u_likelihood);
				Normalize(stat_omni_likelihood);
				Normalize(stat_gpgga_likelihood);
			}
			else{
				Normalize(stat_lid2_l_likelihood);
				Normalize(stat_lid2_u_likelihood);
				Normalize(stat_omni_likelihood);
				Normalize(stat_gpgga_likelihood);
			}

			break;

		case TRIAL_SUYAMA_NONSTAT:
		case TRIAL_3SENSORS_SIMULATNEOUS:
		case TRIAL_3SENSORS_PEARSON:
		case TRIAL_3SENSORS_PEARSON_NONSTAT:
		case TRIAL_3SENSORS_SUYAMA_NONSTAT:
		case TRIAL_3SENSORS_LRF_GPS:
			for (int th = 0; th < LIKELIHOOD_THREAD; th++)
			{
				Lidar2d& lid2_l_tmp = lid2_l[th];
				OmniCamera& omni_tmp = omni[th];
				Gps& gpgga_tmp = gpgga[th];
				likelihood_threads[th] = std::thread([th, &lid2_l_tmp, &omni_tmp, &gpgga_tmp, &stat_lid2_l_likelihood_tmp, &stat_omni_likelihood_tmp, &stat_gpgga_likelihood_tmp, no_tmp, ini_position_tmp, estimated_position_tmp, stat_particles_tmp, map_img_tmp] {
					/*  Lidar2D  */
					lid2_l_tmp.setMeasICP();


					//omni_tmp.calcSimilarityBetweenImages(stat_particles_tmp);


					/*  GPS  */
					gpgga_tmp.setMeasND();


					for (int pi = th; pi < stat_particles_tmp.size(); pi += LIKELIHOOD_THREAD)
					{
						//cv::Point pixel = ToPixel(stat_particles_tmp[pi], map_img_tmp, MAP_qRES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	障害物地図上のパーティクル存在するピクセル
						//if (pixel.x < 0 || pixel.x >= map_img_tmp.cols ||
						//	pixel.y < 0 || pixel.y >= map_img_tmp.rows ||
						//	(int)map_img_tmp.at<unsigned char>(pixel) != 255)	//	存在不可能領域の条件式
						//{
						//	stat_lid2_l_likelihood_tmp[pi] = 0.0;
						//	stat_omni_likelihood_tmp[pi] = 0.0;
						//	stat_gpgga_likelihood_tmp[pi] = 0.0;
						//	continue;
						//}
						/*  尤度算出  */
						double lid2_l_w = lid2_l_tmp.getLikelihoodICP(stat_particles_tmp[pi], ICP_PAIR_NUM_TH_L);
						double omni_w;
						if (USE_BOF){
							omni_w = omni_tmp.getLikelihoodBofSift(stat_particles_tmp[pi]);
						}
						else{
							omni_w = omni_tmp.getLikelihood(stat_particles_tmp[pi], OMNI_FEATURE);
						}
						double gps_w = gpgga_tmp.getLikelihoodND(stat_particles_tmp[pi]);
						stat_lid2_l_likelihood_tmp[pi] = lid2_l_w;
						stat_omni_likelihood_tmp[pi] = omni_w;
						stat_gpgga_likelihood_tmp[pi] = gps_w;
					}
				});
			}
			for (int i = 0; i < LIKELIHOOD_THREAD; i++){
				likelihood_threads[i].join();
			}

			/**********************************************************/
			//	正規化
			/**********************************************************/
			if (ADD_BIAS){
				Normalize(stat_lid2_l_likelihood);
				Normalize(stat_omni_likelihood);
				Normalize(stat_gpgga_likelihood);
				addBias(stat_lid2_l_likelihood);
				addBias(stat_omni_likelihood);
				addBias(stat_gpgga_likelihood);
				Normalize(stat_lid2_l_likelihood);
				Normalize(stat_omni_likelihood);
				Normalize(stat_gpgga_likelihood);
			}
			else{
				Normalize(stat_lid2_l_likelihood);
				Normalize(stat_omni_likelihood);
				Normalize(stat_gpgga_likelihood);
			}


			break;
		default:
			std::cout << "NO TRIAL_TYPE: " << trial_type << ", " << __FILE__ << " LINE: " << __LINE__ << std::endl;
			exit(0);
			break;
		}



	}

	void decideUseSensor(){

		std::cout << std::endl;

		Statistics<double> stat;
		std::vector<bool> use_tmp_;
		std::vector<std::vector<double>> similarity;
		std::vector<std::vector<bool>> similar_;
		float num_th;

		/* SUYAMA用 */
		std::vector<double> average;
		std::vector<double> similarity_tmp;
		double th = 0;
		std::vector<bool> similar_tmp_;

		switch (trial_type)
		{
		case TRIAL_SIMULTANEOUS:
		case TRIAL_NON_TIMESEQUENCE_SIMUL:
		case TRIAL_3SENSORS_SIMULATNEOUS:
			use_tmp_ = std::vector<bool>(sensor_num, true);
			use_sim_.push_back(use_tmp_);
			use_.push_back(use_tmp_);
			break;

		case TRIAL_PEARSON:
			stat.add_data(stat_lid2_l_likelihood);
			stat.add_data(stat_lid2_u_likelihood);
			stat.add_data(stat_omni_likelihood);
			stat.add_data(stat_gpgga_likelihood);

			/*  ピアソンの積率相関係数  */
			similarity = stat.pearsonCorr();
			similarity_table.push_back(similarity);
			std::cout << "Similarity" << std::endl;
			std::cout << similarity;

			/*  閾値処理により類似，非類似に分割  */
			similar_ = ThresholdProcessing(similarity, PEARSON_CORRCOEF_TH);
			similar_table_.push_back(similar_);
			std::cout << "Similar_" << std::endl;
			std::cout << similar_;

			/*  過半数の確率分布に対して非類似となる分布を排除  */
			num_th = sensor_num / 2.0;
			use_tmp_ = UseIndex(similar_, num_th);
			use_sim_.push_back(use_tmp_);
			if (std::find(use_tmp_.begin(), use_tmp_.end(), true) == use_tmp_.end()) {
				use_tmp_ = std::vector<bool>(sensor_num, true);
			}
			use_.push_back(use_tmp_);
			break;

		case TRIAL_PEARSON_NONSTAT:
		case TRIAL_NON_TIMESEQUENCE:

			stat.add_data(lid2_l_likelihood);
			stat.add_data(lid2_u_likelihood);
			stat.add_data(omni_likelihood);
			stat.add_data(gpgga_likelihood);

			/*  ピアソンの積率相関係数  */
			similarity = stat.pearsonCorr();
			similarity_table.push_back(similarity);
			std::cout << "Similarity" << std::endl;
			std::cout << similarity;

			/*  閾値処理により類似，非類似に分割  */
			similar_ = ThresholdProcessing(similarity, PEARSON_CORRCOEF_TH);
			similar_table_.push_back(similar_);
			std::cout << "Similar_" << std::endl;
			std::cout << similar_;

			/*  過半数の確率分布に対して非類似となる分布を排除  */
			num_th = sensor_num / 2.0;
			use_tmp_ = UseIndex(similar_, num_th);
			use_sim_.push_back(use_tmp_);
			if (std::find(use_tmp_.begin(), use_tmp_.end(), true) == use_tmp_.end()) {
				use_tmp_ = std::vector<bool>(sensor_num, true);
			}
			use_.push_back(use_tmp_);
			break;

		case TRIAL_SUYAMA_STAT:

			stat.add_data(stat_lid2_l_likelihood);
			stat.add_data(stat_lid2_u_likelihood);
			stat.add_data(stat_omni_likelihood);
			stat.add_data(stat_gpgga_likelihood);

			std::cout << std::endl;

			/*  average  */
			average = stat.average();

			/*  kl Divergence  */
			similarity_tmp = stat.KlDivergence(average);
			similarity = { similarity_tmp };


			similarity_table.push_back(similarity);

			/*  threshold  */
			th = Statistics<>::rootMeanSquare(similarity_tmp);
			similar_tmp_ = ThresholdProcessing(similarity_tmp, th);
			similar_ = { similar_tmp_ };
			similar_table_.push_back(similar_);


			for (int ni = 0; ni < sensor_num; ni++)
			{
				if (similarity_tmp[ni] <= th){
					use_tmp_.push_back(true);
				}
				else {
					use_tmp_.push_back(false);
				}
			}
			use_sim_.push_back(use_tmp_);

			if (std::find(use_tmp_.begin(), use_tmp_.end(), true) == use_tmp_.end()) {
				use_tmp_ = std::vector<bool>(sensor_num, true);
			}
			use_.push_back(use_tmp_);

			break;

		case TRIAL_SUYAMA_NONSTAT:

			stat.add_data(lid2_l_likelihood);
			stat.add_data(lid2_u_likelihood);
			stat.add_data(omni_likelihood);
			stat.add_data(gpgga_likelihood);

			std::cout << std::endl;

			/*  average  */
			average = stat.average();

			/*  kl Divergence  */
			similarity_tmp = stat.KlDivergence(average);
			similarity = { similarity_tmp };


			similarity_table.push_back(similarity);

			/*  threshold  */
			th = Statistics<>::rootMeanSquare(similarity_tmp);
			similar_tmp_ = ThresholdProcessing(similarity_tmp, th);
			similar_ = { similar_tmp_ };
			similar_table_.push_back(similar_);

			for (int ni = 0; ni < sensor_num; ni++)
			{
				if (similarity_tmp[ni] <= th){
					use_tmp_.push_back(true);
				}
				else {
					use_tmp_.push_back(false);
				}
			}
			use_sim_.push_back(use_tmp_);

			if (std::find(use_tmp_.begin(), use_tmp_.end(), true) == use_tmp_.end()) {
				use_tmp_ = std::vector<bool>(sensor_num, true);
			}
			use_.push_back(use_tmp_);

			break;

		case TRIAL_3SENSORS_LRF_GPS:
			all_th.push_back(0.0);
			use_tmp_ = std::vector<bool>(sensor_num, true);
			use_tmp_[1] = false;
			use_sim_.push_back(use_tmp_);
			use_.push_back(use_tmp_);
			break;

		case TRIAL_3SENSORS_PEARSON:
			stat.add_data(stat_lid2_l_likelihood);
			stat.add_data(stat_omni_likelihood);
			stat.add_data(stat_gpgga_likelihood);

			/*  ピアソンの積率相関係数  */
			similarity = stat.pearsonCorr();
			similarity_table.push_back(similarity);
			std::cout << "Similarity" << std::endl;
			std::cout << similarity;

			/*  閾値処理により類似，非類似に分割  */
			similar_ = ThresholdProcessing(similarity, PEARSON_CORRCOEF_TH);
			similar_table_.push_back(similar_);
			std::cout << "Similar_" << std::endl;
			std::cout << similar_;

			/*  過半数の確率分布に対して非類似となる分布を排除  */
			num_th = (float)(sensor_num) / 2.0;
			use_tmp_ = UseIndex(similar_, num_th);
			use_sim_.push_back(use_tmp_);
			if (std::find(use_tmp_.begin(), use_tmp_.end(), true) == use_tmp_.end()) {
				use_tmp_ = std::vector<bool>(sensor_num, true);
			}
			use_.push_back(use_tmp_);
			break;

		case TRIAL_3SENSORS_PEARSON_NONSTAT:

			stat.add_data(lid2_l_likelihood);
			stat.add_data(omni_likelihood);
			stat.add_data(gpgga_likelihood);

			/*  ピアソンの積率相関係数  */
			similarity = stat.pearsonCorr();
			similarity_table.push_back(similarity);
			std::cout << "Similarity" << std::endl;
			std::cout << similarity;

			/*  閾値処理により類似，非類似に分割  */
			similar_ = ThresholdProcessing(similarity, PEARSON_CORRCOEF_TH);
			similar_table_.push_back(similar_);
			std::cout << "Similar_" << std::endl;
			std::cout << similar_;

			/*  過半数の確率分布に対して非類似となる分布を排除  */
			num_th = (float)(sensor_num) / 2.0;
			use_tmp_ = UseIndex(similar_, num_th);
			use_sim_.push_back(use_tmp_);
			if (std::find(use_tmp_.begin(), use_tmp_.end(), true) == use_tmp_.end()) {
				use_tmp_ = std::vector<bool>(sensor_num, true);
			}
			use_.push_back(use_tmp_);
			break;

		case TRIAL_3SENSORS_SUYAMA_NONSTAT:

			stat.add_data(lid2_l_likelihood);
			stat.add_data(omni_likelihood);
			stat.add_data(gpgga_likelihood);

			std::cout << std::endl;

			/*  average  */
			average = stat.average();

			/*  kl Divergence  */
			similarity_tmp = stat.KlDivergence(average);
			similarity = { similarity_tmp };


			similarity_table.push_back(similarity);

			/*  threshold  */
			th = Statistics<>::rootMeanSquare(similarity_tmp);
			similar_tmp_ = ThresholdProcessing(similarity_tmp, th);
			similar_ = { similar_tmp_ };
			similar_table_.push_back(similar_);

			for (int ni = 0; ni < sensor_num; ni++)
			{
				if (similarity_tmp[ni] <= th){
					use_tmp_.push_back(true);
				}
				else {
					use_tmp_.push_back(false);
				}
			}
			use_sim_.push_back(use_tmp_);

			if (std::find(use_tmp_.begin(), use_tmp_.end(), true) == use_tmp_.end()) {
				use_tmp_ = std::vector<bool>(sensor_num, true);
			}
			use_.push_back(use_tmp_);
			all_th.push_back(th);
			break;


		default:
			std::cout << "Error 'TRIAL': " << TRIAL_TYPE << "FILE: " << __FILE__ << " LINE: " << __LINE__ << std::endl;
			exit(0);
			break;
		}

		all_th.push_back(th);

		std::cout << "Use sensors: " << use_.back() << std::endl;
		std::cout << "Complete decideSensor!" << std::endl;

		std::cout << std::endl;

	}

	void addBias(std::vector<double>& likelihood){
		for (auto& w : likelihood){
			w += W_BIAS;
		}
	}

	void estimateThetaByOnlyLRF()
	{
		std::vector<Particle<Position<>>*>& get_particles = getParticles();
		std::vector<double>& lid2_l_likelihood_tmp = lid2_l_likelihood;
		std::vector<double>& lid2_u_likelihood_tmp = lid2_u_likelihood;
		int no_tmp = no;
		cv::Mat& map_img_tmp = map_img;

		/*  状態遷移に付加する雑音の準備  */
		std::random_device rnd;     // 非決定的な乱数生成器を生成
		std::mt19937_64 mt(rnd());     //  メルセンヌ・ツイスタの64ビット版、引数は初期シード値
		std::normal_distribution<double> par_sys_noise_r(0.0, trans_par_sys_var.r);
		double theta;
		std::vector<double> likelihood_lrf;

		switch (trial_type)
		{
		case TRIAL_SIMULTANEOUS:
		case TRIAL_PEARSON:
		case TRIAL_PEARSON_NONSTAT:
		case TRIAL_SUYAMA_STAT:
			// resize
			lid2_l_likelihood.resize(getParticles().size());
			lid2_u_likelihood.resize(getParticles().size());

			for (int th = 0; th < LIKELIHOOD_THREAD; th++)
			{
				Lidar2d& lid2_l_tmp = lid2_l[th];
				Lidar2d& lid2_u_tmp = lid2_u[th];
				OmniCamera& omni_tmp = omni[th];
				Gps& gpgga_tmp = gpgga[th];
				likelihood_threads[th] = std::thread([th, &lid2_l_tmp, &lid2_u_tmp, &omni_tmp, &gpgga_tmp, &lid2_l_likelihood_tmp, &lid2_u_likelihood_tmp, no_tmp, get_particles, map_img_tmp] {

					/*  Lidar2D  */
					lid2_l_tmp.setMeasICP();
					lid2_u_tmp.setMeasICP();

					/*  Omni Camera  */

					//if (estimated_position_tmp.empty())	omni_tmp.setMeasBofSift1(ini_position_tmp);
					//else
					//{
					//	omni_tmp.setMeasBofSift1(estimated_position_tmp.back());
					//}
					//omni_tmp.calcSimilarityBetweenImages(get_particles);



					for (int pi = th; pi < SAMPLE_SIZE; pi += LIKELIHOOD_THREAD)
					{
						//cv::Point pixel = ToPixel(*get_particles[pi]->getState(), map_img_tmp, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	障害物地図上のパーティクル存在するピクセル
						//if (pixel.x < 0 || pixel.x >= map_img_tmp.cols ||
						//	pixel.y < 0 || pixel.y >= map_img_tmp.rows ||
						//	(int)map_img_tmp.at<unsigned char>(pixel) != 255)	//	存在不可能領域の条件式
						//{
						//	lid2_l_likelihood_tmp[pi] = 0.0;
						//	lid2_u_likelihood_tmp[pi] = 0.0;
						//	omni_likelihood_tmp[pi] = 0.0;
						//	gpgga_likelihood_tmp[pi] = 0.0;
						//	continue;
						//}
						/*  尤度算出  */
						assert(get_particles.size() == lid2_l_likelihood_tmp.size());
						assert(get_particles.size() == lid2_u_likelihood_tmp.size());

						//clock_t lap0 = clock();
						double lid2_l_w = lid2_l_tmp.getLikelihoodICP(*get_particles[pi]->getState(), ICP_PAIR_NUM_TH_L);
						//clock_t lap1 = clock();
						double lid2_u_w = lid2_u_tmp.getLikelihoodICP(*get_particles[pi]->getState(), ICP_PAIR_NUM_TH_U);
						//clock_t lap2 = clock();
						//clock_t lap4 = clock();
						lid2_l_likelihood_tmp[pi] = lid2_l_w;
						lid2_u_likelihood_tmp[pi] = lid2_u_w;
						//std::cout << (int)(lap1 - lap0) << "," << (int)(lap2 - lap1) << "," << (int)(lap3 - lap2) << "," << (int)(lap4 - lap3)  << std::endl;
					}

				});
			}

			for (int i = 0; i < LIKELIHOOD_THREAD; i++)
			{
				likelihood_threads[i].join();
			}


			/* 正規化 */
			if (ADD_BIAS){
				Normalize(lid2_l_likelihood);
				Normalize(lid2_u_likelihood);
				addBias(lid2_l_likelihood);
				addBias(lid2_u_likelihood);
				Normalize(lid2_l_likelihood);
				Normalize(lid2_u_likelihood);
			}
			else{
				Normalize(lid2_l_likelihood);
				Normalize(lid2_u_likelihood);
			}

			/* 統合 */
			for (int i = 0; i < lid2_l_likelihood.size(); i++){
				double w = lid2_l_likelihood[i] * lid2_u_likelihood[i];
				likelihood_lrf.push_back(w);
			}
			Normalize(likelihood_lrf);

			/* theta推定 */
			theta = 0;
			for (int i = 0; i < getParticles().size(); i++){
				theta += getParticles()[i]->getState()->r*likelihood_lrf[i];
			}

			/* thetaを強制 */
			for (int i = 0; i < getParticles().size(); i++){
				getParticles()[i]->getState()->r = theta + par_sys_noise_r(mt);
			}

			lid2_l_likelihood = std::vector<double>(SAMPLE_SIZE, 0);
			lid2_u_likelihood = std::vector<double>(SAMPLE_SIZE, 0);

			break;

		case TRIAL_SUYAMA_NONSTAT:
		case TRIAL_3SENSORS_SIMULATNEOUS:
		case TRIAL_3SENSORS_PEARSON:
		case TRIAL_3SENSORS_PEARSON_NONSTAT:
		case TRIAL_3SENSORS_SUYAMA_NONSTAT:
		case TRIAL_3SENSORS_LRF_GPS:
			// resize
			lid2_l_likelihood.resize(getParticles().size());


			for (int th = 0; th < LIKELIHOOD_THREAD; th++)
			{
				Lidar2d& lid2_l_tmp = lid2_l[th];
				OmniCamera& omni_tmp = omni[th];
				Gps& gpgga_tmp = gpgga[th];
				likelihood_threads[th] = std::thread([th, &lid2_l_tmp, &omni_tmp, &gpgga_tmp, &lid2_l_likelihood_tmp, no_tmp, get_particles, map_img_tmp] {

					/*  Lidar2D  */
					lid2_l_tmp.setMeasICP();

					/*  Omni Camera  */

					//if (estimated_position_tmp.empty())	omni_tmp.setMeasBofSift1(ini_position_tmp);
					//else
					//{
					//	omni_tmp.setMeasBofSift1(estimated_position_tmp.back());
					//}
					//omni_tmp.calcSimilarityBetweenImages(get_particles);

					for (int pi = th; pi < SAMPLE_SIZE; pi += LIKELIHOOD_THREAD)
					{
						//cv::Point pixel = ToPixel(*get_particles[pi]->getState(), map_img_tmp, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	障害物地図上のパーティクル存在するピクセル
						//if (pixel.x < 0 || pixel.x >= map_img_tmp.cols ||
						//	pixel.y < 0 || pixel.y >= map_img_tmp.rows ||
						//	(int)map_img_tmp.at<unsigned char>(pixel) != 255)	//	存在不可能領域の条件式
						//{
						//	lid2_l_likelihood_tmp[pi] = 0.0;
						//	omni_likelihood_tmp[pi] = 0.0;
						//	gpgga_likelihood_tmp[pi] = 0.0;
						//	continue;
						//}
						/*  尤度算出  */
						assert(get_particles.size() == lid2_l_likelihood_tmp.size());

						//clock_t lap0 = clock();
						double lid2_l_w = lid2_l_tmp.getLikelihoodICP(*get_particles[pi]->getState(), ICP_PAIR_NUM_TH_L);
						//clock_t lap1 = clock();
						//clock_t lap2 = clock();
						//clock_t lap4 = clock();
						lid2_l_likelihood_tmp[pi] = lid2_l_w;
					}

				});
			}

			for (int i = 0; i < LIKELIHOOD_THREAD; i++)
			{
				likelihood_threads[i].join();
			}


			/* 正規化 */
			if (ADD_BIAS){
				Normalize(lid2_l_likelihood);
				addBias(lid2_l_likelihood);
				Normalize(lid2_l_likelihood);
			}
			else{
				Normalize(lid2_l_likelihood);
			}

			/* theta推定 */
			theta = 0;
			for (int i = 0; i < getParticles().size(); i++){
				theta += getParticles()[i]->getState()->r*lid2_l_likelihood[i];
			}

			/* thetaを強制 */
			for (int i = 0; i < getParticles().size(); i++){
				getParticles()[i]->getState()->r = theta + par_sys_noise_r(mt);
			}

			lid2_l_likelihood = std::vector<double>(SAMPLE_SIZE, 0);

			break;
		case TRIAL_NON_TIMESEQUENCE:
		case TRIAL_NON_TIMESEQUENCE_SIMUL:
			break;
		default:
			std::cout << "NO TRIAL_TYPE: " << trial_type << ", " << __FILE__ << " LINE: " << __LINE__ << std::endl;
			exit(0);
			break;
		}

	};

	/*  計測モデル  */
	void Likelihood()
	{
		Statistics<> stat2;
		clock_t lap0, lap1, lap2, lap3, lap4, lap5;

		switch (trial_type)
		{
		case TRIAL_SIMULTANEOUS:
			lap0 = clock();

			prepareForCalcLikelihood();

			lap1 = clock();

			/* 画像位置における類似度 */
			//if (!estimated_position.empty()){
			//	cv::Mat map = map_img_tmp.clone();
			//	std::vector<int> sizes;
			//	for (int i = 0; i < omni[0].imgForCalcLikeAll_positioin.size(); i++){
			//		int size = omni[0].matches_used_calc[i].size();
			//		std::cout << size << std::endl;
			//		sizes.push_back(size);
			//	}
			//	//double max = *std::max_element(sizes.begin(),sizes.end());
			//	for (int i = 0; i < sizes.size(); i++){
			//		cv::Point p = ToPixel(omni[0].imgForCalcLikeAll_positioin[i], map, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			//		//int a = (double)(sizes[i]) / max*150.0;
			//		int a = (double)(sizes[i]);
			//		cv::circle(map, p, 3, cv::Scalar(a), -1);
			//	}
			//	/*  画像の切り出し  */
			//	Coor<> upleft(estimated_position_tmp.back().x - CUT_MAP_RADIUS_X, estimated_position_tmp.back().y - CUT_MAP_RADIUS_Y);
			//	//Coor<> upleft(estimated_position.back().x - CUT_MAP_RADIUS_X, estimated_position.back().y - CUT_MAP_RADIUS_Y);
			//	Coor<> rect(CUT_MAP_RADIUS_X*2.0, CUT_MAP_RADIUS_Y*2.0);
			//	cv::Point upleft_pix = ToPixel(upleft, map, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//
			//	cv::Size rect_pix = ToPixelSize(rect, map, MAP_RES);
			//	if (upleft_pix.x < 0)	upleft_pix.x = 0;
			//	if (upleft_pix.y < 0)	upleft_pix.y = 0;
			//	if (upleft_pix.x >= map.cols - rect_pix.width)	upleft_pix.x = map.cols - rect_pix.width;
			//	if (upleft_pix.y >= map.rows - rect_pix.height)	upleft_pix.y = map.rows - rect_pix.height;
			//	map = cv::Mat(map, cv::Rect(upleft_pix, rect_pix));
			//	cv::flip(map, map, 0);
			//	//cv::namedWindow("img_sim");
			//	cv::imshow("img_sim", map);
			//	cv::waitKey(1);
			//}

			lap2 = clock();


			/* ThetaをLRFで強制 */
			estimateThetaByOnlyLRF();

			/**********************************************************/
			//	正規化
			/**********************************************************/

			calcParticleLikelihood();
			Normalize(lid2_l_likelihood);
			Normalize(lid2_u_likelihood);
			Normalize(omni_likelihood);
			Normalize(gpgga_likelihood);


			lap3 = clock();

			decideUseSensor();

			lap4 = clock();


			/**********************************************************/
			//	統合
			/**********************************************************/


			stat2.add_data(lid2_l_likelihood);
			stat2.add_data(lid2_u_likelihood);
			stat2.add_data(omni_likelihood);
			stat2.add_data(gpgga_likelihood);


			/*  バイアス分布以外を積による統合  */
			if (use_.empty()){
				std::cout << "non use" << std::endl;
				exit(0);
			}
			fusion_likelihood = stat2.Product(use_.back(), INTEGRATE_TYPE);
			Normalize(fusion_likelihood);	//	正規化

			/*  パーティクルに尤度をset  */
			for (int pi = 0; pi < SAMPLE_SIZE; pi++) {
				getParticles()[pi]->setWeight(fusion_likelihood[pi]);
			}

			///*  重み付き平均による推定位置算出  */
			//estimated_position.push_back(getWeightedMean());
			lap5 = clock();

			std::cout << (int)(lap1 - lap0) << "," << (int)(lap2 - lap1) << "," << (int)(lap3 - lap2) << "," << (int)(lap4 - lap3) << "," << (int)(lap5 - lap4) << std::endl;
			break;

		case TRIAL_PEARSON:
		case TRIAL_SUYAMA_STAT:
			lap0 = clock();

			prepareForCalcLikelihood();

			/* ThetaをLRFで強制 */
			estimateThetaByOnlyLRF();


			/* 類似性評価用サンプル */
			generateStatParticle();
			calcStatParticleLikelihood();


			lap1 = clock();
			/* 位置推定用パーティクル */
			calcParticleLikelihood();


			/* 画像位置における類似度 */
			//if (!estimated_position.empty()){
			//	cv::Mat map = map_img_tmp.clone();
			//	std::vector<int> sizes;
			//	for (int i = 0; i < omni[0].imgForCalcLikeAll_positioin.size(); i++){
			//		int size = omni[0].matches_used_calc[i].size();
			//		std::cout << size << std::endl;
			//		sizes.push_back(size);
			//	}
			//	//double max = *std::max_element(sizes.begin(),sizes.end());
			//	for (int i = 0; i < sizes.size(); i++){
			//		cv::Point p = ToPixel(omni[0].imgForCalcLikeAll_positioin[i], map, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			//		//int a = (double)(sizes[i]) / max*150.0;
			//		int a = (double)(sizes[i]);
			//		cv::circle(map, p, 3, cv::Scalar(a), -1);
			//	}
			//	/*  画像の切り出し  */
			//	Coor<> upleft(estimated_position_tmp.back().x - CUT_MAP_RADIUS_X, estimated_position_tmp.back().y - CUT_MAP_RADIUS_Y);
			//	//Coor<> upleft(estimated_position.back().x - CUT_MAP_RADIUS_X, estimated_position.back().y - CUT_MAP_RADIUS_Y);
			//	Coor<> rect(CUT_MAP_RADIUS_X*2.0, CUT_MAP_RADIUS_Y*2.0);
			//	cv::Point upleft_pix = ToPixel(upleft, map, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//
			//	cv::Size rect_pix = ToPixelSize(rect, map, MAP_RES);
			//	if (upleft_pix.x < 0)	upleft_pix.x = 0;
			//	if (upleft_pix.y < 0)	upleft_pix.y = 0;
			//	if (upleft_pix.x >= map.cols - rect_pix.width)	upleft_pix.x = map.cols - rect_pix.width;
			//	if (upleft_pix.y >= map.rows - rect_pix.height)	upleft_pix.y = map.rows - rect_pix.height;
			//	map = cv::Mat(map, cv::Rect(upleft_pix, rect_pix));
			//	cv::flip(map, map, 0);
			//	//cv::namedWindow("img_sim");
			//	cv::imshow("img_sim", map);
			//	cv::waitKey(1);
			//}

			lap2 = clock();


			decideUseSensor();


			/**********************************************************/
			//	統合
			/**********************************************************/



			lap3 = clock();


			lap4 = clock();

			stat2.add_data(lid2_l_likelihood);
			stat2.add_data(lid2_u_likelihood);
			stat2.add_data(omni_likelihood);
			stat2.add_data(gpgga_likelihood);


			/*  バイアス分布以外を積による統合  */
			fusion_likelihood = stat2.Product(use_.back(), INTEGRATE_TYPE);
			Normalize(fusion_likelihood);	//	正規化

			/*  パーティクルに尤度をset  */
			for (int pi = 0; pi < SAMPLE_SIZE; pi++) {
				getParticles()[pi]->setWeight(fusion_likelihood[pi]);
			}

			///*  重み付き平均による推定位置算出  */
			//estimated_position.push_back(getWeightedMean());
			lap5 = clock();

			std::cout << (int)(lap1 - lap0) << "," << (int)(lap2 - lap1) << "," << (int)(lap3 - lap2) << "," << (int)(lap4 - lap3) << "," << (int)(lap5 - lap4) << std::endl;

			break;

		case TRIAL_PEARSON_NONSTAT:
		case TRIAL_SUYAMA_NONSTAT:

			lap0 = clock();

			prepareForCalcLikelihood();

			/* ThetaをLRFで強制 */
			estimateThetaByOnlyLRF();


			lap1 = clock();

			/* 画像位置における類似度 */
			//if (!estimated_position.empty()){
			//	cv::Mat map = map_img_tmp.clone();
			//	std::vector<int> sizes;
			//	for (int i = 0; i < omni[0].imgForCalcLikeAll_positioin.size(); i++){
			//		int size = omni[0].matches_used_calc[i].size();
			//		std::cout << size << std::endl;
			//		sizes.push_back(size);
			//	}
			//	//double max = *std::max_element(sizes.begin(),sizes.end());
			//	for (int i = 0; i < sizes.size(); i++){
			//		cv::Point p = ToPixel(omni[0].imgForCalcLikeAll_positioin[i], map, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			//		//int a = (double)(sizes[i]) / max*150.0;
			//		int a = (double)(sizes[i]);
			//		cv::circle(map, p, 3, cv::Scalar(a), -1);
			//	}
			//	/*  画像の切り出し  */
			//	Coor<> upleft(estimated_position_tmp.back().x - CUT_MAP_RADIUS_X, estimated_position_tmp.back().y - CUT_MAP_RADIUS_Y);
			//	//Coor<> upleft(estimated_position.back().x - CUT_MAP_RADIUS_X, estimated_position.back().y - CUT_MAP_RADIUS_Y);
			//	Coor<> rect(CUT_MAP_RADIUS_X*2.0, CUT_MAP_RADIUS_Y*2.0);
			//	cv::Point upleft_pix = ToPixel(upleft, map, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//
			//	cv::Size rect_pix = ToPixelSize(rect, map, MAP_RES);
			//	if (upleft_pix.x < 0)	upleft_pix.x = 0;
			//	if (upleft_pix.y < 0)	upleft_pix.y = 0;
			//	if (upleft_pix.x >= map.cols - rect_pix.width)	upleft_pix.x = map.cols - rect_pix.width;
			//	if (upleft_pix.y >= map.rows - rect_pix.height)	upleft_pix.y = map.rows - rect_pix.height;
			//	map = cv::Mat(map, cv::Rect(upleft_pix, rect_pix));
			//	cv::flip(map, map, 0);
			//	//cv::namedWindow("img_sim");
			//	cv::imshow("img_sim", map);
			//	cv::waitKey(1);
			//}

			lap2 = clock();


			/**********************************************************/
			//	正規化
			/**********************************************************/

			calcParticleLikelihood();
			Normalize(lid2_l_likelihood);
			Normalize(lid2_u_likelihood);
			Normalize(omni_likelihood);
			Normalize(gpgga_likelihood);


			lap3 = clock();

			decideUseSensor();

			lap4 = clock();


			/**********************************************************/
			//	統合
			/**********************************************************/


			stat2.add_data(lid2_l_likelihood);
			stat2.add_data(lid2_u_likelihood);
			stat2.add_data(omni_likelihood);
			stat2.add_data(gpgga_likelihood);


			/*  バイアス分布以外を積による統合  */
			if (use_.empty()){
				std::cout << "non use" << std::endl;
				exit(0);
			}
			fusion_likelihood = stat2.Product(use_.back(), INTEGRATE_TYPE);
			Normalize(fusion_likelihood);	//	正規化

			/*  パーティクルに尤度をset  */
			for (int pi = 0; pi < SAMPLE_SIZE; pi++) {
				getParticles()[pi]->setWeight(fusion_likelihood[pi]);
			}

			///*  重み付き平均による推定位置算出  */
			//estimated_position.push_back(getWeightedMean());
			lap5 = clock();

			std::cout << (int)(lap1 - lap0) << "," << (int)(lap2 - lap1) << "," << (int)(lap3 - lap2) << "," << (int)(lap4 - lap3) << "," << (int)(lap5 - lap4) << std::endl;

			break;

		case TRIAL_NON_TIMESEQUENCE:

			lap0 = clock();

			prepareForCalcLikelihood();

			lap1 = clock();

			/* 画像位置における類似度 */
			//if (!estimated_position.empty()){
			//	cv::Mat map = map_img_tmp.clone();
			//	std::vector<int> sizes;
			//	for (int i = 0; i < omni[0].imgForCalcLikeAll_positioin.size(); i++){
			//		int size = omni[0].matches_used_calc[i].size();
			//		std::cout << size << std::endl;
			//		sizes.push_back(size);
			//	}
			//	//double max = *std::max_element(sizes.begin(),sizes.end());
			//	for (int i = 0; i < sizes.size(); i++){
			//		cv::Point p = ToPixel(omni[0].imgForCalcLikeAll_positioin[i], map, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			//		//int a = (double)(sizes[i]) / max*150.0;
			//		int a = (double)(sizes[i]);
			//		cv::circle(map, p, 3, cv::Scalar(a), -1);
			//	}
			//	/*  画像の切り出し  */
			//	Coor<> upleft(estimated_position_tmp.back().x - CUT_MAP_RADIUS_X, estimated_position_tmp.back().y - CUT_MAP_RADIUS_Y);
			//	//Coor<> upleft(estimated_position.back().x - CUT_MAP_RADIUS_X, estimated_position.back().y - CUT_MAP_RADIUS_Y);
			//	Coor<> rect(CUT_MAP_RADIUS_X*2.0, CUT_MAP_RADIUS_Y*2.0);
			//	cv::Point upleft_pix = ToPixel(upleft, map, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//
			//	cv::Size rect_pix = ToPixelSize(rect, map, MAP_RES);
			//	if (upleft_pix.x < 0)	upleft_pix.x = 0;
			//	if (upleft_pix.y < 0)	upleft_pix.y = 0;
			//	if (upleft_pix.x >= map.cols - rect_pix.width)	upleft_pix.x = map.cols - rect_pix.width;
			//	if (upleft_pix.y >= map.rows - rect_pix.height)	upleft_pix.y = map.rows - rect_pix.height;
			//	map = cv::Mat(map, cv::Rect(upleft_pix, rect_pix));
			//	cv::flip(map, map, 0);
			//	//cv::namedWindow("img_sim");
			//	cv::imshow("img_sim", map);
			//	cv::waitKey(1);
			//}

			lap2 = clock();


			/**********************************************************/
			//	正規化
			/**********************************************************/

			calcParticleLikelihood();

			lap3 = clock();

			decideUseSensor();

			lap4 = clock();


			/**********************************************************/
			//	統合
			/**********************************************************/


			stat2.add_data(lid2_l_likelihood);
			stat2.add_data(lid2_u_likelihood);
			stat2.add_data(omni_likelihood);
			stat2.add_data(gpgga_likelihood);


			/*  バイアス分布以外を積による統合  */
			if (use_.empty()){
				std::cout << "non use" << std::endl;
				exit(0);
			}
			fusion_likelihood = stat2.Product(use_.back(), INTEGRATE_TYPE);
			Normalize(fusion_likelihood);	//	正規化

			/*  パーティクルに尤度をset  */
			for (int pi = 0; pi < SAMPLE_SIZE; pi++) {
				getParticles()[pi]->setWeight(fusion_likelihood[pi]);
			}

			///*  重み付き平均による推定位置算出  */
			//estimated_position.push_back(getWeightedMean());
			lap5 = clock();

			std::cout << (int)(lap1 - lap0) << "," << (int)(lap2 - lap1) << "," << (int)(lap3 - lap2) << "," << (int)(lap4 - lap3) << "," << (int)(lap5 - lap4) << std::endl;

			break;

		case TRIAL_NON_TIMESEQUENCE_SIMUL:

			lap0 = clock();

			prepareForCalcLikelihood();

			lap1 = clock();

			/* 画像位置における類似度 */
			//if (!estimated_position.empty()){
			//	cv::Mat map = map_img_tmp.clone();
			//	std::vector<int> sizes;
			//	for (int i = 0; i < omni[0].imgForCalcLikeAll_positioin.size(); i++){
			//		int size = omni[0].matches_used_calc[i].size();
			//		std::cout << size << std::endl;
			//		sizes.push_back(size);
			//	}
			//	//double max = *std::max_element(sizes.begin(),sizes.end());
			//	for (int i = 0; i < sizes.size(); i++){
			//		cv::Point p = ToPixel(omni[0].imgForCalcLikeAll_positioin[i], map, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			//		//int a = (double)(sizes[i]) / max*150.0;
			//		int a = (double)(sizes[i]);
			//		cv::circle(map, p, 3, cv::Scalar(a), -1);
			//	}
			//	/*  画像の切り出し  */
			//	Coor<> upleft(estimated_position_tmp.back().x - CUT_MAP_RADIUS_X, estimated_position_tmp.back().y - CUT_MAP_RADIUS_Y);
			//	//Coor<> upleft(estimated_position.back().x - CUT_MAP_RADIUS_X, estimated_position.back().y - CUT_MAP_RADIUS_Y);
			//	Coor<> rect(CUT_MAP_RADIUS_X*2.0, CUT_MAP_RADIUS_Y*2.0);
			//	cv::Point upleft_pix = ToPixel(upleft, map, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//
			//	cv::Size rect_pix = ToPixelSize(rect, map, MAP_RES);
			//	if (upleft_pix.x < 0)	upleft_pix.x = 0;
			//	if (upleft_pix.y < 0)	upleft_pix.y = 0;
			//	if (upleft_pix.x >= map.cols - rect_pix.width)	upleft_pix.x = map.cols - rect_pix.width;
			//	if (upleft_pix.y >= map.rows - rect_pix.height)	upleft_pix.y = map.rows - rect_pix.height;
			//	map = cv::Mat(map, cv::Rect(upleft_pix, rect_pix));
			//	cv::flip(map, map, 0);
			//	//cv::namedWindow("img_sim");
			//	cv::imshow("img_sim", map);
			//	cv::waitKey(1);
			//}

			lap2 = clock();


			/**********************************************************/
			//	正規化
			/**********************************************************/

			calcParticleLikelihood();

			lap3 = clock();

			decideUseSensor();

			lap4 = clock();


			/**********************************************************/
			//	統合
			/**********************************************************/


			stat2.add_data(lid2_l_likelihood);
			stat2.add_data(lid2_u_likelihood);
			stat2.add_data(omni_likelihood);
			stat2.add_data(gpgga_likelihood);


			/*  バイアス分布以外を積による統合  */
			if (use_.empty()){
				std::cout << "non use" << std::endl;
				exit(0);
			}
			fusion_likelihood = stat2.Product(use_.back(), INTEGRATE_TYPE);
			Normalize(fusion_likelihood);	//	正規化

			/*  パーティクルに尤度をset  */
			for (int pi = 0; pi < SAMPLE_SIZE; pi++) {
				getParticles()[pi]->setWeight(fusion_likelihood[pi]);
			}

			///*  重み付き平均による推定位置算出  */
			//estimated_position.push_back(getWeightedMean());
			lap5 = clock();

			std::cout << (int)(lap1 - lap0) << "," << (int)(lap2 - lap1) << "," << (int)(lap3 - lap2) << "," << (int)(lap4 - lap3) << "," << (int)(lap5 - lap4) << std::endl;

			break;

		case TRIAL_3SENSORS_SIMULATNEOUS:

			lap0 = clock();

			prepareForCalcLikelihood();

			lap1 = clock();

			/* 画像位置における類似度 */
			//if (!estimated_position.empty()){
			//	cv::Mat map = map_img_tmp.clone();
			//	std::vector<int> sizes;
			//	for (int i = 0; i < omni[0].imgForCalcLikeAll_positioin.size(); i++){
			//		int size = omni[0].matches_used_calc[i].size();
			//		std::cout << size << std::endl;
			//		sizes.push_back(size);
			//	}
			//	//double max = *std::max_element(sizes.begin(),sizes.end());
			//	for (int i = 0; i < sizes.size(); i++){
			//		cv::Point p = ToPixel(omni[0].imgForCalcLikeAll_positioin[i], map, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			//		//int a = (double)(sizes[i]) / max*150.0;
			//		int a = (double)(sizes[i]);
			//		cv::circle(map, p, 3, cv::Scalar(a), -1);
			//	}
			//	/*  画像の切り出し  */
			//	Coor<> upleft(estimated_position_tmp.back().x - CUT_MAP_RADIUS_X, estimated_position_tmp.back().y - CUT_MAP_RADIUS_Y);
			//	//Coor<> upleft(estimated_position.back().x - CUT_MAP_RADIUS_X, estimated_position.back().y - CUT_MAP_RADIUS_Y);
			//	Coor<> rect(CUT_MAP_RADIUS_X*2.0, CUT_MAP_RADIUS_Y*2.0);
			//	cv::Point upleft_pix = ToPixel(upleft, map, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//
			//	cv::Size rect_pix = ToPixelSize(rect, map, MAP_RES);
			//	if (upleft_pix.x < 0)	upleft_pix.x = 0;
			//	if (upleft_pix.y < 0)	upleft_pix.y = 0;
			//	if (upleft_pix.x >= map.cols - rect_pix.width)	upleft_pix.x = map.cols - rect_pix.width;
			//	if (upleft_pix.y >= map.rows - rect_pix.height)	upleft_pix.y = map.rows - rect_pix.height;
			//	map = cv::Mat(map, cv::Rect(upleft_pix, rect_pix));
			//	cv::flip(map, map, 0);
			//	//cv::namedWindow("img_sim");
			//	cv::imshow("img_sim", map);
			//	cv::waitKey(1);
			//}

			lap2 = clock();

			/* ThetaをLRFで強制 */
			estimateThetaByOnlyLRF();

			/**********************************************************/
			//	正規化
			/**********************************************************/

			calcParticleLikelihood();
			Normalize(lid2_l_likelihood);
			Normalize(omni_likelihood);
			Normalize(gpgga_likelihood);


			lap3 = clock();

			decideUseSensor();

			lap4 = clock();


			/**********************************************************/
			//	統合
			/**********************************************************/


			stat2.add_data(lid2_l_likelihood);
			stat2.add_data(omni_likelihood);
			stat2.add_data(gpgga_likelihood);


			/*  バイアス分布以外を積による統合  */
			if (use_.empty()){
				std::cout << "non use" << std::endl;
				exit(0);
			}
			fusion_likelihood = stat2.Product(use_.back(), INTEGRATE_TYPE);
			Normalize(fusion_likelihood);	//	正規化

			/*  パーティクルに尤度をset  */
			for (int pi = 0; pi < SAMPLE_SIZE; pi++) {
				getParticles()[pi]->setWeight(fusion_likelihood[pi]);
			}

			///*  重み付き平均による推定位置算出  */
			//estimated_position.push_back(getWeightedMean());
			lap5 = clock();

			std::cout << (int)(lap1 - lap0) << "," << (int)(lap2 - lap1) << "," << (int)(lap3 - lap2) << "," << (int)(lap4 - lap3) << "," << (int)(lap5 - lap4) << std::endl;
			break;


		case TRIAL_3SENSORS_PEARSON:
			lap0 = clock();

			prepareForCalcLikelihood();

			/* ThetaをLRFで強制 */
			estimateThetaByOnlyLRF();


			/* 類似性評価用サンプル */
			generateStatParticle();
			calcStatParticleLikelihood();


			lap1 = clock();
			/* 位置推定用パーティクル */
			calcParticleLikelihood();


			/* 画像位置における類似度 */
			//if (!estimated_position.empty()){
			//	cv::Mat map = map_img_tmp.clone();
			//	std::vector<int> sizes;
			//	for (int i = 0; i < omni[0].imgForCalcLikeAll_positioin.size(); i++){
			//		int size = omni[0].matches_used_calc[i].size();
			//		std::cout << size << std::endl;
			//		sizes.push_back(size);
			//	}
			//	//double max = *std::max_element(sizes.begin(),sizes.end());
			//	for (int i = 0; i < sizes.size(); i++){
			//		cv::Point p = ToPixel(omni[0].imgForCalcLikeAll_positioin[i], map, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			//		//int a = (double)(sizes[i]) / max*150.0;
			//		int a = (double)(sizes[i]);
			//		cv::circle(map, p, 3, cv::Scalar(a), -1);
			//	}
			//	/*  画像の切り出し  */
			//	Coor<> upleft(estimated_position_tmp.back().x - CUT_MAP_RADIUS_X, estimated_position_tmp.back().y - CUT_MAP_RADIUS_Y);
			//	//Coor<> upleft(estimated_position.back().x - CUT_MAP_RADIUS_X, estimated_position.back().y - CUT_MAP_RADIUS_Y);
			//	Coor<> rect(CUT_MAP_RADIUS_X*2.0, CUT_MAP_RADIUS_Y*2.0);
			//	cv::Point upleft_pix = ToPixel(upleft, map, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//
			//	cv::Size rect_pix = ToPixelSize(rect, map, MAP_RES);
			//	if (upleft_pix.x < 0)	upleft_pix.x = 0;
			//	if (upleft_pix.y < 0)	upleft_pix.y = 0;
			//	if (upleft_pix.x >= map.cols - rect_pix.width)	upleft_pix.x = map.cols - rect_pix.width;
			//	if (upleft_pix.y >= map.rows - rect_pix.height)	upleft_pix.y = map.rows - rect_pix.height;
			//	map = cv::Mat(map, cv::Rect(upleft_pix, rect_pix));
			//	cv::flip(map, map, 0);
			//	//cv::namedWindow("img_sim");
			//	cv::imshow("img_sim", map);
			//	cv::waitKey(1);
			//}

			lap2 = clock();


			decideUseSensor();


			/**********************************************************/
			//	統合
			/**********************************************************/



			lap3 = clock();


			lap4 = clock();

			stat2.add_data(lid2_l_likelihood);
			stat2.add_data(omni_likelihood);
			stat2.add_data(gpgga_likelihood);


			/*  バイアス分布以外を積による統合  */
			fusion_likelihood = stat2.Product(use_.back(), INTEGRATE_TYPE);
			Normalize(fusion_likelihood);	//	正規化

			/*  パーティクルに尤度をset  */
			for (int pi = 0; pi < SAMPLE_SIZE; pi++) {
				getParticles()[pi]->setWeight(fusion_likelihood[pi]);
			}

			///*  重み付き平均による推定位置算出  */
			//estimated_position.push_back(getWeightedMean());
			lap5 = clock();

			std::cout << (int)(lap1 - lap0) << "," << (int)(lap2 - lap1) << "," << (int)(lap3 - lap2) << "," << (int)(lap4 - lap3) << "," << (int)(lap5 - lap4) << std::endl;

			break;


		case TRIAL_3SENSORS_PEARSON_NONSTAT:
		case TRIAL_3SENSORS_SUYAMA_NONSTAT:

			lap0 = clock();

			prepareForCalcLikelihood();

			/* ThetaをLRFで強制 */
			estimateThetaByOnlyLRF();


			lap1 = clock();

			/* 画像位置における類似度 */
			//if (!estimated_position.empty()){
			//	cv::Mat map = map_img_tmp.clone();
			//	std::vector<int> sizes;
			//	for (int i = 0; i < omni[0].imgForCalcLikeAll_positioin.size(); i++){
			//		int size = omni[0].matches_used_calc[i].size();
			//		std::cout << size << std::endl;
			//		sizes.push_back(size);
			//	}
			//	//double max = *std::max_element(sizes.begin(),sizes.end());
			//	for (int i = 0; i < sizes.size(); i++){
			//		cv::Point p = ToPixel(omni[0].imgForCalcLikeAll_positioin[i], map, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			//		//int a = (double)(sizes[i]) / max*150.0;
			//		int a = (double)(sizes[i]);
			//		cv::circle(map, p, 3, cv::Scalar(a), -1);
			//	}
			//	/*  画像の切り出し  */
			//	Coor<> upleft(estimated_position_tmp.back().x - CUT_MAP_RADIUS_X, estimated_position_tmp.back().y - CUT_MAP_RADIUS_Y);
			//	//Coor<> upleft(estimated_position.back().x - CUT_MAP_RADIUS_X, estimated_position.back().y - CUT_MAP_RADIUS_Y);
			//	Coor<> rect(CUT_MAP_RADIUS_X*2.0, CUT_MAP_RADIUS_Y*2.0);
			//	cv::Point upleft_pix = ToPixel(upleft, map, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//
			//	cv::Size rect_pix = ToPixelSize(rect, map, MAP_RES);
			//	if (upleft_pix.x < 0)	upleft_pix.x = 0;
			//	if (upleft_pix.y < 0)	upleft_pix.y = 0;
			//	if (upleft_pix.x >= map.cols - rect_pix.width)	upleft_pix.x = map.cols - rect_pix.width;
			//	if (upleft_pix.y >= map.rows - rect_pix.height)	upleft_pix.y = map.rows - rect_pix.height;
			//	map = cv::Mat(map, cv::Rect(upleft_pix, rect_pix));
			//	cv::flip(map, map, 0);
			//	//cv::namedWindow("img_sim");
			//	cv::imshow("img_sim", map);
			//	cv::waitKey(1);
			//}

			lap2 = clock();


			/**********************************************************/
			//	正規化
			/**********************************************************/

			calcParticleLikelihood();
			Normalize(lid2_l_likelihood);
			Normalize(omni_likelihood);
			Normalize(gpgga_likelihood);


			lap3 = clock();

			decideUseSensor();

			lap4 = clock();


			/**********************************************************/
			//	統合
			/**********************************************************/


			stat2.add_data(lid2_l_likelihood);
			stat2.add_data(omni_likelihood);
			stat2.add_data(gpgga_likelihood);


			/*  バイアス分布以外を積による統合  */
			if (use_.empty()){
				std::cout << "non use" << std::endl;
				exit(0);
			}
			fusion_likelihood = stat2.Product(use_.back(), INTEGRATE_TYPE);
			Normalize(fusion_likelihood);	//	正規化

			/*  パーティクルに尤度をset  */
			for (int pi = 0; pi < SAMPLE_SIZE; pi++) {
				getParticles()[pi]->setWeight(fusion_likelihood[pi]);
			}

			///*  重み付き平均による推定位置算出  */
			//estimated_position.push_back(getWeightedMean());
			lap5 = clock();

			std::cout << (int)(lap1 - lap0) << "," << (int)(lap2 - lap1) << "," << (int)(lap3 - lap2) << "," << (int)(lap4 - lap3) << "," << (int)(lap5 - lap4) << std::endl;

			break;



		case TRIAL_3SENSORS_LRF_GPS:

			lap0 = clock();

			prepareForCalcLikelihood();

			lap1 = clock();

			/* 画像位置における類似度 */
			//if (!estimated_position.empty()){
			//	cv::Mat map = map_img_tmp.clone();
			//	std::vector<int> sizes;
			//	for (int i = 0; i < omni[0].imgForCalcLikeAll_positioin.size(); i++){
			//		int size = omni[0].matches_used_calc[i].size();
			//		std::cout << size << std::endl;
			//		sizes.push_back(size);
			//	}
			//	//double max = *std::max_element(sizes.begin(),sizes.end());
			//	for (int i = 0; i < sizes.size(); i++){
			//		cv::Point p = ToPixel(omni[0].imgForCalcLikeAll_positioin[i], map, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			//		//int a = (double)(sizes[i]) / max*150.0;
			//		int a = (double)(sizes[i]);
			//		cv::circle(map, p, 3, cv::Scalar(a), -1);
			//	}
			//	/*  画像の切り出し  */
			//	Coor<> upleft(estimated_position_tmp.back().x - CUT_MAP_RADIUS_X, estimated_position_tmp.back().y - CUT_MAP_RADIUS_Y);
			//	//Coor<> upleft(estimated_position.back().x - CUT_MAP_RADIUS_X, estimated_position.back().y - CUT_MAP_RADIUS_Y);
			//	Coor<> rect(CUT_MAP_RADIUS_X*2.0, CUT_MAP_RADIUS_Y*2.0);
			//	cv::Point upleft_pix = ToPixel(upleft, map, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//
			//	cv::Size rect_pix = ToPixelSize(rect, map, MAP_RES);
			//	if (upleft_pix.x < 0)	upleft_pix.x = 0;
			//	if (upleft_pix.y < 0)	upleft_pix.y = 0;
			//	if (upleft_pix.x >= map.cols - rect_pix.width)	upleft_pix.x = map.cols - rect_pix.width;
			//	if (upleft_pix.y >= map.rows - rect_pix.height)	upleft_pix.y = map.rows - rect_pix.height;
			//	map = cv::Mat(map, cv::Rect(upleft_pix, rect_pix));
			//	cv::flip(map, map, 0);
			//	//cv::namedWindow("img_sim");
			//	cv::imshow("img_sim", map);
			//	cv::waitKey(1);
			//}

			lap2 = clock();


			/**********************************************************/
			//	正規化
			/**********************************************************/

			calcParticleLikelihood();
			Normalize(lid2_l_likelihood);
			omni_likelihood = std::vector<double>(omni_likelihood.size(), 1.0);
			Normalize(omni_likelihood);
			Normalize(gpgga_likelihood);


			lap3 = clock();

			decideUseSensor();

			lap4 = clock();


			/**********************************************************/
			//	統合
			/**********************************************************/


			stat2.add_data(lid2_l_likelihood);
			stat2.add_data(omni_likelihood);
			stat2.add_data(gpgga_likelihood);


			/*  バイアス分布以外を積による統合  */
			if (use_.empty()){
				std::cout << "non use" << std::endl;
				exit(0);
			}
			fusion_likelihood = stat2.Product(use_.back(), INTEGRATE_TYPE);
			Normalize(fusion_likelihood);	//	正規化

			/*  パーティクルに尤度をset  */
			for (int pi = 0; pi < SAMPLE_SIZE; pi++) {
				getParticles()[pi]->setWeight(fusion_likelihood[pi]);
			}

			///*  重み付き平均による推定位置算出  */
			//estimated_position.push_back(getWeightedMean());
			lap5 = clock();

			std::cout << (int)(lap1 - lap0) << "," << (int)(lap2 - lap1) << "," << (int)(lap3 - lap2) << "," << (int)(lap4 - lap3) << "," << (int)(lap5 - lap4) << std::endl;






		default:
			std::cout << "Error 'TRIAL': " << TRIAL_TYPE << "FILE: " << __FILE__ << " LINE: " << __LINE__ << std::endl;
			exit(0);
			break;
		}

		std::cout << "Complete 'Likelihood' " << std::endl;
	}

	void finalize(int signal) {
		switch (signal)
		{
		case SIGINT:
			writeOutputEstiPositionAll(ofpath_i);
			writeOutputEstiErrorAll(ofpath_i);
			writeUseSensorAll(ofpath_i);
			writeEliminateSensorForAnalysisAll(ofpath_i);
			writeSimilarityTableAll(ofpath_i);
			writeSimilarTableAll_(ofpath_i);
			//loca.writeSimilarTable_();

#if SAVE_PARTICLE_STATES
			writeOutputParticleAll(ofpath_i);
#endif
#if SAVE_STAT_PARTICLE_STATES
			writeOutputStatParticleAll(ofpath_i);
#endif

			writeSizeOfEstiPosition(ofpath_i);
			writeLastInpugFileNumber(ofpath_i);

			writeRoute(ofpath_i);
			writeParState(ofpath_i);
			particle_video.~VideoWriter();
			particle_large_video.~VideoWriter();
			weighted_stat_particle_video.~VideoWriter();
			measurement_data_video.~VideoWriter();
			finish = true;
			break;
		default:
			break;
		}
	}


	void setOutputValue(std::string ofpath) {
		std::vector<Position<>> par;
		for (int i = 0; i < getParticles().size(); i++) {
			par.push_back(*getParticles()[i]->getState());
		}
		all_particles.push_back(par);
		all_stat_particles.push_back(stat_particles);
		all_lid2_l_likelihood.push_back(lid2_l_likelihood);
		all_lid2_u_likelihood.push_back(lid2_u_likelihood);
		all_omni_likelihood.push_back(omni_likelihood);
		all_gpgga_likelihood.push_back(gpgga_likelihood);
		all_fusion_likelihood.push_back(fusion_likelihood);
		all_stat_lid2_l_likelihood.push_back(stat_lid2_l_likelihood);
		all_stat_lid2_u_likelihood.push_back(stat_lid2_u_likelihood);
		all_stat_omni_likelihood.push_back(stat_omni_likelihood);
		all_stat_gpgga_likelihood.push_back(stat_gpgga_likelihood);
		result_time.push_back(esti_time);
		all_stock_tidx.push_back(tidx);
		diff_time_ini_now.push_back(MyTime::diff(ini_time, esti_time));

		std::cout << "Complete 'setOutputValue' " << std::endl;

	}

	void setOutputAllParticlesAfterResampling(){
		std::vector<Position<>> par;
		for (int i = 0; i < getParticles().size(); i++) {
			par.push_back(*getParticles()[i]->getState());
		}
		all_particles_after_resampling.push_back(par);
		std::cout << "Complete setOutputAllParticlesAfterResampling" << std::endl;
	}



	/*  計測データの初期化  */
	void clearMeasurement()
	{
		for (int i = 0; i < LIKELIHOOD_THREAD; i++) {
			lid2_l[i].clearMeasurement();
			lid2_u[i].clearMeasurement();
			omni[i].clearMeasurement();
			gpgga[i].clearMeasurement();
		}
		all_meas_lrf_l.erase(all_meas_lrf_l.begin());
		all_meas_lrf_u.erase(all_meas_lrf_u.begin());
		all_meas_keypoints.erase(all_meas_keypoints.begin());
		all_meas_desriptor.erase(all_meas_desriptor.begin());
		all_meas_gps.erase(all_meas_gps.begin());
		all_meas_odometry.erase(all_meas_odometry.begin());
		all_meas_time.erase(all_meas_time.begin());
		LocalizationPF::swapOdometry();
		stat_particles.clear();
		stat_lid2_l_likelihood.clear();
		stat_lid2_u_likelihood.clear();
		stat_omni_likelihood.clear();
		stat_gpgga_likelihood.clear();
		lid2_l_likelihood.clear();
		lid2_u_likelihood.clear();
		omni_likelihood.clear();
		gpgga_likelihood.clear();
		fusion_likelihood.clear();
		all_omni_img_sim.clear();
		all_omni_img_sim_pos.clear();
	}




	/**********************************************************/
	//  メンバ変数
	/**********************************************************/

	TrialType trial_type;
	bool localization_only_true_position;
	int sensor_num;

	/*  環境データ  */
	GlobalToLocal gl2lc;							//	IniPosには地図の原点GL座標を代入
	cv::Mat map_img;							//  障害物地図（白黒）
	cv::Mat map_img_color;						//  障害物地図（カラー）
	cv::Mat map_img_clone;							//	描画用
	cv::Mat map_img_pointcloud;

	cv::Mat map_img_lower;
	cv::Mat map_img_upper;
	cv::Mat map_img_clone2;
	cv::Mat map_img_clone3;

	/* Reliability Map */
	cv::Mat_<float> reliabity_map_lrf_l;
	cv::Mat_<float> reliabity_map_lrf_u;
	cv::Mat_<float> reliabity_map_omni;
	cv::Mat_<float> reliabity_map_gps;

	bool finish = false;
	bool read_all_measurement_ = false;
	bool fin_read_env = false;

	int meas_step = FIRST_STEP - 1;
	int read_meas_no = FIRST_NO;
	int read_meas_step = FIRST_STEP;

	const bool exist_true_position;

	/* スレッド */
	std::thread set_environment_thread;
	std::thread read_measurement_thread;
	std::thread create_movies_thread;
	bool fin_movie_creator_ = false;


	/*  計測データ */
	Position<>* odometry1 = nullptr;				//	オドメトリデータ
	Position<>* odometry2 = nullptr;

	/* 計測データの保管 */
	std::vector<Position<>> all_meas_odometry;
	std::deque<MyTime> all_meas_time;
	std::vector<std::vector<Polar<>>> all_meas_lrf_l;
	std::vector<std::vector<Polar<>>> all_meas_lrf_u;
	std::vector<cv::Mat> all_meas_img;
	std::vector<std::vector<cv::KeyPoint>> all_meas_keypoints;
	std::vector<cv::Mat> all_meas_desriptor;
	std::vector<std::string> all_meas_gps;
	std::vector<Position<>> all_meas_gps_pos;
	std::vector<Position<>> estimated_position;		//	重み付き平均による推定位置
	std::vector<MyTime> result_time;
	std::vector<std::vector<Position<>>> all_particles;
	std::vector<std::vector<Position<>>> all_particles_after_resampling;
	std::vector<std::vector<Position<>>> all_stat_particles;
	std::vector<std::vector<double>> all_lid2_l_likelihood;
	std::vector<std::vector<double>> all_lid2_u_likelihood;
	std::vector<std::vector<double>> all_omni_likelihood;
	std::vector<std::vector<double>> all_gpgga_likelihood;
	std::vector<std::vector<double>> all_fusion_likelihood;
	std::vector<std::vector<double>> all_stat_lid2_l_likelihood;
	std::vector<std::vector<double>> all_stat_lid2_u_likelihood;
	std::vector<std::vector<double>> all_stat_omni_likelihood;
	std::vector<std::vector<double>> all_stat_gpgga_likelihood;
	std::vector<std::vector<Position<>>> all_omni_img_sim_pos;
	std::vector<std::vector<double>> all_omni_img_sim;
	std::vector<double> all_th;
	std::vector<int> all_stock_tidx;
	std::vector<double> diff_time_ini_now;
	std::vector<Position<>> all_error;
	std::vector<MyTime> error_time;


	/*  ロボットの真の位置  */
	std::vector<Position<>>* true_position = nullptr;	//	真の位置
	std::vector<MyTime> true_time;
	int tidx = TRUE_IDX_INI - 1;	//	真の位置のindex
	int ini_tidx;	// 最初の真の位置
	Position<> ini_position;						//	自己位置推定のスタート（真の位置を与える）

	int movie_step = 0;
	/*  ロボットの推定位置  */

	MyTime esti_time;

	bool init = true;;

	/*  推定位置と真の位置の誤差  */
	//std::vector<Position<>> error;

	/*  パーティクルフィルタのparameter  */
	int sample_size;
	Position<> ini_sample_radius;					//	初期パーティクルのサンプル半径
	Position<> trans_par_sys_var;					//	パーティクル遷移時に誤差を付加
	Polar<> odometry_system_noise;					//	オドメトリ誤差分散

	/*  パーティクルの動画  */
	cv::VideoWriter particle_video;
	cv::Mat particle_img;
	cv::VideoWriter weighted_stat_particle_video;
	cv::Mat weighted_stat_particle_img;
	cv::VideoWriter measurement_data_video;
	cv::Mat measurement_data_img;
	cv::VideoWriter particle_large_video;
	cv::Mat particle_large_img;

	/*  現在のステップ  */
	int now_step = 0;
	int add_measurement_movie_step = 0;
	bool fin_add_measurement_movie_ = false;
	int ini_step;

	/*  何回目の計測か */
	int no = FIRST_NO;
	int ini_no;

	/* 出力用 */


	/* 位置推定開始時刻 */
	MyTime ini_time;

	std::vector<std::thread> likelihood_threads;

	/*  尤度格納  */
	std::vector<double> lid2_l_likelihood;
	std::vector<double> lid2_u_likelihood;
	std::vector<double> omni_likelihood;
	std::vector<double> gpgga_likelihood;
	std::vector<double> fusion_likelihood;

	/*  センサ統合関係  */
	std::vector<Position<>> stat_particles;
	Position<> stat_sample_radius;
	std::vector<double> stat_lid2_l_likelihood;
	std::vector<double> stat_lid2_u_likelihood;
	std::vector<double> stat_omni_likelihood;
	std::vector<double> stat_gpgga_likelihood;

	/*  各センサのクラス  */
	std::vector<Lidar2d> lid2_l;
	std::vector<Lidar2d> lid2_u;
	std::vector<OmniCamera> omni;
	std::vector<Gps> gpgga;


	std::vector<std::vector<bool>> use_;							// 	各ステップで選択されたセンサ
	std::vector<std::vector<std::vector<double>>> similarity_table;	//	センサから得られる確率分布間の類似度
	std::vector<std::vector<std::vector<bool>>> similar_table_;		//	センサから得られる確率分布間を類似・非類似で閾値処理

	std::vector<std::vector<bool>> use_sim_;

	cv::Mat ParticleVideo;
	cv::Mat WeightedStatParImg;

	std::string ofpath_i;
};