// Localization.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"

#include "include\myfun.h"
#include "include/mycv.h"
#include "include/LocalizationPF_3sensors.h"

/**********************************************************/
//	プリプロセッサ
/**********************************************************/



#define FLAG_FIRST 1	//	first stepから実行する場合は1




/**********************************************************/
//	実行クラス
/**********************************************************/

class Localization : public LocalizationPF
{
public:
	/*  コンストラクタ  */
	Localization()
	{
		stat_sample_radius.set(STAT_SAMPLE_RADIUS_X, STAT_SAMPLE_RADIUS_Y, STAT_SAMPLE_RADIUS_R);
	};
	/*  デストラクタ  */
	~Localization() {};


	void initialize() {

		//*odometry1 = Position<>(0.0, 0.0, 0.0);

		//for (auto& tmp : lid2_u)
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
		stat_lid2_u_likelihood.clear();
		stat_omni_likelihood.clear();
		stat_gpgga_likelihood.clear();
		lid2_u_likelihood.clear();
		omni_likelihood.clear();
		gpgga_likelihood.clear();
		fusion_likelihood.clear();

	}

	/**********************************************************/
	//	ファイルの読み込み
	/**********************************************************/


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
			thread_meas = std::thread(&LocalizationPF::addMeasurementVideo2, this);
			thread_par = std::thread(&LocalizationPF::addFlameParticleVideo2, this);
			thread_par_large = std::thread(&LocalizationPF::addFlameParticleLargeVideo2, this);
			thread_meas.join();
			thread_par.join();
			thread_par_large.join();
			break;
		case TRIAL_PEARSON:
		case TRIAL_SUYAMA_STAT:
		case TRIAL_3SENSORS_PEARSON:
			thread_meas = std::thread(&LocalizationPF::addMeasurementVideo2, this);
			thread_par = std::thread(&LocalizationPF::addFlameParticleVideo2, this);
			thread_par_large = std::thread(&LocalizationPF::addFlameParticleLargeVideo2, this);
			thread_stat_par = std::thread(&LocalizationPF::addFlameWeightedStatParImg2, this);
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

	/* Movie 生成 */
	void setMovieCreaterThread() {
		//create_movies_thread = std::thread(&Localization::movieCreater, this);
		//create_movies_thread.detach();
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



	void readEstiPositionSLAM() {

	}

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

	void calcLikelihood() {




	};

	void calcParticleLikelihood(){

		std::vector<Particle<Position<>>*>& get_particles = getParticles();
		std::vector<double>& lid2_u_likelihood_tmp = lid2_u_likelihood;
		std::vector<double>& omni_likelihood_tmp = omni_likelihood;
		std::vector<double>& gpgga_likelihood_tmp = gpgga_likelihood;
		int no_tmp = no;
		Position<> ini_position_tmp = ini_position;
		std::vector<Position<>> estimated_position_tmp = estimated_position;
		cv::Mat& map_img_tmp = map_img;

		// resize
		lid2_u_likelihood.resize(getParticles().size());
		omni_likelihood.resize(getParticles().size());
		gpgga_likelihood.resize(getParticles().size());
		fusion_likelihood.resize(getParticles().size());


		for (int th = 0; th < LIKELIHOOD_THREAD; th++)
		{
			Lidar2d& lid2_u_tmp = lid2_u[th];
			OmniCamera& omni_tmp = omni[th];
			Gps& gpgga_tmp = gpgga[th];
			likelihood_threads[th] = std::thread([th, &lid2_u_tmp, &omni_tmp, &gpgga_tmp, &lid2_u_likelihood_tmp, &omni_likelihood_tmp, &gpgga_likelihood_tmp, no_tmp, ini_position_tmp, estimated_position_tmp, get_particles, map_img_tmp] {

				/*  Lidar2D  */
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
					//	lid2_u_likelihood_tmp[pi] = 0.0;
					//	omni_likelihood_tmp[pi] = 0.0;
					//	gpgga_likelihood_tmp[pi] = 0.0;
					//	continue;
					//}
					/*  尤度算出  */
					assert(get_particles.size() == lid2_u_likelihood_tmp.size());
					assert(get_particles.size() == omni_likelihood_tmp.size());
					assert(get_particles.size() == gpgga_likelihood_tmp.size());

					//clock_t lap0 = clock();
					double lid2_u_w = lid2_u_tmp.getLikelihoodICP(*get_particles[pi]->getState(), ICP_PAIR_NUM_TH_L);
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
			Normalize(lid2_u_likelihood);
			Normalize(omni_likelihood);
			Normalize(gpgga_likelihood);
			addBias(lid2_u_likelihood);
			addBias(omni_likelihood);
			addBias(gpgga_likelihood);
			Normalize(lid2_u_likelihood);
			Normalize(omni_likelihood);
			Normalize(gpgga_likelihood);
		}
		else{
			Normalize(lid2_u_likelihood);
			Normalize(omni_likelihood);
			Normalize(gpgga_likelihood);
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
		stat_lid2_u_likelihood.resize(stat_particles.size());
		stat_omni_likelihood.resize(stat_particles.size());
		stat_gpgga_likelihood.resize(stat_particles.size());


		/**********************************************************/
		//	尤度算出の準備
		/**********************************************************/

		std::vector<double>& stat_lid2_u_likelihood_tmp = stat_lid2_u_likelihood;
		std::vector<double>& stat_omni_likelihood_tmp = stat_omni_likelihood;
		std::vector<double>& stat_gpgga_likelihood_tmp = stat_gpgga_likelihood;
		std::vector<Position<>>& stat_particles_tmp = stat_particles;
		int no_tmp = no;
		Position<>& ini_position_tmp = ini_position;
		std::vector<Position<>>& estimated_position_tmp = estimated_position;
		cv::Mat map_img_tmp = map_img;

		clock_t lap1 = clock();

		for (int th = 0; th < LIKELIHOOD_THREAD; th++)
		{
			Lidar2d& lid2_u_tmp = lid2_u[th];
			OmniCamera& omni_tmp = omni[th];
			Gps& gpgga_tmp = gpgga[th];
			likelihood_threads[th] = std::thread([th, &lid2_u_tmp, &omni_tmp, &gpgga_tmp, &stat_lid2_u_likelihood_tmp, &stat_omni_likelihood_tmp, &stat_gpgga_likelihood_tmp, no_tmp, ini_position_tmp, estimated_position_tmp, stat_particles_tmp, map_img_tmp] {
				/*  Lidar2D  */
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
					//	stat_lid2_u_likelihood_tmp[pi] = 0.0;
					//	stat_omni_likelihood_tmp[pi] = 0.0;
					//	stat_gpgga_likelihood_tmp[pi] = 0.0;
					//	continue;
					//}
					/*  尤度算出  */
					double lid2_u_w = lid2_u_tmp.getLikelihoodICP(stat_particles_tmp[pi], ICP_PAIR_NUM_TH_L);
					double omni_w;
					if (USE_BOF){
						omni_w = omni_tmp.getLikelihoodBofSift(stat_particles_tmp[pi]);
					}
					else{
						omni_w = omni_tmp.getLikelihood(stat_particles_tmp[pi], OMNI_FEATURE);
					}
					double gps_w = gpgga_tmp.getLikelihoodND(stat_particles_tmp[pi]);
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
			Normalize(stat_lid2_u_likelihood);
			Normalize(stat_omni_likelihood);
			Normalize(stat_gpgga_likelihood);
			addBias(stat_lid2_u_likelihood);
			addBias(stat_omni_likelihood);
			addBias(stat_gpgga_likelihood);
			Normalize(stat_lid2_u_likelihood);
			Normalize(stat_omni_likelihood);
			Normalize(stat_gpgga_likelihood);
		}
		else{
			Normalize(stat_lid2_u_likelihood);
			Normalize(stat_omni_likelihood);
			Normalize(stat_gpgga_likelihood);
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
		double th;
		std::vector<bool> similar_tmp_;

		switch (trial_type)
		{
		case TRIAL_SIMULTANEOUS:
		case TRIAL_NON_TIMESEQUENCE_SIMUL:
		case TRIAL_3SENSORS_SIMULATNEOUS:
			all_th.push_back(0.0);
			use_tmp_ = std::vector<bool>(SENSOR_NUM, true);
			use_sim_.push_back(use_tmp_);
			use_.push_back(use_tmp_);
			break;

		case TRIAL_PEARSON:
		case TRIAL_3SENSORS_PEARSON:
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
			num_th = (float)(SENSOR_NUM) / 2.0;
			use_tmp_ = UseIndex(similar_, num_th);
			use_sim_.push_back(use_tmp_);
			if (std::find(use_tmp_.begin(), use_tmp_.end(), true) == use_tmp_.end()) {
				use_tmp_ = std::vector<bool>(SENSOR_NUM, true);
			}
			use_.push_back(use_tmp_);
			break;

		case TRIAL_PEARSON_NONSTAT:
		case TRIAL_NON_TIMESEQUENCE:
		case TRIAL_3SENSORS_PEARSON_NONSTAT:

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
			num_th = (float)(SENSOR_NUM) / 2.0;
			use_tmp_ = UseIndex(similar_, num_th);
			use_sim_.push_back(use_tmp_);
			if (std::find(use_tmp_.begin(), use_tmp_.end(), true) == use_tmp_.end()) {
				use_tmp_ = std::vector<bool>(SENSOR_NUM, true);
			}
			use_.push_back(use_tmp_);
			break;

		case TRIAL_SUYAMA_STAT:

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


			for (int ni = 0; ni < SENSOR_NUM; ni++)
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
				use_tmp_ = std::vector<bool>(SENSOR_NUM, true);
			}
			use_.push_back(use_tmp_);
			all_th.push_back(th);
			break;

		case TRIAL_SUYAMA_NONSTAT:
		case TRIAL_3SENSORS_SUYAMA_NONSTAT:

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

			for (int ni = 0; ni < SENSOR_NUM; ni++)
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
				use_tmp_ = std::vector<bool>(SENSOR_NUM, true);
			}
			use_.push_back(use_tmp_);
			all_th.push_back(th);
			break;

		default:
			std::cout << "Error 'TRIAL': " << TRIAL_TYPE << "FILE: " << __FILE__ << " LINE: " << __LINE__ << std::endl;
			exit(0);
			break;
		}

		std::cout << "Use sensors: " << use_.back() << std::endl;
		std::cout << "Complete decideSensor!" << std::endl;

		std::cout << std::endl;

	}

	void addBias(std::vector<double>& likelihood){
		for (auto& w : likelihood){
			w += W_BIAS;
		}
	}

	/*  計測モデル  */
	void Likelihood()
	{
		Statistics<> stat2;
		clock_t lap0, lap1, lap2, lap3, lap4, lap5;

		switch (trial_type)
		{
		case TRIAL_SIMULTANEOUS:
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


			/**********************************************************/
			//	正規化
			/**********************************************************/

			calcParticleLikelihood();
			Normalize(lid2_u_likelihood);
			Normalize(omni_likelihood);
			Normalize(gpgga_likelihood);


			lap3 = clock();

			decideUseSensor();

			lap4 = clock();


			/**********************************************************/
			//	統合
			/**********************************************************/


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
		case TRIAL_3SENSORS_PEARSON:

			lap0 = clock();

			prepareForCalcLikelihood();

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
		case TRIAL_3SENSORS_PEARSON_NONSTAT:
		case TRIAL_3SENSORS_SUYAMA_NONSTAT:

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
			Normalize(lid2_u_likelihood);
			Normalize(omni_likelihood);
			Normalize(gpgga_likelihood);


			lap3 = clock();

			decideUseSensor();

			lap4 = clock();


			/**********************************************************/
			//	統合
			/**********************************************************/


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
			fin = true;
			break;
		default:
			break;
		}
	}


	/**********************************************************/
	//	ファイル出力
	/**********************************************************/

	///*  途中経過を出力  */
	//void writeOutput()
	//{
	//	/*  推定位置をファイル出力  */
	//	writeOutputEstiPosition(OFPATH_SIMUL);



	//	/*  パーティクル動画にフレームを追加  */
	//	addFlameParticleVideo();

	//};

	/**********************************************************/
	//	メンバ変数
	/**********************************************************/

private:


};





std::vector<Position<>> esti_position2;

Localization loca;

//void finalize(int signal) {
//	/* Finalize */
//	switch (MODE)
//	{
//	
//	case MODE_SIMULTANEOUS:
//		loca.writeOutputEstiPositionAll(loca.ofpath_i);
//		loca.writeOutputEstiErrorAll(loca.ofpath_i);
//		//loca.writeSimilarTable_();
//
//		loca.writeOutputParticleAll(loca.ofpath_i);
//		loca.writeOutputParticleAllAfterResampling(loca.ofpath_i);
//
//		loca.writeSizeOfEstiPosition(loca.ofpath_i);
//		loca.writeLastInpugFileNumber(loca.ofpath_i);
//
//		//while (!loca.fin_movie_creator_);
//
//		loca.measurement_data_video.~VideoWriter();
//		loca.particle_video.~VideoWriter();
//		loca.particle_large_video.~VideoWriter();
//
//		loca.writeRoute(loca.ofpath_i);
//		loca.writeParState(loca.ofpath_i);
//
//		loca.allClear();
//		break;
//
//	case MODE_PEARSON:
//		loca.writeOutputEstiPositionAll(loca.ofpath_i);
//		loca.writeOutputEstiErrorAll(loca.ofpath_i);
//		loca.writeUseSensorAll(loca.ofpath_i);
//		loca.writeEliminateSensorForAnalysisAll(loca.ofpath_i);
//		loca.writeSimilarityTableAll(loca.ofpath_i);
//		loca.writeSimilarTableAll_(loca.ofpath_i);
//		//loca.writeSimilarTable_();
//
//		loca.writeOutputParticleAll(loca.ofpath_i);
//		loca.writeOutputParticleAllAfterResampling(loca.ofpath_i);
//		loca.writeOutputStatParticleAll(loca.ofpath_i);
//
//
//		loca.writeSizeOfEstiPosition(loca.ofpath_i);
//		loca.writeLastInpugFileNumber(loca.ofpath_i);
//
//		//while (!loca.fin_movie_creator_);
//
//		loca.measurement_data_video.~VideoWriter();
//		loca.particle_video.~VideoWriter();
//		loca.particle_large_video.~VideoWriter();
//		loca.weighted_stat_particle_video.~VideoWriter();
//
//
//		loca.writeRoute(loca.ofpath_i);
//		loca.writeParState(loca.ofpath_i);
//
//		loca.allClear();
//		break;
//
//	case MODE_PEARSON_NONSTAT:
//		loca.writeOutputEstiPositionAll(loca.ofpath_i);
//		loca.writeOutputEstiErrorAll(loca.ofpath_i);
//		loca.writeUseSensorAll(loca.ofpath_i);
//		loca.writeEliminateSensorForAnalysisAll(loca.ofpath_i);
//		loca.writeSimilarityTableAll(loca.ofpath_i);
//		loca.writeSimilarTableAll_(loca.ofpath_i);
//		//loca.writeSimilarTable_();
//
//		loca.writeOutputParticleAll(loca.ofpath_i);
//		loca.writeOutputParticleAllAfterResampling(loca.ofpath_i);
//
//		loca.writeSizeOfEstiPosition(loca.ofpath_i);
//		loca.writeLastInpugFileNumber(loca.ofpath_i);
//
//		//while (!loca.fin_movie_creator_);
//
//		loca.measurement_data_video.~VideoWriter();
//		loca.particle_video.~VideoWriter();
//		loca.particle_large_video.~VideoWriter();
//
//		loca.writeRoute(loca.ofpath_i);
//		loca.writeParState(loca.ofpath_i);
//
//		loca.allClear();
//		break;
//
//
//	default:
//		break;
//	}
//}


int _tmain(int argc, _TCHAR* argv[])
{

	loca.setEnvironment();


	//loca.readMeasurement();

	for (int i = FIRST_OUTPUT; i <= LAST_OUTPUT; i++) {


		loca.init = true;
		std::string ofpath_i;

		/* cout MODE */
		switch (loca.trial_type)
		{
		case TRIAL_SIMULTANEOUS:
			std::cout << "SIMULTANEOUS! " << i << std::endl;
			ofpath_i = OFPATH_SIMUL + std::to_string(i) + "/";
			break;
		case TRIAL_PEARSON:
			std::cout << "PEARSON! " << i << std::endl;
			ofpath_i = OFPATH_PEAR + std::to_string(i) + "/";
			break;
		case TRIAL_PEARSON_NONSTAT:
			std::cout << "PEARSON WITHOUT STAT! " << i << std::endl;
			ofpath_i = OFPATH_PEAR_NONSTAT + std::to_string(i) + "/";
			break;
		case TRIAL_NON_TIMESEQUENCE:
			std::cout << "NON TIME SEQUENCE! " << i << std::endl;
			ofpath_i = OFPATH_NON_TIMESEQUENCE + std::to_string(i) + "/";
			break;
		case TRIAL_NON_TIMESEQUENCE_SIMUL:
			std::cout << "NON TIME SEQUENCE SIMULTANEOUS! " << i << std::endl;
			ofpath_i = OFPATH_NON_TIMESEQUENCE_SIMUL + std::to_string(i) + "/";
			break;
		case TRIAL_SUYAMA_STAT:
			std::cout << "SUYAMA STAT! " << i << std::endl;
			ofpath_i = OFPATH_SUYAMA_STAT + std::to_string(i) + "/";
			break;
		case TRIAL_SUYAMA_NONSTAT:
			std::cout << "SUYAMA NONSTAT! " << i << std::endl;
			ofpath_i = OFPATH_SUYAMA_NONSTAT + std::to_string(i) + "/";
			break;
		case TRIAL_3SENSORS_SIMULATNEOUS:
			std::cout << "TRIAL_3SENSORS_SIMULATNEOUS! " << i << std::endl;
			ofpath_i = OFPATH_3SENSORS_SIMULTANEOUS+ std::to_string(i) + "/";
			break;
		case TRIAL_3SENSORS_PEARSON:
			std::cout << "TRIAL_3SENSORS_PEARSON! " << i << std::endl;
			ofpath_i = OFPATH_3SENSORS_PEARSON+ std::to_string(i) + "/";
			break;
		case TRIAL_3SENSORS_PEARSON_NONSTAT:
			std::cout << "TRIAL_3SENSORS_PEARSON_NONSTAT! " << i << std::endl;
			ofpath_i = OFPATH_3SENSORS_PEARSON_NONSTAT + std::to_string(i) + "/";
			break;
		case TRIAL_3SENSORS_SUYAMA_NONSTAT:
			std::cout << "TRIAL_3SENSORS_SUYAMA_NONSTAT!" << std::endl;
			ofpath_i = OFPATH_3SENSORS_SUYAMA_NONSTAT + std::to_string(i) + "/";
			break;
		default:
			std::cout << "Error 'MODE': " << TRIAL_TYPE << "FILE: " << __FILE__ << " LINE: " << __LINE__ << std::endl;
			exit(0);
			break;
		}



		loca.ofpath_i = ofpath_i;

		if (FLAG_FIRST)
		{
			int j = 0;
			loca.initOutput(ofpath_i);

			loca.initPF();

			loca.initialize();


			//loca.iniReadMeasurement();
		}
		else
		{
			loca.readLast();
		}

		//{
		//	std::string filename = "F://Data/Localization/Environment/171031/1237/one_skip/lrf_uower/pos.csv";
		//	std::ifstream ifs(filename);
		//	if (ifs.fail()){
		//		readError(filename);
		//	}
		//	ifs >> esti_position2;
		//}

		while (!loca.fin_read_env){
			std::this_thread::sleep_for(std::chrono::milliseconds(MOVIE_CREATER_SLEEP_MILLISECONDS));
		}

		//loca.setMovieCreaterThread();

		while (!loca.finish)
		{
			//signal(SIGINT, finalize);

			//loca.setMeasurement();
			loca.readMeasurement1();

			if (loca.finish) {
				break;
			}

			if (!loca.init){
				loca.Transition();
			}

			clock_t lap0 = clock();

			loca.Likelihood();

			clock_t lap1 = clock();
			loca.calcEPos();
			clock_t lap2 = clock();
			loca.calcError();
			clock_t lap3 = clock();

			loca.setOutputValue(ofpath_i);
			clock_t lap4 = clock();

			switch (loca.trial_type)
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
				loca.Resample();
				//loca.setOutputAllParticlesAfterResampling();
				break;
			case TRIAL_NON_TIMESEQUENCE:
			case TRIAL_NON_TIMESEQUENCE_SIMUL:
				loca.sampling(loca.estimated_position.back());
				break;
			default:
				std::cout << "Error 'TRIAL': " << TRIAL_TYPE << "FILE: " << __FILE__ << " LINE: " << __LINE__ << std::endl;
				exit(0);
				break;
			}

			clock_t lap5 = clock();
			clock_t lap6 = clock();

			loca.createMovie();
			clock_t lap7 = clock();

			loca.clearMeasurement();
			clock_t lap8 = clock();


			std::cout << (int)(lap1 - lap0) << "," << (int)(lap2 - lap1) << "," << (int)(lap3 - lap2) << "," << (int)(lap4 - lap3) << "," << (int)(lap5 - lap4) << "," << (int)(lap6 - lap5) << "," << (int)(lap7 - lap6) << "," << (int)(lap8 - lap7) << std::endl;

			if (loca.now_step >= LAST_STEP || loca.tidx >= TRUE_IDX_LAST){
				fin = true;
			}

			if (fin) {
				break;
			}
			loca.init = false;
		}

		/* Finalize */
		switch (loca.trial_type)
		{

		case TRIAL_SIMULTANEOUS:
		case TRIAL_3SENSORS_SIMULATNEOUS:
			loca.writeOutputEstiPositionAll(ofpath_i);
			loca.writeOutputEstiErrorAll(ofpath_i);
			//loca.writeSimilarTable_();


			loca.writeOutputParticleAll(ofpath_i);
			//loca.writeOutputParticleAllAfterResampling(ofpath_i);
			loca.writeSizeOfEstiPosition(ofpath_i);
			loca.writeLastInpugFileNumber(ofpath_i);

			//while (!loca.fin_movie_creator_);

			loca.measurement_data_video.~VideoWriter();
			loca.particle_video.~VideoWriter();
			loca.particle_large_video.~VideoWriter();

			loca.writeRoute(ofpath_i);
			loca.writeParState(ofpath_i);

			loca.allClear();
			break;

		case TRIAL_PEARSON:
		case TRIAL_SUYAMA_STAT:
		case TRIAL_3SENSORS_PEARSON:
			loca.writeOutputEstiPositionAll(ofpath_i);
			loca.writeOutputEstiErrorAll(ofpath_i);
			loca.writeUseSensorAll(ofpath_i);
			loca.writeEliminateSensorForAnalysisAll(ofpath_i);
			loca.writeSimilarityTableAll(ofpath_i);
			loca.writeSimilarTableAll_(ofpath_i);
			//loca.writeSimilarTable_();

			loca.writeOutputThAll(ofpath_i);


			loca.writeOutputParticleAll(ofpath_i);
			//loca.writeOutputParticleAllAfterResampling(ofpath_i);
			loca.writeOutputStatParticleAll(ofpath_i);


			loca.writeSizeOfEstiPosition(ofpath_i);
			loca.writeLastInpugFileNumber(ofpath_i);

			//while (!loca.fin_movie_creator_);

			loca.measurement_data_video.~VideoWriter();
			loca.particle_video.~VideoWriter();
			loca.particle_large_video.~VideoWriter();
			loca.weighted_stat_particle_video.~VideoWriter();


			loca.writeRoute(ofpath_i);
			loca.writeParState(ofpath_i);

			loca.allClear();
			break;

		case TRIAL_PEARSON_NONSTAT:
		case TRIAL_SUYAMA_NONSTAT:
		case TRIAL_3SENSORS_PEARSON_NONSTAT:
		case TRIAL_3SENSORS_SUYAMA_NONSTAT:
			loca.writeOutputEstiPositionAll(loca.ofpath_i);
			loca.writeOutputEstiErrorAll(loca.ofpath_i);
			loca.writeUseSensorAll(loca.ofpath_i);
			loca.writeEliminateSensorForAnalysisAll(loca.ofpath_i);
			loca.writeSimilarityTableAll(loca.ofpath_i);
			loca.writeSimilarTableAll_(loca.ofpath_i);
			//loca.writeSimilarTable_();

			loca.writeOutputParticleAll(loca.ofpath_i);
			//loca.writeOutputParticleAllAfterResampling(loca.ofpath_i);

			loca.writeSizeOfEstiPosition(loca.ofpath_i);
			loca.writeLastInpugFileNumber(loca.ofpath_i);

			//while (!loca.fin_movie_creator_);

			loca.measurement_data_video.~VideoWriter();
			loca.particle_video.~VideoWriter();
			loca.particle_large_video.~VideoWriter();

			loca.writeRoute(loca.ofpath_i);
			loca.writeParState(loca.ofpath_i);

			loca.allClear();
			break;

		case TRIAL_NON_TIMESEQUENCE:
			loca.writeOutputEstiPositionAll(loca.ofpath_i);
			loca.writeOutputEstiErrorAll(loca.ofpath_i);
			loca.writeUseSensorAll(loca.ofpath_i);
			loca.writeEliminateSensorForAnalysisAll(loca.ofpath_i);
			loca.writeSimilarityTableAll(loca.ofpath_i);
			loca.writeSimilarTableAll_(loca.ofpath_i);
			//loca.writeSimilarTable_();

			loca.writeOutputParticleAll(loca.ofpath_i);

			loca.writeSizeOfEstiPosition(loca.ofpath_i);
			loca.writeLastInpugFileNumber(loca.ofpath_i);

			//while (!loca.fin_movie_creator_);

			loca.measurement_data_video.~VideoWriter();
			loca.particle_video.~VideoWriter();
			loca.particle_large_video.~VideoWriter();

			loca.writeRoute(loca.ofpath_i);
			loca.writeParState(loca.ofpath_i);

			loca.allClear();
			break;

		case TRIAL_NON_TIMESEQUENCE_SIMUL:
			loca.writeOutputEstiPositionAll(loca.ofpath_i);
			loca.writeOutputEstiErrorAll(loca.ofpath_i);

			loca.writeOutputParticleAll(loca.ofpath_i);

			loca.writeSizeOfEstiPosition(loca.ofpath_i);
			loca.writeLastInpugFileNumber(loca.ofpath_i);

			//while (!loca.fin_movie_creator_);

			loca.measurement_data_video.~VideoWriter();
			loca.particle_video.~VideoWriter();
			loca.particle_large_video.~VideoWriter();

			loca.writeRoute(loca.ofpath_i);
			loca.writeParState(loca.ofpath_i);

			loca.allClear();
			break;

		default:
			std::cout << "Error 'TRIAL': " << TRIAL_TYPE << "FILE: " << __FILE__ << " LINE: " << __LINE__ << std::endl;
			exit(0);
			break;
		}
	}

	return 0;
}