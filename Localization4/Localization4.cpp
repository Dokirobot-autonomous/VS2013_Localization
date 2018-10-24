// Localization.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"

#include "include\myfun.h"
#include "include/mycv.h"
#include "include/LocalizationPF.h"

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




	/**********************************************************/
	//	ファイルの読み込み
	/**********************************************************/




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

bool finish = false;

void finalize(int signal){
	finish = true;
	return;
}

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
	std::vector<Position<>> esti_position2;

	Localization loca;

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
			ofpath_i = OFPATH_3SENSORS_SIMULTANEOUS + std::to_string(i) + "/";
			break;
		case TRIAL_3SENSORS_PEARSON:
			std::cout << "TRIAL_3SENSORS_PEARSON! " << i << std::endl;
			ofpath_i = OFPATH_3SENSORS_PEARSON + std::to_string(i) + "/";
			break;
		case TRIAL_3SENSORS_PEARSON_NONSTAT:
			std::cout << "TRIAL_3SENSORS_PEARSON_NONSTAT! " << i << std::endl;
			ofpath_i = OFPATH_3SENSORS_PEARSON_NONSTAT + std::to_string(i) + "/";
			break;
		case TRIAL_3SENSORS_SUYAMA_NONSTAT:
			std::cout << "TRIAL_3SENSORS_SUYAMA_NONSTAT!" << std::endl;
			ofpath_i = OFPATH_3SENSORS_SUYAMA_NONSTAT + std::to_string(i) + "/";
			break;
		case TRIAL_3SENSORS_LRF_GPS:
			std::cout << "TRIAL_3SENSORS_LRF_GPS!" << std::endl;
			ofpath_i = OFPATH_3SENSORS_LRF_GPS + std::to_string(i) + "/";
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
		//	std::string filename = "F://Data/Localization/Environment/171031/1237/one_skip/lrf_lower/pos.csv";
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

		while (!finish && !loca.finish)
		{
			signal(SIGINT, finalize);

			//loca.setMeasurement();
			loca.readMeasurement1();

			if (loca.localization_only_true_position){
				std::cout << loca.true_time[loca.tidx] << "," << loca.esti_time << std::endl;
				if (loca.true_time[loca.tidx] != loca.esti_time){
					loca.clearMeasurement();
					continue;
				}
				else{
					loca.sampling(loca.true_position->at(loca.tidx));
				}
			}

			if (finish || loca.finish) {
				break;
			}

			if (!loca.init && !loca.localization_only_true_position){
				loca.Transition();
			}

			clock_t lap0 = clock();

			loca.Likelihood();

			clock_t lap1 = clock();
			loca.calcEPos();
			clock_t lap2 = clock();
			if (loca.exist_true_position){
				loca.calcError();
			}
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
			case TRIAL_3SENSORS_LRF_GPS:
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

			if (MODE_TEST){
				loca.debug();
			}

			loca.clearMeasurement();
			clock_t lap8 = clock();


			std::cout << (int)(lap1 - lap0) << "," << (int)(lap2 - lap1) << "," << (int)(lap3 - lap2) << "," << (int)(lap4 - lap3) << "," << (int)(lap5 - lap4) << "," << (int)(lap6 - lap5) << "," << (int)(lap7 - lap6) << "," << (int)(lap8 - lap7) << std::endl;

			if (loca.now_step >= LAST_STEP - FIRST_STEP || loca.tidx >= TRUE_IDX_LAST){
				finish = true;
			}

			//// errorが２ｍをこえたばあい、しゅうりょう
			//if (!loca.all_error.empty()){
			//	double error = std::sqrt(loca.all_error.back().x*loca.all_error.back().x + loca.all_error.back().y*loca.all_error.back().y);
			//	if (error > 2000){
			//		break;
			//	}
			//}



			if (finish || loca.finish) {
				break;
			}

			if (loca.init = true) loca.init = false;

			if (loca.localization_only_true_position){
				loca.tidx++;
			}

		}

		/* Finalize */
		switch (loca.trial_type)
		{

		case TRIAL_SIMULTANEOUS:
		case TRIAL_3SENSORS_SIMULATNEOUS:
		case TRIAL_3SENSORS_LRF_GPS:
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