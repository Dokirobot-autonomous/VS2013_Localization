// test.cpp : �R���\�[�� �A�v���P�[�V�����̃G���g�� �|�C���g���`���܂��B
//

#include "stdafx.h"

#include "include\myfun.h"
#include "include\mycv.h"
#include "include\parameter.h"

/* �ŏ��Ɏ��s */
//#define END_NO 2
//int _tmain(int argc, _TCHAR* argv[])
//{
//	///* �摜�Q�𓮉�ŕۑ� */
//	//{
//	//	std::string filename = IFPATH_MEAS + "img/img_no1_1th.bmp";
//	//	cv::Mat ini = cv::imread(filename, 1);
//
//	//	/*  �t�@�C���o��  */
//	//	/*  �r�f�I�֌W  */
//	//	// �t�@�C�����I�[�v�����C�r�f�I���C�^��������
//	//	// filename - �o�̓t�@�C����
//	//	// fourcc - �R�[�f�b�N
//	//	// fps - 1 �b������̃t���[����
//	//	// frameSize - �r�f�I�t���[���̃T�C�Y
//	//	// isColor - �r�f�I�X�g���[�����J���[���C�O���[�X�P�[�������w��
//	//	cv::VideoWriter vwr;
//	//	{
//	//		std::string filename = IFPATH_MEAS + "img.avi";
//	//		vwr.open(filename, CV_FOURCC('D', 'I', 'B', ' '), 1, cv::Size(ini.cols, ini.rows));
//	//		if (!vwr.isOpened())	writeError(filename);
//	//	}
//
//	//	int no = 1;
//	//	int fnum = 1;	//	���ݓǂݍ���ł���t�@�C���ԍ�
//
//	//	while (true){
//	//		std::string filename = IFPATH_MEAS + "img / img_no" + std::to_string(no) + "_" + std::to_string(fnum) + "th.bmp";
//	//		cv::Mat mat = cv::imread(filename, 1);
//	//		if (mat.empty()){
//	//			no++;
//	//			fnum = 1;
//	//			if (no > END_NO){
//	//				break;
//	//			}
//	//			continue;
//	//		}
//
//	//		vwr << mat;
//
//	//		std::cout << "Read No: " << no << " Fnum: " << fnum << std::endl;
//
//	//		fnum++;
//	//	}
//	//}
//
//	/* gps���܂Ƃ߂� */
//	{
//		std::vector<std::string> out;	//	�o�̓f�[�^
//
//		std::string filename;
//
//		int no = 1;
//		int now = 0;	//	���ݓǂݍ���ł���t�@�C���ԍ�
//		while (1)
//		{
//			now++;
//			filename = IFPATH_MEAS + "gps/gps_no" + std::to_string(no) + "_" + std::to_string(now) + "th.txt";
//			std::ifstream ifs(filename);
//			if (ifs.fail())	{
//				no++;
//				now = 0;
//				if (no > END_NO){
//					break;
//				}
//				continue;
//			}
//
//			std::string str;
//			ifs >> str;
//			out.push_back(str);
//		}
//
//
//		/*  �t�@�C���o��  */
//		filename = IFPATH_MEAS + "gps.csv";
//		std::ofstream ofs(filename);
//		for (const auto& tmp : out)
//		{
//			ofs << tmp << std::endl;
//		}
//	}
//
//	/* time���܂Ƃ߂� */
//	{
//		std::vector<std::string> out;	//	�o�̓f�[�^
//
//		std::string filename;
//
//		int no = 1;
//		int now = 0;	//	���ݓǂݍ���ł���t�@�C���ԍ�
//		while (1)
//		{
//			now++;
//			filename = IFPATH_MEAS + "time/time_no" + std::to_string(no) + "_" + std::to_string(now) + "th.csv";
//			std::ifstream ifs(filename);
//			if (ifs.fail())	{
//				no++;
//				now = 0;
//				if (no > END_NO){
//					break;
//				}
//				continue;
//			}
//
//			std::string str;
//			ifs >> str;
//			out.push_back(str);
//		}
//
//
//		/*  �t�@�C���o��  */
//		filename = IFPATH_MEAS + "time.csv";
//		std::ofstream ofs(filename);
//		for (const auto& tmp : out)
//		{
//			ofs << tmp << std::endl;
//		}
//	}
//
//	/* odometry���܂Ƃ߂� */
//	{
//		std::vector<std::string> out;	//	�o�̓f�[�^
//
//		std::string filename;
//
//		int no = 1;
//		int now = 0;	//	���ݓǂݍ���ł���t�@�C���ԍ�
//		while (1)
//		{
//			now++;
//			filename = IFPATH_MEAS + "odometry/odometry_no" + std::to_string(no) + "_" + std::to_string(now) + "th.csv";
//			std::ifstream ifs(filename);
//			if (ifs.fail())	{
//				no++;
//				now = 0;
//				if (no > END_NO){
//					break;
//				}
//				continue;
//			}
//
//			std::string str;
//			ifs >> str;
//			out.push_back(str);
//		}
//
//
//		/*  �t�@�C���o��  */
//		filename = IFPATH_MEAS + "odometry.csv";
//		std::ofstream ofs(filename);
//		for (const auto& tmp : out)
//		{
//			ofs << tmp << std::endl;
//		}
//	}
//
//
//	return 0;
//}

/* test */
//int _tmain(int argc, _TCHAR* argv[])
//{
//	for (int i = 0; i < 6; i++){
//#define HOGE i
//		std::cout << HOGE << std::endl;
//	}
//
//	std::cout << 18E+10 *10E-8 << std::endl;
//
//	return 0;
//}

/* video writer�̃e�X�g */
//int _tmain(int argc, _TCHAR* argv[])
//{
//	cv::Mat img11(cv::Size(300, 300), CV_8UC1, cv::Scalar(0));
//	cv::Mat img12(cv::Size(300, 300), CV_8UC1, cv::Scalar(100));
//	cv::Mat img21(cv::Size(300, 300), CV_8UC1, cv::Scalar(200));
//	cv::Mat img22(cv::Size(300, 300), CV_8UC1, cv::Scalar(255));
//
//	//cv::Mat img(cv::Size(600, 600), CV_8UC1,cv::Scalar(255));
//	//img11.copyTo(img(cv::Rect(cv::Point(0, 0), cv::Size(img11.cols, img11.rows))));
//	//img12.copyTo(img(cv::Rect(cv::Point(img11.cols, 0), cv::Size(img12.cols, img12.rows))));
//	//img21.copyTo(img(cv::Rect(cv::Point(0, img11.rows), cv::Size(img21.cols, img21.rows))));
//	//img22.copyTo(img(cv::Rect(cv::Point(img11.cols, img11.rows), cv::Size(img22.cols, img22.rows))));
//
//	cv::VideoWriter vwr;
//	/*  �r�f�I�֌W  */
//	// �t�@�C�����I�[�v�����C�r�f�I���C�^��������
//	// filename - �o�̓t�@�C����
//	// fourcc - �R�[�f�b�N
//	// fps - 1 �b������̃t���[����
//	// frameSize - �r�f�I�t���[���̃T�C�Y
//	// isColor - �r�f�I�X�g���[�����J���[���C�O���[�X�P�[�������w��
//	std::string filename = "./output/particle.avi";
//	vwr.open(filename, CV_FOURCC('X', 'V', 'I', 'D'), 30.0, cv::Size(300, 300));
//	if (!vwr.isOpened())	writeError(filename);
//
//	for (int i = 0; i < 500; i++){
//		cv::Mat img_img(cv::Size(300, 300), CV_8UC3, cv::Scalar(128,128,128));
//		vwr << img_img;
//	}
//
//	// ��320px�A����240px�A3�`�����l���̃C���X�^���X�𐶐�����
//	cv::Mat img(cv::Size(320, 240), CV_8UC3, cv::Scalar(0, 0, 255));
//
//	int fourcc = CV_FOURCC('X', 'V', 'I', 'D');
//	double fps = 30.0;
//	bool isColor = true;
//
//	// �i1�j����t�@�C���������o���������s��
//	cv::VideoWriter writer("./output/videofile.avi", fourcc, fps, img.size(), isColor);
//
//	// �i2�j����t�@�C�������o���̏����ɐ����������`�F�b�N����i���s���Ă���΃G���[�I������j
//	if (!writer.isOpened())
//		return -1;
//
//	for (int i = 0; i < 300; i++)
//	{
//		// �i3�j�摜�f�[�^�𓮉�t�@�C���ɏ����o��
//		writer << img;
//	}
//
//	cv::imshow("img", img);
//	cv::waitKey();
//	cv::imwrite("./output/img.bmp",img);
//
//
//	return 0;
//}

/* �^�̈ʒu�̏o�� */
//#include "include\leica.h"
//#define TH_DISTANCE 1000
//int _tmain(int argc, _TCHAR* argv[])
//{
//	GlobalToLocal gl2lc;
//	std::vector<leica::Data<>> dataset_leica;
//
//	/*  ���Ȉʒu����̏����ʒu  */
//	//	GL���W�n����LC���W�n�ɕϊ�
//	gl2lc.setOriginal(MAP_ORG_LAT, MAP_ORG_LON, MAP_ORG_ELE, MAP_ORG_HEAD);
//
//
//	/* leica �ǂݍ��� */
//	gl2lc.setOriginal(MAP_ORG_LAT, MAP_ORG_LON, MAP_ORG_ELE, MAP_ORG_HEAD);
//	Coor<> leica_coor = gl2lc.getCoor(LEICA_ORG_LAT, LEICA_ORG_LON, LEICA_ORG_ELE);
//	leica::Param<> param_leica(leica_coor);
//	param_leica.setHorizontalErrorDeg(LEICA_HORIZONTAL_ERROR);
//	{
//		std::string filename = IFPATH_MEAS + "true_position/true_position.csv";
//		std::ifstream ifs(filename);
//		if (ifs.fail())	readError(filename);
//		leica::readDeg(ifs, param_leica, dataset_leica);
//	}
//
//	/* Map�̓ǂݍ��� */
//	cv::Mat map_img;
//	{
//		std::string filename = IFPATH_ENV + "lrf_lower/gridmap.bmp";
//		map_img = cv::imread(filename, CV_LOAD_IMAGESCALE);
//		if (map_img.empty())	readError(filename);
//	}
//
//	///* �����ʒu��map���ɂȂ�悤�ɂ��� */
//	//int tidx = 0;
//	//while (true){
//	//	int tmp = tidx + 1;
//	//	while (leica::distance(dataset_leica[tidx], dataset_leica[tmp]) < 500){
//	//		tmp++;
//	//	}
//	//	Position<> ini_position = dataset_leica[tidx].position(dataset_leica[tmp]);
//	//	cv::Point pixel = ToPixel(ini_position, map_img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
//	//	if ((int)map_img.at<unsigned char>(pixel) != 0)	break;
//	//	tidx++;
//	//}
//
//	/* �^�̈ʒu�̎Z�o */
//	std::vector<Position<>> true_position;
//	std::vector<MyTime> true_time;
//	for (int i = tidx; i < dataset_leica.size(); i++){
//		int tmp = i;
//		while (tmp < dataset_leica.size() && leica::distance(dataset_leica[i], dataset_leica[tmp]) < 500){
//			tmp++;
//		}
//		if (tmp >= dataset_leica.size()){
//			tmp--;
//		}
//		Position<> pos = dataset_leica[i].position(dataset_leica[tmp]);
//		MyTime time = dataset_leica[i].time;
//		if (i == tmp){
//			pos.r = true_position.back().r;
//		}
//		true_position.push_back(pos);
//		true_time.push_back(time);
//	}
//
//	/* �^�̈ʒu�̕`�� */
//	for (int i = 0; i < true_position.size(); i++){
//		cv::Point pix = ToPixel(true_position[i], map_img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
//		cv::circle(map_img, pix, 3, cv::Scalar(0, 255, 0), -1);
//	}
//
//	/* imshow */
//	cv::imshow("map", map_img);
//	cv::waitKey();
//
//	return 0;
//}

/* �^�̈ʒu��map��ɏo�� */
//#include "include\leica.h"
//#define TH_DISTANCE 1000
//int _tmain(int argc, _TCHAR* argv[])
//{
//	GlobalToLocal gl2lc;
//	std::vector<leica::Data<>> dataset_leica;
//
//	/*  ���Ȉʒu����̏����ʒu  */
//	//	GL���W�n����LC���W�n�ɕϊ�
//	gl2lc.setOriginal(MAP_ORG_LAT, MAP_ORG_LON, MAP_ORG_ELE, MAP_ORG_HEAD);
//
//
//	/* leica �ǂݍ��� */
//	gl2lc.setOriginal(MAP_ORG_LAT, MAP_ORG_LON, MAP_ORG_ELE, MAP_ORG_HEAD);
//	Coor<> leica_coor = gl2lc.getCoor(LEICA_ORG_LAT, LEICA_ORG_LON, LEICA_ORG_ELE);
//	leica::Param<> param_leica(leica_coor);
//	param_leica.setHorizontalErrorDeg(LEICA_HORIZONTAL_ERROR);
//	{
//		std::string filename = IFPATH_MEAS + "true_position/true_position.csv";
//		std::ifstream ifs(filename);
//		if (ifs.fail())	readError(filename);
//		leica::readDeg(ifs, param_leica, dataset_leica);
//	}
//
//	/* Map�̓ǂݍ��� */
//	cv::Mat map_img;
//	{
//		std::string filename = IFPATH_ENV + "lrf_lower/gridmap.bmp";
//		map_img = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
//		if (map_img.empty())	readError(filename);
//	}
//
//	cv::Point leica_pix = ToPixel(leica_coor, map_img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
//	cv::circle(map_img, leica_pix, 3, cv::Scalar(255, 0, 0), -1);
//
//	/* �^�̈ʒu�̎Z�o */
//	std::vector<Coor<>> true_position;
//	std::vector<MyTime> true_time;
//	for (int i = 0; i < dataset_leica.size(); i++){
//		Coor<> pos = dataset_leica[i].coor();
//		MyTime time = dataset_leica[i].time;
//		true_position.push_back(pos);
//		true_time.push_back(time);
//	}
//
//	/* �^�̈ʒu�̏o�� */
//	std::ofstream ofs("true_position.csv");
//	ofs << "time,x,y,rad" << std::endl;
//	for (int i = 0; i < true_position.size();i++){
//		ofs << true_time[i] << "," << true_position[i] << std::endl;
//	}
//	
//	std::cout << true_position.size() << std::endl;
//	std::cout << true_time.size() << std::endl;
//
//	/* �^�̈ʒu�̕`�� */
//	for (int i = 0; i < true_position.size(); i++){
//	//for (int i = 0; i < 3; i++){
//			cv::Point pix = ToPixel(true_position[i], map_img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
//		cv::circle(map_img, pix, 3, cv::Scalar(0, 255, 0), -1);
//	}
//
//	/* �k�� */
//	cv::resize(map_img, map_img, cv::Size(), 0.5, 0.5);
//	/* imshow */
//	cv::flip(map_img, map_img, 0);
//	cv::imshow("map", map_img);
//	cv::waitKey();
//	cv::imwrite("map.bmp", map_img);
//
//	return 0;
//}

/* �^�̈ʒu�Ɛ���ʒu��map��ɏo�� */
//#include "include\leica.h"
//#define ESTIMATED_POSITION_RED "F://Data/Localization/Result/Visual Studio/Localization.ver2/parking/3sensors_���U��/lower/3sensors_simul1/Data/estimated_position_tmp.csv"
//#define ESTIMATED_POSITION_BLUE "F://Data/Localization/Result/Visual Studio/Localization.ver2/parking/3sensors_���U��/lower/3sensors_pear_nonstat1/Data/estimated_position_tmp.csv"
//#define TH_DISTANCE 1000
//int _tmain(int argc, _TCHAR* argv[])
//{
//	GlobalToLocal gl2lc;
//	std::vector<leica::Data<>> dataset_leica;
//
//	/*  ���Ȉʒu����̏����ʒu  */
//	//	GL���W�n����LC���W�n�ɕϊ�
//	gl2lc.setOriginal(MAP_ORG_LAT, MAP_ORG_LON, MAP_ORG_ELE, MAP_ORG_HEAD);
//
//
//	/* leica �ǂݍ��� */
//	gl2lc.setOriginal(MAP_ORG_LAT, MAP_ORG_LON, MAP_ORG_ELE, MAP_ORG_HEAD);
//	Coor<> leica_coor = gl2lc.getCoor(LEICA_ORG_LAT, LEICA_ORG_LON, LEICA_ORG_ELE);
//	leica::Param<> param_leica(leica_coor);
//	param_leica.setHorizontalErrorDeg(LEICA_HORIZONTAL_ERROR);
//	{
//		std::string filename = IFPATH_MEAS + "true_position/true_position.csv";
//		std::ifstream ifs(filename);
//		if (ifs.fail())	readError(filename);
//		leica::readDeg(ifs, param_leica, dataset_leica);
//	}
//
//	/* ����ʒu�̓ǂݍ��� */
//	//RED
//	std::vector<Position<>> estimated_position_red;
//	{
//		std::string filename = ESTIMATED_POSITION_RED;
//		std::ifstream ifs(filename);
//		if (ifs.fail()){
//			readError(filename);
//		}
//		std::string str;
//		std::getline(ifs, str);
//		ifs >> estimated_position_red;
//	}
//	//BLUE
//	std::vector<Position<>> estimated_position_blue;
//	{
//		std::string filename = ESTIMATED_POSITION_BLUE;
//		std::ifstream ifs(filename);
//		if (ifs.fail()){
//			readError(filename);
//		}
//		std::string str;
//		std::getline(ifs, str);
//		ifs >> estimated_position_blue;
//	}
//		
//	/* Map�̓ǂݍ��� */
//	cv::Mat map_img;
//	{
//		std::string filename = IFPATH_ENV + "lrf_lower/gridmap.bmp";
//		map_img = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
//		if (map_img.empty())	readError(filename);
//	}
//
//	//cv::Point leica_pix = ToPixel(leica_coor, map_img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
//	//cv::circle(map_img, leica_pix, 3, cv::Scalar(255, 0, 0), -1);
//
//	/* �^�̈ʒu�̎Z�o */
//	std::vector<Coor<>> true_position;
//	std::vector<MyTime> true_time;
//	for (int i = 0; i < dataset_leica.size(); i++){
//		Coor<> pos = dataset_leica[i].coor();
//		MyTime time = dataset_leica[i].time;
//		true_position.push_back(pos);
//		true_time.push_back(time);
//	}
//
//	/* �^�̈ʒu�̏o�� */
//	std::ofstream ofs("true_position.csv");
//	ofs << "time,x,y,rad" << std::endl;
//	for (int i = 0; i < true_position.size();i++){
//		ofs << true_time[i] << "," << true_position[i] << std::endl;
//	}
//	
//	std::cout << true_position.size() << std::endl;
//	std::cout << true_time.size() << std::endl;
//
//	/* ����ʒu�̕`�� */
//	for (int i = 0; i < estimated_position_red.size()-1; i++){
//		cv::Point pix1 = ToPixel(estimated_position_red[i], map_img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
//		cv::Point pix2 = ToPixel(estimated_position_red[i+1], map_img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
//		cv::line(map_img, pix1, pix2, cv::Scalar(255, 0, 255), 2);
//	}
//	for (int i = 0; i < estimated_position_blue.size() - 1; i++){
//		cv::Point pix1 = ToPixel(estimated_position_blue[i], map_img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
//		cv::Point pix2 = ToPixel(estimated_position_blue[i + 1], map_img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
//		cv::line(map_img, pix1, pix2, cv::Scalar(255, 0, 0), 2);
//	}
//
//	/* �^�̈ʒu�̕`�� */
//	for (int i = 6; i < true_position.size(); i++){
//	//for (int i = 0; i < 3; i++){
//			cv::Point pix = ToPixel(true_position[i], map_img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
//		cv::circle(map_img, pix, 3, cv::Scalar(0, 255, 0), -1);
//	}
//
//
//
//	/* �k�� */
//	cv::resize(map_img, map_img, cv::Size(), 0.5, 0.5);
//	/* imshow */
//	cv::flip(map_img, map_img, 0);
//	cv::imshow("map", map_img);
//	cv::waitKey();
//	cv::imwrite("map.bmp", map_img);
//
//	return 0;
//}

/* �J�����ʒu��map��ɕ`�� */
//#define TH_DISTANCE 1000
//int _tmain(int argc, _TCHAR* argv[])
//{
//	/* Map�̓ǂݍ��� */
//	cv::Mat map_img;
//	{
//		std::string filename = IFPATH_ENV + "lrf_lower/gridmap.bmp";
//		map_img = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
//		if (map_img.empty())	readError(filename);
//	}
//
//	/* ���摜�B�e�ʒu�̓ǂݍ��� */
//	std::vector<Position<>> env_img_position;
//	{
//		std::string filename = IFPATH_ENV_OMNI + "omni/img_pos.csv";
//		std::ifstream ifs(filename);
//		if (ifs.fail()) readError(filename);
//		std::string str;
//		std::getline(ifs, str);
//		ifs >> env_img_position;
//	}
//
//
//	/* �^�̈ʒu�̕`�� */
//	for (int i = 0; i < env_img_position.size(); i++){
//		cv::Point pix = ToPixel(env_img_position[i], map_img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
//		cv::circle(map_img, pix, IMAGE_PARTICLE_RADIUS, cv::Scalar(0, 255, 0), 2, 8, 0);	//	�p�[�e�B�N���̕`��
//		// ���̕`��
//		double l = IMAGE_PARTICLE_ARROW_LENGTH;
//		Coor<> coor2;
//		coor2.x = l*std::cos(env_img_position[i].r) + env_img_position[i].x;
//		coor2.y = l*std::sin(env_img_position[i].r) + env_img_position[i].y;
//		cv::Point p2 = ToPixel(coor2, map_img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	�p�[�e�B�N���s�N�Z��
//		cv::line(map_img, pix, p2, cv::Scalar(0, 255, 0), IMAGE_PARTICLE_ARROW_THICKNESS);
//	}
//
//	/* imshow */
//	cv::flip(map_img, map_img, 0);
//	cv::resize(map_img, map_img, cv::Size(), 0.5, 0.5);
//	cv::imshow("map", map_img);
//	cv::waitKey();
//	cv::imwrite("map.bmp", map_img);
//
//	return 0;
//}

/* sigma */
//int _tmain(int argc, _TCHAR* argv[])
//{
//	/* �摜�B�e�ʒu�̓ǂݍ��� */
//	std::vector<Position<>> env_img_position;
//	{
//		std::string filename = IFPATH_ENV_OMNI + "omni/img_pos.csv";
//		std::ifstream ifs(filename);
//		if (ifs.fail()) readError(filename);
//		std::string str;
//		std::getline(ifs, str);
//		ifs >> env_img_position;
//	}
//	std::cout << "Size of Environmental Images: "<<env_img_position.size() << std::endl;
//
//	/* Keypoint��Descriptor�̓ǂݍ��� */
//	std::vector<std::vector<cv::KeyPoint>> env_keypoints;
//	std::vector<cv::Mat> env_descriptors;
//	env_keypoints.reserve(env_img_position.size());
//	env_descriptors.reserve(env_img_position.size());
//	int fnum = 1;
//	while (true){
//		std::string filename = IFPATH_ENV_OMNI + "omni/surf/img/img_" + std::to_string(fnum) + "th.bmp";
//		cv::Mat img;
//		img = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
//		if (img.empty()){
//			break;
//		}
//
//		//SURF
//		cv::SurfFeatureDetector detector(1000);
//		cv::SurfDescriptorExtractor extractor;
//
//		//�摜��������_�����o
//		std::vector<cv::KeyPoint> key;
//		detector.detect(img, key);
//		//�摜�̓����_�ɂ���������ʂ𒊏o
//		cv::Mat desc;
//		extractor.compute(img, key, desc);
//
//		env_keypoints.push_back(key);
//		env_descriptors.push_back(desc);
//		std::cout << "Environment Image: " << fnum << std::endl;
//		fnum++;
//		//if (fnum > 2){
//		//	break;
//		//}
//	}
//	if (env_keypoints.empty() || env_descriptors.empty()){
//		readError("Descriptor or Keypoints");
//	}
//
//
//	/* �����Ɨގ��x */
//	std::vector<double> distances;
//	std::vector<double> similarities;
//	for (int i = 0; i < env_img_position.size(); i++){
//		for (int j = i + 1; j < env_img_position.size(); j++){
//			double distance_images = env_img_position[i] | env_img_position[j];
//			distances.push_back(distance_images);
//
//			/* matches_used_calc[i][j]: �g�p������摜i�ԖڂƊϑ��摜�Ԃ�j�Ԗڂ̑Ή��_ */
//			//�����_�̑Ή��t��
//			std::vector<cv::DMatch> matches;
//			cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(matching_type);;
//			matcher->match(env_descriptors[i], env_descriptors[j], matches);
//
//			///* �\�[�g����SURF_SIZE_USE_FUTURES�Ԗڂ܂ł̑Ή�����\��������Bnth_element�͗v�f����v�f������O�Ɉړ������邠���̃\�[�g */
//			//nth_element(matches.begin(), matches.begin() + SURF_SIZE_USE_FUTURES - 1, matches.end());
//			//std::sort(matches.begin(), matches.end());
//			//matches.erase(matches.begin() + SURF_SIZE_USE_FUTURES, matches.end());
//
//			/* 臒l���� */
//			std::vector<cv::DMatch> matches_good;
//			for (int k = 0; k < matches.size(); k++){
//				if (matches[k].distance < SURF_DMATCH_DIST_TH){
//					matches_good.push_back(matches[k]);
//				}
//			}
//
//			double similarity = matches_good.size() / 1000.0;
//
//			similarities.push_back(similarity);
//			std::cout << "Finish " << i << "-" << j << ": " << similarities.back() << std::endl;
//		}
//	}
//
//	/* �t�@�C���o�� */
//	{
//		std::string filename = "output/sim_dis.csv";
//		std::ofstream ofs(filename);
//		if (ofs.fail()) writeError(filename);
//		for (int i = 0; i < distances.size(); i++){
//			ofs << similarities[i] << "," << distances[i] << std::endl;
//		}
//	}
//
//
//}

/* �n���C�e�X�g */
//int _tmain(int argc, _TCHAR* argv[])
//{
//
//
//>>>>>>> master
//
//	/* �o�� */
//	{
//		std::string filename = OFPATH_ALL + "true_position.csv";
//		std::ofstream ofs(filename);
//		if (ofs.fail()){
//			writeError(filename);
//		}
//		ofs << "time,x,y,rad" << std::endl;
//		for (int i = 0; i < true_position.size(); i++){
//			ofs << true_time[i] << "," << true_position[i] << std::endl;
//		}
//	}
//	{
//		std::string filename = OFPATH_ALL + "true_time.csv";
//		std::ofstream ofs(filename);
//		if (ofs.fail()){
//			writeError(filename);
//		}
//		ofs << true_time << std::endl;
//	}
//
//
//
//
//	return 0;
//}

/* SIFT�ɂ��ގ����`�F�b�N */
//#include "include\parameter.h"
//int _tmain(int argc, _TCHAR* argv[])
//{
//	//�摜�ǂݍ���
//	cv::Mat colorImg1 = cv::imread("input/img_no1_1th.bmp");
//	//cv::Mat colorImg2 = cv::imread("input/img_no1_2th.bmp");
//	cv::Mat colorImg2 = cv::imread("input/img_no1_66th.bmp");
//	//cv::Mat colorImg2 = cv::imread("input/img_no1_549th.bmp");
//	if (colorImg1.empty() || colorImg2.empty()){
//		std::cout << "No Image" << std::endl;
//		return -1;
//	}
//
//	{
//		/*  Operator�̓h��Ԃ�  */
//		cv::Point center(colorImg1.cols / 1.9, colorImg1.rows / 2);
//		cv::Size radius(colorImg1.rows / 3, colorImg1.rows / 3);
//		double start_angle = -105;	//	��n�̒��S�p�x
//		double angle = 35;	//	�p�x
//		cv::ellipse(colorImg1, center, radius, start_angle, 0, angle, cv::Scalar(0, 0, 0), -1, CV_AA);
//
//		/* �O�g�̓h��Ԃ� */
//		cv::circle(colorImg1, center, 510, cv::Scalar(0, 0, 0), 150);
//
//		//�@���Ԋu�̉摜�ɐ؂�o��
//		cv::Rect roi_rect(ROI_ORG_X, ROI_ORG_Y, ROI_SIZE_X, ROI_SIZE_Y); // x,y,w,h
//		colorImg1 = colorImg1(roi_rect);
//
//	}
//
//	{
//		/*  Operator�̓h��Ԃ�  */
//		cv::Point center(colorImg2.cols / 1.9, colorImg2.rows / 2);
//		cv::Size radius(colorImg2.rows / 3, colorImg2.rows / 3);
//		double start_angle = -105;	//	��n�̒��S�p�x
//		double angle = 35;	//	�p�x
//		cv::ellipse(colorImg2, center, radius, start_angle, 0, angle, cv::Scalar(0, 0, 0), -1, CV_AA);
//
//		/* �O�g�̓h��Ԃ� */
//		cv::circle(colorImg2, center, 510, cv::Scalar(0, 0, 0), 150);
//
//		//�@���Ԋu�̉摜�ɐ؂�o��
//		cv::Rect roi_rect(ROI_ORG_X, ROI_ORG_Y, ROI_SIZE_X, ROI_SIZE_Y); // x,y,w,h
//		colorImg2 = colorImg2(roi_rect);
//	}
//
//
//	//�����_���o�p�̃O���[�摜�p��
//	cv::Mat grayImg1, grayImg2;
//	cv::cvtColor(colorImg1, grayImg1, CV_BGR2GRAY);
//	cv::normalize(grayImg1, grayImg1, 0, 255, cv::NORM_MINMAX);
//	cv::cvtColor(colorImg2, grayImg2, CV_BGR2GRAY);
//	cv::normalize(grayImg2, grayImg2, 0, 255, cv::NORM_MINMAX);
//
//	//SIFT
//	//    cv::SiftFeatureDetector detector;
//	//    cv::SiftDescriptorExtractor extractor;    
//	//SURF
//	cv::SiftFeatureDetector detector(1000);
//	cv::SiftDescriptorExtractor extractor;
//
//	//�摜��������_�����o
//	std::vector<cv::KeyPoint> keypoints1;
//	detector.detect(grayImg1, keypoints1);
//	std::vector<cv::KeyPoint> keypoints2;
//	detector.detect(grayImg2, keypoints2);
//
//	//�摜�̓����_�ɂ���������ʂ𒊏o
//	cv::Mat descriptors1;
//	extractor.compute(grayImg1, keypoints1, descriptors1);
//	cv::Mat descriptors2;
//	extractor.compute(grayImg2, keypoints2, descriptors2);
//
//	while (true){
//
//		//�����_�̑Ή��t��
//		std::vector<cv::DMatch> matches;
//		cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(matching_type);;
//		matcher.match(descriptors1, descriptors2, matches);
//
//		////�\�[�g����n�Ԗڂ܂ł̑Ή�����\��������Bnth_element�͗v�f����v�f������O�Ɉړ������邠���̃\�[�g
//		//int N;
//		//std::cout << "N: " << std::endl;
//		//std::cin >> N;
//		//nth_element(matches.begin(), matches.begin() + N - 1, matches.end());
//		//std::sort(matches.begin(), matches.end());
//		//matches.erase(matches.begin() + N, matches.end());
//		//for (int i = 0; i < matches.size(); i++){
//		//	std::cout << matches[i].distance << std::endl;
//		//}
//		////�Ή��Â����ꂽ�摜�̗p��
//		//cv::Mat matchedImg;
//		//cv::drawMatches(colorImg1, keypoints1, colorImg2, keypoints2, matches, matchedImg);
//
//		///* N���ӂ�DMATCH_DISTANCE���`�F�b�N */
//		int N;
//		std::cout << "N: " << std::endl;
//		std::cin >> N;
//		std::sort(matches.begin(), matches.end());
//		matches.erase(matches.begin(), matches.end() - N);
//		matches.erase(matches.end() - N+10, matches.end());
//		for (int i = 0; i < matches.size(); i++){
//			std::cout << matches[i].distance << std::endl;
//		}
//		//�Ή��Â����ꂽ�摜�̗p��
//		cv::Mat matchedImg;
//		cv::drawMatches(colorImg1, keypoints1, colorImg2, keypoints2, matches, matchedImg);
//
//		//std::vector<cv::DMatch> matches_good;
//		//for (int i = 0; i < matches.size(); i++){
//		//	if (matches[i].distance < SURF_DMATCH_DIST_TH){
//		//		matches_good.push_back(matches[i]);
//		//	}
//		//}
//		////�Ή��Â����ꂽ�摜�̗p��
//		//cv::Mat matchedImg;
//		//cv::drawMatches(colorImg1, keypoints1, colorImg2, keypoints2, matches_good, matchedImg);
//
//		/// �摜��\������E�B���h�E�̖��O�C�v���p�e�B
//		// CV_WINDOW_AUTOSIZE : �E�B���h�E�T�C�Y���摜�T�C�Y�ɍ��킹��
//		// CV_WINDOW_FREERATIO : �E�B���h�E�̃A�X�y�N�g����Œ肵�Ȃ�
//		cv::namedWindow("image", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
//		// �E�B���h�E���ŃE�B���h�E���w�肵�āC�����ɉ摜��`��
//		cv::imshow("image", matchedImg);
//
//		// �L�[���͂��i�����Ɂj�҂�
//		char c = cv::waitKey();
//
//		if (c == 'q'){
//			break;
//		}
//	}
//
//	return 0;
//
//}

/* SURF�ɂ��ގ����`�F�b�N */
//#include "include\parameter.h"
//int _tmain(int argc, _TCHAR* argv[])
//{
//	//�摜�ǂݍ���
//	cv::Mat colorImg1 = cv::imread("input/img_no1_1th.bmp");
//	//cv::Mat colorImg2 = cv::imread("input/img_no1_2th.bmp");
//	cv::Mat colorImg2 = cv::imread("input/img_no1_25th.bmp");
//	//cv::Mat colorImg2 = cv::imread("input/img_no1_549th.bmp");
//	if (colorImg1.empty() || colorImg2.empty()){
//		std::cout << "No Image" << std::endl;
//		return -1;
//	}
//
//	/* ���H */
//	{
//		/*  Operator�̓h��Ԃ�  */
//		cv::Point center(colorImg1.cols / 1.9, colorImg1.rows / 2);
//		cv::Size radius(colorImg1.rows / 3, colorImg1.rows / 3);
//		double start_angle = -105;	//	��n�̒��S�p�x
//		double angle = 35;	//	�p�x
//		cv::ellipse(colorImg1, center, radius, start_angle, 0, angle, cv::Scalar(0, 0, 0), -1, CV_AA);
//
//		/* �O�g�̓h��Ԃ� */
//		cv::circle(colorImg1, center, 510, cv::Scalar(0, 0, 0), 150);
//
//		//�@���Ԋu�̉摜�ɐ؂�o��
//		cv::Rect roi_rect(ROI_ORG_X, ROI_ORG_Y, ROI_SIZE_X, ROI_SIZE_Y); // x,y,w,h
//		colorImg1 = colorImg1(roi_rect);
//
//	}
//
//	{
//		/*  Operator�̓h��Ԃ�  */
//		cv::Point center(colorImg2.cols / 1.9, colorImg2.rows / 2);
//		cv::Size radius(colorImg2.rows / 3, colorImg2.rows / 3);
//		double start_angle = -105;	//	��n�̒��S�p�x
//		double angle = 35;	//	�p�x
//		cv::ellipse(colorImg2, center, radius, start_angle, 0, angle, cv::Scalar(0, 0, 0), -1, CV_AA);
//
//		/* �O�g�̓h��Ԃ� */
//		cv::circle(colorImg2, center, 510, cv::Scalar(0, 0, 0), 150);
//
//		//�@���Ԋu�̉摜�ɐ؂�o��
//		cv::Rect roi_rect(ROI_ORG_X, ROI_ORG_Y, ROI_SIZE_X, ROI_SIZE_Y); // x,y,w,h
//		colorImg2 = colorImg2(roi_rect);
//	}
//
//	///* �摜�̉�] */
//	//{
//	//	double angle = -40;	// [deg]
//	//	cv::Point center(colorImg2.cols*0.5, colorImg2.rows*0.5);
//	//	cv::Mat affin_mat = cv::getRotationMatrix2D(center, angle, 1.0);
//	//	cv::warpAffine(colorImg2, colorImg2, affin_mat, colorImg2.size());
//	//}
//
//
//
//
//	//�����_���o�p�̃O���[�摜�p��
//	cv::Mat grayImg1, grayImg2;
//	cv::cvtColor(colorImg1, grayImg1, CV_BGR2GRAY);
//	cv::normalize(grayImg1, grayImg1, 0, 255, cv::NORM_MINMAX);
//	cv::cvtColor(colorImg2, grayImg2, CV_BGR2GRAY);
//	cv::normalize(grayImg2, grayImg2, 0, 255, cv::NORM_MINMAX);
//
//	//SIFT
//	//    cv::SiftFeatureDetector detector;
//	//    cv::SiftDescriptorExtractor extractor;    
//	//SURF
//	cv::SurfFeatureDetector detector(1000);
//	cv::SurfDescriptorExtractor extractor;
//
//	//�摜��������_�����o
//	std::vector<cv::KeyPoint> keypoints1;
//	detector.detect(grayImg1, keypoints1);
//	std::vector<cv::KeyPoint> keypoints2;
//	detector.detect(grayImg2, keypoints2);
//
//	//�摜�̓����_�ɂ���������ʂ𒊏o
//	cv::Mat descriptors1;
//	extractor.compute(grayImg1, keypoints1, descriptors1);
//	cv::Mat descriptors2;
//	extractor.compute(grayImg2, keypoints2, descriptors2);
//
//	while (true){
//
//		//�����_�̑Ή��t��
//		std::vector<cv::DMatch> matches;
//		cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(matching_type);;
//		matcher->match(descriptors1, descriptors2, matches);
//
//		/* �\�[�g����n�Ԗڂ܂ł̑Ή�����\��������Bnth_element�͗v�f����v�f������O�Ɉړ������邠���̃\�[�g */
//		int N;
//		std::cout << "N: " << std::endl;
//		std::cin >> N;
//		nth_element(matches.begin(), matches.begin() + N - 1, matches.end());
//		std::sort(matches.begin(), matches.end());
//		matches.erase(matches.begin() + N, matches.end());
//		for (int i = 0; i < matches.size(); i++){
//			//std::cout << matches[i].distance << std::endl;
//			std::cout << keypoints1[matches[i].queryIdx].pt << "," << keypoints2[matches[i].trainIdx].pt << "," << matches[i].imgIdx << std::endl;
//		}
//		//�Ή��Â����ꂽ�摜�̗p��
//		cv::Mat matchedImg;
//		cv::drawMatches(colorImg1, keypoints1, colorImg2, keypoints2, matches, matchedImg);
//
//		///* N���ӂ�DMATCH_DISTANCE���`�F�b�N */
//		//int N;
//		//std::cout << "N: " << std::endl;
//		//std::cin >> N;
//		//std::sort(matches.begin(), matches.end());
//		//matches.erase(matches.begin(), matches.end() - N);
//		//matches.erase(matches.end() - N + 10, matches.end());
//		//for (int i = 0; i < matches.size(); i++){
//		//	std::cout << matches[i].distance << std::endl;
//		//}
//		////�Ή��Â����ꂽ�摜�̗p��
//		//cv::Mat matchedImg;
//		//cv::drawMatches(colorImg1, keypoints1, colorImg2, keypoints2, matches, matchedImg);
//
//		///* 臒l���� */
//		//std::vector<cv::DMatch> matches_good;
//		//for (int i = 0; i < matches.size(); i++){
//		//	if (matches[i].distance < SURF_DMATCH_DIST_TH){
//		//		matches_good.push_back(matches[i]);
//		//	}
//		//}
//		////�Ή��Â����ꂽ�摜�̗p��
//		//cv::Mat matchedImg;
//		//cv::drawMatches(colorImg1, keypoints1, colorImg2, keypoints2, matches_good, matchedImg);
//
//		/// �摜��\������E�B���h�E�̖��O�C�v���p�e�B
//		// CV_WINDOW_AUTOSIZE : �E�B���h�E�T�C�Y���摜�T�C�Y�ɍ��킹��
//		// CV_WINDOW_FREERATIO : �E�B���h�E�̃A�X�y�N�g����Œ肵�Ȃ�
//		cv::namedWindow("image", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
//		// �E�B���h�E���ŃE�B���h�E���w�肵�āC�����ɉ摜��`��
//		cv::imshow("image", matchedImg);
//
//		// �L�[���͂��i�����Ɂj�҂�
//		char c = cv::waitKey();
//
//		if (c == 'q'){
//			break;
//		}
//	}
//
//	return 0;
//
//}

/* �摜�Q���r�f�I�ɂ܂Ƃ߂� */
//#define END_NO 2
//int _tmain(int argc, _TCHAR* argv[])
//{
//	std::string filename = "F://Data/Localization/Measurement/" + MEASUREMENT_DATE + "/" + MEASUREMENT_TIME + "/img/img_no1_1th.bmp";
//	cv::Mat ini = cv::imread(filename, 1);
//	if (ini.empty()){
//		readError(filename);
//	}
//
//	/*  �t�@�C���o��  */
//	/*  �r�f�I�֌W  */
//	// �t�@�C�����I�[�v�����C�r�f�I���C�^��������
//	// filename - �o�̓t�@�C����
//	// fourcc - �R�[�f�b�N
//	// fps - 1 �b������̃t���[����
//	// frameSize - �r�f�I�t���[���̃T�C�Y
//	// isColor - �r�f�I�X�g���[�����J���[���C�O���[�X�P�[�������w��
//	cv::VideoWriter vwr;
//	{
//		std::string filename = IFPATH_MEAS + "img.avi";
//		vwr.open(filename, CV_FOURCC('D', 'I', 'B', ' '), 1, cv::Size(ini.cols, ini.rows));
//		if (!vwr.isOpened())	writeError(filename);
//	}
//
//	int no = 1;
//	int fnum = 1;	//	���ݓǂݍ���ł���t�@�C���ԍ�
//
//	while (true){
//		std::string filename = "F://Data/Localization/Measurement/" + MEASUREMENT_DATE + "/" + MEASUREMENT_TIME + "/img/img_no" + std::to_string(no) + "_" + std::to_string(fnum) + "th.bmp";
//		cv::Mat mat = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
//		if (mat.empty()){
//			std::cout << filename << std::endl;
//			no++;
//			fnum = 1;
//			if (no > END_NO){
//				break;
//			}
//			continue;
//		}
//
//		vwr << mat;
//
//		std::cout << "Read No: " << no << " Fnum: " << fnum << std::endl;
//
//		fnum++;
//	}
//
//
//	return 0;
//}

/* �摜�Q�̍s�� */
//int _tmain(int argc, _TCHAR* argv[])
//{
//	/* �r�f�I�t�@�C���̃I�[�v�� */
//	cv::VideoCapture capture;
//	{
//		std::string filename = IFPATH_MEAS + "img.avi";
//		capture.open(filename);
//		if (!capture.isOpened()){
//			readError(filename);
//		}
//	}
//
//	int i = 1;
//	while (true){
//		cv::Mat img;
//
//		capture >> img;
//		if (img.empty()){
//			break;
//		}
//
//		cv::imshow("hoge", img);
//		cv::waitKey(100);
//	}
//
//	return 0;
//}

/* ���摜�Q���r�f�I�ɂ܂Ƃ߂� */
//#define END_NO 2
//int _tmain(int argc, _TCHAR* argv[])
//{
//	std::string filename = IFPATH_ENV_OMNI + "omni/surf/img/img_1th.bmp";
//	cv::Mat ini = cv::imread(filename, 1);
//
//	/*  �t�@�C���o��  */
//	/*  �r�f�I�֌W  */
//	// �t�@�C�����I�[�v�����C�r�f�I���C�^��������
//	// filename - �o�̓t�@�C����
//	// fourcc - �R�[�f�b�N
//	// fps - 1 �b������̃t���[����
//	// frameSize - �r�f�I�t���[���̃T�C�Y
//	// isColor - �r�f�I�X�g���[�����J���[���C�O���[�X�P�[�������w��
//	cv::VideoWriter vwr;
//	{
//		std::string filename = IFPATH_ENV_OMNI + "omni/surf/img.avi";
//		vwr.open(filename, CV_FOURCC('D', 'I', 'B', ' '), 1, cv::Size(ini.cols, ini.rows));
//		if (!vwr.isOpened())	writeError(filename);
//	}
//
//	int no = 1;
//	int fnum = 1;	//	���ݓǂݍ���ł���t�@�C���ԍ�
//
//	while (true){
//		std::string filename = IFPATH_ENV_OMNI + "omni/surf/img/img_" + std::to_string(fnum) + "th.bmp";
//		cv::Mat mat = cv::imread(filename, 1);
//		if (mat.empty()){
//				break;
//		}
//
//		vwr << mat;
//
//		std::cout << "Fnum: " << fnum << std::endl;
//
//		fnum++;
//	}
//
//	return 0;
//}

/* ���摜�Q�̓ǂݍ��݂�Keypoint�ۑ�(SURF) */
//int _tmain(int argc, _TCHAR* argv[])
//{
//	clock_t start = clock();
//
//	/* �r�f�I�t�@�C���̃I�[�v�� */
//	cv::VideoCapture capture;
//	{
//		std::string filename = IFPATH_ENV_OMNI + "omni/surf/img.avi";
//		capture.open(filename);
//		if (!capture.isOpened()){
//			readError(filename);
//		}
//	}
//
//	clock_t end = clock();
//
//	std::cout << "���s����: " << (int)(end - start) << std::endl;
//
//	int i = 1;
//	while (true){
//		cv::Mat img;
//
//		capture >> img;
//		if (img.empty()){
//			break;
//		}
//
//		//SURF
//		cv::SurfFeatureDetector detector(1000);
//		std::vector<cv::KeyPoint> keypoints;
//
//		//�摜��������_�����o
//		detector.detect(img, keypoints);
//
//		/* keypoint�̕ۑ� */
//		std::string filename = IFPATH_ENV_OMNI + "omni/surf/keypoint/keypoint_" + std::to_string(i) + "th.yml";
//		cv::FileStorage fs(filename, cv::FileStorage::WRITE);
//		if (!fs.isOpened()){
//			writeError(filename);
//		}
//		cv::write(fs, "keypoints", keypoints);
//
//
//		std::cout << i << std::endl;
//		i++;
//	}
//
//	return 0;
//}

/* ���摜�Q�̓ǂݍ��݂�Keypoint�ۑ�(SIFT) */
//int _tmain(int argc, _TCHAR* argv[])
//{
//	clock_t start = clock();
//
//	/* �r�f�I�t�@�C���̃I�[�v�� */
//	cv::VideoCapture capture;
//	{
//		std::string filename = IFPATH_ENV_OMNI + "omni/img.avi";
//		capture.open(filename);
//		if (!capture.isOpened()){
//			readError(filename);
//		}
//	}
//
//	clock_t end = clock();
//
//	std::cout << "���s����: " << (int)(end - start) << std::endl;
//
//	int i = 1;
//	while (true){
//		cv::Mat img;
//
//		capture >> img;
//		if (img.empty()){
//			break;
//		}
//
//		//SIFT
//		cv::SiftFeatureDetector detector(1000);
//		std::vector<cv::KeyPoint> keypoints;
//
//		//�摜��������_�����o
//		detector.detect(img, keypoints);
//
//		/* keypoint�̕ۑ� */
//		std::string filename = IFPATH_ENV_OMNI + "omni/sift/keypoint/keypoint_" + std::to_string(i) + "th.yml";
//		cv::FileStorage fs(filename, cv::FileStorage::WRITE);
//		if (!fs.isOpened()){
//			writeError(filename);
//		}
//		cv::write(fs, "keypoints", keypoints);
//
//
//		std::cout << i << std::endl;
//		i++;
//	}
//
//	return 0;
//}

/* ���摜�Q�̓ǂݍ��݂�Keypoint�EDescriptor�ۑ�(SURF) */
//int _tmain(int argc, _TCHAR* argv[])
//{
//	clock_t start = clock();
//
//	/* �r�f�I�t�@�C���̃I�[�v�� */
//	cv::VideoCapture capture;
//	{
//		//std::string filename = "\\\\Desktop-mt35ltg/f/Data/Localization/Environment/" + ENVIRONMENT_DATE_OMNI + "/" + ENVIRONMENT_TIME_OMNI + "/omni/img.avi";
//		std::string filename = "F://Data/Localization/Environment/Omni/integrate1/omni/img.avi";
//		capture.open(filename);
//		if (!capture.isOpened()){
//			readError(filename);
//		}
//	}
//
//	clock_t end = clock();
//
//	std::cout << "���s����: " << (int)(end - start) << std::endl;
//
//	std::vector<cv::Mat> imgs;
//	int i = 1;
//	while (true){
//		cv::Mat img;
//
//		capture >> img;
//		if (img.empty()){
//			break;
//		}
//
//		imgs.push_back(img);
//	}
//
//	int size_thread = 1;
//	//int size_thread = std::thread::hardware_concurrency() / 16;
//	std::vector<std::thread> threads(size_thread);
//	for (int th = 0; th < size_thread; th++){
//		threads[th] = std::thread([=]{
//			
//			for (int i = th; i < imgs.size(); i += size_thread){
//				//SURF
//				cv::SurfFeatureDetector detector(1000);
//				std::vector<cv::KeyPoint> keypoints;
//
//				//�摜��������_�����o
//				detector.detect(imgs[i], keypoints);
//
//				/* keypoint�̕ۑ� */
//				std::string filename = IFPATH_ENV_OMNI + "omni/surf/keypoint/keypoint_" + std::to_string(i + 1) + "th.yml";
//				cv::FileStorage fs(filename, cv::FileStorage::WRITE);
//				if (!fs.isOpened()){
//					writeError(filename);
//				}
//				cv::write(fs, "keypoints", keypoints);
//
//				cv::SurfDescriptorExtractor extractor;
//
//				cv::Mat descriptors;
//				extractor.compute(imgs[i], keypoints, descriptors);
//
//				/*  descriptor�̏����o��  */
//				filename = IFPATH_ENV_OMNI + "omni/surf/descriptor/desc_" + std::to_string(i + 1) + "th.csv";
//				std::ofstream ofs(filename);
//				if (ofs.fail())	readError(filename);
//				ofs << cv::format(descriptors, "csv");
//
//				std::cout << i << std::endl;
//
//			}
//		});
//	}
//	for (auto& thread : threads){
//		thread.join();
//	}
//
//
//	return 0;
//}

/* ���摜�Q�̓ǂݍ��݂�Keypoint�EDescriptor�ۑ�(SIFT) */
//int _tmain(int argc, _TCHAR* argv[])
//{
//	clock_t start = clock();
//
//	/* �r�f�I�t�@�C���̃I�[�v�� */
//	cv::VideoCapture capture;
//	{
//		std::string filename = "\\\\Desktop-mt35ltg/f/Data/Localization/Environment/" + ENVIRONMENT_DATE_OMNI + "/" + ENVIRONMENT_TIME_OMNI + "/omni/img.avi";
//		capture.open(filename);
//		if (!capture.isOpened()){
//			readError(filename);
//		}
//	}
//
//	clock_t end = clock();
//
//	std::cout << "���s����: " << (int)(end - start) << std::endl;
//
//	std::vector<cv::Mat> imgs;
//	int i = 1;
//	while (true){
//		cv::Mat img;
//
//		capture >> img;
//		if (img.empty()){
//			break;
//		}
//
//		imgs.push_back(img);
//	}
//
//	int size_thread = 1;
//	//int size_thread = std::thread::hardware_concurrency() / 16;
//	std::vector<std::thread> threads(size_thread);
//	for (int th = 0; th < size_thread; th++){
//		threads[th] = std::thread([=]{
//			
//			for (int i = th; i < imgs.size(); i += size_thread){
//				//SIFT
//				cv::SiftFeatureDetector detector(1000);
//				std::vector<cv::KeyPoint> keypoints;
//
//				//�摜��������_�����o
//				detector.detect(imgs[i], keypoints);
//
//				/* keypoint�̕ۑ� */
//				std::string filename = IFPATH_ENV_OMNI + "omni/sift/keypoint/keypoint_" + std::to_string(i) + "th.yml";
//				cv::FileStorage fs(filename, cv::FileStorage::WRITE);
//				if (!fs.isOpened()){
//					writeError(filename);
//				}
//				cv::write(fs, "keypoints", keypoints);
//
//				cv::SiftDescriptorExtractor extractor;
//
//				cv::Mat descriptors;
//				extractor.compute(imgs[i], keypoints, descriptors);
//
//				/*  descriptor�̏����o��  */
//				filename = IFPATH_ENV_OMNI + "omni/sift/descriptor/desc_" + std::to_string(i) + "th.csv";
//				std::ofstream ofs(filename);
//				if (ofs.fail())	readError(filename);
//				ofs << cv::format(descriptors, "csv");
//
//				std::cout << i << std::endl;
//
//			}
//		});
//	}
//	for (auto& thread : threads){
//		thread.join();
//	}
//
//
//	return 0;
//}

/* cv::Point�̉�] */
//int _tmain(int argc, _TCHAR* argv[])
//{
//
//	cv::Mat img(400, 400, CV_8UC3, cv::Scalar(255, 255, 255));
//	cv::Point center(img.cols / 2.0, img.rows / 2.0);
//	cv::circle(img, center, 3, cv::Scalar(0, 0, 0), -1);
//	cv::imshow("origin", img);
//
//
//	/* ��] */
//	cv::Point p1(center.x, center.y / 2.0);
//	std::cout << "p1: " << p1 << std::endl;
//	cv::circle(img, p1, 3, cv::Scalar(0, 0, 255), -1);
//	double angle = -90;	// [deg](����]��)
//	cv::Mat affin_mat = cv::getRotationMatrix2D(center, angle, 1.0);
//	std::cout << affin_mat << std::endl;
//	cv::warpAffine(img, img, affin_mat, img.size());
//	cv::imshow("rotate1", img);
//
//	/* p1�̉�] */
//	cv::Point p2;
//	p2.x = std::cos(angle / 180.0*M_PI)*(p1.x - center.x) + std::sin(angle / 180.0*M_PI)*(p1.y - center.y) + center.x;
//	p2.y = -std::sin(angle / 180.0*M_PI)*(p1.x - center.x) + std::cos(angle / 180.0*M_PI)*(p1.y - center.y) + center.y;
//
//
//
//	//std::cout << affin_mat_clone(1, 1)*(p1.x - center.x) << "1," << affin_mat_clone(, 2) << std::endl;
//
//	std::cout << "p2: " << p2 << std::endl;
//
//	//cv::Mat p2_mat = affin_mat*p1_mat.t();
//	//std::cout << p2_mat << std::endl;
//
//
//
//
//
//
//
//
//
//	cv::waitKey();
//	return 0;
//
//}

/* �����ʂɂ��}�b�`���O�x�`�F�b�N */
//#include "include\parameter.h"
//int _tmain(int argc, _TCHAR* argv[])
//{
//	//�摜�ǂݍ���
//	cv::Mat colorImg1 = cv::imread("input/img_no1_1th.bmp");
//	//cv::Mat colorImg2 = cv::imread("input/img_no1_2th.bmp");
//	cv::Mat colorImg2 = cv::imread("input/img_no1_549th.bmp");
//	if (colorImg1.empty() || colorImg2.empty()){
//		std::cout << "No Image" << std::endl;
//		return -1;
//	}
//	
//	{
//		/*  Operator�̓h��Ԃ�  */
//		cv::Point center(colorImg1.cols / 1.9, colorImg1.rows / 2);
//		cv::Size radius(colorImg1.rows / 3, colorImg1.rows / 3);
//		double start_angle = -105;	//	��n�̒��S�p�x
//		double angle = 35;	//	�p�x
//		cv::ellipse(colorImg1, center, radius, start_angle, 0, angle, cv::Scalar(0, 0, 0), -1, CV_AA);
//
//		/* �O�g�̓h��Ԃ� */
//		cv::circle(colorImg1, center, 510, cv::Scalar(0, 0, 0), 150);
//
//		//�@���Ԋu�̉摜�ɐ؂�o��
//		cv::Rect roi_rect(ROI_ORG_X, ROI_ORG_Y, ROI_SIZE_X, ROI_SIZE_Y); // x,y,w,h
//		colorImg1 = colorImg1(roi_rect);
//		
//	}
//	
//	{
//		/*  Operator�̓h��Ԃ�  */
//		cv::Point center(colorImg2.cols / 1.9, colorImg2.rows / 2);
//		cv::Size radius(colorImg2.rows / 3, colorImg2.rows / 3);
//		double start_angle = -105;	//	��n�̒��S�p�x
//		double angle = 35;	//	�p�x
//		cv::ellipse(colorImg2, center, radius, start_angle, 0, angle, cv::Scalar(0, 0, 0), -1, CV_AA);
//
//		/* �O�g�̓h��Ԃ� */
//		cv::circle(colorImg2, center, 510, cv::Scalar(0, 0, 0), 150);
//
//		//�@���Ԋu�̉摜�ɐ؂�o��
//		cv::Rect roi_rect(ROI_ORG_X, ROI_ORG_Y, ROI_SIZE_X, ROI_SIZE_Y); // x,y,w,h
//		colorImg2 = colorImg2(roi_rect);
//	}
//
//
//	//�����_���o�p�̃O���[�摜�p��
//	cv::Mat grayImg1, grayImg2;
//	cv::cvtColor(colorImg1, grayImg1, CV_BGR2GRAY);
//	cv::normalize(grayImg1, grayImg1, 0, 255, cv::NORM_MINMAX);
//	cv::cvtColor(colorImg2, grayImg2, CV_BGR2GRAY);
//	cv::normalize(grayImg2, grayImg2, 0, 255, cv::NORM_MINMAX);
//
//	//SIFT
//	//    cv::SiftFeatureDetector detector;
//	//    cv::SiftDescriptorExtractor extractor;    
//	//SURF
//	cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create();
//
//	//cv::SurfFeatureDetector detector(1000);
//	//cv::SurfDescriptorExtractor extractor;
//
//	//�摜��������_�����o
//	std::vector<cv::KeyPoint> keypoints1;
//	surf->detect(grayImg1, keypoints1);
//	std::vector<cv::KeyPoint> keypoints2;
//	surf->detect(grayImg2, keypoints2);
//	//std::vector<cv::KeyPoint> keypoints1;
//	//detector.detect(grayImg1, keypoints1);
//	//std::vector<cv::KeyPoint> keypoints2;
//	//detector.detect(grayImg2, keypoints2);
//
//	//�摜�̓����_�ɂ���������ʂ𒊏o
//	cv::Mat descriptors1;
//	surf->compute(grayImg1, keypoints1, descriptors1);
//	cv::Mat descriptors2;
//	surf->compute(grayImg2, keypoints2, descriptors2);
//
//	
//	//�����_�̑Ή��t��
//	std::vector<cv::DMatch> matches;
//	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(matching_type);;
//	matcher->match(descriptors1, descriptors2, matches);
//
//	//�\�[�g����n�Ԗڂ܂ł̑Ή�����\��������Bnth_element�͗v�f����v�f������O�Ɉړ������邠���̃\�[�g
//	//int N = 300;
//	//nth_element(matches.begin(), matches.begin() + N - 1, matches.end());
//	//std::sort(matches.begin(), matches.end());
//	//matches.erase(matches.begin() + N, matches.end());
//
//	/* N���ӂ�DMATCH_DISTANCE���`�F�b�N */
//	//int N = 480;
//	//std::sort(matches.begin(), matches.end());
//	//matches.erase(matches.begin(), matches.end() - N);
//	//matches.erase(matches.end() - N+10, matches.end());
//	//for (int i = 0; i < matches.size(); i++){
//	//	std::cout << matches[i].distance << std::endl;
//	//}
//
//	std::vector<cv::DMatch> matches_good;
//	for (int i = 0; i < matches.size(); i++){
//		if (matches[i].distance < SURF_DMATCH_DIST_TH){
//			matches_good.push_back(matches[i]);
//		}
//	}
//
//	//�Ή��Â����ꂽ�摜�̗p��
//	cv::Mat matchedImg;
//	cv::drawMatches(colorImg1, keypoints1, colorImg2, keypoints2, matches_good, matchedImg);
//
//	/// �摜��\������E�B���h�E�̖��O�C�v���p�e�B
//	// CV_WINDOW_AUTOSIZE : �E�B���h�E�T�C�Y���摜�T�C�Y�ɍ��킹��
//	// CV_WINDOW_FREERATIO : �E�B���h�E�̃A�X�y�N�g����Œ肵�Ȃ�
//	cv::namedWindow("image", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
//	// �E�B���h�E���ŃE�B���h�E���w�肵�āC�����ɉ摜��`��
//	cv::imshow("image", matchedImg);
//
//	// �L�[���͂��i�����Ɂj�҂�
//	cv::waitKey(0);
//	return 0;
//
//}

/* �v���摜��������ʂ�keypoint��ۑ�(SURF) */
//#include "include\parameter.h"
//int _tmain(int argc, _TCHAR* argv[])
//{
//	int no = 1;
//
//	int thread_num = 4;
//	//int thread_num = std::thread::hardware_concurrency() / 2;
//
//	std::vector<std::thread> thread;
//	for (int th = 0; th < thread_num; th++)
//	{
//		/*  thread�̐���  */
//		thread.push_back(std::thread([th, no, thread_num] {
//			int step = th + 1;
//			//int step = th * 5 + 2;
//			while (1)
//			{
//				cv::Mat img;	//	�S���ʃJ�����摜
//
//								/*  �t�@�C���̓ǂݍ���  */
//				std::string filename = "F://Data/Localization/Measurement/" + MEASUREMENT_DATE + "/" + MEASUREMENT_TIME + "/img/img_no" + std::to_string(no) + "_" + std::to_string(step) + "th.bmp";
//				img = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
//				//if (img.empty()) break;	//	�I������
//				if (img.empty()) readError(filename);	//	�I������
//
//														/*  Operator�̓h��Ԃ�  */
//				cv::Point center(img.cols / 1.9, img.rows / 2);
//				cv::Size radius(img.rows / 3, img.rows / 3);
//				double start_angle = -105;	//	��n�̒��S�p�x
//				double angle = 35;	//	�p�x
//				cv::ellipse(img, center, radius, start_angle, 0, angle, cv::Scalar(0, 0, 0), -1, CV_AA);
//
//				/* �O�g�̓h��Ԃ� */
//				cv::circle(img, center, 510, cv::Scalar(0, 0, 0), 150);
//
//				//�@���Ԋu�̉摜�ɐ؂�o��
//				cv::Rect roi_rect(ROI_ORG_X, ROI_ORG_Y, ROI_SIZE_X, ROI_SIZE_Y); // x,y,w,h
//				img = img(roi_rect);
//
//				cv::SurfFeatureDetector detector(1000);
//				cv::SurfDescriptorExtractor extractor;
//
//				//clock_t lap1 = clock();
//
//				//�摜��������_�����o
//				std::vector<cv::KeyPoint> keypoints;
//				detector.detect(img, keypoints);
//
//				//clock_t lap2 = clock();
//
//				//�摜�̓����_�ɂ���������ʂ𒊏o
//				cv::Mat descriptors;
//				extractor.compute(img, keypoints, descriptors);
//
//				//clock_t lap3 = clock();
//
//				/* keypoint�̕ۑ� */
//				filename = IFPATH_MEAS + "surf/keypoint/keypoint_no" + std::to_string(no) + "_" + std::to_string(step) + "th.yml";
//				cv::FileStorage fs(filename, cv::FileStorage::WRITE);
//				if (!fs.isOpened()) {
//					writeError(filename);
//				}
//				cv::write(fs, "keypoints", keypoints);
//
//				/*  descriptor�̏����o��  */
//				filename = IFPATH_MEAS + "surf/descriptor/desc_no" + std::to_string(no) + "_" + std::to_string(step) + "th.csv";
//				std::ofstream ofs(filename);
//				if (ofs.fail())	readError(filename);
//				ofs << cv::format(descriptors, "csv");
//
//
//				std::cout << "Finish Step: " << step << std::endl;
//				//if (step > 1)	break;
//
//
//				step += thread_num;
//			}
//		}));	//	thread�̒�`�����܂�
//	}
//
//	for (auto& th : thread)
//	{
//		th.join();
//	}
//
//	return 0;
//
//}

/* �v���摜��������ʂ�keypoint��ۑ�(SIFT) */
//#include "include\parameter.h"
//int _tmain(int argc, _TCHAR* argv[])
//{
//	int no = 1;
//
//	int thread_num =4;
//	//int thread_num = std::thread::hardware_concurrency() / 2;
//
//	std::vector<std::thread> thread;
//	for (int th = 0; th < thread_num; th++)
//	{
//		/*  thread�̐���  */
//		thread.push_back(std::thread([th, no, thread_num] {
//			int step = th + 2200;
//			//int step = th * 5 + 2;
//			while (1)
//			{
//				cv::Mat img;	//	�S���ʃJ�����摜
//
//								/*  �t�@�C���̓ǂݍ���  */
//				//std::string filename = "\\\\Desktop-mt35ltg/f/Data/Localization/Measurement/" + MEASUREMENT_DATE + "/" + MEASUREMENT_TIME + "/img/img_no" + std::to_string(no) + "_" + std::to_string(step) + "th.bmp";
//				std::string filename = "F://Data/Localization/Measurement/" + MEASUREMENT_DATE + "/" + MEASUREMENT_TIME + "/img/img_no" + std::to_string(no) + "_" + std::to_string(step) + "th.bmp";
//				img = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
//				//if (img.empty()) break;	//	�I������
//				if (img.empty()) readError(filename);	//	�I������
//
//														/*  Operator�̓h��Ԃ�  */
//				cv::Point center(img.cols / 1.9, img.rows / 2);
//				cv::Size radius(img.rows / 3, img.rows / 3);
//				double start_angle = -105;	//	��n�̒��S�p�x
//				double angle = 35;	//	�p�x
//				cv::ellipse(img, center, radius, start_angle, 0, angle, cv::Scalar(0, 0, 0), -1, CV_AA);
//
//				/* �O�g�̓h��Ԃ� */
//				cv::circle(img, center, 510, cv::Scalar(0, 0, 0), 150);
//
//				//�@���Ԋu�̉摜�ɐ؂�o��
//				cv::Rect roi_rect(ROI_ORG_X, ROI_ORG_Y, ROI_SIZE_X, ROI_SIZE_Y); // x,y,w,h
//				img = img(roi_rect);
//
//				cv::SiftFeatureDetector detector(1000);
//				cv::SiftDescriptorExtractor extractor;
//
//				//clock_t lap1 = clock();
//
//				//�摜��������_�����o
//				std::vector<cv::KeyPoint> keypoints;
//				detector.detect(img, keypoints);
//
//				//clock_t lap2 = clock();
//
//				//�摜�̓����_�ɂ���������ʂ𒊏o
//				cv::Mat descriptors;
//				extractor.compute(img, keypoints, descriptors);
//
//				//clock_t lap3 = clock();
//
//				/* keypoint�̕ۑ� */
//				filename = IFPATH_MEAS + "sift/keypoint/keypoint_no" + std::to_string(no) + "_" + std::to_string(step) + "th.yml";
//				cv::FileStorage fs(filename, cv::FileStorage::WRITE);
//				if (!fs.isOpened()) {
//					writeError(filename);
//				}
//				cv::write(fs, "keypoints", keypoints);
//
//				/*  descriptor�̏����o��  */
//				filename = IFPATH_MEAS + "sift/descriptor/desc_no" + std::to_string(no) + "_" + std::to_string(step) + "th.csv";
//				std::ofstream ofs(filename);
//				if (ofs.fail())	readError(filename);
//				ofs << cv::format(descriptors, "csv");
//
//
//				std::cout << "Finish Step: " << step << std::endl;
//				//if (step > 2260)	break;
//
//
//				step += thread_num;
//			}
//		}));	//	thread�̒�`�����܂�
//	}
//
//	for (auto& th : thread)
//	{
//		th.join();
//	}
//
//	return 0;
//
//}

/* keypoint��descriptor�̓ǂݍ��� */
//const std::string matching_type = "BruteForce-SL2";
//int _tmain(int argc, _TCHAR* argv[])
//{
//
//	clock_t start = clock();
//	std::vector<cv::KeyPoint> keypoints1;
//	{
//		//std::string filename = IFPATH_MEAS + "surf/keypoint/keypoint_no1_1th.yml";
//		std::string filename = "F://Data/Localization/Measurement/171031/1237/surf/keypoint/keypoint_no1_1th.yml";
//		cv::FileStorage fs(filename, cv::FileStorage::READ);
//		cv::FileNode fn = fs["keypoints"];
//		cv::read(fn, keypoints1);
//		if (keypoints1.empty())	readError(filename);
//	}
//
//	clock_t lap1= clock();
//
//	//std::cout << keypoints1[0].pt << std::endl;
//
//	clock_t lap2;
//	cv::Mat descriptors1;
//	{
//		//std::string filename = IFPATH_MEAS + "surf/descriptor/descriptor_no1_1th.yml";
//		std::string filename = "F://Data/Localization/Measurement/171031/1237/surf/descriptor/desc_no1_1th.csv";
//		std::ifstream ifs(filename);
//		if (ifs.fail()){
//			readError(filename);
//		}
//		std::vector<std::vector<float>> v;
//		ifs >> v;
//		lap2 = clock();
//		for (int i = 0; i < v.size(); i++){
//			cv::Mat mat(v[i], true);
//			mat = mat.t();
//			descriptors1.push_back(mat);
//		}
//		//std::cout << v.size() <<","<< v[0].size() << std::endl;
//		//std::cout << descriptors1.size() << std::endl;
//	}
//
//
//	std::cout << (int)(lap1 - start) << "," << (int)(lap2 - lap1) << std::endl;
//
//	return 0;
//}

/* ���l�e�X�g */
//#define HOGE 8E+09
//int _tmain(int argc, _TCHAR* argv[])
//{
//	std::cout << HOGE << std::endl;
//
//	std::cout << HOGE / std::pow(10, 9) << std::endl;
//
//	return 0;
//}

/* �p�[�e�B�N������`�� */
//#include "include\parameter.h"
//#define FIRST_STEP 1
//#define LAST_STEP 2
//static std::string filename;
//int _tmain(int argc, _TCHAR* argv[])
//{
//	cv::VideoWriter vwr;
//	/*  �r�f�I�֌W  */
//	// �t�@�C�����I�[�v�����C�r�f�I���C�^��������
//	// filename - �o�̓t�@�C����
//	// fourcc - �R�[�f�b�N
//	// fps - 1 �b������̃t���[����
//	// frameSize - �r�f�I�t���[���̃T�C�Y
//	// isColor - �r�f�I�X�g���[�����J���[���C�O���[�X�P�[�������w��
//	filename = OFPATH_PEAR + "Movie/particle.avi";
//	vwr.open(filename, CV_FOURCC('X', 'V', 'I', 'D'), 30.0, cv::Size(300, 300));
//	if (!vwr.isOpened())	writeError(filename);
//
//	/* �n�}�̓ǂݍ��� */
//
//
//
//
//
//
//
//	}

/* string test */
//int _tmain(int argc, _TCHAR* argv[])
//{
//	std::string str="hoege,";
//	std::cout << str << std::endl;
//
//	str.pop_back();
//	std::cout << str << std::endl;
//
//
//
//
//	return 0;
//
//}

/* std::thread�e�X�g */
//int _tmain(int argc, _TCHAR* argv[])
//{
//	std::vector<std::thread> th(3);
//
//	for (int i = 0; i < 3; i++){
//		th[i] = std::thread([i]{
//			std::cout << i << std::endl;
//		});
//	}
//
//	for (auto& t : th){
//		t.join();
//	}
//
//	for (int i = 0; i < 3; i++){
//		th[i] = std::thread([i]{
//			std::cout << i-5 << std::endl;
//		});
//	}
//
//	for (auto& t : th){
//		t.join();
//	}
//
//
//}

/* Particle Large Video �̍쐻(4sensors) */
//#include "include\LocalizationPF.h"
//#define SIZE_PARTICLES 100
//std::string TEST_PATH = "F://Data/Localization/Result/Visual Studio/Localization.ver2/parking/���U��_addbias_lrf�̂��؂�/���U��7_addbias_lrf�̂��؂�/pearson1/";
//// �m�����z�֌W
//#define MAP_RES_TEST MAP_RES
//class Hoge : public LocalizationPF
//{
//public:
//
//	Hoge(){};
//	~Hoge(){};
//
//	void initParticleVideo(std::string ofpath)
//	{
//		/*  �r�f�I�֌W  */
//		// �t�@�C�����I�[�v�����C�r�f�I���C�^��������
//		// filename - �o�̓t�@�C����
//		// fourcc - �R�[�f�b�N
//		// fps - 1 �b������̃t���[����
//		// frameSize - �r�f�I�t���[���̃T�C�Y
//		// isColor - �r�f�I�X�g���[�����J���[���C�O���[�X�P�[�������w��
//
//		std::string filename = ofpath + "movie2/if_nonstat_particle.avi";
//		//std::string filename = "./output/Movie/particle.avi";
//		Coor<> rect(CUT_MAP_RADIUS_X*6.0, CUT_MAP_RADIUS_Y*4.0);
//		cv::Size rect_pix = ToPixelSize(rect, map_img_clone, MAP_RES);
//		rect_pix.width *= MOVIE_SCALE_W;
//		rect_pix.height *= MOVIE_SCALE_H;
//		particle_video.open(filename, CV_FOURCC('X', 'V', 'I', 'D'), PARTICLE_FPS, rect_pix);
//		if (!particle_video.isOpened())	writeError(filename);
//	}
//	void initParticleVideoLarge(std::string ofpath)
//	{
//		/*  �r�f�I�֌W  */
//		// �t�@�C�����I�[�v�����C�r�f�I���C�^��������
//		// filename - �o�̓t�@�C����
//		// fourcc - �R�[�f�b�N
//		// fps - 1 �b������̃t���[����
//		// frameSize - �r�f�I�t���[���̃T�C�Y
//		// isColor - �r�f�I�X�g���[�����J���[���C�O���[�X�P�[�������w��
//
//		std::string filename = ofpath + "movie2/if_nonstat_particle_large.avi";
//		//std::string filename = "./output/Movie/particle.avi";
//		Coor<> rect(CUT_MAP_RADIUS_X*6.0, CUT_MAP_RADIUS_Y*4.0);
//		cv::Size rect_pix = ToPixelSize(rect, map_img_color, MAP_RES);
//		rect_pix.width *= MOVIE_SCALE_W;
//		rect_pix.height *= MOVIE_SCALE_H;
//		particle_large_video.open(filename, CV_FOURCC('X', 'V', 'I', 'D'), PARTICLE_FPS, rect_pix);
//		if (!particle_large_video.isOpened())	writeError(filename);
//	}
//
//	void decideUseSensor(){
//
//		//std::cout << std::endl;
//
//		Statistics<double> stat;
//		std::vector<bool> use_tmp_;
//		std::vector<std::vector<double>> similarity;
//		std::vector<std::vector<bool>> similar_;
//		float num_th;
//
//		/* SUYAMA�p */
//		std::vector<double> average;
//		std::vector<double> similarity_tmp;
//		double th;
//		std::vector<bool> similar_tmp_;
//
//		switch (trial_type)
//		{
//		case TRIAL_SIMULTANEOUS:
//		case TRIAL_NON_TIMESEQUENCE_SIMUL:
//
//			use_tmp_ = std::vector<bool>(SENSOR_NUM, true);
//			use_sim_.push_back(use_tmp_);
//			use_.push_back(use_tmp_);
//			break;
//
//		case TRIAL_PEARSON:
//			stat.add_data(stat_lid2_l_likelihood);
//			stat.add_data(stat_lid2_u_likelihood);
//			stat.add_data(stat_omni_likelihood);
//			stat.add_data(stat_gpgga_likelihood);
//
//			/*  �s�A�\���̐ϗ����֌W��  */
//			similarity = stat.pearsonCorr();
//			similarity_table.push_back(similarity);
//			//std::cout << "Similarity" << std::endl;
//			//std::cout << similarity;
//
//			/*  臒l�����ɂ��ގ��C��ގ��ɕ���  */
//			similar_ = ThresholdProcessing(similarity, PEARSON_CORRCOEF_TH);
//			similar_table_.push_back(similar_);
//			//std::cout << "Similar_" << std::endl;
//			//std::cout << similar_;
//
//			/*  �ߔ����̊m�����z�ɑ΂��Ĕ�ގ��ƂȂ镪�z��r��  */
//			num_th = SENSOR_NUM / 2.0;
//			use_tmp_ = UseIndex(similar_, num_th);
//			use_sim_.push_back(use_tmp_);
//			if (std::find(use_tmp_.begin(), use_tmp_.end(), true) == use_tmp_.end()) {
//				use_tmp_ = std::vector<bool>(SENSOR_NUM, true);
//			}
//			use_.push_back(use_tmp_);
//			break;
//
//		case TRIAL_PEARSON_NONSTAT:
//		case TRIAL_NON_TIMESEQUENCE:
//
//			stat.add_data(lid2_l_likelihood);
//			stat.add_data(lid2_u_likelihood);
//			stat.add_data(omni_likelihood);
//			stat.add_data(gpgga_likelihood);
//
//			/*  �s�A�\���̐ϗ����֌W��  */
//			similarity = stat.pearsonCorr();
//			similarity_table.push_back(similarity);
//			//std::cout << "Similarity" << std::endl;
//			//std::cout << similarity;
//
//			/*  臒l�����ɂ��ގ��C��ގ��ɕ���  */
//			similar_ = ThresholdProcessing(similarity, PEARSON_CORRCOEF_TH);
//			similar_table_.push_back(similar_);
//			//std::cout << "Similar_" << std::endl;
//			//std::cout << similar_;
//
//			/*  �ߔ����̊m�����z�ɑ΂��Ĕ�ގ��ƂȂ镪�z��r��  */
//			num_th = SENSOR_NUM / 2.0;
//			use_tmp_ = UseIndex(similar_, num_th);
//			use_sim_.push_back(use_tmp_);
//			if (std::find(use_tmp_.begin(), use_tmp_.end(), true) == use_tmp_.end()) {
//				use_tmp_ = std::vector<bool>(SENSOR_NUM, true);
//			}
//			use_.push_back(use_tmp_);
//			break;
//
//		case TRIAL_SUYAMA_STAT:
//
//			stat.add_data(stat_lid2_l_likelihood);
//			stat.add_data(stat_lid2_u_likelihood);
//			stat.add_data(stat_omni_likelihood);
//			stat.add_data(stat_gpgga_likelihood);
//
//			std::cout << std::endl;
//
//			/*  average  */
//			average = stat.average();
//
//			/*  kl Divergence  */
//			similarity_tmp = stat.KlDivergence(average);
//			similarity = { similarity_tmp };
//
//
//			similarity_table.push_back(similarity);
//
//			/*  threshold  */
//			th = Statistics<>::rootMeanSquare(similarity_tmp);
//			similar_tmp_ = ThresholdProcessing(similarity_tmp, th);
//			similar_ = { similar_tmp_ };
//			similar_table_.push_back(similar_);
//
//
//			for (int ni = 0; ni < SENSOR_NUM; ni++)
//			{
//				if (similarity_tmp[ni] <= th){
//					use_tmp_.push_back(true);
//				}
//				else {
//					use_tmp_.push_back(false);
//				}
//			}
//			use_sim_.push_back(use_tmp_);
//
//			if (std::find(use_tmp_.begin(), use_tmp_.end(), true) == use_tmp_.end()) {
//				use_tmp_ = std::vector<bool>(SENSOR_NUM, true);
//			}
//			use_.push_back(use_tmp_);
//			all_th.push_back(th);
//			break;
//
//		case TRIAL_SUYAMA_NONSTAT:
//
//			stat.add_data(lid2_l_likelihood);
//			stat.add_data(lid2_u_likelihood);
//			stat.add_data(omni_likelihood);
//			stat.add_data(gpgga_likelihood);
//
//			std::cout << std::endl;
//
//			/*  average  */
//			average = stat.average();
//
//			/*  kl Divergence  */
//			similarity_tmp = stat.KlDivergence(average);
//			similarity = { similarity_tmp };
//
//
//			similarity_table.push_back(similarity);
//
//			/*  threshold  */
//			th = Statistics<>::rootMeanSquare(similarity_tmp);
//			similar_tmp_ = ThresholdProcessing(similarity_tmp, th);
//			similar_ = { similar_tmp_ };
//			similar_table_.push_back(similar_);
//
//			for (int ni = 0; ni < SENSOR_NUM; ni++)
//			{
//				if (similarity_tmp[ni] <= th){
//					use_tmp_.push_back(true);
//				}
//				else {
//					use_tmp_.push_back(false);
//				}
//			}
//			use_sim_.push_back(use_tmp_);
//
//			if (std::find(use_tmp_.begin(), use_tmp_.end(), true) == use_tmp_.end()) {
//				use_tmp_ = std::vector<bool>(SENSOR_NUM, true);
//			}
//			use_.push_back(use_tmp_);
//			all_th.push_back(th);
//			break;
//
//		default:
//			std::cout << "Error 'TRIAL': " << TRIAL_TYPE << "FILE: " << __FILE__ << " LINE: " << __LINE__ << std::endl;
//			exit(0);
//			break;
//		}
//
//		//std::cout << "Use sensors: " << use_.back() << std::endl;
//		//std::cout << "Complete decideSensor!" << std::endl;
//
//		//std::cout << std::endl;
//
//	}
//
//
//	void readParticles(){
//
//		/* ����ʒu�̓ǂݍ��� */
//		std::vector<Position<>> esti_pos;
//		{
//			std::string filename = TEST_PATH + "Data/estimated_position.csv";
//			std::ifstream ifs(filename);
//			if (ifs.fail()){
//				readError(filename);
//			}
//			std::string str;
//			std::getline(ifs, str);
//			ifs >> esti_pos;
//		}
//
//
//		/* �f�[�^�̐��� */
//		std::vector<std::vector<std::string>> particle_data;
//		{
//			std::string filename = TEST_PATH + "Data/gitignore/particle.csv";
//			std::ifstream ifs(filename);
//			if (ifs.fail()){
//				readError(filename);
//			}
//			std::string str;
//			std::getline(ifs, str);
//			ifs >> particle_data;
//		}
//		std::cout << particle_data.size() << std::endl;
//
//		//std::vector<std::vector<std::string>> particle_data_after_resampling;
//		//{
//		//	std::string filename = TEST_PATH + "Data/gitignore/particle_before_resampling.csv";
//		//	std::ifstream ifs(filename);
//		//	if (ifs.fail()){
//		//		readError(filename);
//		//	}
//		//	std::string str;
//		//	std::getline(ifs, str);
//		//	ifs >> particle_data_after_resampling;
//		//}
//
//		std::vector<std::vector<std::string>> similar_table_str_;
//		{
//			std::string filename = TEST_PATH + "Data/gitignore/similar_table_.csv";
//			std::ifstream ifs(filename);
//			if (ifs.fail()){
//				readError(filename);
//			}
//			std::string str;
//			std::getline(ifs, str);
//			ifs >> similar_table_str_;
//		}
//
//		//std::vector<std::vector<bool>> use_sim_tmp_;
//		//for (int i = 0; i < similar_table_str_.size(); i += 4){
//		//	std::vector<std::vector<bool>> similar_tables_;
//		//	for (int j = 0; j < 4; j++){
//		//		bool s1 = std::stod(similar_table_str_[i + j][3]);
//		//		bool s2 = std::stod(similar_table_str_[i + j][4]);
//		//		bool s3 = std::stod(similar_table_str_[i + j][5]);
//		//		bool s4 = std::stod(similar_table_str_[i + j][6]);
//		//		similar_tables_.push_back({ s1, s2, s3, s4 });
//		//	}
//		//	std::vector<bool> use_sim_tmp_tmp = UseIndex(similar_tables_, 2);
//		//	use_sim_tmp_.push_back(use_sim_tmp_tmp);
//		//}
//		//use_sim_ = use_sim_tmp_;
//
//		trial_type = TRIAL_PEARSON_NONSTAT;
//
//
//		std::vector<int> step;
//		std::vector<MyTime> time;
//		std::vector<std::vector<Position<>>> particles;
//		std::vector<std::vector<double>> fusion_likelihood;
//		std::vector<std::vector<double>> lrf_l_likelihood;
//		std::vector<std::vector<double>> lrf_u_likelihood;
//		std::vector<std::vector<double>> cmr_likelihood;
//		std::vector<std::vector<double>> gps_likelihood;
//		std::vector<std::vector<Position<>>> particle_after_resampling;
//		for (int i = 0; i < particle_data.size(); i += SIZE_PARTICLES){
//
//			int step_tmp = std::stoi(particle_data[i][0]);
//			MyTime time_tmp(particle_data[i][1]);
//			std::vector<Position<>> particles_tmp;
//			std::vector<double> fusion_likelihood_tmp;
//			std::vector<double> lrf_l_likelihood_tmp;
//			std::vector<double> lrf_u_likelihood_tmp;
//			std::vector<double> cmr_likelihood_tmp;
//			std::vector<double> gps_likelihood_tmp;
//			std::vector<Position<>> particle_after_resampling_tmp;
//			for (int j = 0; j < SIZE_PARTICLES; j++){
//				double x = std::stod(particle_data[i + j][3]);
//				double y = std::stod(particle_data[i + j][4]);
//				double rad = std::stod(particle_data[i + j][5]);
//				particles_tmp.push_back(Position<>(x, y, rad));
//				fusion_likelihood_tmp.push_back(std::stod(particle_data[i + j][6]));
//				lrf_l_likelihood_tmp.push_back(std::stod(particle_data[i + j][7]));
//				lrf_u_likelihood_tmp.push_back(std::stod(particle_data[i + j][8]));
//				cmr_likelihood_tmp.push_back(std::stod(particle_data[i + j][9]));
//				gps_likelihood_tmp.push_back(std::stod(particle_data[i + j][10]));
//				//double x1 = std::stod(particle_data_after_resampling[i + j][3]);
//				//double y1 = std::stod(particle_data_after_resampling[i + j][4]);
//				//double rad1 = std::stod(particle_data_after_resampling[i + j][5]);
//				//particle_after_resampling_tmp.push_back(Position<>(x1, y1, rad1));
//			}
//
//			assert(particles_tmp.size() == fusion_likelihood_tmp.size());
//			assert(particles_tmp.size() == lrf_l_likelihood_tmp.size());
//			assert(particles_tmp.size() == lrf_u_likelihood_tmp.size());
//			assert(particles_tmp.size() == cmr_likelihood_tmp.size());
//			assert(particles_tmp.size() == gps_likelihood_tmp.size());
//			std::cout << i/SIZE_PARTICLES << std::endl;
//
//			step.push_back(step_tmp);
//			time.push_back(time_tmp);
//			particles.push_back(particles_tmp);
//			fusion_likelihood.push_back(fusion_likelihood_tmp);
//			lrf_l_likelihood.push_back(lrf_l_likelihood_tmp);
//			lrf_u_likelihood.push_back(lrf_u_likelihood_tmp);
//			cmr_likelihood.push_back(cmr_likelihood_tmp);
//			gps_likelihood.push_back(gps_likelihood_tmp);
//
//			this->lid2_l_likelihood = lrf_l_likelihood_tmp;
//			this->lid2_u_likelihood = lrf_u_likelihood_tmp;
//			this->omni_likelihood = cmr_likelihood_tmp;
//			this->gpgga_likelihood = gps_likelihood_tmp;
//
//			decideUseSensor();
//
//			//particle_after_resampling.push_back(particle_after_resampling_tmp);
//		}
//		assert(step.size() == time.size());
//		assert(step.size() == particles.size());
//		assert(step.size() == fusion_likelihood.size());
//		assert(step.size() == lrf_l_likelihood.size());
//		assert(step.size() == lrf_u_likelihood.size());
//		assert(step.size() == cmr_likelihood.size());
//		assert(step.size() == gps_likelihood.size());
//		assert(step.size() == esti_pos.size());
//
//		all_fusion_likelihood = fusion_likelihood;
//		all_lid2_l_likelihood = lrf_l_likelihood;
//		all_lid2_u_likelihood = lrf_u_likelihood;
//		all_omni_likelihood = cmr_likelihood;
//		all_gpgga_likelihood = gps_likelihood;
//		result_time = time;
//		all_particles = particles;
//		estimated_position = esti_pos;
//		//all_particles_after_resampling = particle_after_resampling;
//
//		
//
//
//	}
//
//	void createParticleMovie(){
//		for (int i = 0; i < result_time.size(); i++){
//			while (tidx < true_time.size() && result_time[movie_step] > true_time[tidx]) {
//				tidx++;
//			}
//			if (tidx >= true_time.size()){
//				break;
//			}
//			all_stock_tidx.push_back(tidx);
//			addFlameParticleVideo2();
//			//addFlameWeightedStatParImg2()
//			//addFlameParticleLargeVideo2();
//			std::cout << movie_step << std::endl;
//			movie_step++;
//			//if (movie_step > 10){
//			//	break;
//			//}
//		}
//	}
//
//
//	void Likelihood(){};
//
//};
//int _tmain(int argc, _TCHAR* argv[])
//{
//
//	Hoge hoge;
//
//	hoge.readGridMap(IFPATH_ENV + "lrf_upper/gridmap.bmp");
//	if (hoge.map_img_color.empty()){
//		std::cout << 1 << std::endl;
//		exit(0);
//	}
//
//	hoge.initParticleVideo(TEST_PATH);
//	hoge.initParticleVideoLarge(TEST_PATH);
//
//	hoge.readTruePosition(1);
//
//	hoge.readParticles();
//
//	hoge.createParticleMovie();
//
//	return 0;
//}

/* �J����180�x��] */
#include "include\parameter.h"
int _tmain(int argc, _TCHAR* argv[])
{
	int no = 3;

	int thread_num = 1;
	//int thread_num = std::thread::hardware_concurrency() / 2;

	std::vector<std::thread> thread;
	for (int th = 0; th < thread_num; th++)
	{
		/*  thread�̐���  */
		thread.push_back(std::thread([th, no, thread_num] {
			int step = th + 1;
			//int step = th * 5 + 2;
			while (1)
			{
				cv::Mat img;	//	�S���ʃJ�����摜

				/*  �t�@�C���̓ǂݍ���  */
				//std::string filename = "\\\\Desktop-mt35ltg/f/Data/Localization/Measurement/" + MEASUREMENT_DATE + "/" + MEASUREMENT_TIME + "/img/img_no" + std::to_string(no) + "_" + std::to_string(step) + "th.bmp";
				std::string filename = "F://Data/Localization/Measurement/" + MEASUREMENT_DATE + "/" + MEASUREMENT_TIME + "/img_180/img_no" + std::to_string(no) + "_" + std::to_string(step) + "th.bmp";
				img = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
				//if (img.empty()) break;	//	�I������

				if (img.empty()){
					break;
				}

				cv::flip(img, img, 0);
				cv::flip(img, img, 1);
				filename = "F://Data/Localization/Measurement/" + MEASUREMENT_DATE + "/" + MEASUREMENT_TIME + "/img/img_no" + std::to_string(no) + "_" + std::to_string(step) + "th.bmp";
				cv::imwrite(filename, img);


				std::cout << "Finish Step: " << step << std::endl;
				//if (step > 2260)	break;


				step += thread_num;
			}
		}));	//	thread�̒�`�����܂�
	}

	for (auto& th : thread)
	{
		th.join();
	}

	return 0;

}
