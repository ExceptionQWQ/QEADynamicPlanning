#include "ransac.h"

/*
 * @brief 画直线
 * @param view 图像
 * @param k 斜率
 * @param z 截距
 * @param color 颜色
 */
void DrawLine(cv::Mat view, double k, double z, cv::Scalar color)
{
	cv::line(view, cv::Point(0, z), cv::Point(view.cols, view.cols * k + z), color, 3);
}

/*
 * @brief 画圆
 * @param x 圆心x坐标
 * @param y 圆心y坐标
 * @param r 圆半径
 * @param color 颜色
 */
void DrawCircle(cv::Mat view, double x, double y, double r, cv::Scalar color)
{
    cv::circle(view, cv::Point(x, y), r, color, 3);
}

/*
 * @brief 生成含有一条随机直线的图像
 */
cv::Mat GenerateLineView()
{
	cv::Mat view = cv::Mat::zeros(cv::Size(512, 512), CV_8UC1);
	double z = rand() % 10;
	double k = 1 + (double)rand() / RAND_MAX;
	DrawLine(view, k, z);
	return view;
}

/*
 * @brief 生成含有一条随机线段的图像
 */
cv::Mat GenerateLineSegmentView()
{
	cv::Mat view = cv::Mat::zeros(cv::Size(512, 512), CV_8UC1);
	double x1 = rand() % 512, y1 = rand() % 512;
	double x2 = rand() % 512, y2 = rand() % 512;
	cv::line(view, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255), 3);
	return view;
}

/*
 * @brief 生成含有一个随机圆的图像
 */
cv::Mat GenerateCircleView()
{
	cv::Mat view = cv::Mat::zeros(cv::Size(512, 512), CV_8UC1);
	double x = rand() % 300 + 100;
	double y = rand() % 300 + 100;
	double r = rand() % 100 + 20;
	DrawCircle(view, x, y, r);
	return view;
}

/*
 * @brief 制造噪声
 */
cv::Mat MakeNoise(cv::Mat view)
{
	for (int i = 0; i < 1000; ++i) {
		int row = rand() % view.rows;
		int col = rand() % view.cols;
		view.at<uint8_t>(row, col) = 255;
	}
	return view;
}

/*
 * @brief 根据3个点求解圆
 * @param p1 第一个点 p2 第二个点 p3 第三个点
 * @return 圆心坐标， 半径
 */
std::pair<cv::Point, double> CalcCircleFromThreePoints(cv::Point p1, cv::Point p2, cv::Point p3)
{
	double a = 2 * (p2.x - p1.x);
    double b = 2 * (p2.y - p1.y);
    double c = p2.x * p2.x + p2.y * p2.y - p1.x * p1.x - p1.y * p1.y;
    double d = 2 * (p3.x - p2.x);
    double e = 2 * (p3.y - p2.y);
    double f = p3.x * p3.x + p3.y * p3.y - p2.x * p2.x - p2.y * p2.y;
	double x = (b * f - e * c) / (b * d - e * a);
    double y = (d * c - a * f) / (b * d - e * a);
    double r = sqrt((x - p1.x) * (x - p1.x) + (y - p1.y) * (y - p1.y));
    return std::make_pair(cv::Point(x, y), r);
}

/*
 * @brief 将图像转换成点的向量
 */
std::vector<cv::Point2d> ConvertViewToPoints(cv::Mat view)
{
	double cx = view.cols / 2, cy = view.rows / 2;
	std::vector<cv::Point2d> points;
	for (int y = 0; y < view.rows; ++y) {
		for (int x = 0; x < view.cols; ++x) {
			if (view.at<uint8_t>(y, x) != 255) continue ;
			points.emplace_back(x, y);
		}
	}
	return points;
}

/*
 * @brief 利用ransac算法寻找直线
 * @param points 点的向量
 * @return 斜率，截距
 */
std::optional<std::pair<double, double>> FindLineRansac(const std::vector<cv::Point2d>& points)
{
	int maxCnt = 0;
	double matchK, matchZ;
	std::vector<cv::Point2d> seq = points; //拷贝一份点的数据
	std::shuffle(std::begin(seq), std::end(seq), std::random_device()); //随机打乱顺序
	int randIndex = 0;
	for (int i = 0; i < 100; ++i) { //迭代
		if (randIndex + 1 >= seq.size()) {
			randIndex = 0;
			std::shuffle(std::begin(seq), std::end(seq), std::random_device()); //随机打乱顺序
		}
		//随机获取2个点
		cv::Point2d p1 = seq[randIndex++];
		cv::Point2d p2 = seq[randIndex++];

		//计算斜率及截距
		double k = (p2.y - p1.y) / (p2.x - p1.x);
		double z = p1.y - k * p1.x;

		int cnt = 0;

		//枚举每个点是否在这条直线上
		for (size_t j = 0; j != points.size(); ++j) {
			cv::Point2d p = points[j];
			double dis = std::abs(k * p.x - p.y + z) / std::sqrt(std::pow(k, 2) + 1);
			
			if (dis < 3) ++cnt;
		}

		//如果有更多的点在直线上，则更新最终匹配的直线。
		if (cnt > maxCnt) {
			maxCnt = cnt;
			matchK = k;
			matchZ = z;
		}
	}

	if (maxCnt < 10) return std::nullopt;
	return std::make_pair(matchK, matchZ);
}

/*
 * @brief 利用ransac算法寻找线段
 * @param points 点的向量
 * @return 线段两端点的坐标
 */
std::optional<std::pair<cv::Point, cv::Point>> FindLineSegmentRansac(const cv::Mat& view, double minLen, double maxLen)
{
	auto ret = FindLineRansac(ConvertViewToPoints(view));
	if (!ret.has_value()) return std::nullopt;

	double k = ret->first, z = ret->second;
	int kernel = 1 + 2 * 3;
	int startX = -1, startY;
	for (int x = 0; x < view.cols; ++x) {
		int y = k * x + z;
		if (y < 0 || y >= view.rows) continue;
		int cnt = 0;
		for (int i = 0; i < kernel; ++i) {
			for (int j = 0; j < kernel; ++j) {
				int tx = x + i - kernel / 2;
				int ty = y + j - kernel / 2;
				if (tx < 0 || tx >= view.cols || ty < 0 || ty >= view.rows) continue;
				if (view.at<uint8_t>(ty, tx) == 255) ++cnt;
			}
		}
		if (startX == -1 && cnt != 0) {
			startX = x;
			startY = y;
		} else if (startX != -1 && cnt == 0) { //线段结束
			double len = std::sqrt(std::pow(x - startX, 2) + std::pow(y - startY, 2));
			if (len >= minLen && len <= maxLen) {
				return std::make_pair(cv::Point(startX, startY), cv::Point(x, y));
			} else {
				startX = -1;
			}
		}
	}
	if (startX != -1) {
		int x = view.cols;
		int y = k * x + z;
		double len = std::sqrt(std::pow(x - startX, 2) + std::pow(y - startY, 2));
		if (len >= minLen && len <= maxLen) {
			return std::make_pair(cv::Point(startX, startY), cv::Point(x, y));
		} 
	}
	return std::nullopt;
}

/*
 * @brief 利用ransac算法寻找圆/圆弧
 * @param points 点的向量
 * @param minR 要检测的圆的最小半径
 * @param maxR 要检测的圆的最大半径
 * @return 圆心坐标， 半径
 */
std::optional<std::pair<cv::Point, double>> FindCircleRansac(const std::vector<cv::Point2d>& points, double minR, double maxR)
{
	int maxCnt = 0;
	double matchX, matchY, matchR;
	std::vector<cv::Point2d> seq = points;
	std::shuffle(std::begin(seq), std::end(seq), std::random_device()); //随机打乱点的顺序
	int randIndex = 0;
	for (int i = 0; i < 10000; ++i) { //迭代
		if (randIndex + 2 >= seq.size()) {
			randIndex = 0;
			std::shuffle(std::begin(seq), std::end(seq), std::random_device()); //随机打乱点的顺序
		}
		//随机获取3个点的坐标
		cv::Point2d p1 = seq[randIndex++];
		cv::Point2d p2 = seq[randIndex++];
		cv::Point2d p3 = seq[randIndex++];

		//求解圆心坐标及半径
		auto [pc, r] = CalcCircleFromThreePoints(p1, p2, p3);

		//筛选目标圆半径
		if (r < minR || r > maxR) continue;

		int cnt = 0;

		//枚举每个点是否在圆上
		for (size_t j = 0; j != points.size(); ++j) {
			cv::Point2d p = points[j];
			double dis = std::sqrt(std::pow(p.x - pc.x, 2) + std::pow(p.y - pc.y, 2));
			if (std::fabs(dis - r) < 3) ++cnt;
		}
		//如果有更多的点在圆上，则更新最终匹配结果。
		if (cnt > maxCnt) {
			maxCnt = cnt;
			matchX = pc.x;
			matchY = pc.y;
			matchR = r;
		}
	}
	if (maxCnt < 10) return std::nullopt;
	return std::make_pair(cv::Point(matchX, matchY), matchR);
}