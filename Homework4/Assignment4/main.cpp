#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void* userdata)
{
	if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4)
	{
		std::cout << "Left button of the mouse is clicked - position (" << x << ", "
			<< y << ")" << '\n';
		control_points.emplace_back(x, y);
	}
}

void naive_bezier(const std::vector<cv::Point2f>& points, cv::Mat& window)
{
	auto& p_0 = points[0];
	auto& p_1 = points[1];
	auto& p_2 = points[2];
	auto& p_3 = points[3];

	for (double t = 0.0; t <= 1.0; t += 0.001)
	{
		auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
			3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

		window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
	}
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f>& control_points, float t)
{
	// TODO: Implement de Casteljau's algorithm
	// Ŀ����ʵ���ǲ��ϵݹ�ֱ��ֻʣ��һ���㣬��ʱ�����������ϵ�һ����
	if (control_points.size() == 2) {
		// ֹͣ����
		return control_points[0] + t * (control_points[1] - control_points[0]);
	}
	std::vector<cv::Point2f> control_points_tmp;
	for (int i = 0; i < control_points.size() - 1; i++) {
		control_points_tmp.push_back(control_points[i] + t * (control_points[i + 1] - control_points[i]));
	}
	return recursive_bezier(control_points_tmp, t);

}

void bezier(const std::vector<cv::Point2f>& control_points, cv::Mat& window)
{
	// TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
	// recursive Bezier algorithm.
	for (double t = 0; t <= 1; t += 0.001) {
		// �����õ��ľ���һ��������
		auto point = recursive_bezier(control_points, t);
		window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
		// ������������,�ȶ�������仯��
		const float x[4] = { 0,0,0.5,-0.5 };
		const float y[4] = { 0.5,-0.5,0,0 };
		// Ѱ�������������
		int xNow = round(point.x), yNow = round(point.y);
		// �������
		float d = std::sqrt(std::pow(point.x - xNow, 2) + std::pow(point.y - yNow, 2));
		for (int i = 0; i < 4; i++) {
			// ������Χ�����ɫֵ
			float xNeighbor = floor(point.x + x[i]);
			float yNeighbor = floor(point.y + y[i]);
			// ��֤��Χ�ĵ�û�г��߽�
			if (xNeighbor >= 0 && xNeighbor < 700 && yNeighbor >= 0 && yNeighbor < 700) {
				float w = d / std::sqrt(std::pow(xNeighbor - point.x, 2) + std::pow(yNeighbor - point.y, 2));
				// ȡMax��֤���ظ�����
				//window.at<cv::Vec3b>(yNeighbor, xNeighbor)[1] = 255*w;
				window.at<cv::Vec3b>(yNeighbor, xNeighbor)[1] = std::max(float(window.at<cv::Vec3b>(yNeighbor, xNeighbor)[1]), 255 * w);
			}
		}
	}


}

int main()
{
	cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
	cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
	cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

	cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

	int key = -1;
	while (key != 27)
	{
		for (auto& point : control_points)
		{
			cv::circle(window, point, 3, { 255, 255, 255 }, 3);
		}

		if (control_points.size() == 4)
		{
			//naive_bezier(control_points, window);
			bezier(control_points, window);

			cv::imshow("Bezier Curve", window);
			cv::imwrite("my_bezier_curve.png", window);
			key = cv::waitKey(0);

			return 0;
		}

		cv::imshow("Bezier Curve", window);
		key = cv::waitKey(20);
	}

	return 0;
}
