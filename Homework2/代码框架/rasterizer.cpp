// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f>& positions)
{
	auto id = get_next_id();
	pos_buf.emplace(id, positions);

	return { id };
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i>& indices)
{
	auto id = get_next_id();
	ind_buf.emplace(id, indices);

	return { id };
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f>& cols)
{
	auto id = get_next_id();
	col_buf.emplace(id, cols);

	return { id };
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
	return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

// 这里一定要注意哇需要将形参改为float类型，否则MSAA是没有效果的
static bool insideTriangle(float x, float y, const Vector3f* _v)
{
	// TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
	Eigen::Vector2f p;
	p << x, y;
	// 取三个点的坐标比较取最小或者最大值从而创建一个boundingbox
	Eigen::Vector2f AB = _v[1].head(2) - _v[0].head(2);
	Eigen::Vector2f BC = _v[2].head(2) - _v[1].head(2);
	Eigen::Vector2f CA = _v[0].head(2) - _v[2].head(2);

	Eigen::Vector2f AP = p - _v[0].head(2);
	Eigen::Vector2f BP = p - _v[1].head(2);
	Eigen::Vector2f CP = p - _v[2].head(2);

	// 这里只需要判断是不是叉乘以后都是同方向的向量
	// 所以我们只需要通过坐标运算得到z坐标看是否同向，这里是判断是否大于零
	return AB[0] * AP[1] - AB[1] * AP[0] > 0
		&& BC[0] * BP[1] - BC[1] * BP[0] > 0
		&& CA[0] * CP[1] - CA[1] * CP[0] > 0;
}


static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
	float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
	float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
	float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
	return { c1,c2,c3 };
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
	auto& buf = pos_buf[pos_buffer.pos_id];
	auto& ind = ind_buf[ind_buffer.ind_id];
	auto& col = col_buf[col_buffer.col_id];

	float f1 = (50 - 0.1) / 2.0;
	float f2 = (50 + 0.1) / 2.0;

	Eigen::Matrix4f mvp = projection * view * model;
	for (auto& i : ind)
	{
		Triangle t;
		Eigen::Vector4f v[] = {
				mvp * to_vec4(buf[i[0]], 1.0f),
				mvp * to_vec4(buf[i[1]], 1.0f),
				mvp * to_vec4(buf[i[2]], 1.0f)
		};
		//Homogeneous division
		for (auto& vec : v) {
			vec /= vec.w();
		}
		//Viewport transformation
		for (auto& vert : v)
		{
			vert.x() = 0.5 * width * (vert.x() + 1.0);
			vert.y() = 0.5 * height * (vert.y() + 1.0);
			vert.z() = vert.z() * f1 + f2;
		}

		for (int i = 0; i < 3; ++i)
		{
			t.setVertex(i, v[i].head<3>());
			t.setVertex(i, v[i].head<3>());
			t.setVertex(i, v[i].head<3>());
		}

		auto col_x = col[i[0]];
		auto col_y = col[i[1]];
		auto col_z = col[i[2]];

		t.setColor(0, col_x[0], col_x[1], col_x[2]);
		t.setColor(1, col_y[0], col_y[1], col_y[2]);
		t.setColor(2, col_z[0], col_z[1], col_z[2]);

		rasterize_triangle(t);
	}
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
	auto v = t.toVector4();

	// TODO : Find out the bounding box of current triangle.
	// iterate through the pixel and find if the current pixel is inside the triangle

	int min_x = std::min(v[0].x(), std::min(v[1].x(), v[2].x()));
	int max_x = std::max(v[0].x(), std::max(v[1].x(), v[2].x()));
	int min_y = std::min(v[0].y(), std::min(v[1].y(), v[2].y()));
	int max_y = std::max(v[0].y(), std::max(v[1].y(), v[2].y()));


	// TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
	// 是否开启MSAA进行抗锯齿处理
	bool MSAA = true;
	if (MSAA) {
		std::cout << "开启了MSAA" <<std::endl;;
		std::vector<Eigen::Vector2f> pos{
			// 对于每一个pixel进一步进行划分成四个小点从而达到MSAA边界抗锯齿处理
			{0.25,0.25},{0.75,0.25},{0.25,0.75},{0.75,0.75},
		};
		// 接下来进行光栅化着色
		for (int x = min_x; x <= max_x; x++) {
			for (int y = min_y; y <= max_y; y++) {
				// 遍历之前处理出来的boundingbox中的每一个像素点
				// 深度是小的在前面，大的在后面，初始时将最小深度值设置为最大开始遍历
				float minDepth = FLT_MAX;
				//用来统计一个像素点中有多少个小点在三角形内
				int count = 0;
				for (int i = 0; i < 4; i++) {
					if (insideTriangle((float)x + pos[i][0], (float)y + pos[i][1], t.v)) {
						// If so, use the following code to get the interpolated z value.
						// 如果在，对深度z进行插值,注意这里和非MSAA有变化，这里需要判断每一个小点，所以加的不是0.5
						auto tup = computeBarycentric2D((float)x + pos[i][0], (float)y + pos[i][1], t.v);
						float alpha;
						float beta;
						float gamma;
						std::tie(alpha, beta, gamma) = tup;
						float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
						float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
						z_interpolated *= w_reciprocal;
						// 与当前最小深度进行比较，如果比其小说明在其前面，应该着色，更新最浅深度同时说明这个像素内的小点应着色
						minDepth = std::min(minDepth, z_interpolated);
						count++;
					}
				}
				//std::cout << count << std::endl;
				if (count != 0) {
					// 说明这个像素至少有一个小点在三角形内同时在前面，则着色，因此要更新存储的应该着色的像素点和颜色值
					if (depth_buf[get_index(x, y)] > minDepth) {
						// 根据小点数量计算要更新的颜色值，同时替换要上色的深度值，因为是按比例上色，显然边界会有模糊
						Vector3f color = t.getColor() * count / 4.0;
						Vector3f point(3);
						point << (float)x, (float)y, minDepth;
						depth_buf[get_index(x, y)] = minDepth;
						// 着色像素
						set_pixel(point, color);
					}
				}
			}
		}
	}
	else {
		std::cout << "未开启MSAA" << std::endl;
		//不开启MSAA那么会造成锯齿严重，边界会出现明显黑边
		for (int x = min_x; x <= max_x; x++) {
			for (int y = min_y; y <= max_y; y++) {
				if (insideTriangle((float)x + 0.5, (float)y + 0.5, t.v)) {
					// If so, use the following code to get the interpolated z value.
					// 如果在，对深度z进行插值
					auto tup = computeBarycentric2D((float)x + 0.5, (float)y + 0.5, t.v);
					float alpha;
					float beta;
					float gamma;
					std::tie(alpha, beta, gamma) = tup;
					float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
					float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
					z_interpolated *= w_reciprocal;

					if (depth_buf[get_index(x, y)] > z_interpolated) {
						Vector3f color = t.getColor();
						Vector3f point(3);
						point << (float)x, (float)y, z_interpolated;
						depth_buf[get_index(x, y)] = z_interpolated;
						set_pixel(point, color);
					}
				}
			}
		}
	}
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
	model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
	view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
	projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
	if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
	{
		std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
	}
	if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
	{
		std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
	}
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
	frame_buf.resize(w * h);
	depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
	return (height - 1 - y) * width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
	//old index: auto ind = point.y() + point.x() * width;
	auto ind = (height - 1 - point.y()) * width + point.x();
	frame_buf[ind] = color;

}

// clang-format on