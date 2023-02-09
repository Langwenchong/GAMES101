#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

	Eigen::Matrix4f translate;
	translate << 1, 0, 0, -eye_pos[0],
		0, 1, 0, -eye_pos[1],
		0, 0, 1, -eye_pos[2],
		0, 0, 0, 1;

	view = translate * view;

	return view;
}

Eigen::Matrix4f get_model_matrix(float angle)
{
	Eigen::Matrix4f rotation;
	angle = angle * MY_PI / 180.f;
	rotation << cos(angle), 0, sin(angle), 0,
		0, 1, 0, 0,
		-sin(angle), 0, cos(angle), 0,
		0, 0, 0, 1;

	Eigen::Matrix4f scale;
	scale << 2.5, 0, 0, 0,
		0, 2.5, 0, 0,
		0, 0, 2.5, 0,
		0, 0, 0, 1;

	Eigen::Matrix4f translate;
	translate << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
	// TODO: Use the same projection matrix from the previous assignments
	// 先透视投影
	Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f m;
	m << zNear, 0, 0, 0, \
		0, zNear, 0, 0, \
		0, 0, zNear + zFar, -zNear * zFar, \
		0, 0, 1, 0;
	// 通过垂直角度FovY和zNear计算出高度的一半即top
	float halve = eye_fov / 2 * acos(-1) / 180.0;
	float top = -tan(halve) * zNear;
	// 底部就是-top，height=top-bottom;
	float bottom = -top;
	//再通过长宽比计算出宽度width
	float right = top * aspect_ratio;
	float left = -right;
	Eigen::Matrix4f n, p;
	// 正交化
	n << 2 / (right - left), 0, 0, 0,
		0, 2 / (top - bottom), 0, 0,
		0, 0, 2 / (zNear - zFar), 0,
		0, 0, 0, 1;
	//再视口变换移到以左下角为原点
	p << 1, 0, 0, -(right + left) / 2,
		0, 1, 0, -(top + bottom) / 2,
		0, 0, 1, -(zFar + zNear) / 2,
		0, 0, 0, 1;
	// 要注意左乘顺序
	projection = n * p * m;
	return projection;

}

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
	return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
	Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
	Eigen::Vector3f result;
	result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
	return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
	auto costheta = vec.dot(axis);
	return (2 * costheta * axis - vec).normalized();
}

struct light
{
	Eigen::Vector3f position;
	Eigen::Vector3f intensity;
};

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
	Eigen::Vector3f return_color = { 0, 0, 0 };
	if (payload.texture)
	{
		// TODO: Get the texture value at the texture coordinates of the current fragment
		// 这里提供的代码优点问题会触发一个assert错误，需要修改一下getColor()函数
		return_color = payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y());
	}
	Eigen::Vector3f texture_color;
	texture_color << return_color.x(), return_color.y(), return_color.z();

	Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
	Eigen::Vector3f kd = texture_color / 255.f;
	Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

	auto l1 = light{ {20, 20, 20}, {500, 500, 500} };
	auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };

	std::vector<light> lights = { l1, l2 };
	Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
	Eigen::Vector3f eye_pos{ 0, 0, 10 };

	float p = 150;

	Eigen::Vector3f color = texture_color;
	Eigen::Vector3f point = payload.view_pos;
	Eigen::Vector3f normal = payload.normal;

	Eigen::Vector3f result_color = { 0, 0, 0 };

	for (auto& light : lights)
	{
		// TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
		// components are. Then, accumulate that result on the *result_color* object.
		// 首先计算出l,v,h等向量坐标，并且需要归一化处理
		Eigen::Vector3f light_dir = (light.position - point).normalized();
		Eigen::Vector3f view_dir = (eye_pos - point).normalized();
		Eigen::Vector3f half_vector = (light_dir + view_dir).normalized();

		// 定义距离衰减r方，可以通过向量自身点乘实现求距离的平方
		float r2 = (light.position - point).dot(light.position - point);

		// 环境光
		// cwiseProduct可以实现矩阵的点对点相乘
		Eigen::Vector3f La = ka.cwiseProduct(amb_light_intensity);

		// 漫反射
		Eigen::Vector3f Ld = kd.cwiseProduct(light.intensity / r2);
		Ld *= std::max(0.0f, normal.normalized().dot(light_dir));

		// 高光
		Eigen::Vector3f Ls = ks.cwiseProduct(light.intensity / r2);
		Ls *= std::pow(std::max(0.0f, normal.normalized().dot(half_vector)), p);

		result_color += (La + Ld + Ls);

	}

	return result_color * 255.f;
}

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
	Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
	Eigen::Vector3f kd = payload.color;
	Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

	auto l1 = light{ {20, 20, 20}, {500, 500, 500} };
	auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };

	std::vector<light> lights = { l1, l2 };
	Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
	Eigen::Vector3f eye_pos{ 0, 0, 10 };

	// 用来快速衰减
	float p = 150;

	Eigen::Vector3f color = payload.color;
	Eigen::Vector3f point = payload.view_pos;
	Eigen::Vector3f normal = payload.normal;

	Eigen::Vector3f result_color = { 0, 0, 0 };
	for (auto& light : lights)
	{
		// TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
		// components are. Then, accumulate that result on the *result_color* object.
		// 首先计算出l,v,h等向量坐标，并且需要归一化处理
		Eigen::Vector3f light_dir = (light.position - point).normalized();
		Eigen::Vector3f view_dir = (eye_pos - point).normalized();
		Eigen::Vector3f half_vector = (light_dir + view_dir).normalized();

		// 定义距离衰减r方，可以通过向量自身点乘实现求距离的平方
		float r2 = (light.position - point).dot(light.position - point);

		// 环境光
		// cwiseProduct可以实现矩阵的点对点相乘
		Eigen::Vector3f La = ka.cwiseProduct(amb_light_intensity);

		// 漫反射
		Eigen::Vector3f Ld = kd.cwiseProduct(light.intensity / r2);
		Ld *= std::max(0.0f, normal.normalized().dot(light_dir));

		// 高光
		Eigen::Vector3f Ls = ks.cwiseProduct(light.intensity / r2);
		Ls *= std::pow(std::max(0.0f, normal.normalized().dot(half_vector)), p);

		result_color += (La + Ld + Ls);
	}

	return result_color * 255.f;
}



Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{

	Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
	Eigen::Vector3f kd = payload.color;
	Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

	auto l1 = light{ {20, 20, 20}, {500, 500, 500} };
	auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };

	std::vector<light> lights = { l1, l2 };
	Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
	Eigen::Vector3f eye_pos{ 0, 0, 10 };

	float p = 150;

	Eigen::Vector3f color = payload.color;
	Eigen::Vector3f point = payload.view_pos;
	Eigen::Vector3f normal = payload.normal;

	float kh = 0.2, kn = 0.1;

	// TODO: Implement displacement mapping here
	// Let n = normal = (x, y, z)
	// Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
	// Vector b = n cross product t
	// Matrix TBN = [t b n]
	// dU = kh * kn * (h(u+1/w,v)-h(u,v))
	// dV = kh * kn * (h(u,v+1/h)-h(u,v))
	// Vector ln = (-dU, -dV, 1)
	// Position p = p + kn * n * h(u,v)
	// Normal n = normalize(TBN * ln)
	float x = normal.x();
	float y = normal.y();
	float z = normal.z();

	Eigen::Vector3f t = Eigen::Vector3f(x * y / std::sqrt(x * x + z * z), std::sqrt(x * x + z * z), z * y / std::sqrt(x * x + z * z));
	Eigen::Vector3f b = normal.cross(t);

	Eigen::Matrix3f TBN;
	// 用来根据求得的切线去计算垂直的法向量变换矩阵
	TBN << t.x(), b.x(), normal.x(),
		t.y(), b.y(), normal.y(),
		t.z(), b.z(), normal.z();

	// 用来求导数的指标，其中用w/h主要是要取一个小单位长度
	float u = payload.tex_coords.x();
	float v = payload.tex_coords.y();
	float w = payload.texture->width;
	float h = payload.texture->height;

	// 求导数值再交换得扰动后的法向量值
	float dU = kh * kn * (payload.texture->getColor(u + 1.0f / w, v).norm() - payload.texture->getColor(u, v).norm());
	float dV = kh * kn * (payload.texture->getColor(u, v + 1.0f / h).norm() - payload.texture->getColor(u, v).norm());

	Eigen::Vector3f ln = Eigen::Vector3f(-dU, -dV, 1.0f);
	// 点处也要有变化，这样即实现了点的高度变化即位移
	point += (kn * normal * payload.texture->getColor(u, v).norm());
	// 得到扰动以后的向量即得凹凸效果
	normal = TBN * ln;

	Eigen::Vector3f result_color = { 0, 0, 0 };

	for (auto& light : lights)
	{
		// TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
		// components are. Then, accumulate that result on the *result_color* object.
		// 首先计算出l,v,h等向量坐标，并且需要归一化处理
		Eigen::Vector3f light_dir = (light.position - point).normalized();
		Eigen::Vector3f view_dir = (eye_pos - point).normalized();
		Eigen::Vector3f half_vector = (light_dir + view_dir).normalized();

		// 定义距离衰减r方，可以通过向量自身点乘实现求距离的平方
		float r2 = (light.position - point).dot(light.position - point);

		// 环境光
		// cwiseProduct可以实现矩阵的点对点相乘
		Eigen::Vector3f La = ka.cwiseProduct(amb_light_intensity);

		// 漫反射
		Eigen::Vector3f Ld = kd.cwiseProduct(light.intensity / r2);
		Ld *= std::max(0.0f, normal.normalized().dot(light_dir));

		// 高光
		Eigen::Vector3f Ls = ks.cwiseProduct(light.intensity / r2);
		Ls *= std::pow(std::max(0.0f, normal.normalized().dot(half_vector)), p);

		result_color += (La + Ld + Ls);

	}

	return result_color * 255.f;
}


Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{

	Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
	Eigen::Vector3f kd = payload.color;
	Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

	auto l1 = light{ {20, 20, 20}, {500, 500, 500} };
	auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };

	std::vector<light> lights = { l1, l2 };
	Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
	Eigen::Vector3f eye_pos{ 0, 0, 10 };

	float p = 150;

	Eigen::Vector3f color = payload.color;
	Eigen::Vector3f point = payload.view_pos;
	Eigen::Vector3f normal = payload.normal;


	float kh = 0.2, kn = 0.1;

	// TODO: Implement bump mapping here
	// Let n = normal = (x, y, z)
	// Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
	// Vector b = n cross product t
	// Matrix TBN = [t b n]
	// dU = kh * kn * (h(u+1/w,v)-h(u,v))
	// dV = kh * kn * (h(u,v+1/h)-h(u,v))
	// Vector ln = (-dU, -dV, 1)
	// Normal n = normalize(TBN * ln)
	float x = normal.x();
	float y = normal.y();
	float z = normal.z();

	Eigen::Vector3f t = Eigen::Vector3f(x * y / std::sqrt(x * x + z * z), std::sqrt(x * x + z * z), z * y / std::sqrt(x * x + z * z));
	Eigen::Vector3f b = normal.cross(t);

	Eigen::Matrix3f TBN;
	// 用来根据求得的切线去计算垂直的法向量变换矩阵
	TBN << t.x(), b.x(), normal.x(),
		t.y(), b.y(), normal.y(),
		t.z(), b.z(), normal.z();

	// 用来求导数的指标，其中用w/h主要是要取一个小单位长度
	float u = payload.tex_coords.x();
	float v = payload.tex_coords.y();
	float w = payload.texture->width;
	float h = payload.texture->height;

	// 求导数值再交换得扰动后的法向量值
	float dU = kh * kn * (payload.texture->getColor(u + 1.0f / w, v).norm() - payload.texture->getColor(u, v).norm());
	float dV = kh * kn * (payload.texture->getColor(u, v + 1.0f / h).norm() - payload.texture->getColor(u, v).norm());

	Eigen::Vector3f ln = Eigen::Vector3f(-dU, -dV, 1.0f);
	// 得到扰动以后的向量即得凹凸效果
	normal = TBN * ln;

	Eigen::Vector3f result_color = { 0, 0, 0 };
	result_color = normal;

	return result_color * 255.f;
}

int main(int argc, const char** argv)
{
	std::vector<Triangle*> TriangleList;

	float angle = 140.0;
	bool command_line = false;

	std::string filename = "output.png";
	objl::Loader Loader;
	std::string obj_path = "C:/Users/86159/Desktop/Homework3/Homework3/Assignment3/models/spot/";

	// Load .obj File
	bool loadout = Loader.LoadFile("C:/Users/86159/Desktop/Homework3/Homework3/Assignment3/models/spot/spot_triangulated_good.obj");
	for (auto mesh : Loader.LoadedMeshes)
	{
		for (int i = 0; i < mesh.Vertices.size(); i += 3)
		{
			Triangle* t = new Triangle();
			for (int j = 0; j < 3; j++)
			{
				t->setVertex(j, Vector4f(mesh.Vertices[i + j].Position.X, mesh.Vertices[i + j].Position.Y, mesh.Vertices[i + j].Position.Z, 1.0));
				t->setNormal(j, Vector3f(mesh.Vertices[i + j].Normal.X, mesh.Vertices[i + j].Normal.Y, mesh.Vertices[i + j].Normal.Z));
				t->setTexCoord(j, Vector2f(mesh.Vertices[i + j].TextureCoordinate.X, mesh.Vertices[i + j].TextureCoordinate.Y));
			}
			TriangleList.push_back(t);
		}
	}

	rst::rasterizer r(700, 700);

	auto texture_path = "spot_texture.png";
	r.set_texture(Texture(obj_path + texture_path));

	std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = texture_fragment_shader;

	if (argc >= 2)
	{
		command_line = true;
		filename = std::string(argv[1]);

		if (argc == 3 && std::string(argv[2]) == "texture")
		{
			std::cout << "Rasterizing using the texture shader\n";
			active_shader = texture_fragment_shader;
			texture_path = "spot_texture.png";
			r.set_texture(Texture(obj_path + texture_path));
		}
		else if (argc == 3 && std::string(argv[2]) == "normal")
		{
			std::cout << "Rasterizing using the normal shader\n";
			active_shader = normal_fragment_shader;
		}
		else if (argc == 3 && std::string(argv[2]) == "phong")
		{
			std::cout << "Rasterizing using the phong shader\n";
			active_shader = phong_fragment_shader;
		}
		else if (argc == 3 && std::string(argv[2]) == "bump")
		{
			std::cout << "Rasterizing using the bump shader\n";
			active_shader = bump_fragment_shader;
		}
		else if (argc == 3 && std::string(argv[2]) == "displacement")
		{
			std::cout << "Rasterizing using the bump shader\n";
			active_shader = displacement_fragment_shader;
		}
	}

	Eigen::Vector3f eye_pos = { 0,0,10 };

	r.set_vertex_shader(vertex_shader);
	r.set_fragment_shader(active_shader);

	int key = 0;
	int frame_count = 0;

	if (command_line)
	{
		r.clear(rst::Buffers::Color | rst::Buffers::Depth);
		r.set_model(get_model_matrix(angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

		r.draw(TriangleList);
		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
		image.convertTo(image, CV_8UC3, 1.0f);
		cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

		cv::imwrite(filename, image);

		return 0;
	}

	while (key != 27)
	{
		r.clear(rst::Buffers::Color | rst::Buffers::Depth);

		r.set_model(get_model_matrix(angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

		//r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
		r.draw(TriangleList);
		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
		image.convertTo(image, CV_8UC3, 1.0f);
		cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

		cv::imshow("image", image);
		cv::imwrite(filename, image);
		key = cv::waitKey(10);

		if (key == 'a')
		{
			angle -= 0.1;
		}
		else if (key == 'd')
		{
			angle += 0.1;
		}

	}
	return 0;
}
