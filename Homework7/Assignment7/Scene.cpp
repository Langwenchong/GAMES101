//
// Created by Göksu Güvendiren on 2019-05-14.
//
#include "Scene.hpp"


void Scene::buildBVH() {
	printf(" - Generating BVH...\n\n");
	this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray& ray) const
{
	return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection& pos, float& pdf) const
{
	float emit_area_sum = 0;
	for (uint32_t k = 0; k < objects.size(); ++k) {
		if (objects[k]->hasEmit()) {
			emit_area_sum += objects[k]->getArea();
		}
	}
	float p = get_random_float() * emit_area_sum;
	emit_area_sum = 0;
	for (uint32_t k = 0; k < objects.size(); ++k) {
		if (objects[k]->hasEmit()) {
			emit_area_sum += objects[k]->getArea();
			if (p <= emit_area_sum) {
				objects[k]->Sample(pos, pdf);
				break;
			}
		}
	}
}

bool Scene::trace(
	const Ray& ray,
	const std::vector<Object*>& objects,
	float& tNear, uint32_t& index, Object** hitObject)
{
	*hitObject = nullptr;
	for (uint32_t k = 0; k < objects.size(); ++k) {
		float tNearK = kInfinity;
		uint32_t indexK;
		Vector2f uvK;
		if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
			*hitObject = objects[k];
			tNear = tNearK;
			index = indexK;
		}
	}


	return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray& ray, int depth) const
{
	// TO DO Implement Path Tracing Algorithm here
	Vector3f L_dir = { 0,0,0 }, L_indir = { 0,0,0 };
	// 引一条光线求与场景中的交点
	Intersection intersection = Scene::intersect(ray);
	if (!intersection.happened) {
		// 没有和场景相交
		return {};
	}
	if (intersection.m->hasEmission()) {
		// 交点就是光源，那么直接返回光源辐射强度
		return intersection.m->getEmission();
	}
	// 交点是物体： 1）首先是向光源采样直接光照
	Intersection lightpos;
	float lightpdf = 0.0f;
	// 采样光源，包括光源的位置和采样的pdf
	sampleLight(lightpos, lightpdf);
	// 通过与物体交点与光源点坐标相减计算出光线向量的坐标表示
	Vector3f collisionlight = lightpos.coords - intersection.coords;
	// 计算距离
	float dis = dotProduct(collisionlight, collisionlight);
	// 表示光线方向的单位向量
	Vector3f collisionlightdir = collisionlight.normalized();
	// 初始化射到物体的光线
	Ray light_to_object_ray(intersection.coords, collisionlightdir);
	// 计算这条光线与场景的交点，如果没有遮挡的话那么长度应该和collisionLight差距不大即没有发生中途遮挡
	Intersection light_to_anything_ray = Scene::intersect(light_to_object_ray);
	// 计算f_r值
	auto f_r = intersection.m->eval(ray.direction, collisionlightdir, intersection.normal);
	if (light_to_anything_ray.distance - collisionlight.norm() > -0.005 && lightpdf > EPSILON) {
		// 没有发生遮挡，那么就计算直接光照反射的辐射强度
		L_dir = lightpos.emit * f_r * dotProduct(collisionlightdir, intersection.normal) * dotProduct(collisionlightdir, intersection.normal) * dotProduct(-collisionlightdir, lightpos.normal) / dis / lightpdf;
	}

	// 交点是物体 ：2）来自于其他物体的光线，采用递归模拟，轮盘赌停止
	if (get_random_float() > RussianRoulette) {
		return L_dir;
	}
	// 采样一个出射方向
	Vector3f w0 = intersection.m->sample(ray.direction, intersection.normal).normalized();
	// 定义一个其他物体射到本物体的光线向量
	Ray object_to_object_ray(intersection.coords, w0);
	// 判断这个其他物体是否为光源
	Intersection isLight = Scene::intersect(object_to_object_ray);
	if (isLight.happened && !isLight.m->hasEmission()) {
		// 确实与某个物体相交了同时这个物体不是光源
		float pdf = intersection.m->pdf(ray.direction, w0, intersection.normal);
		if (pdf > EPSILON) {
			f_r = intersection.m->eval(ray.direction, w0, intersection.normal);
			// 递归调用以其他物体为基准再次调用castRay获取间接辐射强度
			L_indir = castRay(object_to_object_ray, depth + 1) * f_r * dotProduct(w0, intersection.normal) / pdf / RussianRoulette;
		}
	}
	// 求和即得该点的总反射光的强度
	return L_dir + L_indir;
}