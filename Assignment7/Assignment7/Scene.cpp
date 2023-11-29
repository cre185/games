//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
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
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    const float EPSILON = 0.0001;

    Intersection p_inter = intersect(ray);
    if(!p_inter.happened) return Vector3f();
    if(p_inter.m->hasEmission()) return p_inter.m->getEmission();

    Intersection x_inter;
    float pdf_light = 0.0f;
    sampleLight(x_inter, pdf_light);

    Vector3f p = p_inter.coords;
    Vector3f x = x_inter.coords;
    float dist = (x-p).norm();
    Vector3f N = p_inter.normal;
    Vector3f NN = x_inter.normal;
    Vector3f wo = ray.direction;
    Vector3f ws = (x-p).normalized();
    Vector3f emit = x_inter.emit;

    Ray ws_ray(p, ws);
    Intersection ws_inter = intersect(ws_ray);
    Vector3f l_dir(0.0), l_indir(0.0);
    
    if(ws_inter.distance-dist > -EPSILON){
        //std::cout<<emit<<' '<<p_inter.m->eval(wo,ws,N)<<' '<<dotProduct(ws,N)<<' '<<dotProduct(ws,NN)<<std::endl;
        l_dir = emit*p_inter.m->eval(wo,ws,N)*dotProduct(-ws,N)*dotProduct(ws,NN)/(dist*dist)/pdf_light;
    }

    if(get_random_float() < RussianRoulette){
        Vector3f wi = p_inter.m->sample(wo, N).normalized();
        Ray wi_ray(p, wi);
        Intersection wi_inter = intersect(wi_ray);
        float pdf_value=p_inter.m->pdf(wo,wi,N);
        if(wi_inter.happened && (!wi_inter.m->hasEmission()) && pdf_value >= 0.000001){
            l_indir = castRay(wi_ray,depth+1)*p_inter.m->eval(wo,wi,N)*dotProduct(wi,N)/pdf_value/RussianRoulette;
        }
    }
    return l_dir+l_indir;
}