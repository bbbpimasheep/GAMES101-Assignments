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
    /*
    shade(p, wo)
        sampleLight(inter , pdf_light)
        Get x, ws, NN, emit from inter
        Shoot a ray from p to x
        If the ray is not blocked in the middle
            L_dir = emit * eval(wo, ws, N) * dot(ws, N) * dot(ws,
            NN) / |x-p|^2 / pdf_light
  
  
        L_indir = 0.0
        //Test Russian Roulette with probability RussianRoulette
        wi = sample(wo, N)
        Trace a ray r(p, wi)
        If ray r hit a non-emitting object at q
            L_indir = shade(q, wi) * eval(wo, wi, N) * dot(wi, N)
            / pdf(wo, wi, N) / RussianRoulette
  
        Return L_dir + L_indir
    */

    Vector3f L_emit(0.0f), L_dir(0.0f), L_indir(0.0f);

    Intersection mn_pos = this->intersect(ray);
    if (!mn_pos.happened) return L_dir;
    if (mn_pos.m->hasEmission()) L_emit = mn_pos.m->getEmission();

    Intersection ls_pos; float ls_pdf;
    sampleLight(ls_pos, ls_pdf);

    Vector3f ls_p = ls_pos.coords, ls_n = ls_pos.normal.normalized(), 
             mn_p = mn_pos.coords, mn_n = mn_pos.normal.normalized(),
             ws_dir = (ls_p - mn_p).normalized(), wo_dir = ray.direction; 
    Ray w_s(mn_p + mn_n * EPSILON, ws_dir);

    Intersection ws_isect = this->intersect(w_s);
    float ws_dist = (ws_isect.coords - mn_p).norm(), 
          ls_dist = (ls_p - mn_p).norm();
    if (mn_pos.m->getType() == MaterialType::DIFFUSE && 
        ws_isect.happened && ls_dist - ws_dist <= EPSILON) {
        Vector3f BRDF = mn_pos.m->eval(wo_dir, ws_dir, mn_n),
                 radiance = ls_pos.emit;
        // if (mn_pos.m->getType() == MaterialType::MICROFACET)
            // printf("BRDF: %f %f %f\n", BRDF.x, BRDF.y, BRDF.z);
            // printf("Radiance: %f %f %f\n", radiance.x, radiance.y, radiance.z);
        L_dir = radiance * BRDF * dotProduct(ws_dir, mn_n) * dotProduct(-ws_dir, ls_n) 
                / (ls_dist * ls_dist * ls_pdf);
    }

    if (get_random_float() <= this->RussianRoulette) {
        Vector3f wi_dir = mn_pos.m->sample(wo_dir, mn_n).normalized();
        if (dotProduct(wi_dir, mn_n) >= 0) {
            Ray w_i(mn_p, wi_dir);
            Intersection wi_isect = this->intersect(w_i);
            if (wi_isect.happened && 
                ((mn_pos.m->getType() == MaterialType::DIFFUSE && !wi_isect.m->hasEmission()) ||
                 (mn_pos.m->getType() == MaterialType::MICROFACET))) {
                Vector3f BRDF = mn_pos.m->eval(wo_dir, wi_dir, mn_n),
                         radiance = this->castRay(w_i, depth+1);
                float wi_pdf = mn_pos.m->pdf(wo_dir, wi_dir, mn_n);
                L_indir = wi_pdf >= EPSILON
                        ? radiance * BRDF * dotProduct(wi_dir, mn_n) / (wi_pdf * this->RussianRoulette)
                        : Vector3f(0.0f);
            }
        }
    }

    return L_emit + Vector3f::Min(Vector3f::Max(L_dir + L_indir, Vector3f(0)), Vector3f(5));
;
;
}