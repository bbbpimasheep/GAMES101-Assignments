//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"

enum MaterialType { DIFFUSE, MICROFACET };

class Material{
private:

    // Compute reflection direction
    Vector3f reflect(const Vector3f &I, const Vector3f &N) const
    {
        return I - 2 * dotProduct(I, N) * N;
    }

    // Compute refraction direction using Snell's law
    //
    // We need to handle with care the two possible situations:
    //
    //    - When the ray is inside the object
    //
    //    - When the ray is outside.
    //
    // If the ray is outside, you need to make cosi positive cosi = -N.I
    //
    // If the ray is inside, you need to invert the refractive indices and negate the normal N
    Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        Vector3f n = N;
        if (cosi < 0) { cosi = -cosi; } else { std::swap(etai, etat); n= -N; }
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
    }

    // Compute Fresnel equation
    //
    // \param I is the incident view direction
    //
    // \param N is the normal at the intersection point
    //
    // \param ior is the material refractive index
    //
    // \param[out] kr is the amount of light reflected
    void fresnel(const Vector3f &I, const Vector3f &N, const float &ior, float &kr) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        if (cosi > 0) {  std::swap(etai, etat); }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1) {
            kr = 1;
        }
        else {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
        // As a consequence of the conservation of energy, transmittance is given by:
        // kt = 1 - kr;
    }

    inline Vector3f fresnel_schlick(float cosTheta, const Vector3f &F0)
    {
        return F0 + (Vector3f(1.0) - F0) * std::pow(1.0 - cosTheta, 5.0);
    }

    // MicroFacet Material Model
    //
    // Compute the normal distribution function (NDF)
    //
    // \param N is the normal at the intersection point
    //
    // \param H is the half-vector
    //
    // \param alpha is the roughness parameter
    float NDFunc(const Vector3f &N, const Vector3f &H, const float &alpha)
    {
        float alpha_sq = alpha * alpha, 
              dot_nh = dotProduct(N, H), 
              dot_sq = dot_nh * dot_nh;
        float denom_sqrt = dot_sq * (alpha_sq - 1) + 1;
        return alpha_sq / (M_PI * denom_sqrt * denom_sqrt);
    }

    // Geometry term of the microfacet model
    //
    // \param I is the incident view direction
    //
    // \param O is the outgoing view direction
    float GGXFunc(const Vector3f &N, const Vector3f &V, const float &alpha)
    {
        float dot_nv = std::max(EPSILON, dotProduct(N, V)),
              k = (alpha+1) * (alpha+1) / 8.0f;
        return dot_nv / (dot_nv * (1 - k) + k);
    }

    float GeoFunc(const Vector3f &N, const Vector3f &I, const Vector3f &O, const float &alpha)
    {
        return GGXFunc(N, I, alpha) * GGXFunc(N, O, alpha);
    }

    Vector3f toWorld(const Vector3f &a, const Vector3f &N)
    {
        Vector3f B, C;
        if (std::fabs(N.x) > std::fabs(N.y)){
            float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
            C = Vector3f(N.z * invLen, 0.0f, -N.x *invLen);
        }
        else {
            float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
            C = Vector3f(0.0f, N.z * invLen, -N.y *invLen);
        }
        B = crossProduct(C, N);
        return a.x * B + a.y * C + a.z * N;
    }

public:
    MaterialType m_type;
    //Vector3f m_color;
    Vector3f m_emission;
    float ior;
    float roughness;
    float metalness;
    Vector3f Kd, Ks;
    float specularExponent;
    //Texture tex;

    inline Material(MaterialType t=DIFFUSE, Vector3f e=Vector3f(0,0,0));
    inline MaterialType getType();
    //inline Vector3f getColor();
    inline Vector3f getColorAt(double u, double v);
    inline Vector3f getEmission();
    inline bool hasEmission();

    // sample a ray by Material properties
    inline Vector3f sample(const Vector3f &wi, const Vector3f &N);
    // given a ray, calculate the PdF of this ray
    inline float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
    // given a ray, calculate the contribution of this ray
    inline Vector3f eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);

};

Material::Material(MaterialType t, Vector3f e){
    m_type = t;
    //m_color = c;
    m_emission = e;
}

MaterialType Material::getType(){return m_type;}
///Vector3f Material::getColor(){return m_color;}
Vector3f Material::getEmission() {return m_emission;}
bool Material::hasEmission() {
    if (m_emission.norm() > EPSILON) return true;
    else return false;
}

Vector3f Material::getColorAt(double u, double v) {
    return Vector3f();
}


Vector3f Material::sample(const Vector3f &wi, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // uniform sample on the hemisphere
            float x_1 = get_random_float(), x_2 = get_random_float();
            float z = std::fabs(1.0f - 2.0f * x_1);
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
            Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z);
            return toWorld(localRay, N);
            
            break;
        }
        case MICROFACET:
        {
            // sample the GGX distribution
            float alpha_sq = roughness * roughness;

            float x_1 = get_random_float(), x_2 = get_random_float();
            float phi = 2 * M_PI * x_1;
            float cosTheta = std::sqrt((1 - x_2) / (1 + (alpha_sq - 1) * x_2)),
                  sinTheta = std::sqrt(1 - cosTheta * cosTheta);
            
            Vector3f H;
            H.x = sinTheta * std::cos(phi);
            H.y = sinTheta * std::sin(phi);
            H.z = cosTheta;
            Vector3f H_w = toWorld(H, N);
            return reflect(wi, H_w);

            break;
        }
    }
}

float Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // uniform sample probability 1 / (2 * PI)
            if (dotProduct(wo, N) > 0.0f)
                return 0.5f / M_PI;
            else
                return 0.0f;
            break;
        }
        
        case MICROFACET:
        {
            // calculate the pdf of GGX distribution
            Vector3f H = (wo - wi).normalized();
            float normalDis = NDFunc(N, H, roughness);
            float cosTheta = dotProduct(H, N);
            float pdf_halfv = normalDis * cosTheta;
            return pdf_halfv / (4 * dotProduct(wo, H));
            break; 
        }
    }
}

Vector3f Material::eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // calculate the contribution of diffuse model
            float cosalpha = dotProduct(N, wo);
            if (cosalpha > 0.0f) {
                Vector3f diffuse = Kd / M_PI;
                return diffuse;
            }
            else
                return Vector3f(0.0f);
            break;
        }
        case MICROFACET:
        {
            if (dotProduct(wo, N) <= 0.0f) 
                return Vector3f(0.0f);
            else {
                Vector3f H = (wo - wi).normalized();
                // calculate the contribution of specular
                float denominator = std::max(dotProduct(-wi, N) * dotProduct(wo, N) * 4, EPSILON);
                float G = GeoFunc(N, -wi, wo, roughness),
                      D = NDFunc(N, H, roughness);
                Vector3f F0 = Vector3f(0.04f); F0 = lerp(F0, Kd, metalness);
                Vector3f F = fresnel_schlick(dotProduct(-wi, H), F0);

                Vector3f diffuse = (Vector3f(1.0f) - F) * Kd / M_PI;
                Vector3f specular = F * G * D / denominator;
                return specular + diffuse;
            }
            break;
        }
    }
}

#endif //RAYTRACING_MATERIAL_H