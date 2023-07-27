#include "rtutils.h"

#include "color.h"
#include "hittable_list.h"
#include "sphere.h"
#include "camera.h"
#include "material.h"

#include <fstream>
#include <vector>
#include <iostream>
#include <future>

double hit_sphere(const point3& center, double radius, const ray& r) {
    vec3 oc = r.origin() - center;
    auto a = r.direction().length_squared();
    auto half_b = dot(oc, r.direction());
    auto c = oc.length_squared() - radius*radius;
    auto discriminant = half_b*half_b - a*c;
    if (discriminant < 0){
        return -1.0;
    } else {
        return (-half_b - sqrt(discriminant)) / a;
    }
}

color ray_color(const ray& r, const hittable& world, int depth) {
    hit_record rec;

    if (depth <= 0)
        return color(0, 0, 0);

    if (world.hit(r, 0.001, infinity, rec)) {
        ray scattered;
        color attenuation;
        if (rec.mat_ptr->scatter(r, rec, attenuation, scattered))
            return attenuation * ray_color(scattered, world, depth-1);
        return color(0, 0, 0);
    }

    vec3 unit_direction = unit_vector(r.direction());
    auto t = 0.5 * (unit_direction.y() + 1.0);
    return (1.0-t)*color(1.0,1.0,1.0) + t*color(0.5, 0.7, 1.0);
}

struct RayResult {
    unsigned int index;
    vec3 color;
};

int main() {
    // image
    const auto aspect_ratio = 16.0 / 9.0;
    const int image_width = 400;
    const int image_height = static_cast<int>(image_width / aspect_ratio);
    const int samples_per_pixel = 100;
    const int max_depth = 50;
    const int pixelCount = image_width * image_height;

    // world 
    auto R = cos(pi/4);
    hittable_list world;
    auto material_ground = make_shared<lambertian>(color(0.8, 0.8, 0.0));
    auto material_center = make_shared<lambertian>(color(0.1, 0.2, 0.5));
    auto material_left   = make_shared<dielectric>(1.5);
    auto material_right  = make_shared<metal>(color(0.8,0.6,0.2), 0.0);

    world.add(make_shared<sphere>(point3( 0.0, -100.5, -1.0), 100.0, material_ground));
    world.add(make_shared<sphere>(point3( 0.0,    0.0, -1.0),   0.5, material_center));
    world.add(make_shared<sphere>(point3(-1.0,    0.0, -1.0),   0.5, material_left));
    world.add(make_shared<sphere>(point3(-1.0,    0.0, -1.0), -0.45, material_left));
    world.add(make_shared<sphere>(point3( 1.0,    0.0, -1.0),   0.5, material_right));
    
    //camer
    point3 lookfrom(3,3,2);
    point3 lookat(0,0,-1);
    vec3 vup(0,1,0);
    auto dist_to_focus = (lookfrom-lookat).length();
    auto aperture = 2.0;

    camera cam(lookfrom, lookat, vup, 20, aspect_ratio, aperture, dist_to_focus);

    std::ofstream outputFile("output_image.ppm", std::ofstream::out);

    if (!outputFile) {
        std::cerr << "Error opening the output file.";
        return 1;
    }

    vec3* image = new vec3[image_width * image_height];
	memset(&image[0], 0, image_width * image_height * sizeof(vec3));

    outputFile << "P3\n" << image_width << ' ' << image_height
 << "\n255\n";

    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    
    std::mutex mutex;
    std::condition_variable cvResults;
	std::vector<std::future<RayResult>> m_futures;

    for (int j = 0; j < image_height; ++j) {
        for (int i = 0; i < image_width; ++i) {
            auto future = std::async(std::launch::async | std::launch::deferred,
                [&cam, &world, &samples_per_pixel, i, j, image_width, image_height, &cvResults]() -> RayResult {
                const unsigned int index = j * image_width + i;
                vec3 color(0, 0, 0);
                for (int s = 0; s < samples_per_pixel; ++s) {
                    auto u = (i + random_double()) / (image_width-1);
                    auto v = (j + random_double()) / (image_height-1);
                    ray r = cam.get_ray(u, v);
                    color += ray_color(r, world, max_depth);
                }
                color /= float(samples_per_pixel);
                RayResult result;
                result.index = index;
                result.color = vec3(sqrt(color[0]), sqrt(color[1]), sqrt(color[2]));
                return result;
                // write_color(outputFile, pixel_color, samples_per_pixel);
            });
            {
                std::lock_guard<std::mutex> lock(mutex);
                m_futures.push_back(std::move(future));
            }
        }
    }
{
    std::unique_lock<std::mutex> lock(mutex);
    cvResults.wait(lock, [&m_futures, &pixelCount] {
        return m_futures.size() == pixelCount;
    });
}

for (std::future<RayResult>& rr : m_futures)
{
    RayResult result = rr.get();
    image[result.index] = result.color;
}

for (unsigned int i = 0; i < pixelCount; ++i)
{
    // BGR to RGB
    // 2 = r;
    // 1 = g;
    // 0 = b;
    outputFile
        << static_cast<int>(255.99f * image[i].e[2]) << " "
        << static_cast<int>(255.99f * image[i].e[1]) << " "
        << static_cast<int>(255.99f * image[i].e[0]) << "\n";
}


    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();   
    std::cerr << "\nDone in " << std::chrono::duration_cast<std::chrono::seconds> (end - start).count() 
        << " seconds";

    outputFile.close();
}