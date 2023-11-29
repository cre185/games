#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives, splitMethod);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects, SplitMethod splitMethod)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]}, splitMethod);
        node->right = recursiveBuild(std::vector{objects[1]}, splitMethod);

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        if(splitMethod==SplitMethod::NAIVE){
            auto beginning = objects.begin();
            auto middling = objects.begin() + (objects.size() / 2);
            auto ending = objects.end();

            auto leftshapes = std::vector<Object*>(beginning, middling);
            auto rightshapes = std::vector<Object*>(middling, ending);

            assert(objects.size() == (leftshapes.size() + rightshapes.size()));

            node->left = recursiveBuild(leftshapes, splitMethod);
            node->right = recursiveBuild(rightshapes, splitMethod);

            node->bounds = Union(node->left->bounds, node->right->bounds);
        }
        else{
            float min_cost=std::numeric_limits<float>::infinity();
            int min_cut=1;
            auto beginning = objects.begin();
            auto ending = objects.end();
            int num_slice=10;

            for(int i=1;i<num_slice;++i){
                auto current=objects.begin()+(objects.size()*i/num_slice);
                auto leftshapes = std::vector<Object*>(beginning, current);
                auto rightshapes = std::vector<Object*>(current, ending);
                auto leftBound = Bounds3(), rightBound=Bounds3();
                for(int j=0;j<leftshapes.size();++j){
                    leftBound=Union(leftBound, leftshapes[j]->getBounds().Centroid());
                }
                for(int j=0;j<rightshapes.size();++j){
                    rightBound=Union(rightBound, rightshapes[j]->getBounds().Centroid());
                }
                float cost = leftshapes.size()*leftBound.SurfaceArea()+rightshapes.size()*rightBound.SurfaceArea();
                if(min_cost>cost){
                    min_cost=cost;
                    min_cut=i;
                }
            }
            auto finalcut=objects.begin()+(objects.size()*min_cut/num_slice);
            auto leftshapes = std::vector<Object*>(beginning, finalcut);
            auto rightshapes = std::vector<Object*>(finalcut, ending);
            assert(objects.size() == (leftshapes.size() + rightshapes.size()));

            node->left = recursiveBuild(leftshapes, splitMethod);
            node->right = recursiveBuild(rightshapes, splitMethod);

            node->bounds = Union(node->left->bounds, node->right->bounds);
        }
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Vector3f invDir=Vector3f(1./ray.direction.x,1./ray.direction.y,1./ray.direction.z);
    std::array<int, 3>isNeg{ray.direction.x>0, ray.direction.y>0, ray.direction.z>0};
    if(!node->bounds.IntersectP(ray, invDir, isNeg)) return Intersection();
    if(node->left==nullptr && node->right==nullptr)
        return node->object->getIntersection(ray);
    Intersection leftint=Intersection(),rightint=Intersection();
    if(node->left) leftint=getIntersection(node->left, ray);
    if(node->right) rightint=getIntersection(node->right, ray);
    return leftint.distance<rightint.distance?leftint:rightint;
}