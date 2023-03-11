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

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
    //printf("递归%i 次",count);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();
    if (splitMethod == SplitMethod::SAH) 
    {
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
            node->left = recursiveBuild(std::vector{ objects[0] });
            node->right = recursiveBuild(std::vector{ objects[1] });

            node->bounds = Union(node->left->bounds, node->right->bounds);
            return node;
        }
        else {
            Bounds3 centroidBounds;
            for (int i = 0; i < objects.size(); ++i)
                centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
            float Sn = centroidBounds.SurfaceArea();
            int B = 10; //B<=32
            int minCostIndex = 0, minCostCoor = 0;
            float minCost = std::numeric_limits<float>::infinity();
            for (int i = 0; i < 3; i++) {
                switch (i) {
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
                for (int j = 0; j < B; j++) {
                    auto beginning = objects.begin();
                    auto middling = objects.begin() + (objects.size() *j/ B);
                    auto ending = objects.end();
                    auto leftshapes = std::vector<Object*>(beginning, middling);
                    auto rightshapes = std::vector<Object*>(middling, ending);
                    Bounds3 leftBound, rightBound;
                    for(int k = 0; k < leftshapes.size(); k++){
                        leftBound = Union(leftBound,leftshapes[k]->getBounds().Centroid());
                    }
                    for (int k = 0; k < rightshapes.size(); k++) {
                        rightBound = Union(rightBound, rightshapes[k]->getBounds().Centroid());
                    }
                    float SA = leftBound.SurfaceArea();
                    float SB = rightBound.SurfaceArea();
                    float cost = (SA * leftshapes.size() + SB * rightshapes.size()) / Sn;
                    if (cost < minCost) {
                        minCost = cost;
                        minCostCoor = i;
                        minCostIndex = j;
                    }
                }
            }


            switch (minCostCoor) {
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
            auto beginning = objects.begin();
            auto middling = objects.begin() + (objects.size() * minCostIndex / B);
            auto ending = objects.end();
            auto leftshapes = std::vector<Object*>(beginning, middling);
            auto rightshapes = std::vector<Object*>(middling, ending);
            assert(objects.size() == (leftshapes.size() + rightshapes.size()));
            node->left = recursiveBuild(leftshapes);
            node->right = recursiveBuild(rightshapes);
            node->bounds = Union(node->left->bounds, node->right->bounds);
        }
    }
    else {
        //count++;
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
            node->left = recursiveBuild(std::vector{ objects[0] });
            node->right = recursiveBuild(std::vector{ objects[1] });

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

            auto beginning = objects.begin();
            auto middling = objects.begin() + (objects.size() / 2);
            auto ending = objects.end();

            auto leftshapes = std::vector<Object*>(beginning, middling);
            auto rightshapes = std::vector<Object*>(middling, ending);

            assert(objects.size() == (leftshapes.size() + rightshapes.size()));

            node->left = recursiveBuild(leftshapes);
            node->right = recursiveBuild(rightshapes);

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
    Intersection inter;
    auto Obj = node->object; 
    Vector3f invdir(1 / ray.direction.x, 1 / ray.direction.y, 1 / ray.direction.z);
    std::array<int, 3>dirIsNeg = { ray.direction.x > 0 ? 0:1,ray.direction.y > 0 ? 0 : 1,ray.direction.z > 0 ? 0 : 1 };
    //没有交点
    if (!node->bounds.IntersectP(ray, invdir, dirIsNeg)) {
       return inter;
    }
    //有交点 且为叶子节点
    if (node->left == nullptr && node->right == nullptr)
    {
        return Obj->getIntersection(ray);
    }
    Intersection inter_left = getIntersection(node->left,ray);
    Intersection inter_right = getIntersection(node->right, ray);
    return inter_left.distance<inter_right.distance?inter_left:inter_right;
}