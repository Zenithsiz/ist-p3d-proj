#include "rayAccelerator.h"
#include "macros.h"

using namespace std;

BVH::BVHNode::BVHNode(void) {}

void BVH::BVHNode::setAABB(AABB& bbox_) { this->bbox = bbox_; }

void BVH::BVHNode::makeLeaf(unsigned int index_, unsigned int n_objs_) {
	this->leaf = true;
	this->index = index_;
	this->n_objs = n_objs_;
}

void BVH::BVHNode::makeNode(unsigned int left_index_) {
	this->leaf = false;
	this->index = left_index_;
}


BVH::BVH(void) {}

int BVH::getNumObjects() { return objects.size(); }


void BVH::Build(vector<Object *> &objs) {


			BVHNode *root = new BVHNode();

			Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
			AABB world_bbox = AABB(min, max);

			for (Object* obj : objs) {
				AABB bbox = obj->GetBoundingBox();
				world_bbox.extend(bbox);
				objects.push_back(obj);
			}
			world_bbox.min.x -= EPSILON; world_bbox.min.y -= EPSILON; world_bbox.min.z -= EPSILON;
			world_bbox.max.x += EPSILON; world_bbox.max.y += EPSILON; world_bbox.max.z += EPSILON;
			root->setAABB(world_bbox);
			nodes.push_back(root);
			build_recursive(0, objects.size(), root); // -> root node takes all the objects
		}

void BVH::build_recursive(int left_index, int right_index, BVHNode *node) {

	//PUT YOUR CODE HERE

		//right_index, left_index and split_index refer to the indices in the objects vector
	   // do not confuse with left_nodde_index and right_node_index which refer to indices in the nodes vector.
	    // node.index can have a index of objects vector or a index of nodes vector

	}

bool BVH::Traverse(Ray& ray, const Object** hit_obj, HitRecord& hitRec) const {
			float tmp;
			bool hit = false;
			stack<StackItem> hit_stack;
			HitRecord rec;   //rec.isHit initialized to false and rec.t initialized with FLT_MAX

			BVHNode* currentNode = nodes[0];

			//PUT YOUR CODE HERE

			return hit;

	}

bool BVH::Traverse(Ray& ray) const {  //shadow ray with length
			float tmp;
			stack<StackItem> hit_stack;
			HitRecord rec;

			double length = ray.direction.length(); //distance between light and intersection point
			ray.direction.normalize();


			return false;  //no primitive intersection

	}
