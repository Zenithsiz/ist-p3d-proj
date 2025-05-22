#include "macros.h"
#include "rayAccelerator.h"
#include "scene.h"

#include <algorithm>
#include <cassert>
#include <cfloat>
#include <cmath>

using namespace std;

BVH::BVHNode::BVHNode(void) {}

void BVH::BVHNode::setAABB(AABB &bbox_) {
	this->bbox = bbox_;
}

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

int BVH::getNumObjects() {
	return objects.size();
}

void BVH::Build(vector<Object *> &objs) {
	Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	AABB world_bbox = AABB(min, max);

	// Reserve space for all nodes to ensure we don't invalidate any pointers
	// during construction.
	this->nodes.reserve(2 * objs.size() - 1);

	// Calculate the world hit box
	for (Object *obj: objs) {
		AABB bbox = obj->GetBoundingBox();
		world_bbox.extend(bbox);
		objects.push_back(obj);
	}
	world_bbox.min.x -= EPSILON;
	world_bbox.min.y -= EPSILON;
	world_bbox.min.z -= EPSILON;
	world_bbox.max.x += EPSILON;
	world_bbox.max.y += EPSILON;
	world_bbox.max.z += EPSILON;

	// Then build the root node recursively with all of the objects.
	auto &root = nodes.emplace_back();
	root.setAABB(world_bbox);

	this->build_recursive(0, objects.size(), root);
}

void BVH::build_recursive(unsigned int left_index, unsigned int right_index, BVHNode &node) {
	if (right_index - left_index <= this->Threshold) {
		node.makeLeaf(left_index, right_index - left_index);
		return;
	}

	auto aabb = node.getAABB();
	auto size = aabb.max - aabb.min;

	// Find the dimension to sort by and sort all objects in our range
	auto sort_dimension = (size.x > size.y && size.x > size.z) ? 0 : (size.y > size.x && size.y > size.z) ? 1 : 2;
	std::sort(&this->objects[left_index], &this->objects[right_index], Comparator{sort_dimension});

	// Then find the split index
	// TODO: The top approach is way too slow, should we speed it up or just use the middlepoint?
	/*
	auto aabb_centroid = aabb.centroid();
	auto split_obj = std::partition_point(
	    &this->objects[left_index],
	    &this->objects[right_index],
	    [aabb_centroid, sort_dimension](const auto &obj) {
	        return obj->getCentroid().getAxisValue(sort_dimension) < aabb_centroid.getAxisValue(sort_dimension) / 2;
	    }
	);
	unsigned int split_idx = std::distance(&this->objects[left_index], split_obj);
	split_idx = std::clamp(split_idx, left_index + this->Threshold, right_index - this->Threshold);
	*/
	unsigned int split_idx = (left_index + right_index) / 2;

	// Make this node a non-leaf
	node.makeNode(nodes.size());

	// And create the sub-nodes
	assert(this->nodes.size() + 2 <= this->nodes.capacity());
	auto &lhs = this->nodes.emplace_back();
	auto &rhs = this->nodes.emplace_back();

	// Build left
	Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	AABB lhs_bbox = AABB(min, max);
	for (unsigned int i = left_index; i < split_idx; i++) {
		AABB bbox = this->objects[i]->GetBoundingBox();
		lhs_bbox.extend(bbox);
	}
	lhs.setAABB(lhs_bbox);
	this->build_recursive(left_index, split_idx, lhs);

	// Then build right
	AABB rhs_bbox = AABB(min, max);
	for (unsigned int i = split_idx; i < right_index; i++) {
		AABB bbox = this->objects[i]->GetBoundingBox();
		rhs_bbox.extend(bbox);
	}
	rhs.setAABB(rhs_bbox);
	this->build_recursive(split_idx, right_index, rhs);

	// right_index, left_index and split_index refer to the indices in the objects vector
	// do not confuse with left_nodde_index and right_node_index which refer to indices in the nodes vector.
	// node.index can have a index of objects vector or a index of nodes vector
}

bool BVH::Traverse(Ray &ray, const Object **hit_obj, HitRecord &hitRec) const {
	thread_local std::vector<const BVH::BVHNode *> hit_stack;
	hit_stack.clear();

	const auto *currentNode = &nodes[0];
	HitRecord closest_hit;

	if (!currentNode->getAABB().hit(ray, hitRec.t)) {
		return false;
	}

	while (true) {
		if (currentNode->isLeaf()) {
			auto start_idx = currentNode->getIndex();
			auto end_idx = start_idx + currentNode->getNObjs();
			for (unsigned int i = start_idx; i < end_idx; i++) {
				const auto &obj = this->objects[i];
				auto curHitRec = obj->hit(ray);
				if (curHitRec.isHit && curHitRec.t < closest_hit.t) {
					closest_hit = curHitRec;
					*hit_obj = obj;
				}
			}
		} else {
			auto lhs_node_idx = currentNode->getIndex();
			const auto *lhs_node = &this->nodes[lhs_node_idx];
			const auto *rhs_node = &this->nodes[lhs_node_idx + 1];

			float lhs_t;
			auto lhs_hit = lhs_node->getAABB().hit(ray, lhs_t);
			if (lhs_node->getAABB().isInside(ray.origin)) {
				lhs_t = 0;
			}

			float rhs_t;
			auto rhs_hit = rhs_node->getAABB().hit(ray, rhs_t);
			if (rhs_node->getAABB().isInside(ray.origin)) {
				rhs_t = 0;
			}

			if (lhs_hit && rhs_hit) {
				if (lhs_t < rhs_t) {
					hit_stack.push_back(rhs_node);
					currentNode = lhs_node;
				} else {
					hit_stack.push_back(lhs_node);
					currentNode = rhs_node;
				}
				continue;
			} else if (lhs_hit) {
				currentNode = lhs_node;
				continue;
			} else if (rhs_hit) {
				currentNode = rhs_node;
				continue;
			}
		}

		if (hit_stack.empty()) {
			break;
		}

		currentNode = hit_stack.back();
		hit_stack.pop_back();
	}

	if (!closest_hit.isHit) {
		return false;
	}

	hitRec = closest_hit;
	return true;
}

bool BVH::Traverse(Ray &ray) const { // shadow ray with length
	thread_local std::vector<const BVH::BVHNode *> hit_stack;
	hit_stack.clear();

	double ray_length = ray.direction.length(); // distance between light and intersection point
	ray.direction.normalize();

	const BVHNode *currentNode = &nodes[0];

	if (float t; !currentNode->getAABB().hit(ray, t)) {
		return false;
	}

	while (true) {
		if (currentNode->isLeaf()) {
			auto start_idx = currentNode->getIndex();
			auto end_idx = start_idx + currentNode->getNObjs();
			for (unsigned int i = start_idx; i < end_idx; i++) {
				const auto &obj = this->objects[i];
				auto curHitRec = obj->hit(ray);
				if (curHitRec.isHit && curHitRec.t < ray_length) {
					return true;
				}
			}
		} else {
			auto lhs_node_idx = currentNode->getIndex();
			const auto *lhs_node = &this->nodes[lhs_node_idx];
			const auto *rhs_node = &this->nodes[lhs_node_idx + 1];

			float lhs_t;
			auto lhs_hit = lhs_node->getAABB().hit(ray, lhs_t);

			float rhs_t;
			auto rhs_hit = rhs_node->getAABB().hit(ray, rhs_t);

			if (lhs_hit && rhs_hit) {
				hit_stack.push_back(rhs_node);
				currentNode = lhs_node;
				continue;
			} else if (lhs_hit) {
				currentNode = lhs_node;
				continue;
			} else if (rhs_hit) {
				currentNode = rhs_node;
				continue;
			}
		}

		if (hit_stack.empty()) {
			break;
		}

		currentNode = hit_stack.back();
		hit_stack.pop_back();
	}

	return false; // no primitive intersection
}
