#ifndef ACCELERATOR_H
#define ACCELERATOR_H

#include "scene.h"
#include <cmath>

using namespace std;

class Grid {
  public:
	Grid(void);
	//~Grid(void);
	int getNumObjects() const;
	void addObject(Object *o);
	void setAABB(AABB &bbox_);
	Object *getObject(unsigned int index) const;
	void Build(vector<Object *> &objs); // set up grid cells
	bool Traverse(Ray &ray, const Object **hitobject, HitRecord &hitRec) const;
	bool Traverse(Ray &ray) const; // Traverse for shadow ray

  private:
	vector<Object *> objects;
	vector<vector<Object *>> cells;

	int nx, ny, nz; // number of cells in the x, y, and z directions
	float m = 2.0f; // factor that allows to vary the number of cells

	// Setup function for Grid traversal
	bool Init_Traverse(
		Ray &ray,
		int &ix,
		int &iy,
		int &iz,
		double &dtx,
		double &dty,
		double &dtz,
		double &tx_next,
		double &ty_next,
		double &tz_next,
		int &ix_step,
		int &iy_step,
		int &iz_step,
		int &ix_stop,
		int &iy_stop,
		int &iz_stop
	) const;

	AABB bbox;
};

/*********************************BVH*****************************************************************/
class BVH {
	struct BVHObj {
		const Object *obj;
		AABB bb;
	};

	class Comparator {
	  public:
		int dimension;

		bool operator()(const BVHObj &a, const BVHObj &b) {
			float ca = a.bb.centroid().getAxisValue(dimension);
			float cb = b.bb.centroid().getAxisValue(dimension);
			return ca < cb;
		}
	};

	class BVHNode {
	  private:
		AABB bbox;
		bool leaf;
		unsigned int n_objs;
		unsigned int index; // if leaf == false: index to left child node,
		                    // else if leaf == true: index to first Intersectable (Object *) in objects vector

	  public:
		BVHNode(void);
		void setAABB(AABB &bbox_);
		void makeLeaf(unsigned int index_, unsigned int n_objs_);
		void makeNode(unsigned int left_index_);
		bool isLeaf() const {
			return leaf;
		}
		unsigned int getIndex() const {
			return index;
		}
		unsigned int getNObjs() const {
			return n_objs;
		}
		const AABB &getAABB() const {
			return bbox;
		};
	};

  private:
	unsigned int Threshold = 2;
	vector<BVHObj> objects;

	vector<BVH::BVHNode> nodes;

  public:
	BVH(void);
	int getNumObjects();

	void Build(vector<Object *> &objects);
	void build_recursive(unsigned int left_index, unsigned int right_index, BVHNode &node);
	bool Traverse(Ray &ray, const Object **hit_obj, HitRecord &hitRec) const;
	bool Traverse(Ray &ray) const;
};
#endif
