#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <fstream>

#include "maths.h"
#include "scene.h"
#include "macros.h"


Triangle::Triangle(Vector& P0, Vector& P1, Vector& P2)
{
	points[0] = P0; points[1] = P1; points[2] = P2;

	// Calculate the Min and Max for bounding box
	Min = Vector(+FLT_MAX, +FLT_MAX, +FLT_MAX);
	Max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);

	for (auto &point : points) {
		if (point.x < Min.x)
			Min.x = point.x;
		if (point.x > Max.x)
			Max.x = point.x;
		if (point.y < Min.y)
			Min.y = point.y;
		if (point.y > Max.y)
			Max.y = point.y;
		if (point.z < Min.z)
			Min.z = point.z;
		if (point.z > Max.z)
			Max.z = point.z;
	}
	// enlarge the bounding box a bit just in case...
	Min -= EPSILON;
	Max += EPSILON;
}

AABB Triangle::GetBoundingBox() {
	return(AABB(Min, Max));
}


//
// Ray/Triangle intersection test using Tomas Moller-Ben Trumbore algorithm.
//

HitRecord Triangle::hit(Ray& r) const {

	HitRecord rec;
	rec.t = FLT_MAX;  //not necessary
	rec.isHit = false;  //not necessary

	/* Calculate the normal */
	Vector normal = (points[1] - points[0]) % (points[2] - points[1]);  //cross product
	normal.normalize();

	Vector edge1 = points[1] - points[0];
	Vector edge2 = points[2] - points[0];
	Vector ray_cross_e2 = r.direction % edge2;
	float det = edge1 * ray_cross_e2;

	if (det > -EPSILON && det < EPSILON)
		return rec;

	float inv_det = 1.0 / det;
	Vector s = r.origin - points[0];
	float u = inv_det * (s * ray_cross_e2);

	if ((u < 0 && abs(u) > EPSILON) || (u > 1 && abs(u-1) > EPSILON))
		return rec;

	Vector s_cross_e1 = s % edge1;
	float v = inv_det * r.direction * s_cross_e1;

	if ((v < 0 && abs(v) > EPSILON) || (u + v > 1 && abs(u + v - 1) > EPSILON))
		return rec;

	// At this stage we can compute t to find out where the intersection point is on the line.
	rec.t = inv_det * (edge2 * s_cross_e1);
	rec.isHit = rec.t > EPSILON;

	rec.normal = normal;
	if (rec.normal * r.direction > 0) {
		rec.normal = -rec.normal;
	}

	return (rec);
}


Plane::Plane(Vector& a_PN, float a_D)
	: PN(a_PN), D(a_D)
{}

Plane::Plane(Vector& P0, Vector& P1, Vector& P2)
{
   float l;

   PN = (P1 - P0) % (P2 - P0); // counter-clockwise vectorial product.
   if ((l=PN.length()) == 0.0)
   {
     cerr << "DEGENERATED PLANE!\n";
   }
   else
   {
     PN.normalize();
     D  = -(PN * P0);
   }
}

//
// Ray/Plane intersection test.
//


HitRecord Plane::hit( Ray& r) const
{
	HitRecord rec;
	rec.t = FLT_MAX;
	rec.isHit = false;
	float PNxRd;

	PNxRd = PN * r.direction;

	// if ray is parallel to the plane the ray misses.
	if (fabs(PNxRd) < EPSILON)
		return rec;

	float t = -((PN * r.origin) + D)/PNxRd;
	if (t > 0) {
		rec.t = t;
		rec.normal = PN;
		rec.isHit = true;
		return rec;
	}

	return (rec);
}

HitRecord Sphere::hit(Ray& r) const
{
	HitRecord rec;
	rec.t = FLT_MAX;
	rec.isHit = false;

	auto offset = r.origin - this->center;

	auto ray_center_offset = offset * r.direction;
	auto distance_from_surface = offset * offset - this->radius * this->radius;

	if (distance_from_surface > 0.0 && ray_center_offset > 0.0) {
		return rec;
	}

	auto disc = ray_center_offset*ray_center_offset - distance_from_surface;

	if (disc < 0.0) {
		return rec;
	}

	rec.t = -(ray_center_offset + sqrtf(disc));

	auto hit_point = r.origin + r.direction * rec.t;
	rec.normal = (hit_point - this->center).normalize();
	rec.isHit = true;

	return (rec);
}


AABB Sphere::GetBoundingBox() {
	Vector a_min = this->center - Vector(this->radius, this->radius, this->radius);
	Vector a_max = this->center - Vector(this->radius, this->radius, this->radius);

	return(AABB(a_min, a_max));
}

aaBox::aaBox(Vector& minPoint, Vector& maxPoint) //Axis aligned Box: another geometric object
{
	this->min = minPoint;
	this->max = maxPoint;
}

AABB aaBox::GetBoundingBox() {
	return(AABB(min, max));
}

HitRecord aaBox::hit(Ray& ray) const
{
	HitRecord rec;
	rec.t = FLT_MAX;
	rec.isHit = false;

	double ox = ray.origin.x;
	double oy = ray.origin.y;
	double oz = ray.origin.z;

	double dx = ray.direction.x;
	double dy = ray.direction.y;
	double dz = ray.direction.z;

	double tx_min, ty_min, tz_min;
	double tx_max, ty_max, tz_max;

	double a = 1.0 / dx;
	if (a >= 0) {
		tx_min = (min.x - ox) * a;
		tx_max = (max.x - ox) * a;
	} else {
		tx_min = (max.x - ox) * a;
		tx_max = (min.x - ox) * a;
	}

	double b = 1.0 / dy;
	if (b >= 0) {
		ty_min = (min.y - oy) * b;
		ty_max = (max.y - oy) * b;
	} else {
		ty_min = (max.y - oy) * b;
		ty_max = (min.y - oy) * b;
	}

	double c = 1.0 / dz;
	if (c >= 0) {
		tz_min = (min.z - oz) * c;
		tz_max = (max.z - oz) * c;
	} else {
		tz_min = (max.z - oz) * c;
		tz_max = (min.z - oz) * c;
	}

	float tE, tL; //entering and leaving t values
	Vector face_in, face_out; // normals
	// find largest tE, entering t value
	if (tx_min > ty_min) {
	tE = tx_min;
	face_in = (a >= 0.0) ? Vector(-1, 0, 0) : Vector(1, 0, 0);
	}
	else {
	tE = ty_min;
	face_in = (b >= 0.0) ? Vector(0, -1, 0) : Vector(0, 1, 0);
	}
	if (tz_min > tE) {
		tE = tz_min;
		face_in = (c >= 0.0) ? Vector(0, 0, -1) : Vector(0, 0, 1);
	}
	// find smallest tL, leaving t value
	if (tx_max < ty_max) {
		tL = tx_max;
		face_out = (a >= 0.0) ? Vector(1, 0, 0) : Vector(-1, 0, 0);
	}
	else {
		tL = ty_max;
		face_out = (b >= 0.0) ? Vector(0, 1, 0) : Vector(0, -1, 0);
	}
	if (tz_max < tL) {
		tL = tz_max;
		face_out = (c >= 0.0) ? Vector(0, 0, 1) : Vector(0, 0, -1);
	}
	if (tE < tL && tL > 0) { // condition for a hit
		rec.isHit = true;
		if (tE > 0) {
			rec.t = tE; // ray hits outside surface
			rec.normal = face_in;
		}
		else {
			rec.t = tL;// ray hits inside surface
			rec.normal = face_out;
		}

		return (rec);
	}
	else return (rec); //initialized to false

}


Scene::Scene()
{}

Scene::~Scene()
{
	objects.erase(objects.begin()+1, objects.end()-1);
}

int Scene::getNumObjects()
{
	return objects.size();
}


void Scene::addObject(Object* o)
{
	objects.push_back(o);
}


Object* Scene::getObject(unsigned int index)
{
	if (index >= 0 && index < objects.size())
		return objects[index];
	return NULL;
}


int Scene::getNumLights()
{
	return lights.size();
}


void Scene::addLight(Light* l)
{
	lights.push_back(l);
}


Light* Scene::getLight(unsigned int index)
{
	if (index >= 0 && index < lights.size())
		return lights[index];
	return NULL;
}

void Scene::LoadSkybox(const char *sky_dir)
{
	char *filenames[6];
	char buffer[100];
	const char *maps[] = { "/right.jpg", "/left.jpg", "/top.jpg", "/bottom.jpg", "/front.jpg", "/back.jpg" };

	for (int i = 0; i < 6; i++) {
		strncpy(buffer, sky_dir, sizeof(buffer));
		strncat(buffer, maps[i], sizeof(buffer)-strlen(buffer)-1);
		filenames[i] = (char *)malloc(sizeof(buffer));
		strncpy(filenames[i], buffer, sizeof(buffer));
	}


	ILuint ImageName;

	ilEnable(IL_ORIGIN_SET);
	ilOriginFunc(IL_ORIGIN_LOWER_LEFT);

	for (int i = 0; i < 6; i++) {
		ilGenImages(1, &ImageName);
		ilBindImage(ImageName);

		if (ilLoadImage(filenames[i]))  //Image loaded with lower left origin
			printf("Skybox face %d: Image sucessfully loaded.\n", i);
		else
			exit(0);

		ILint bpp = ilGetInteger(IL_IMAGE_BITS_PER_PIXEL);

		ILenum format = IL_RGB;
		printf("bpp=%d\n", bpp);
		if (bpp == 24)
			format = IL_RGB;
		else if (bpp == 32)
			format = IL_RGBA;

		ilConvertImage(format, IL_UNSIGNED_BYTE);

		int size = ilGetInteger(IL_IMAGE_SIZE_OF_DATA);
		skybox_img[i].img = (ILubyte *)malloc(size);
		ILubyte *bytes = ilGetData();
		memcpy(skybox_img[i].img, bytes, size);
		skybox_img[i].resX = ilGetInteger(IL_IMAGE_WIDTH);
		skybox_img[i].resY = ilGetInteger(IL_IMAGE_HEIGHT);
		format == IL_RGB ? skybox_img[i].BPP = 3 : skybox_img[i].BPP = 4;
		ilDeleteImages(1, &ImageName);
	}
	ilDisable(IL_ORIGIN_SET);
}

Color Scene::GetSkyboxColor(Ray& r) {
	float t_intersec;
	Vector cubemap_coords; //To index the skybox

	float ma;
	CubeMap img_side;
	float sc, tc, s, t;
	unsigned int xp, yp, width, height, bytesperpixel;

	//skybox indexed by the ray direction
	cubemap_coords = r.direction;


	if (fabs(cubemap_coords.x) > fabs(cubemap_coords.y)) {
		ma = fabs(cubemap_coords.x);
		cubemap_coords.x >= 0 ? img_side = LEFT : img_side = RIGHT;    //left cubemap at X = +1 and right at X = -1
	}
	else {
		ma = fabs(cubemap_coords.y);
		cubemap_coords.y >= 0 ? img_side = TOP : img_side = BOTTOM; //top cubemap at Y = +1 and bottom at Y = -1
	}

	if (fabs(cubemap_coords.z) > ma) {
		ma = fabs(cubemap_coords.z);
		cubemap_coords.z >= 0 ? img_side = FRONT : img_side = BACK;   //front cubemap at Z = +1 and back at Z = -1
	}

	switch (img_side) {

	case 0:  //right
		sc = -cubemap_coords.z;
		tc = cubemap_coords.y;
		break;

	case 1:  //left
		sc = cubemap_coords.z;
		tc = cubemap_coords.y;
		break;

	case 2:  //top
		sc = -cubemap_coords.x;
		tc = -cubemap_coords.z;
		break;

	case 3: //bottom
		sc = -cubemap_coords.x;
		tc = cubemap_coords.z;
		break;

	case 4:  //front
		sc = -cubemap_coords.x;
		tc = cubemap_coords.y;
		break;

	case 5: //back
		sc = cubemap_coords.x;
		tc = cubemap_coords.y;
		break;
	}

	double invMa = 1 / ma;
	s = (sc * invMa + 1) / 2;
	t = (tc * invMa + 1) / 2;

	width = skybox_img[img_side].resX;
	height = skybox_img[img_side].resY;
	bytesperpixel = skybox_img[img_side].BPP;

	xp = int((width - 1) * s);
	xp < 0 ? 0 : (xp > (width - 1) ? width - 1 : xp);
	yp = int((height - 1) * t);
	yp < 0 ? 0 : (yp > (height - 1) ? height - 1 : yp);

	float red = u8tofloat(skybox_img[img_side].img[(yp*width + xp) * bytesperpixel]);
	float green = u8tofloat(skybox_img[img_side].img[(yp*width + xp) * bytesperpixel + 1]);
	float blue = u8tofloat(skybox_img[img_side].img[(yp*width + xp) * bytesperpixel + 2]);

	return(Color(red, green, blue));
}




////////////////////////////////////////////////////////////////////////////////
// P3F file parsing methods.
//
void next_token(ifstream& file, char *token, const char *name)
{
  file >> token;
  if (strcmp(token, name))
    cerr << "'" << name << "' expected.\n";
}


bool Scene::load_p3f(const char *name)
{
  const	int	lineSize = 1024;
  string	cmd;
  char		token	[256];
  ifstream	file(name, ios::in);
  Material *	material;

  material = NULL;
  this->SetSkyBoxFlg(false);  //init with no skybox

  if (file >> cmd)
  {
    while (true)
    {
      if (cmd == "accel") {  //Acceleration data structure
		string accel_type; // type of acceleration data structure
		file >> accel_type;
		if (accel_type == "none")
			this->SetAccelStruct(NONE);
		else if (accel_type == "grid")
			this->SetAccelStruct(GRID_ACC);
		else if (accel_type == "bvh")
			this->SetAccelStruct(BVH_ACC);
		else {
			printf("Unsupported acceleration type\n");
			break;
		}

	  }

	  else if (cmd == "spp")    //samples per pixel
	  {
		  unsigned int spp; // number of samples per pixel

		  file >> spp;
		  this->SetSamplesPerPixel(spp);
	  }
	  else if (cmd == "mat")   //Material
      {
		  double Kd, Ks, Shine, T, ior;
		  Color cd, cs;

		  file >> cd >> Kd >> cs >> Ks >> Shine >> T >> ior;

		  material = new Material(cd, Kd, cs, Ks, Shine, T, ior);
      }

      else if (cmd == "s")    //Sphere
      {
	     Vector center;
    	 float radius;
         Sphere* sphere;

	    file >> center >> radius;
        sphere = new Sphere(center,radius);
	    if (material) sphere->SetMaterial(material);
        this->addObject( (Object*) sphere);
      }

	  else if (cmd == "box")    //axis aligned box
	  {
		  Vector minpoint, maxpoint;
		  aaBox	*box;

		  file >> minpoint >> maxpoint;
		  box = new aaBox(minpoint, maxpoint);
		  if (material) box->SetMaterial(material);
		  this->addObject((Object*)box);
	  }
	  else if (cmd == "p")  // Polygon: just accepts triangles for now
      {
		  Vector P0, P1, P2;
		  Triangle* triangle;
		  unsigned total_vertices;

		  file >> total_vertices;
		  if (total_vertices == 3)
		  {
			  file >> P0 >> P1 >> P2;
			  triangle = new Triangle(P0, P1, P2);
			  if (material) triangle->SetMaterial(material);
			  this->addObject( (Object*) triangle);
		  }
		  else
		  {
			  cerr << "Unsupported number of vertices.\n";
			  break;
		  }
      }

	  else if (cmd == "mesh") {
		  unsigned total_vertices, total_faces;
		  unsigned P0, P1, P2;
		  Triangle* triangle;
		  Vector* verticesArray, vertex;

		  file >> total_vertices >> total_faces;
		  verticesArray = (Vector*)malloc(total_vertices * sizeof(Vector));
		  for (int i = 0; i < total_vertices; i++) {
			  file >> vertex;
			  verticesArray[i] = vertex;
		  }

		  for (int i = 0; i < total_faces; i++) {
			  file >> P0 >> P1 >> P2;
			  if (P0 > 0) {
				  P0 -= 1;
				  P1 -= 1;
				  P2 -= 1;
			  }
			  else {
				  P0 += total_vertices;
				  P1 += total_vertices;
				  P2 += total_vertices;
			  }
			  triangle = new Triangle(verticesArray[P0], verticesArray[P1], verticesArray[P2]); //vertex index start at 1
			  if (material) triangle->SetMaterial(material);
			  this->addObject((Object*)triangle);
		  }
	  }

      else if (cmd == "npl")  //Plane in Hessian form
	  {
          Vector N;
          float D;
          Plane* plane;

          file >> N >> D;
          plane = new Plane(N, D);
	      if (material) plane->SetMaterial(material);
          this->addObject( (Object*) plane);
	  }
	  else if (cmd == "pl")  // General Plane
	  {
          Vector P0, P1, P2;
		  Plane* plane;

          file >> P0 >> P1 >> P2;
          plane = new Plane(P0, P1, P2);
	      if (material) plane->SetMaterial(material);
          this->addObject( (Object*) plane);
	  }

      else if (cmd == "light")  // Need to check light color since by default is white
      {
	    Vector pos, at;
        Color color;
		Vector v1, v2;
		unsigned int grid_res;

		string type;
		file >> type;

		if (type == "punctual") {
			file >> pos >> color;
			this->addLight(new Light(pos, color));
		}
		else if(type == "quad") {
			file >> pos >> color >> v1 >> v2 >> grid_res;
			this->addLight(new Light(pos, color, v1, v2, grid_res));
		}
		else
		{
			cerr << "Unsupported light type.\n";
			break;
		}

      }
      else if (cmd == "camera")
      {
	    Vector up, from, at;
	    float fov, hither;
	    int xres, yres;
        Camera* camera;
		float focal_ratio; //ratio beteween the focal distance and the viewplane distance
		float aperture_ratio; // number of times to be multiplied by the size of a pixel

	    next_token (file, token, "eye");
	    file >> from;

	    next_token (file, token, "at");
	    file >> at;

	    next_token (file, token, "up");
	    file >> up;

	    next_token (file, token, "angle");
	    file >> fov;

	    next_token (file, token, "hither");
	    file >> hither;

	    next_token (file, token, "resolution");
	    file >> xres >> yres;

		next_token(file, token, "aperture");
		file >> aperture_ratio;

		next_token(file, token, "focal");
		file >> focal_ratio;
	    // Create Camera
		camera = new Camera( from, at, up, fov, hither, 1000.0*hither, xres, yres, aperture_ratio, focal_ratio);
        this->SetCamera(camera);
      }

	  else if (cmd == "bclr")   //Background color
	  {
	  Color bgcolor;
	  file >> bgcolor;
	  this->SetBackgroundColor(bgcolor);
	  }

	  else if (cmd == "env")
	  {
	  file >> token;

	  this->LoadSkybox(token);
	  this->SetSkyBoxFlg(true);
	  }

	  /*
      else if (cmd == "b")   //Background color
      {
		Color bgcolor;
		bool skybox_flg;

		file >> token;
		if (!strcmp(token, "clr")) {
			skybox_flg = false;
			this->SetSkyBoxFlg(skybox_flg);
			file >> bgcolor;
			this->SetBackgroundColor(bgcolor);
		}
		else if (!strcmp(token, "cubemap")) {
			skybox_flg = true;
			this->SetSkyBoxFlg(skybox_flg);
			file >> token;
			this->LoadSkybox(token);
		}
		else {
			skybox_flg = false;
			this->SetSkyBoxFlg(skybox_flg);
			file.ignore(lineSize, '\n');
			this->SetBackgroundColor(Color(0, 0, 0));
		}

      }
	  */

      else if (cmd[0] == '#')
      {
	    file.ignore (lineSize, '\n');
      }
      else
      {
	    cerr << "unknown command '" << cmd << "'.\n";
	    break;
      }
      if (!(file >> cmd))
        break;
    }
  }

  file.close();
  return true;
};

void Scene::create_random_scene() {
	Camera* camera;
	Material* material;
	Sphere* sphere;

	set_rand_seed(time(NULL)* time(NULL)* time(NULL));
	material = NULL;
	this->SetSkyBoxFlg(false);  //init with no skybox

	this->SetBackgroundColor(Color(0.5, 0.7, 1.0));
	this->SetAccelStruct(NONE);
	//this->SetAccelStruct(GRID_ACC);
	this->SetSamplesPerPixel(0);
	camera = new Camera(Vector(-5.312192, 4.456562, 11.963158), Vector(0.0, 0.0, 0), Vector(0.0, 1.0, 0.0), 40.0, 0.01, 10000.0, 800, 600, 0, 1.5f);
	this->SetCamera(camera);

	this->addLight(new Light(Vector(7, 10, -5), Color(1.0, 1.0, 1.0)));
	this->addLight(new Light(Vector(-7, 10, -5), Color(1.0, 1.0, 1.0)));
	this->addLight(new Light(Vector(0, 10, 7), Color(1.0, 1.0, 1.0)));

	material = new Material(Color(0.5, 0.5, 0.5), 1.0, Color(0.0, 0.0, 0.0), 0.0, 10, 0, 1);


	sphere = new Sphere(Vector(0.0, -1000, 0.0), 1000.0);
	if (material) sphere->SetMaterial(material);
	this->addObject((Object*)sphere);

	for(int a = -5; a < 5; a++)
		for (int b = -5; b < 5; b++) {

			double choose_mat = rand_double();

			Vector center = Vector(a + 0.9 * rand_double(), 0.2, b + 0.9 * rand_double());

			if ((center - Vector(4.0, 0.2, 0.0)).length() > 0.9) {
				if (choose_mat < 0.4) {  //diffuse
					material = new Material(Color(rand_double(), rand_double(), rand_double()), 1.0, Color(0.0, 0.0, 0.0), 0.0, 10, 0, 1);
					sphere = new Sphere(center, 0.2);
					if (material) sphere->SetMaterial(material);
					this->addObject((Object*)sphere);
				}
				else if (choose_mat < 0.7) {   //metal
					material = new Material(Color(0.0, 0.0, 0.0), 0.0, Color(rand_double(0.5, 1), rand_double(0.5, 1), rand_double(0.5, 1)), 1.0, 220, 0, 1);
					sphere = new Sphere(center, 0.2);
					if (material) sphere->SetMaterial(material);
					this->addObject((Object*)sphere);
				}
				else {   //glass
					//material = new Material(Color(0.8, 0.3, 0.3), 0.0, Color(1.0, 1.0, 1.0), 0.7, 20, 1, 1.5);
					material = new Material(Color(rand_double(0.6, 1), rand_double(0.6, 1), rand_double(0.6, 1)), 0.0, Color(1.0, 1.0, 1.0), 0.7, 20, 1, 1.5);
					sphere = new Sphere(center, 0.2);
					if (material) sphere->SetMaterial(material);
					this->addObject((Object*)sphere);
				}

			}

		}

	material = new Material(Color(1.0, 1.0, 1.0), 0.0, Color(1.0, 1.0, 1.0), 0.7, 20, 1, 1.5);
	sphere = new Sphere(Vector(0.0, 1.0, 0.0), 1.0);
	if (material) sphere->SetMaterial(material);
	this->addObject((Object*)sphere);

	material = new Material(Color(0.4, 0.2, 0.1), 0.9, Color(1.0, 1.0, 1.0), 0.0, 10, 0, 1.0);
	sphere = new Sphere(Vector(-4.0, 1.0, 0.0), 1.0);
	if (material) sphere->SetMaterial(material);
	this->addObject((Object*)sphere);

	material = new Material(Color(0.4, 0.2, 0.1), 0.0, Color(0.7, 0.6, 0.5), 1.0, 220, 0, 1.0);
	sphere = new Sphere(Vector(4.0, 1.0, 0.0), 1.0);
	if (material) sphere->SetMaterial(material);
	this->addObject((Object*)sphere);
}
