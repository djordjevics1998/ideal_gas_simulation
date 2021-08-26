#include "geometry.h"
#include <math.h>
#include <string>
#include <sstream>
#include <algorithm>
#include <random>
#include <chrono>
#include <set>

#define SQR(a) (a * a)
#define COLL_A(a, b) ((a - b) * (a - b))
#define COLL_B(c_1, v_1, c_2, v_2) ((c_1 - c_2) * (v_1 - v_2))

Event::Event() {
	this->o1 = NULL;
	this->o2 = NULL;
	this->t = 0;
	this->dt = 0;
}

Event::Event(PhObject* o1, PhObject* o2, double t, double dt) {
	this->o1 = o1;
	this->o2 = o2;
	this->t = t;
	this->dt = dt;
}

bool Event::compare(Event* e1, Event* e2) {
	if (e1->dt <= -0.5 && e2->dt <= -0.5) return e1->dt > e2->dt;
	if (e1->dt <= -0.5) return false;
	if (e2->dt <= -0.5) return true;
	if (e1->t + e1->dt == e2->t + e2->dt) return e1 < e2;
	return e1->t + e1->dt < e2->t + e2->dt;
}

Point2D::Point2D(double x, double y) {
	this->x = x;
	this->y = y;
}

Point2D::Point2D(Point2D *p) {
	this->x = p->x;
	this->y = p->y;
}

double Point2D::getX() {
	return this->x;
}

double Point2D::getY() {
	return this->y;
}

void Point2D::set(double x, double y) {
	this->x = x;
	this->y = y;
}

void Point2D::add(double dx, double dy) {
	this->x += dx;
	this->y += dy;
}


std::string Point2D::toString() {
	std::ostringstream ssx, ssy;
	ssx << std::scientific << this->x;
	ssy << std::scientific << this->y;
	return "Point2D(" + ssx.str() + ", " + ssy.str() + ")";
}

double Point2D::distance(Point2D *p1, Point2D *p2) {
	return sqrt(COLL_A(p1->x, p2->x) + COLL_A(p1->y, p2->y));
}

Point3D::Point3D(double x, double y, double z) {
	this->x = x;
	this->y = y;
	this->z = z;
}

Point3D::Point3D(Point3D* p) {
	this->x = p->x;
	this->y = p->y;
	this->z = p->z;
}

double Point3D::getX() {
	return this->x;
}

double Point3D::getY() {
	return this->y;
}

double Point3D::getZ() {
	return this->z;
}

void Point3D::set(double x, double y, double z) {
	this->x = x;
	this->y = y;
	this->z = z;
}

void Point3D::add(double dx, double dy, double dz) {
	this->x += dx;
	this->y += dy;
	this->z += dz;
}


std::string Point3D::toString() {
	std::ostringstream ssx, ssy, ssz;
	ssx << std::scientific << this->x;
	ssy << std::scientific << this->y;
	ssz << std::scientific << this->z;
	return "Point3D(" + ssx.str() + ", " + ssy.str() + ", " + ssz.str() + ")";
}

double Point3D::distance(Point3D* p1, Point3D* p2) {
	return sqrt(COLL_A(p1->x, p2->x) + COLL_A(p1->y, p2->y) + COLL_A(p1->z, p2->z));
}

void PhObject::initEvents(int n) {
	this->events_n = n;
	this->events = new Event*[n];
}

Event** PhObject::getEvents() {
	return this->events;
}

int PhObject::getEventsLen() {
	return this->events_n;
}

PhObject::PhObject() {
	this->events = NULL;
	this->events_n = 0;
}

double PhObject::collision(PhObject *o1, PhObject *o2, double act) {
	double t = act;
	if(o1->getType() == LINE_2D && o2->getType() == LINE_2D) return NOT_COLLIDING;
	else if((o1->getType() == LINE_2D && o2->getType() == PARTICLE_2D) ||
		(o1->getType() == PARTICLE_2D && o2->getType() == LINE_2D)) {
		Line2D *sw = static_cast<Line2D *>(o1->getType() == LINE_2D ? o1 : o2);
		Particle2D *p = static_cast<Particle2D *>(o1->getType() == LINE_2D ? o2 : o1);
		Point2D *p1 = sw->getFirstPoint(), *p2 = sw->getSecondPoint(), *ctr = p->getCenter();
		Vector2D *v = p->getVelocity();
		
		Vector2D tang(p1, p2, true),
			d1(ctr, p2),
			ort(-tang.getY(), tang.getX()),
			ortsw = Vector2D::projection(&d1, &ort),
			ortv = Vector2D::projection(v, &ort);
		if(act < -0.5) {
			if (ortsw.getX() * ortv.getX() + ortsw.getY() * ortv.getY() <= 0) {
				//std::cout << triangle->toString() << " " << v->toString() << std::endl;
				//std::cout << orttriangle.toString() << " " << ortv.toString() << std::endl << std::endl;
 				return NOT_COLLIDING;
			}
			else {
				t = (ortsw.len() - p->getRadius()) / ortv.len();
				if (t <= 0) return NOT_COLLIDING;
			}
		} else {

			Vector2D tangv = Vector2D::projection(v, &tang);
			ortv.multiply(-1);
			v->set(tangv.getX() + ortv.getX(), tangv.getY() + ortv.getY());

			//std::cout << triangle->toString() << std::endl;
			t = 2 * p->getMass() * ortv.len();
		}
	} else if(o1->getType() == PARTICLE_2D && o2->getType() == PARTICLE_2D) {
		Particle2D* p1 = static_cast<Particle2D*>(o1), * p2 = static_cast<Particle2D*>(o2);
		Vector2D *v1 = p1->getVelocity(), *v2 = p2->getVelocity();
		Point2D *c1 = p1->getCenter(), *c2 = p2->getCenter();
		if(act < -0.5) {
			if(Point2D::distance(c1, c2) < p1->getRadius() + p2->getRadius()) return INSIDE_EACH_OTHER;
			else if (Point2D::distance(c1, c2) == p1->getRadius() + p2->getRadius()) {
				Vector2D d(c1, c2, true),
					tang1 = Vector2D::projection(v1, &d),
					tang2 = Vector2D::projection(v2, &d),
					tangs = Vector2D::sub(&tang1, &tang2);
				if (tangs.len() == 0 || tangs.getX() / (c1->getX() - c2->getX()) < 0 || tangs.getY() / (c1->getY() - c2->getY()) < 0) return NOT_COLLIDING;
			}
			double a = COLL_A(v1->getX(), v2->getX()) + COLL_A(v1->getY(), v2->getY()),
				b = 2 * (COLL_B(c1->getX(), v1->getX(), c2->getX(), v2->getX()) + COLL_B(c1->getY(), v1->getY(), c2->getY(), v2->getY())),
				c = COLL_A(c1->getX(), c2->getX()) + COLL_A(c1->getY(), c2->getY()) - COLL_A(p1->getRadius(), -p2->getRadius());
			if(b * b - 4 * a * c < 0) return NOT_COLLIDING;
			double t1 = (-b - sqrt(b * b - 4 * a * c)) / 2 / a,
				t2 = (-b + sqrt(b * b - 4 * a * c)) / 2 / a;
			if(t1 < 0 && t2 < 0) return NOT_COLLIDING;
			else t = (t1 < 0) ? t2 : t1;
			if (t == 0) { // check if going to collide or get away
				Vector2D d(c1, c2, true),
					tang1 = Vector2D::projection(v1, &d),
					tang2 = Vector2D::projection(v2, &d),
					tangs = Vector2D::sub(&tang1, &tang2);
				//std::cout << v1->toString() << " " << v2->toString() << std::endl;
				if (tangs.len() == 0 || tangs.getX() / (c1->getX() - c2->getX()) < 0 || tangs.getY() / (c1->getY() - c2->getY()) < 0) return NOT_COLLIDING;
			}
			else if (t < 0) return NOT_COLLIDING; // quantisation error
		} else {
			Vector2D tang(c1, c2, true),
				ort(-tang.getY(), tang.getX()),
				tang1 = Vector2D::projection(v1, &tang),
				tang2 = Vector2D::projection(v2, &tang),
				ort1 = Vector2D::projection(v1, &ort),
				ort2 = Vector2D::projection(v2, &ort);
			double m1 = p1->getMass(), m2 = p2->getMass(),
				a11 = (m1 - m2) / (m1 + m2), a12 = 2 * m2 / (m1 + m2),
				a21 = 2 * m1 / (m1 + m2), a22 = (m2 - m1) / (m1 + m2);
			v1->set(a11 * tang1.getX() + a12 * tang2.getX() + ort1.getX(), a11 * tang1.getY() + a12 * tang2.getY() + ort1.getY());
			v2->set(a21 * tang1.getX() + a22 * tang2.getX() + ort2.getX(), a21 * tang1.getY() + a22 * tang2.getY() + ort2.getY());
			
			t = 0; // promena impulsa nula
		}
	} else if (o1->getType() == TRIANGLE && o2->getType() == TRIANGLE) return NOT_COLLIDING;
	else if ((o1->getType() == TRIANGLE && o2->getType() == PARTICLE_3D) ||
		(o1->getType() == PARTICLE_3D && o2->getType() == TRIANGLE)) {
		Triangle* triangle = static_cast<Triangle*>(o1->getType() == TRIANGLE ? o1 : o2);
		Particle3D* p = static_cast<Particle3D*>(o1->getType() == TRIANGLE ? o2 : o1);
		Point3D* p1 = triangle->getFirstPoint(), * p2 = triangle->getSecondPoint(), * p3 = triangle->getThirdPoint(), * ctr = p->getCenter();
		Vector3D* v = p->getVelocity();

		Vector3D a(p1, p2, true),
			b(p1, p3, true),
			d1(ctr, p2),
			ort = Vector3D::vector(&a, &b, true),
			orttriangle = Vector3D::projection(&d1, &ort),
			ortv = Vector3D::projection(v, &ort);

		if (act < -0.5) {
			if (orttriangle.getX() * ortv.getX() + orttriangle.getY() * ortv.getY() + orttriangle.getZ() * ortv.getZ() <= 0) {
				//std::cout << triangle->toString() << " " << v->toString() << std::endl;
				//std::cout << orttriangle.toString() << " " << ortv.toString() << std::endl << std::endl;
				return NOT_COLLIDING;
			}
			else {
				t = (orttriangle.len() - p->getRadius()) / ortv.len();
				if (t <= 0) return NOT_COLLIDING;
			}
		}
		else {
			Vector3D tangv = Vector3D::sub(v, &ortv);
			ortv.multiply(-1);
			v->set(tangv.getX() + ortv.getX(), tangv.getY() + ortv.getY(), tangv.getZ() + ortv.getZ());

			//std::cout << triangle->toString() << std::endl;
			t = 2 * p->getMass() * ortv.len();
		}
	}
	else if (o1->getType() == PARTICLE_3D && o2->getType() == PARTICLE_3D) {
		Particle3D* p1 = static_cast<Particle3D*>(o1), * p2 = static_cast<Particle3D*>(o2);
		Vector3D* v1 = p1->getVelocity(), * v2 = p2->getVelocity();
		Point3D* c1 = p1->getCenter(), * c2 = p2->getCenter();
		if (act < -0.5) {
			if (Point3D::distance(c1, c2) < p1->getRadius() + p2->getRadius()) return INSIDE_EACH_OTHER;
			else if (Point3D::distance(c1, c2) == p1->getRadius() + p2->getRadius()) {
				Vector3D d(c1, c2, true),
					tang1 = Vector3D::projection(v1, &d),
					tang2 = Vector3D::projection(v2, &d),
					tangs = Vector3D::sub(&tang1, &tang2);
				if (tangs.len() == 0 || tangs.getX() / (c1->getX() - c2->getX()) < 0 || tangs.getY() / (c1->getY() - c2->getY()) < 0 || tangs.getZ() / (c1->getZ() - c2->getZ()) < 0) return NOT_COLLIDING;
			}
			double a = COLL_A(v1->getX(), v2->getX()) + COLL_A(v1->getY(), v2->getY()) + COLL_A(v1->getZ(), v2->getZ()),
				b = 2 * (COLL_B(c1->getX(), v1->getX(), c2->getX(), v2->getX()) + COLL_B(c1->getY(), v1->getY(), c2->getY(), v2->getY()) + COLL_B(c1->getZ(), v1->getZ(), c2->getZ(), v2->getZ())),
				c = COLL_A(c1->getX(), c2->getX()) + COLL_A(c1->getY(), c2->getY()) + COLL_A(c1->getZ(), c2->getZ()) - COLL_A(p1->getRadius(), -p2->getRadius());
			if (b * b - 4 * a * c < 0) return NOT_COLLIDING;
			double t1 = (-b - sqrt(b * b - 4 * a * c)) / 2 / a,
				t2 = (-b + sqrt(b * b - 4 * a * c)) / 2 / a;
			if (t1 < 0 && t2 < 0) return NOT_COLLIDING;
			else t = (t1 < 0) ? t2 : t1;
			if (t == 0) { // check if going to collide or get away
				Vector3D d(c1, c2, true),
					tang1 = Vector3D::projection(v1, &d),
					tang2 = Vector3D::projection(v2, &d),
					tangs = Vector3D::sub(&tang1, &tang2);
				//std::cout << v1->toString() << " " << v2->toString() << std::endl;
				if (tangs.len() == 0 || tangs.getX() / (c1->getX() - c2->getX()) < 0 || tangs.getY() / (c1->getY() - c2->getY()) < 0 || tangs.getZ() / (c1->getZ() - c2->getZ()) < 0) return NOT_COLLIDING;
			}
			else if (t < 0) return NOT_COLLIDING; // quantisation error
		}
		else {
			Vector3D tang(c1, c2, true),
				tang1 = Vector3D::projection(v1, &tang),
				tang2 = Vector3D::projection(v2, &tang),
				ort1 = Vector3D::sub(v1, &tang1),
				ort2 = Vector3D::sub(v2, &tang2);


			double m1 = p1->getMass(), m2 = p2->getMass(),
				a11 = (m1 - m2) / (m1 + m2), a12 = 2 * m2 / (m1 + m2),
				a21 = 2 * m1 / (m1 + m2), a22 = (m2 - m1) / (m1 + m2);
			v1->set(a11* tang1.getX() + a12 * tang2.getX() + ort1.getX(), a11* tang1.getY() + a12 * tang2.getY() + ort1.getY(), a11* tang1.getZ() + a12 * tang2.getZ() + ort1.getZ());
			v2->set(a21* tang1.getX() + a22 * tang2.getX() + ort2.getX(), a21* tang1.getY() + a22 * tang2.getY() + ort2.getY(), a21* tang1.getZ() + a22 * tang2.getZ() + ort2.getZ());

			//v1->set(tang2.getX() + ort1.getX(), tang2.getY() + ort1.getY(), tang2.getZ() + ort1.getZ()); // #TODO azurirati za razlicite mase, mada u eksperimentu nama nisu razlicite
			//v2->set(tang1.getX() + ort2.getX(), tang1.getY() + ort2.getY(), tang1.getZ() + ort2.getZ());

			t = 0; // promena impulsa nula
		}
	} else return UNKNOWN;
	
	return t;
}

bool PhObject::compare(PhObject* o1, PhObject* o2) {
	return o1->events_n == 0 ? false : o2->events_n == 0 ? true : Event::compare(o1->events[0], o2->events[0]);
}

PhObject::~PhObject() {
	delete[] events;
}

Line2D::Line2D(Point2D *p1, Point2D *p2) {
	this->p1 = new Point2D(p1);
	this->p2 = new Point2D(p2);
}

Point2D * Line2D::getFirstPoint() {
	return this->p1;
}

Point2D * Line2D::getSecondPoint() {
	return this->p2;
}

TYPE Line2D::getType() {
	return LINE_2D;
}

std::string Line2D::toString() {
	return "Line2D(" + this->p1->toString() + ", " + this->p2->toString() + ")";
}

void Line2D::progress(double t) {

}

Line2D::~Line2D() {
	delete p1;
	delete p2;
}

Triangle::Triangle(Point3D* p1, Point3D* p2, Point3D* p3) {
	this->p1 = new Point3D(p1);
	this->p2 = new Point3D(p2);
	this->p3 = new Point3D(p3);
}

Point3D* Triangle::getFirstPoint() {
	return this->p1;
}

Point3D* Triangle::getSecondPoint() {
	return this->p2;
}

Point3D* Triangle::getThirdPoint() {
	return this->p3;
}

TYPE Triangle::getType() {
	return TRIANGLE;
}

std::string Triangle::toString() {
	return "Triangle(" + this->p1->toString() + ", " + this->p2->toString() + ", " + this->p3->toString() + ")";
}

void Triangle::progress(double t) {

}

Triangle::~Triangle() {
	delete p1;
	delete p2;
	delete p3;
}

Vector2D::Vector2D(double x, double y) {
	set(x, y);
}

Vector2D::Vector2D(Point2D *p1, Point2D *p2) : Vector2D(p2->getX() - p1->getX(), p2->getY() - p1->getY()) {
}

Vector2D::Vector2D(Point2D *p1, Point2D *p2, bool normalize) : Vector2D(p1, p2) {
	if(normalize) this->multiply(1 / this->len());
}

Vector2D::Vector2D(Vector2D *v) : Vector2D(v->x, v->y) {
}

double Vector2D::getX() {
	return this->x;
}

double Vector2D::getY() {
	return this->y;
}

void Vector2D::set(double x, double y) {
	this->x = x;
	this->y = y;
}

double Vector2D::len() {
	return sqrt(SQR(this->x) + SQR(this->y));
}

void Vector2D::multiply(double m) {
	this->x *= m;
	this->y *= m;
}

double Vector2D::scalar(Vector2D *v) {
	return this->x * v->x + this->y * v->y;
}

std::string Vector2D::toString() {
	std::ostringstream ssx, ssy;
	ssx << std::scientific << this->x;
	ssy << std::scientific << this->y;
	return "Vector2D(" + ssx.str() + ", " + ssy.str() + ")";
}

Vector2D Vector2D::projection(Vector2D *v_orig, Vector2D *v_on) {
	Vector2D projection(v_on);
	if(projection.x != 0 || projection.y != 0) projection.multiply(1 / projection.len());
	projection.multiply(projection.scalar(v_orig));
	return projection;
}

Vector2D Vector2D::add(Vector2D *v1, Vector2D *v2) {
	return Vector2D(v1->x + v2->x, v1->y + v2->y);
}

Vector2D Vector2D::sub(Vector2D* from, Vector2D* what) {
	return Vector2D(from->x - what->x, from->y - what->y);
}

Vector3D::Vector3D(double x, double y, double z) {
	set(x, y, z);
}

Vector3D::Vector3D(Point3D* p1, Point3D* p2) : Vector3D(p2->getX() - p1->getX(), p2->getY() - p1->getY(), p2->getZ() - p1->getZ()) {
}

Vector3D::Vector3D(Point3D* p1, Point3D* p2, bool normalize) : Vector3D(p1, p2) {
	if (normalize) this->multiply(1 / this->len());
}

Vector3D::Vector3D(Vector3D* v) : Vector3D(v->x, v->y, v->z) {
}

double Vector3D::getX() {
	return this->x;
}

double Vector3D::getY() {
	return this->y;
}

double Vector3D::getZ() {
	return this->z;
}

void Vector3D::set(double x, double y, double z) {
	this->x = x;
	this->y = y;
	this->z = z;
}

double Vector3D::len() {
	return sqrt(SQR(this->x) + SQR(this->y) + SQR(this->z));
}

void Vector3D::multiply(double m) {
	this->x *= m;
	this->y *= m;
	this->z *= m;
}

double Vector3D::scalar(Vector3D* v) {
	return this->x * v->x + this->y * v->y + this->z * v->z;
}

std::string Vector3D::toString() {
	std::ostringstream ssx, ssy, ssz;
	ssx << std::scientific << this->x;
	ssy << std::scientific << this->y;
	ssz << std::scientific << this->z;
	return "Vector3D(" + ssx.str() + ", " + ssy.str() + ", " + ssz.str() + ")";
}

Vector3D Vector3D::vector(Vector3D* v1, Vector3D* v2, bool normalize) {
	Vector3D product(v1->y * v2->z - v1->z * v2->y, v1->z * v2->x - v1->x * v2->z, v1->x * v2->y - v1->y * v2->x);
	if (normalize && (product.x != 0 || product.y != 0 || product.z != 0)) product.multiply(1 / product.len());
	return product;
}

Vector3D Vector3D::projection(Vector3D* v_orig, Vector3D* v_on) {
	Vector3D projection(v_on);
	if (projection.x != 0 || projection.y != 0 || projection.z != 0) projection.multiply(1 / projection.len());
	projection.multiply(projection.scalar(v_orig));
	return projection;
}

Vector3D Vector3D::add(Vector3D* v1, Vector3D* v2) {
	return Vector3D(v1->x + v2->x, v1->y + v2->y, v1->z + v2->z);
}

Vector3D Vector3D::sub(Vector3D* from, Vector3D* what) {
	return Vector3D(from->x - what->x, from->y - what->y, from->z - what->z);
}

ParticleConfig::ParticleConfig(int id, double r, double m) {
	this->id = id;
	this->r = r;
	this->m = m;
}

ParticleConfig::ParticleConfig(ParticleConfig* pc) : ParticleConfig::ParticleConfig(pc->id, pc->r, pc->m) {

}

int ParticleConfig::getId() {
	return this->id;
}

double ParticleConfig::getRadius() {
	return this->r;
}

double ParticleConfig::getMass() {
	return this->m;
}

Particle2D::Particle2D(int id, Point2D *c, double r, double m, double vx, double vy) : ParticleConfig(id, r, m) {
	this->c = new Point2D(c);
	this->v = new Vector2D(vx, vy);
}

Particle2D::Particle2D(ParticleConfig* pc, Point2D* c, double vx, double vy) : Particle2D::Particle2D(pc->getId(), c, pc->getRadius(), pc->getMass(), vx, vy) {

}

Point2D * Particle2D::getCenter() {
	return this->c;
}

int Particle2D::getId() {
	return ParticleConfig::getId();
}

double Particle2D::getRadius() {
	return ParticleConfig::getRadius();
}

double Particle2D::getMass() {
	return ParticleConfig::getMass();
}

Vector2D * Particle2D::getVelocity() {
	return this->v;
}

void Particle2D::progress(double t) {
	this->c->add(this->v->getX() * t, this->v->getY() * t);
}

TYPE Particle2D::getType() {
	return PARTICLE_2D;
}

std::string Particle2D::toString() {
	std::ostringstream ssr;
	ssr << std::scientific << this->r;
	return "Particle2D(" + this->c->toString() + ", " + ssr.str() + ", " + this->v->toString() + ")";
}

Particle2D::~Particle2D() {
	delete c;
	delete v;
}

Particle3D::Particle3D(int id, Point3D* c, double r, double m, double vx, double vy, double vz) : ParticleConfig::ParticleConfig(id, r, m) {
	this->c = new Point3D(c);
	this->v = new Vector3D(vx, vy, vz);
}

Particle3D::Particle3D(ParticleConfig* pc, Point3D* c, double vx, double vy, double vz) : Particle3D::Particle3D(pc->getId(), c, pc->getRadius(), pc->getMass(), vx, vy, vz) {

}

Point3D* Particle3D::getCenter() {
	return this->c;
}

int Particle3D::getId() {
	return ParticleConfig::getId();
}

double Particle3D::getRadius() {
	return ParticleConfig::getRadius();
}

double Particle3D::getMass() {
	return ParticleConfig::getMass();
}

Vector3D* Particle3D::getVelocity() {
	return this->v;
}

void Particle3D::progress(double t) {
	this->c->add(this->v->getX() * t, this->v->getY() * t, this->v->getZ() * t);
}

TYPE Particle3D::getType() {
	return PARTICLE_3D;
}

std::string Particle3D::toString() {
	return "Particle3D(" + this->c->toString() + ", " + this->v->toString() + ")";
}

Particle3D::~Particle3D() {
	delete c;
	delete v;
}

void Simulation::simulate() {
	std::mt19937 rng(std::chrono::steady_clock::now().time_since_epoch().count());
	std::uniform_real_distribution<> distR(0, 1);
	double t = 0, avg_pv = 0, temp, dp = 0, dt = 0;

	Event** events_1, ** events_2;
	all_events_p = new Event * [(objs_len * (objs_len - 1)) / 2];
	int b = 0;
	for (int l = 0; l < objs_len; l++) objs[l]->initEvents(objs_len - 1);
	for (int l = 0; l < objs_len; l++) {
		events_1 = objs[l]->getEvents();
		for (int j = l + 1; j < objs_len; j++) {
			events_2 = objs[j]->getEvents();
			all_events_p[b++] = events_1[j - 1] = new Event(objs[l], objs[j], t, PhObject::collision(objs[l], objs[j], -1));
			if (events_1[j - 1]->dt <= -0.5) events_1[j - 1]->dt -= distR(rng);
			events_2[l] = events_1[j - 1];
		}
	}
	std::multiset<Event*, decltype(Event::compare)*> allEvs(all_events_p, all_events_p + (objs_len * (objs_len - 1)) / 2, Event::compare);

	if (listener != nullptr) listener->OnSimulationStart(objs, objs_len);
	Event* tEv;
	for (int b = 0; b < sim_count * sim_step; b++) {

		tEv = (*allEvs.begin());
		for (int l = 0; l < objs_len; l++) objs[l]->progress((tEv->t - t) + tEv->dt);

		temp = PhObject::collision(tEv->o1, tEv->o2, tEv->dt);
		dp += temp;
		dt += (tEv->t - t) + tEv->dt;
		t = tEv->t + tEv->dt;

		if (tEv->o1->getType() != LINE_2D && tEv->o1->getType() != TRIANGLE) {

			events_1 = tEv->o1->getEvents();
			for (int l = 0; l < objs_len - 1; l++) {
				auto itt = allEvs.find(events_1[l]);
				if (itt != allEvs.end()) {
					while (*itt != events_1[l])	++itt;

					allEvs.erase(itt);
					events_1[l]->t = t;
					events_1[l]->dt = PhObject::collision(events_1[l]->o1, events_1[l]->o2, -1);
					if (events_1[l]->dt <= -0.5) events_1[l]->dt -= distR(rng);
					allEvs.insert(events_1[l]);
				}
			}
		}

		if (tEv->o2->getType() != LINE_2D && tEv->o2->getType() != TRIANGLE) {

			events_1 = tEv->o2->getEvents();
			for (int l = 0; l < objs_len - 1; l++) {
				auto itt = allEvs.find(events_1[l]);
				if (itt != allEvs.end()) {
					while (*itt != events_1[l])	++itt;

					allEvs.erase(itt);
					events_1[l]->t = t;
					events_1[l]->dt = PhObject::collision(events_1[l]->o1, events_1[l]->o2, -1);
					if (events_1[l]->dt <= -0.5) events_1[l]->dt -= distR(rng);
					allEvs.insert(events_1[l]);
				}
			}
		}

		if ((b + 1) % sim_step == 0) {
			avg_pv += Vs * dp / dt;
			if (listener != nullptr) listener->OnSimulationStep(Vs * dp / dt, N * kB * T, (b + 1) / sim_step - 1);
			dp = 0;
			dt = 0;
			//for (int l = walls_len; l < objs_len; l++) myfile << static_cast<Particle2D*>(objs[l])->getVelocity()->len() << endl;
		}
		if (listener != nullptr) listener->OnSimulationIteration(objs, objs_len, b);
	}
	if (listener != nullptr) listener->OnSimulationEnd(objs, objs_len);

	//std::cout << avg_pv / sim_count << std::endl;
}

void Simulation::setOnSimulationListener(IOnSimulationListener* listener) {
	this->listener = listener;
}

Simulation::Simulation(double kB, double T, double hfw, ParticleConfig* pc1, ParticleConfig* pc2, double rate, long long sim_step, long long sim_count, int row, int col) {
	this->kB = kB;
	this->T = T;
	this->hfw = hfw;
	this->pc1 = new ParticleConfig(pc1);
	this->pc2 = new ParticleConfig(pc2);
	this->rate = rate;
	this->sim_step = sim_step;
	this->sim_count = sim_count;
	this->row = row;
	this->col = col;
	this->listener = nullptr;
	N = 0;
	Vs = 0;
	walls_len = 0;
	objs_len = 0;
	objs = nullptr;
	all_events_p = nullptr;
}

Simulation::~Simulation() {
	delete pc1;
	delete pc2;
	if (objs != nullptr) {
		for (int l = 0; l < objs_len; l++) delete objs[l];
		delete[] objs;
	}
	if (all_events_p != nullptr) {
		for (int l = 0; l < (objs_len * (objs_len - 1)) / 2; l++) delete all_events_p[l];
		delete[] all_events_p;
	}
	if (listener != nullptr) delete listener;
}

Simulation2D::Simulation2D(double kB, double T, double hfw, ParticleConfig* pc1, ParticleConfig* pc2, double rate, long long sim_step, long long sim_count, int row, int col) : Simulation(kB, T, hfw, pc1, pc2, rate, sim_step, sim_count, row, col) {
	N = row * col;
	walls_len = 4;
	Vs = hfw / 2;
}

void Simulation2D::setOnSimulationListener(IOnSimulationListener* listener) {
	Simulation::setOnSimulationListener(listener);
}

void Simulation2D::run() {
	if (objs_len != 0) return;
	objs_len = N + walls_len;
	std::mt19937 rng(std::chrono::steady_clock::now().time_since_epoch().count());
	std::normal_distribution<double> distM_1(0, sqrt(kB * T / pc1->getMass())), distM_2(0, sqrt(kB * T / pc2->getMass()));
	std::uniform_real_distribution<> distR(0, 1);

	Point2D** exts2D = new Point2D * [4];
	exts2D[0] = new Point2D(-hfw, -hfw);
	exts2D[1] = new Point2D(-hfw, hfw);
	exts2D[2] = new Point2D(hfw, hfw);
	exts2D[3] = new Point2D(hfw, -hfw);

	objs = new PhObject * [objs_len];
	objs[0] = new Line2D(exts2D[0], exts2D[1]);
	objs[1] = new Line2D(exts2D[1], exts2D[2]);
	objs[2] = new Line2D(exts2D[2], exts2D[3]);
	objs[3] = new Line2D(exts2D[3], exts2D[0]);

	for (int l = 0; l < 4; l++) delete exts2D[l];
	delete[] exts2D;

	double stepw = 2 * hfw / (row + 1), steph = 2 * hfw / (col + 1);
	bool isFirstParticle;
	for (int l = 0; l < row; l++)
		for (int j = 0; j < col; j++) {
			isFirstParticle = distR(rng) < rate;
			objs[walls_len + l * col + j] = new Particle2D(isFirstParticle ? this->pc1 : this->pc2, new Point2D((l - row / 2 + 0.5) * stepw, (j - col / 2 + 0.5) * steph), isFirstParticle ? distM_1(rng) : distM_2(rng), isFirstParticle ? distM_1(rng) : distM_2(rng));
		}
	simulate();
}

Simulation2D::~Simulation2D() {

}

Simulation3D::Simulation3D(double kB, double T, double hfw, ParticleConfig* pc1, ParticleConfig* pc2, double rate, long long sim_step, long long sim_count, int row, int col, int stack) : Simulation(kB, T, hfw, pc1, pc2, rate, sim_step, sim_count, row, col) {
	this->stack = stack;
	N = row * col * stack;
	walls_len = 12;
	Vs = hfw / 3;
}

void Simulation3D::setOnSimulationListener(IOnSimulationListener* listener) {
	Simulation::setOnSimulationListener(listener);
}

void Simulation3D::run() {
	if (objs_len != 0) return;
	objs_len = N + walls_len;
	std::mt19937 rng(std::chrono::steady_clock::now().time_since_epoch().count());
	std::normal_distribution<double> distM_1(0, sqrt(kB * T / pc1->getMass())), distM_2(0, sqrt(kB * T / pc2->getMass()));
	std::uniform_real_distribution<> distR(0, 1);

	Point3D** exts3D = new Point3D * [8];
	exts3D[0] = new Point3D(-hfw, -hfw, hfw);
	exts3D[1] = new Point3D(-hfw, hfw, hfw);
	exts3D[2] = new Point3D(hfw, hfw, hfw);
	exts3D[3] = new Point3D(hfw, -hfw, hfw);
	exts3D[4] = new Point3D(-hfw, -hfw, -hfw);
	exts3D[5] = new Point3D(-hfw, hfw, -hfw);
	exts3D[6] = new Point3D(hfw, hfw, -hfw);
	exts3D[7] = new Point3D(hfw, -hfw, -hfw);

	objs = new PhObject * [objs_len];
	objs[0] = new Triangle(exts3D[0], exts3D[1], exts3D[2]);
	objs[1] = new Triangle(exts3D[0], exts3D[1], exts3D[3]);
	objs[2] = new Triangle(exts3D[4], exts3D[5], exts3D[6]);
	objs[3] = new Triangle(exts3D[4], exts3D[5], exts3D[7]);
	objs[4] = new Triangle(exts3D[0], exts3D[1], exts3D[4]);
	objs[5] = new Triangle(exts3D[0], exts3D[1], exts3D[5]);
	objs[6] = new Triangle(exts3D[2], exts3D[3], exts3D[6]);
	objs[7] = new Triangle(exts3D[2], exts3D[3], exts3D[7]);
	objs[8] = new Triangle(exts3D[0], exts3D[3], exts3D[4]);
	objs[9] = new Triangle(exts3D[0], exts3D[3], exts3D[7]);
	objs[10] = new Triangle(exts3D[1], exts3D[2], exts3D[5]);
	objs[11] = new Triangle(exts3D[1], exts3D[2], exts3D[6]);

	for (int l = 0; l < 8; l++) delete exts3D[l];
	delete[] exts3D;

	double stepw = 2 * hfw / (row + 1), steph = 2 * hfw / (col + 1), steps = 2 * hfw / (stack + 1);
	bool isFirstParticle;
	for (int l = 0; l < row; l++)
		for (int j = 0; j < col; j++)
			for (int k = 0; k < stack; k++) {
				isFirstParticle = distR(rng) < rate;
				objs[walls_len + l * col * stack + j * stack + k] = new Particle3D(isFirstParticle ? this->pc1 : this->pc2, new Point3D((l - row / 2 + 0.5) * stepw, (j - col / 2 + 0.5) * steph, (k - stack / 2 + 0.5) * steps), isFirstParticle ? distM_1(rng) : distM_2(rng), isFirstParticle ? distM_1(rng) : distM_2(rng), isFirstParticle ? distM_1(rng) : distM_2(rng));
			}
	simulate();
}

Simulation3D::~Simulation3D() {

}