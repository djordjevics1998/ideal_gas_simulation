#include <string>
#ifndef H_GEOMETRY
#define H_GEOMETRY

#define NOT_COLLIDING -1
#define INSIDE_EACH_OTHER -2
#define UNKNOWN -3
enum TYPE {LINE_2D, PARTICLE_2D, TRIANGLE, PARTICLE_3D};

class Object;

class Event {
	public:
		Object* o1, *o2;
		long double t, dt;
		Event();
		Event(Object* o1, Object* o2, long double t, long double dt);
		static bool compare(Event* e1, Event* e2);
};

class Point2D {
	protected:
		long double x, y;
	
	public:
		Point2D(long double x, long double y);
		Point2D(Point2D *p);
		
		long double getX();
		long double getY();
		
		void set(long double x, long double y);
		void add(long double dx, long double dy);
		std::string toString();
		static long double distance(Point2D *p1, Point2D *p2);
};

class Point3D {
protected:
	long double x, y, z;

public:
	Point3D(long double x, long double y, long double z);
	Point3D(Point3D* p);

	long double getX();
	long double getY();
	long double getZ();

	void set(long double x, long double y, long double z);
	void add(long double dx, long double dy, long double dz);
	std::string toString();
	static long double distance(Point3D* p1, Point3D* p2);
};

class Object {
	protected:
		Event** events;
		int events_n;
	public:
		Object();
		void initEvents(int n);
		int getEventsLen();
		Event** getEvents();
		virtual void progress(long double t) = 0;
		static long double collision(Object *o1, Object *o2, long double act);
		static bool compare(Object* o1, Object* o2);
		virtual TYPE getType() = 0;
		virtual std::string toString() = 0;
		~Object();
};

class Line2D : public Object {
	protected:
		Point2D *p1, *p2;
		TYPE getType();
	
	public:
		void initEvents(int n);
		Event** getEvents();
		Line2D(Point2D *p1, Point2D *p2);
		Point2D * getFirstPoint();
		Point2D * getSecondPoint();
		void progress(long double t);
		std::string toString();
		~Line2D();
};

class Triangle : public Object {
protected:
	Point3D* p1, * p2, * p3;
	TYPE getType();

public:
	void initEvents(int n);
	Event** getEvents();
	Triangle(Point3D* p1, Point3D* p2, Point3D* p3);
	Point3D* getFirstPoint();
	Point3D* getSecondPoint();
	Point3D* getThirdPoint();
	void progress(long double t);
	std::string toString();
	~Triangle();
};

class Vector2D {
	protected:
		long double x, y;
	
	public:
		Vector2D(long double x, long double y);
		Vector2D(Point2D *p1, Point2D *p2);
		Vector2D(Point2D *p1, Point2D *p2, bool normalize);
		Vector2D(Vector2D *v);
		
		long double getX();
		long double getY();
		void set(long double x, long double y);
		long double len();
		void multiply(long double m);
		long double scalar(Vector2D *v);
		std::string toString();
		
		static Vector2D projection(Vector2D *v_orig, Vector2D *v_on);
		static Vector2D add(Vector2D *v1, Vector2D *v2);
		static Vector2D sub(Vector2D* from, Vector2D *what);
};

class Vector3D {
protected:
	long double x, y, z;

public:
	Vector3D(long double x, long double y, long double z);
	Vector3D(Point3D* p1, Point3D* p2);
	Vector3D(Point3D* p1, Point3D* p2, bool normalize);
	Vector3D(Vector3D* v);

	long double getX();
	long double getY();
	long double getZ();
	void set(long double x, long double y, long double z);
	long double len();
	void multiply(long double m);
	long double scalar(Vector3D* v);
	std::string toString();
	
	static Vector3D vector(Vector3D* v1, Vector3D* v2, bool normalize);
	static Vector3D projection(Vector3D* v_orig, Vector3D* v_on);
	static Vector3D add(Vector3D* v1, Vector3D* v2);
	static Vector3D sub(Vector3D* from, Vector3D* what);
};

class Particle2D : public Object {
	protected:
		Point2D *c;
		Vector2D *v;
		long double r, m;
		TYPE getType();
	
	public:
		void initEvents(int n);
		Event** getEvents();
		Particle2D(Point2D *c, long double r, long double m, long double vx, long double vy);
		Point2D * getCenter();
		Vector2D *getVelocity();
		long double getRadius();
		long double getMass();
		void progress(long double t);
		std::string toString();
		~Particle2D();
};

class Particle3D : public Object {
protected:
	Point3D* c;
	Vector3D* v;
	long double r, m;
	TYPE getType();

public:
	void initEvents(int n);
	Event** getEvents();
	Particle3D(Point3D* c, long double r, long double m, long double vx, long double vy, long double vz);
	Point3D* getCenter();
	Vector3D* getVelocity();
	long double getRadius();
	long double getMass();
	void progress(long double t);
	std::string toString();
	~Particle3D();
};

class IOnSimulationListener {
public:
	virtual void OnSimulationIteration(Object** objs, int objs_len) = 0;
	virtual void OnSimulationStep(long double pV) = 0;
};

class Simulation {
protected:
	int row, col, N, walls_len, objs_len;
	long long sim_step, sim_count;
	long double kB, T, hfw, r_1, r_2, m_1, m_2, Vs, rate;
	Object** objs;
	Event** all_events_p;
	IOnSimulationListener* listener;
	void simulate();

public:
	Simulation(long double kB, long double T, long double hfw, long double r_1, long double r_2, long double m_1, long double m_2, long double rate, long long sim_step, long long sim_count, int row, int col);
	void setOnSimulationListener(IOnSimulationListener* listener);
	virtual void run() = 0;
	~Simulation();
};

class Simulation2D : Simulation {

public:
	Simulation2D(long double kB, long double T, long double hfw, long double r_1, long double r_2, long double m_1, long double m_2, long double rate, long long sim_step, long long sim_count, int row, int col);
	void setOnSimulationListener(IOnSimulationListener* listener);
	void run();
	~Simulation2D();
};

class Simulation3D : Simulation {
protected:
	int stack;

public:
	Simulation3D(long double kB, long double T, long double hfw, long double r_1, long double r_2, long double m_1, long double m_2, long double rate, long long sim_step, long long sim_count, int row, int col, int stack);
	void setOnSimulationListener(IOnSimulationListener* listener);
	void run();
	~Simulation3D();
};

#endif
