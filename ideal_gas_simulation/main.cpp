#include "geometry.h"
#include <sstream>
#include <iostream>
#include <fstream>
#include <Windows.h>

/* run this program using the console pauser or add your own getch, system("pause") or input loop */

using namespace std;

class ICustomOnSimulationListener : public IOnSimulationListener {
protected:
	std::ofstream myfile;
	std::string prefix, name;

public:
	ICustomOnSimulationListener(std::string prefix, std::string name) {
		this->prefix = prefix;
		this->name = name;
		system(("mkdir " + prefix).c_str());
		myfile.open(prefix + "/" + name);
	}

	void OnSimulationStart(PhObject** objs, int objs_len) {
	}

	void OnSimulationIteration(PhObject** objs, int objs_len, int sim_ite) {
		/*if (sim_ite % 100 == 0) {
			for (int l = 0; l < objs_len; l++) cout << sim_ite << " " << l << " " << objs[l]->toString() << endl;
		}*/
	}

	void OnSimulationStep(double pV, double NkBT, int sim_step) {
		myfile << pV << std::endl;
		//std::cout << sim_step << ". " << pV << " " << NkBT << std::endl;
	}

	void OnSimulationEnd(PhObject** objs, int objs_len) {
		myfile.close();
		myfile.open(prefix + "/" + name + "_intensities.txt");
		for (int l = 0; l < objs_len; l++)
			if (objs[l]->getType() == PARTICLE_2D) myfile << (static_cast<Particle2D*>(objs[l]))->getVelocity()->len() << std::endl;
	}

	~ICustomOnSimulationListener() {
		myfile.close();
	}
};

string toStringScientific(double t) {

	std::ostringstream ss;
	ss << std::scientific << t;
	return ss.str();
}

int main(int argc, char** argv) {
	const double kB = 1.3806503e-23, T = 273 + 30,
		hfw = 1e5, r_1 = 1e-10, r_2 = 5e-6, m_1 = 1, m_2 = 2;
	const int row = 4, col = 4, stack = 12;
	const long long sim_step = 50, sim_count = 1000;
	ParticleConfig pc1(0, r_1, m_1),
		pc2(1, r_2, m_2);
	int rows[] = /*{1, 2, 3, 4, 5}; */ {1, 2, 3, 4, 5, 7, 10, 12, 15, 17, 20, 23, 25, 30, 35, 40};
	string prefix, name;
	for (double hfw = 1e-4; hfw <= 1e7; hfw *= 10) {
		for (int l = 0; l < 9 /*16*/; l++) {
			/*string name = "pv_";
			name += (is3D ? "3" : "2");
			name += "d_" + std::to_string(N) + ".txt";*/
			prefix = toStringScientific(hfw) + "_" + toStringScientific(r_1) + "_" + toStringScientific(m_1);// +"/";
			name = std::to_string(rows[l] * rows[l]) + ".txt";
			std::cout << "Pocetak simulacije " << prefix << " N = " << rows[l] * rows[l] << std::endl;
			Simulation2D sim2d(kB, T, hfw, &pc1, &pc2, 1.1, sim_step, sim_count, 0, rows[l] * rows[l], rows[l], rows[l]);
			ICustomOnSimulationListener* listener = new ICustomOnSimulationListener(prefix, name);
			sim2d.setOnSimulationListener(listener);
			sim2d.run();

			std::cout << std::endl;
		}
	}
	std::cout << "Zavrseno!" << std::endl;
	/*Simulation2D sim2d(kB, T, hfw, &pc1, &pc2, 1, sim_step, sim_count, row, col);
	ICustomOnSimulationListener* listener = new ICustomOnSimulationListener();
	sim2d.setOnSimulationListener(listener);
	sim2d.run();*/
	/*Simulation3D sim3d(kB, T, hfw, &pc1, &pc2, 0.5, sim_step, sim_count, row, col, stack);
	ICustomOnSimulationListener* listener = new ICustomOnSimulationListener();
	sim3d.setOnSimulationListener(listener);
	sim3d.run();*/
	
	return 0;
}
