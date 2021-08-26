#include "geometry.h"
#include <iostream>
#include <fstream>

/* run this program using the console pauser or add your own getch, system("pause") or input loop */

using namespace std;

class ICustomOnSimulationListener : public IOnSimulationListener {
protected:
	std::ofstream myfile;

public:
	ICustomOnSimulationListener() {
		myfile.open("pv_3d_1728.txt");
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
		std::cout << sim_step << ". " << pV << " " << NkBT << std::endl;
	}

	void OnSimulationEnd(PhObject** objs, int objs_len) {

	}

	~ICustomOnSimulationListener() {
		myfile.close();
	}
};

int main(int argc, char** argv) {
	const double kB = 1.3806503e-23, T = 273 + 30,
		hfw = 1e-4, r_1 = 1e-7, r_2 = 5e-6, m_1 = 1, m_2 = 2;
	const int row = 4, col = 4, stack = 12;
	const long long sim_step = 100, sim_count = 1000;
	ParticleConfig pc1(0, r_1, m_1),
		pc2(1, r_2, m_2);
	Simulation2D sim2d(kB, T, hfw, &pc1, &pc2, 0.5, sim_step, sim_count, row, col);
	ICustomOnSimulationListener* listener = new ICustomOnSimulationListener();
	sim2d.setOnSimulationListener(listener);
	sim2d.run();
	/*Simulation3D sim3d(kB, T, hfw, &pc1, &pc2, 0.5, sim_step, sim_count, row, col, stack);
	ICustomOnSimulationListener* listener = new ICustomOnSimulationListener();
	sim3d.setOnSimulationListener(listener);
	sim3d.run();*/
	
	return 0;
}
