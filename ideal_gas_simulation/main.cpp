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

	void OnSimulationIteration(Object** objs, int objs_len, int sim_ite) {

	}
	void OnSimulationStep(double pV, double NkBT, int sim_step) {
		myfile << pV << std::endl;
		std::cout << sim_step << ". " << pV << " " << NkBT << std::endl;
	}

	~ICustomOnSimulationListener() {
		myfile.close();
	}
};

int main(int argc, char** argv) {
	const double kB = 1.3806503e-23, T = 273 + 30,
		hfw = 1e5, r_1 = 1e-7, r_2 = 5e-6, m_1 = 1, m_2 = 2;
	const int row = 12, col = 12, stack = 12;
	const long long sim_step = 1e2, sim_count = 1e3;
	Simulation3D sim3d(kB, T, hfw, r_1, r_2, m_1, m_2, 0.5, sim_step, sim_count, row, col, stack);
	ICustomOnSimulationListener* listener = new ICustomOnSimulationListener();
	sim3d.setOnSimulationListener(listener);
	sim3d.run();
	Simulation2D sim2d(kB, T, hfw, r_1, r_2, m_1, m_2, 0.5, sim_step, sim_count, row, col);
	sim2d.run();
	
	return 0;
}
