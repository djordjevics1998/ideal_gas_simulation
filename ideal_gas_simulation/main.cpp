#include "geometry.h"

/* run this program using the console pauser or add your own getch, system("pause") or input loop */

using namespace std;

int main(int argc, char** argv) {
	const long double kB = 1.3806503e-23, T = 273 + 30,
		hfw = 1e5, r_1 = 1e-7, r_2 = 5e-6, m_1 = 1, m_2 = 2;
	const int row = 12, col = 12, stack = 12;
	const long long sim_step = 1e2, sim_count = 1e3;
	Simulation3D sim3d(kB, T, hfw, r_1, r_2, m_1, m_2, 0.5, sim_step, sim_count, row, col, stack);
	sim3d.setOnSimulationListener(nullptr);
	sim3d.run();
	Simulation2D sim2d(kB, T, hfw, r_1, r_2, m_1, m_2, 0.5, sim_step, sim_count, row, col);
	sim2d.run();
	
	return 0;
}
