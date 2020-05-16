#ifndef SOLVER_INCL
#define SOLVER_INCL

// Windows Header Files:
#include <windows.h>

// C RunTime Header Files:
#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
#include <wchar.h>
#include <math.h>




//
// Macros
//

// IX macro (calculates index into the given array)
#ifndef IX
#define IX(i,j) ((gridx+2)*(j) + (i))
#endif




class SmokeSolver
{

private:

	// Simulation constants
	// Physical constants for air obtained from https://www.me.psu.edu/cimbala/me433/Links/Table_A_9_CC_Properties_of_Air.pdf
	int relax = 70;						// Number of iterations for Gauss-Seidel relaxation
	float dt = 0.0416667f;				// Default set to 1/24 of a second, equivalent to standard film frame rate
	float diff = 0.00002074f;			// Density diffusion rate (based on thermal diffusivity @ 20C)
	float visc = 0.00001825f;			// Viscosity of the fluid being simulated, in this case air @ 20C
	float dissip = 0.05f;				// Arbitrary rate of dissipation for the density field (to prevent endless accumulation)

	// Simulation grid dimension defaults (if not otherwise specified)
	int gridx = 150;					// Size of the simulation grid in x
	int gridy = 150;					// Size of the simulation grid in y
	int gridsize = (gridx + 2) * (gridy + 2);

	// Simulation variables
	//float u[gridsize] = { };
	//float u_prev[gridsize] = { };
	//float v[gridsize] = { };
	//float v_prev[gridsize] = { };
	//float dens[gridsize] = { };
	//float dens_prev[gridsize] = { };
	float* u;
	float* u_prev;
	float* v;
	float* v_prev;
	float* dens;
	float* dens_prev;
	float* dens_source;
	float* u_source;
	float* v_source;



	// Set boundaries - this handles the simulation grid boundary conditions
	// for the field passed as a parameter
	void set_bnd(int gridx, int gridy, int b, float* x);


	// Dissipate (to prevent eternal accumulation)
	void dissipate(int gridsize, float* x, float dt);

	
	// Add source (density or velocity - doesn't matter which - same implementation!)
	void add_source(int gridsize, float* x, float* s, float dt);


	// Diffuse (density or velocity - doesn't matter which - same implementation!)
	void diffuse(int gridx, int gridy, int b, float* x, float* x0, float diff, float dt);


	// Advect (density or velocity - doesn't matter which - same implementation!)
	void advect(int gridx, int gridy, int b, float* d, float* d0, float* u, float* v, float dt);






	// Projection
	// This is an important step for the velocity field that helps the fluid to 
	// conserve mass, i.e. so that we do not see mass accumulate in some places and 
	// less in others. This process introduces vortices.
	void project(int gridx, int gridy, float* u, float* v, float* p, float* div);




	// Density step:
	// 1) Adds new sources of smoke density
	// 2) Diffuses the smoke density
	// 3) Advects the smoke density along the velocity field vectors
	void dens_step(int gridx, int gridy, float* x, float* x0, float* xs, float* u, float* v, float diff, float dt);


	// Velocity step:
	// 1) Adds any new velocity sources
	// 2) Diffuses the velocity vectors
	// 3) Advects the velocity field vectors along themselves
	void vel_step(int gridx, int gridy, float* u, float* v, float* u0, float* v0, float* us, float* vs, float visc, float dt);


	//
	// Array initializers
	//
	void init_dens(int gridsize);
	void init_dens_prev(int gridsize);
	void init_dens_source(int gridsize);
	void init_u(int gridsize);
	void init_u_prev(int gridsize);
	void init_u_source(int gridsize);
	void init_v(int gridsize);
	void init_v_prev(int gridsize);
	void init_v_source(int gridsize);





public:

	// Constructor
	SmokeSolver();

	// Destructor
	~SmokeSolver();



	// Performs a single iteration of the solver
	// This is initiated by the calling app
	//
	// NOTE: The calling app is expected to take the
	// density field resulting from this step and
	// render it how it sees fit
	//
	// The calling app can get access to the density
	// field by calling get_dens() which returns
	// a pointer to the float array
	void iterate();





	//
	//
	// Clear source fields
	//
	//

	// Clears the density source field (sets everything to zero)
	void clear_dens_source();



	// Clears the u,v source fields (sets everything to zero)
	void clear_vel_source();






	//
	//
	// Get / Set members
	//
	//


	// relax variable - determines the number of steps for Gauss-Seidel relaxation
	// Default value is 20 steps

	// Gets the value of the relax variable
	int get_relax();


	// Sets the value of the relax variable
	void set_relax(int new_relax);





	// dt variable - the length of the time step in sec.

	// Gets the value of dt
	float get_dt();


	// Sets the value of dt
	void set_dt(float new_dt);




	// diff variable - the diffusion rate (m2 / s)

	// Gets the diffusion rate
	float get_diff();


	// Sets the diffusion rate
	void set_diff(float new_diff);




	// visc variable - the viscosity of the fluid (in this case, air) (kg/m*s)

	// Gets the viscosity
	float get_visc();


	// Sets the viscosity
	void set_visc(float new_visc);




	// gridx - the width of the simulation grid (in cells)
	// Currently cannot set the grid size

	// Get the simulation grid size (x)
	int get_gridx();



	// gridy - the height of the simulation grid (in cells)
	// Currently cannot set the grid size

	// Get the simulation grid size (y)
	int get_gridy();



	// gridsize - the size of the simulation grid arrays
	// Currently cannot set the grid size

	// Get the simulation grid array size (applies to all field arrays)
	int get_gridsize();




	// dens - expose the density field as calculated by the simulation

	// Get the density field
	float* get_dens();




	// dens_source - expose the density source field which is used to add density to the simulation

	// Get the density source field
	float* get_dens_source();




	// u,v source - expose the velocity source field which is used to add velocity to the simulation
	// NOTE: The u,v components are treated as separate fields to simplify implementation

	// Get the u_source field
	float* get_u_source();


	// Get the v_source field
	float* get_v_source();


};

#endif