#include "smokesolver.h"

//
//
// smokesolver class
// by Karl Paetow
//
// Created:     April 22, 2020
// Updated:     --
// Iteration:   1
//
// ABOUT
//
// This is an implementation of the basic 2D fluid simulation solver described in "Real-Time Fluid Dynamics for 
// Games" by Jos Stam (2001.)
//
// This is a strict implementionation of the content in the paper by Stam. It does not deviate from the methods
// described there. The intent is to provide a working 2D simulation which can then be used as a starting point
// for experimentation, such as altering the algorithms used to calculate the variables.
//
// Two variables are simulated:
// + (u,v) = velocity (vector field)
// + dens = smoke density (scalar field)
//
// Other simulation variables:
// + dt = the length of the time step (in sec.) - defaults to 1/24 sec (to match standard film frame rate)
// + diff = the diffusion rate - defaults to 0.00002074
// + visc = viscosity of the fluid - defaults to 0.00001825
//
// This solver is written in platform-agnostic C++ to enable portability to the extent possible, however it has
// not been tested on platforms other than Win x64.
//
//
// Iteration 1
// + Initial implementation of the fluid solver class, hard-coded for 2D
//




// Macros
#ifndef SWAP
#define SWAP(x0,x) { float* tmp = x0; x0 = x; x = tmp; }
#endif




// Set boundaries - this handles the simulation grid boundary conditions
// for the field passed as a parameter
void SmokeSolver::set_bnd(int gridx, int gridy, int b, float* x)
{
	int i, j;

	for (j = 1; j <= gridy; j++)
	{
		x[IX(0, j)] = b == 1 ? -x[IX(1, j)] : x[IX(1, j)];
		x[IX(gridx + 1, j)] = b == 1 ? -x[IX(gridx, j)] : x[IX(gridx, j)];
	}

	for (i = 1; i <= gridx; i++)
	{
		x[IX(i, 0)] = b == 2 ? -x[IX(i, 1)] : x[IX(1, 1)];
		x[IX(i, gridy + 1)] = b == 2 ? -x[IX(i, gridy)] : x[IX(i, gridy)];
	}

	x[IX(0, 0)] = 0.5f * (x[IX(1, 0)] + x[IX(0, 1)]);
	x[IX(0, gridy)] = 0.5f * (x[IX(1, gridy + 1)] + x[IX(0, gridy)]);
	x[IX(gridx + 1, 0)] = 0.5f * (x[IX(gridx, 0)] + x[IX(gridx + 1, 1)]);
	x[IX(gridx + 1, gridy + 1)] = 0.5f * (x[IX(gridx, gridy + 1)] + x[IX(gridx + 1, gridy)]);
}



// Dissipate (to prevent eternal accumulation)
void SmokeSolver::dissipate(int gridsize, float* x, float dt)
{
	int i;

	for (i = 0; i < gridsize; i++)
		x[i] *= 1.0f - dissip * dt;
}


// Add source (density or velocity - doesn't matter which - same implementation!)
void SmokeSolver::add_source(int gridsize, float* x, float* s, float dt)
{
	int i;

	for (i = 0; i < gridsize; i++)
		x[i] += dt * s[i];
}


// Diffuse (density or velocity - doesn't matter which - same implementation!)
void SmokeSolver::diffuse(int gridx, int gridy, int b, float* x, float* x0, float diff, float dt)
{
	int i, j, k;
	float a = dt * diff * gridx * gridy;

	// This calculation uses Gauss-Seidel relaxation for numerical stability
	for (k = 0; k < relax; k++)
	{
		for (j = 1; j <= gridy; j++)
		{
			for (i = 1; i <= gridx; i++)
			{
				x[IX(i, j)] = (
					x0[IX(i, j)] +
					a * (
						x[IX(i - 1, j)] +
						x[IX(i + 1, j)] +
						x[IX(i, j - 1)] +
						x[IX(i, j + 1)]
						)
					) / (1.0f + 4.0f * a);
			}
		}
		set_bnd(gridx, gridy, b, x);
	}
}


// Advect (density or velocity - doesn't matter which - same implementation!)
void SmokeSolver::advect(int gridx, int gridy, int b, float* d, float* d0, float* u, float* v, float dt)
{
	int i, j, i0, j0, i1, j1;
	float x, y, s0, t0, s1, t1, dtx, dty;

	dtx = dt * gridx;
	dty = dt * gridy;

	for (j = 1; j <= gridy; j++)
	{
		for (i = 1; i <= gridx; i++)
		{
			x = (float)i - (dtx * u[IX(i, j)]);
			y = (float)j - (dty * v[IX(i, j)]);

			if (x < 0.5f) x = 0.5f;
			if (x > (float)gridx + 0.5f)
				x = (float)gridx + 0.5f;
			i0 = (int)x;
			i1 = i0 + 1;

			if (y < 0.5f) y = 0.5f;
			if (y > (float)gridy + 0.5f)
				y = (float)gridy + 0.5f;
			j0 = (int)y;
			j1 = j0 + 1;

			s1 = x - (float)i0;
			s0 = 1.0f - s1;
			t1 = y - (float)j0;
			t0 = 1.0f - t1;

			d[IX(i, j)] =
				s0 * (
					t0 * d0[IX(i0, j0)] +
					t1 * d0[IX(i0, j1)]
					) +
				s1 * (
					t0 * d0[IX(i1, j0)] +
					t1 * d0[IX(i1, j1)]
					);
		}
	}

	set_bnd(gridx, gridy, b, d);
}






// Projection
// This is an important step for the velocity field that helps the fluid to 
// conserve mass, i.e. so that we do not see mass accumulate in some places and 
// less in others. This process introduces vortices.
void SmokeSolver::project(int gridx, int gridy, float* u, float* v, float* p, float* div)
{
	// TODO: Determine what variables div and p are for

	int i, j, k;
	float hx, hy;

	hx = 1.0f / (float)gridx;
	hy = 1.0f / (float)gridy;

	for (j = 1; j <= gridy; j++)
	{
		for (i = 1; i <= gridx; i++)
		{
			div[IX(i, j)] = (-0.5f * hx * (u[IX(i + 1, j)] - u[IX(i - 1, j)])) + (-0.5f * hy * (v[IX(i, j + 1)] - v[IX(i, j - 1)]));
			p[IX(i, j)] = 0.0f;
		}
	}
	set_bnd(gridx, gridy, 0, div);
	set_bnd(gridx, gridy, 0, p);

	// Gauss-Sidel relaxation used here
	// Refers to 'relax' variable which determines how many iterations to use
	for (k = 0; k < relax; k++)
	{
		for (j = 1; j <= gridy; j++)
		{
			for (i = 1; i <= gridx; i++)
			{
				p[IX(i, j)] = (
					div[IX(i, j)] +
					p[IX(i - 1, j)] +
					p[IX(i + 1, j)] +
					p[IX(i, j - 1)] +
					p[IX(i, j + 1)]
					) / 4.0f;
			}
		}
		set_bnd(gridx, gridy, 0, p);
	}

	for (j = 1; j <= gridy; j++)
	{
		for (i = 1; i <= gridx; i++)
		{
			u[IX(i, j)] -= 0.5f * (p[IX(i + 1, j)] - p[IX(i - 1, j)]) / hx;
			v[IX(i, j)] -= 0.5f * (p[IX(i, j + 1)] - p[IX(i, j - 1)]) / hy;
		}
	}
	set_bnd(gridx, gridy, 1, u);
	set_bnd(gridx, gridy, 2, v);
}




// Density step:
// 1) Adds new sources of smoke density
// 2) Diffuses the smoke density
// 3) Advects the smoke density along the velocity field vectors
void SmokeSolver::dens_step(int gridx, int gridy, float* x, float* x0, float* xs, float* u, float* v, float diff, float dt)
{
	dissipate(gridsize, dens, dt);
	add_source(gridsize, x, xs, dt);
	SWAP(x0, x);
	diffuse(gridx, gridy, 0, x, x0, diff, dt);
	SWAP(x0, x);
	advect(gridx, gridy, 0, x, x0, u, v, dt);
}


// Velocity step:
// 1) Adds any new velocity sources
// 2) Diffuses the velocity vectors
// 3) Advects the velocity field vectors along themselves
void SmokeSolver::vel_step(int gridx, int gridy, float* u, float* v, float* u0, float* v0, float* us, float* vs, float visc, float dt)
{
	dissipate(gridsize, u, dt);
	dissipate(gridsize, v, dt);
	add_source(gridsize, u, us, dt);
	add_source(gridsize, v, vs, dt);
	SWAP(u0, u);
	diffuse(gridx, gridy, 1, u, u0, visc, dt);
	SWAP(v0, v);
	diffuse(gridx, gridy, 2, v, v0, visc, dt);
	project(gridx, gridy, u, v, u0, v0);
	SWAP(u0, u);
	SWAP(v0, v);
	advect(gridx, gridy, 1, u, u0, u0, v0, dt);
	advect(gridx, gridy, 2, v, v0, u0, v0, dt);
	project(gridx, gridy, u, v, u0, v0);
}




//
//
// Array initializers
//
//

void SmokeSolver::init_dens(int gridsize)
{
	int i;

	for (i = 0; i < gridsize; i++)
		dens[i] = 0.0f;
}


void SmokeSolver::init_dens_prev(int gridsize)
{
	int i;

	for (i = 0; i < gridsize; i++)
		dens_prev[i] = 0.0f;
}


void SmokeSolver::init_dens_source(int gridsize)
{
	int i;

	for (i = 0; i < gridsize; i++)
		dens_source[i] = 0.0f;
}


void SmokeSolver::init_u(int gridsize)
{
	int i;

	for (i = 0; i < gridsize; i++)
		u[i] = 0.0f;
}


void SmokeSolver::init_u_prev(int gridsize)
{
	int i;

	for (i = 0; i < gridsize; i++)
		u_prev[i] = 0.0f;
}


void SmokeSolver::init_u_source(int gridsize)
{
	int i;

	for (i = 0; i < gridsize; i++)
		u_source[i] = 0.0f;
}


void SmokeSolver::init_v(int gridsize)
{
	int i;

	for (i = 0; i < gridsize; i++)
		v[i] = 0.0f;
}


void SmokeSolver::init_v_prev(int gridsize)
{
	int i;

	for (i = 0; i < gridsize; i++)
		v_prev[i] = 0.0f;
}


void SmokeSolver::init_v_source(int gridsize)
{
	int i;

	for (i = 0; i < gridsize; i++)
		v_source[i] = 0.0f;
}







// Constructor
SmokeSolver::SmokeSolver()
{
	// Dynamically allocate the variable arrays
	// (They cannot be statically declared due to the requirement to allow for grid size changes)
	u = new float[gridsize];
	init_u(gridsize);

	u_prev = new float[gridsize];
	init_u_prev(gridsize);

	v = new float[gridsize];
	init_v(gridsize);

	v_prev = new float[gridsize];
	init_v_prev(gridsize);

	dens = new float[gridsize];
	init_dens(gridsize);

	dens_prev = new float[gridsize];
	init_dens_prev(gridsize);

	dens_source = new float[gridsize];
	init_dens_source(gridsize);

	u_source = new float[gridsize];
	init_u_source(gridsize);

	v_source = new float[gridsize];
	init_v_source(gridsize);
}


// Destructor
SmokeSolver::~SmokeSolver()
{
	// Destroy the dynamically-allocated variable arrays to ensure clean shutdown
	delete u;
	delete u_prev;
	delete v;
	delete v_prev;
	delete dens;
	delete dens_prev;

	delete dens_source;
	delete u_source;
	delete v_source;
}





// Iteration step:
// Completes an iteration of the solver calculations
void SmokeSolver::iterate()
{
	// Calculate the velocity and advect it
	vel_step(gridx, gridy, u, v, u_prev, v_prev, u_source, v_source, visc, dt);

	// Calculate the density and advect it
	dens_step(gridx, gridy, dens, dens_prev, dens_source, u, v, diff, dt);

	// We leave the calling app to draw the density field
	// after this function ends and returns to the caller
}


//
//
// Clear source fields
//
//

// Clears the density source field (sets everything to zero)
void SmokeSolver::clear_dens_source()
{
	int i, j;

	for (j = 0; j < gridy; j++)
	{
		for (i = 0; i < gridx; i++)
		{
			dens_source[IX(i, j)] = 0.0f;
		}
	}
}


// Clears the u,v source fields (sets everything to zero)
void SmokeSolver::clear_vel_source()
{
	int i, j;

	for (j = 0; j < gridy; j++)
	{
		for (i = 0; i < gridx; i++)
		{
			u_source[IX(i, j)] = 0.0f;
			v_source[IX(i, j)] = 0.0f;
		}
	}
}





//
//
// Get / Set members
//
//


// relax variable - determines the number of steps for Gauss-Seidel relaxation
// Default value is 20 steps

// Gets the value of the relax variable
int SmokeSolver::get_relax()
{
	return relax;
}

// Sets the value of the relax variable
void SmokeSolver::set_relax(int new_relax)
{
	if (new_relax != NULL)
		relax = new_relax;
}




// dt variable - the length of the time step in sec.

// Gets the value of dt
float SmokeSolver::get_dt()
{
	return dt;
}

// Sets the value of dt
void SmokeSolver::set_dt(float new_dt)
{
	if (new_dt != NULL)
		dt = new_dt;
}



// diff variable - the diffusion rate (m2 / s)

// Gets the diffusion rate
float SmokeSolver::get_diff()
{
	return diff;
}

// Sets the diffusion rate
void SmokeSolver::set_diff(float new_diff)
{
	if (new_diff != NULL)
		diff = new_diff;
}



// visc variable - the viscosity of the fluid (in this case, air) (kg/m*s)

// Gets the viscosity
float SmokeSolver::get_visc()
{
	return visc;
}

// Sets the viscosity
void SmokeSolver::set_visc(float new_visc)
{
	if (new_visc != NULL)
		visc = new_visc;
}



// gridx - the width of the simulation grid (in cells)
// Currently cannot set the grid size

// Get the simulation grid size (x)
int SmokeSolver::get_gridx()
{
	return gridx;
}


// gridy - the height of the simulation grid (in cells)
// Currently cannot set the grid size

// Get the simulation grid size (y)
int SmokeSolver::get_gridy()
{
	return gridy;
}


// gridsize - the size of the simulation grid field arrays
// Currently cannot set the grid size

// Get the simulation grid size (applies to all field arrays)
int SmokeSolver::get_gridsize()
{
	return gridsize;
}


// dens - expose the density field as calculated by the simulation

// Get the density field
float* SmokeSolver::get_dens()
{
	return dens;
}



// dens_source - expose the density source field which is used to add density to the simulation

// Get the density source field
float* SmokeSolver::get_dens_source()
{
	return dens_source;
}



// u,v source - expose the velocity source field which is used to add velocity to the simulation
// NOTE: The u,v components are treated as separate fields to simplify implementation

// Get the u_source field
float* SmokeSolver::get_u_source()
{
	return u_source;
}

// Get the v_source field
float* SmokeSolver::get_v_source()
{
	return v_source;
}

