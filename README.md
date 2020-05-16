# smoke-sim-1

This is an attempt to build a Windows 10 native implementation of the basic fluid simulation described in _Real-Time Fluid Dynamics for Games_ written by Jos Stam (Alias | Wavefront) in 2001. (See PDF included in this repo.)

It is a 64-bit Direct2D implementation written in C++ which adheres strictly to the article's content in that no attempt is made to deviate from the algorithms provided in the article.

The objective is to get a working implementation of a fluid sim together, based on the simple, but stable, methods presented by Stam. From this, other versions can later be derived for the purposes of experimentation.

There are several advantages to this implementation, as it:
+ Provides a good introduction to fluid simulations in general
+ Is a fast and lightweight implementation
+ Is a simple fluid simulation, being constrained to 2D and to 2 variables (velocity field and smoke density)
+ Provides immediate visual feedback to the user as its output (vs large, specialized dataset files)
+ Allows the user to interact with the simulation and affect its outcome

Disadvantages of this implementation:
+ It is a native Windows desktop application which can't be run in a browser
+ It targets Direct2D which sacrifices portability to other platforms
+ It's a very basic fluid simulation, constrained to 2 variables (smoke density and velocity)
+ It's limited to 2D
+ Uses a fixed simulation grid size that can't be changed interactively to simplify implementation
+ The application is single-threaded (which requires processing both the application's Windows message loop as well as iterating the simulation in the same thread)
