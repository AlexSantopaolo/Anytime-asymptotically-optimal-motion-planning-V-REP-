# Anytime-asymptotically-optimal-motion-planning

Developed by: A. Santopaolo (2018).

Supervisor: prof. G. Oriolo.

Achievement: Autonomous and mobile robotics exam.


<h3> Abstract </h3>

The goal of our project is to use an optimal motion planning algorithm to move a mobile that is assigned a navigation task without taking into account the dynamic constraints of this robot. The optimal motion planning algorithm chosen is the anytime motion planning based on the RRT*, which is a sampling-based algorithm with an asymptotic optimality property. Contrarily to the basic RRT, it accounts for the quality of the generated solution trajectories. On the other hand, it introduces some computational overhead that is problematic when planning under time limitations. The anytime version of RRT* addresses this issue by quickly computing an initial sub-optimal solution and repeatedly improving it during the simultaneous execution. The proposed motion planning algorithm was thoroughly evaluated in several scenarios of increasing complexity. The algorithm is in C + + and simulation environment is V-Rep. The robot on which the algorithm is tested is YouBot.


> For more details about the implementation open the [Report](Report.pdf)

<h3>Implementation Details</h3>

The algorithm has been written in C + +. The simulation environment is V-Rep. The robot on which the algorithm is tested is YouBot. The code is available. It's suggested to run in Linux environment.

<h3> *Referecenes* </h3>

1. Paolo Ferrari, Marco Cognetti, and Giuseppe Oriolo. Anytime wholebody planning/replanning for humanoid robots. 2018 IEEE-RAS 18th International Conference on Humanoid Robots (Humanoids), 2018.
1. Sertac Karaman and Emilio Frazzoli. Sampling-based algorithms for optimal motion planning. The international journal of robotics research, 30(7):846{894, 2011.
1. Sertac Karaman and Emilio Frazzoli. Incremental sampling-based algorithms for optimal motion planning. Robotics Science and Systems VI, 104:2, 2010.
