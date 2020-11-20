# Anytime-asymptotically-optimal-motion-planning

<h2> Project Descrpition </h2>
The goal of our project is to use an optimal motion planning algorithm to
move a mobile that is assigned a navigation task without taking into account
the dynamic constraints of this robot. The optimal motion planning algo-
rithm chosen is the anytime motion planning based on the RRT*, which is a
sampling-based algorithm with an asymptotic optimality property. Contrar-
ily to the basic RRT, it accounts for the quality of the generated solution
trajectories. On the other hand, it introduces some computational overhead
that is problematic when planning under time limitations. The anytime ver-
sion of RRT* addresses this issue by quickly computing an initial sub-optimal
solution and repeatedly improving it during the simultaneous execution. The
proposed motion planning algorithm was thoroughly evaluated in several sce-
narios of increasing complexity. We programmed this algorithm in C + +
and we tested it on the mobile robot Y oubot which is available in the V-rep
simulator.

> For more detail open the [report](report.pdf)
