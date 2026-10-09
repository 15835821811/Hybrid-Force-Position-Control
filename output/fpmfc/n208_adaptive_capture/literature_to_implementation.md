# Literature to implementation, checked 2026-10-09

Material Passport: source/formula review for a new numerical implementation; no
claim of reproducing an entire paper, spacecraft hardware, camera, or gripper.

| Source | Verified usable content | N208 implementation / limitation |
|---|---|---|
| [R1 Wensing et al., IJRR 2024](https://journals.sagepub.com/doi/10.1177/02783649241258215) | Geometric observability and unobservable inertial directions | Data-only SVD after parameter scaling and row noise weighting; no regularization rank credit; finite task data cannot prove universal identifiability |
| [R2 Khorshidi et al., arXiv v2](https://arxiv.org/html/2409.09850v2) | Section IV defines origin inertia, first moment, 4x4 pseudo-inertia with scalar mass, physical triangle/support constraints | Nondegenerate dimensionless PSD constraint, COM box and necessary second-moment support bounds; tested constrained least squares. These simple bounds do not prove a complete cube-support moment problem. Legged-robot experiments are not space-capture evidence |
| [R3 Uchida et al., v1](https://arxiv.org/html/2512.21886v1) | Sections II/III discuss linear inertial regressors, rank and floating-system momentum | Use fixed-world-origin momentum differences; robot known-body sum excludes target. Does not assume zero initial H. HTML equation (7) repeats Sigma in the bottom-right cell, inconsistent with dimensions: implementation uses scalar m. Parameter order is explicitly remapped, not copied. Grasped-object identification does not validate pregrasp capture |
| [R4 Candan and Servadio](https://arxiv.org/html/2603.27361) | Attitude/motion and normalized inertia estimation in free rotation | Absolute mass/scale not observable without known interaction. N208 uses one SO(3) EKF, not a second UKF implementation |
| [R5 Ren and Shan, institutional record](https://pure.bit.edu.cn/en/publications/a-unified-framework-for-compliant-control-and-trajectory-planning/) | Public abstract supports coordination of approach planning and compliant control | Motivation only; no unverified equations or claim of complete reproduction |
| [R6 PRIME paper](https://arxiv.org/html/2605.17681v1), [RSS proceedings](https://www.roboticsproceedings.org/rss22/p029.html) | Contact, motion and inertia consistency; optimization-based estimation | N208 remains causal lightweight estimation; no claim of transplanting an offline long-window estimator into a real-time spacecraft loop |
| [R7 Jing et al.](https://www.sciopen.com/article/10.1016/j.cja.2024.06.029) | Identification uses electrostatic interaction | Not an available actuator. No electrostatic forcing added |
| [MuJoCo 3.3.2 computation](https://mujoco.readthedocs.io/en/3.3.2/computation/) | Generalized constraint force and free-body dynamics conventions | Reconstruct declared ideal interface F/T from target free-coordinate constraint force and point Jacobian; never use target qacc as estimator input. Forward copies confined to sensors/evaluation, control forward calls use independent prior model |

Frame contract: all p/v/w and wrench axes are world axes; B is the visible target
geometry origin, G is the fixed declared grasp site; O=(0,0,0) is fixed in world.
pi=[m,hx,hy,hz,IBxx,IByy,IBzz,IBxy,IBxz,IByz], h=m c. IB is about B;
IC=IB-m[(c.c)I-c c^T]. P=m vB+w cross (R h),
HO=(pB-O) cross P+(R h) cross vB+R IB R^T w. Wrench transfer is
MO=MG+(pG-O) cross F. Linear/angular momentum noise scales carry distinct units.

The independently computed COM formula is the regression oracle in selftests.
Physical constraint tests precede any parameter feedback. The interface integral
is a diagnostic cross-check of robot momentum differences, not a second independent
likelihood using the same motion data. No full-identification label follows merely
from optimizer convergence, constraints, small fitting residual, or task success.
