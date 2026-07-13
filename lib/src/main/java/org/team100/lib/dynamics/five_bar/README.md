# Five-Bar Dynamics

Dynamic analysis of the five-bar parallel linkage is somewhat more involved
than the serial linkages covered elsewhere.

Following Khalil 2020, for now, ignore the end mass in the real apparatus,
and also ignore the operator force in the paper.

## Mass

The joints in Khalil are

* $q_1$: one of the actuators
* $q_2$: an elbow
* $q_3$: the other elbow
* $q_4$: the other actuator

All these angles are measured from the global x axis -- the "elbow" angles are not
measured relative to their parent links.

the middle joint is not modeled.

The masses of the links are $m_A$, $m_B$, $m_C$, and $m_D$.

The lengths are $l_1$, $l_2$, $l_3$, and $l_4$.

Using these coordinates, Khalil claims this mass matrix:

```math
M = 
\begin{bmatrix}
(\frac{m_A}{3} + m_B)l_1^2 & \frac{m_Bl_1l_2}{4}cos(q_1-q_2) & 0 & 0 \\
\frac{m_Bl_1l_2}{4}cos(q_1-q_2) & \frac{m_Bl_2^2}{3} & 0 & 0 \\
0 & 0 & \frac{m_Cl_3^2}{3} & \frac{m_Cl_3l_4}{4}cos(q_3-q_4) \\
0 & 0 & \frac{m_Cl_3l_4}{4}cos(q_3-q_4) & (\frac{m_D}{3}+m_C)l_4^2 \\
\end{bmatrix}
```


## Centrifugal and Coriolis

Khalil claims this C matrix:

```math
C = 
\begin{bmatrix}
\frac{m_Bl_1l_2}{4}\dot{q_2}sin(q_1-q_2)&0&0&0\\
0&0&0&0\\
0&0&0&0\\
0&0&0&0\\
\end{bmatrix}
```

## Gravity

For a horizontal mechanism, there's no gravity effect.

TODO: the vertical case.

## Equation of Motion

Start with the definition:

```math
\tau = M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q)
```

TODO: finish this


## References

* [Khalil 2007](https://www.researchgate.net/publication/29602307_General_Solution_for_the_Dynamic_Modeling_of_Parallel_Robots)
* [Khalil 2020](https://www.scribd.com/document/741490236/geomatrix-modeling-5bar-robot)
* [Saeed 2021](https://www.researchgate.net/publication/349813240_Modeling_and_Control_of_a_Pantograph_Mechanism_Interacting_with_the_Environment)
* [Sen 2022](https://www.mdpi.com/2075-1702/10/11/971)
