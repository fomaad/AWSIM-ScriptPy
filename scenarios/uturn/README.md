## U-turn Scenarios

This folder contains the specification in AWSIM-ScriptPy for U-turn scenarios from the [JAMA Standard](https://www.jama.or.jp/english/reports/docs/Automated_Driving_Safety_Evaluation_Framework_Ver3.0.pdf).
The following figure (from the JAMA Standard) illustrates an example of a U-turn scenario.

<img src="../../assets/fig-uturn.png" alt="drawing" width="240"/>

The ego vehicle (in blue) is traveling straight in its lane at a constant speed of $ve$.
An oncoming NPC vehicle (in green) approaches from the opposite direction at a constant speed of $vo$.
At the moment when the longitudinal distance between the two vehicles is $dx_0$, the NPC vehicle begins to make a U-turn maneuver.
The relevant parameters for this scenario are as follows:
- $ve$: Ego vehicle speed (m/s)
- $vo$: Oncoming vehicle speed (m/s)
- $dx_0$: Longitudinal distance between the two vehicles (m)
- $dy_0$: Lateral displacement between two vehicles before the U-turn (m). However, in the scenario implementation, we consider only two concrete values for $dy_0$ corresponding to two cases: (1) when the ego vehicle is on the rightmost lane and (2) when it is on the adjacent lane to the rightmost lane.

For each parameter setting of $ve$, $vo$, $dx_0$, and $dy_0$, based on the JAMA's good driver model, we can determine whether a collision can be avoided through braking alone or if a collision is unavoidable for an ideal ADS.
Readers are referred to the JAMA standard (Section 2.3.3.1) for more details.

