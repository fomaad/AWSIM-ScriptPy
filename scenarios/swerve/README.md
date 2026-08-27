## Swerve Scenarios

This folder contains the specification in AWSIM-ScriptPy for swerve scenarios from the [JAMA Standard](https://www.jama.or.jp/english/reports/docs/Automated_Driving_Safety_Evaluation_Framework_Ver3.0.pdf).
The following figure (from the JAMA Standard) illustrates an example of a swerve scenario.

![Swerve Scenario](../../assets/fig-swerve.png)

The ego vehicle (in blue) is traveling straight in its lane at a constant speed of $ve$.
An oncoming NPC vehicle (in orange) approaches from the opposite direction at a constant speed of $vo$.
At the moment when the longitudinal distance between the two vehicles is $dx_0$, the NPC vehicle begins to temporarily swerve out of its lane to avoid an obstacle, moving laterally by $ny$ with a lateral velocity of $vy$.
The NPC's trajectory is represented by four waypoints $P_0$, $P_1$, $P_2$, and $P_3$, corresponding to its front-center point at four key stages of the maneuver, as shown in the figure. 

For each parameter setting of $ve$, $vo$, etc., based on the JAMA's good driver model, we can determine whether a collision can be avoided through braking alone or if a collision is unavoidable for an ideal ADS.
Readers are referred to the JAMA standard (Section 2.3.3.1) for more details.
