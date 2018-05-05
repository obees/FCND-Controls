# Rotation matrix
- To transform between body-frame and world frame we can use

euler2RM(roll,pitch,yaw)


# Testing
## Control parameter tuning

T_rise = time to get to the desired value in seconds
T_rise = 1,57 * T

T = Period in seconds
W_n = natural angular speed in rad / second. Choose a large W_n

T = 1 / W_n

0.7 < delta < 1.0

k_p = W_n^2

k_d = 2 * delta * W_n

- Utiliser le paramétrage Wn et ... pour vérifier les valeurs k_p / k_d
- Isoler les controleurs pour les tester individuellement
- Identifier les k_p et k_d qui permettent d'optimiser à la fois la vitesse de réaction sans trop de sur compensation

- Traiter les controleurs dans cet ordre:
- altitude_control()
- yaw_control()
- body_rate_control()
- roll_pitch_control()

- The altitude/yaw should be stabilized and the roll/pitch will slowly drift to unstable (instead of immediately flipping over).
- Next, the roll_pitch_control() can be tested by passing in zero acceleration commands.
- This should stabilize the drone's orientation completely.
- Tuning a faster and smoother inner loop will make tuning the outer loop easier due to less decoupling of the modes.

- For each step, you may also find it is easier to see the effects of your controller,
- so consider replacing the target local position (in the waypoint transition function) with the following:

- replace this

self.local_position_target = np.array((self.target_position[0], self.target_position[1], self.target_position[2]))

- with this

self.local_position_target = np.array([0.0, 0.0, -3.0])

- This will have your controller attempt to hold at a fixed point just about the origin.

Dans le tout digital, des apps permettront de piloter l'ensemble de ses activités et interactions avec son ecoysteme.
L'optimisation de la valeur business de ces apps s'appuiera sur la data (data-driven / ai driven)
Comme la techno, la data fait partie de l'ADN des apps qui permet de fluidifier les itérations de création de valeur produit



Piloiter l'ensemble de ses activités et inrecations avec son ecoysteme avec des apps digitals et d'optimiser la valeur business de ces apps par la data (data-driven / ai driven) .
En mode agile, comme la technologie, la data fait partie du produit (par le biais d'indicateurs de décision) ce qui fluidifie les itérations de création de valeur produit.


Here are my gains, do they look about the right magnitude?

# Position control gains
kpPosXY = 34.7
kpPosZ  = 32
KiPosZ  = 19

# Velocity control gains
kpVelXY = 14.4
kpVelZ  = 8

# Angle control gains
kpBank  = 10.5
kpYaw   = 2.9

# Angle rate gains
kpPQR  = 80,80,1.3
