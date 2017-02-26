import math
import ode

rotor_displacements = [
  (0.5, 0.0, -0.5),
  (0.5, 0.0, 0.5),
  (-0.5, 0.0, 0.5),
  (-0.5, 0.0, -0.5),
]

def clamp_duty_cycle(x):
  if x < 0.0:
    return 0.0
  if x > 1.0:
    return 1.0
  return x

def differentiate(old, new, dt):
  return (
    (new[0] - old[0]) / dt,
    (new[1] - old[1]) / dt,
    (new[2] - old[2]) / dt,
  )

def quaternion_rotate(q, v):
  """Apply the rotation specified by q to the vector v."""
  qr, qi, qj, qk = q
  vx, vy, vz = v
  return (
    (1-2*qj*qj-2*qk*qk)*vx + 2*(qi*qj-qk*qr)*vy + 2*(qi*qk+qj*qr)*vz,
    2*(qi*qj+qk*qr)*vx + (1-2*qi*qi-2*qk*qk)*vy + 2*(qj*qk-qi*qr)*vz,
    2*(qi*qk-qj*qr)*vx + 2*(qj*qk+qi*qr)*vy + (1-2*qi*qi-2*qj*qj)*vz,
  )

def quaternion_roll(q):
  """Get the roll of a quaternion."""
  qr, qi, qj, qk = q
  return -math.atan2(2*(qj*qk-qi*qr),1-2*(qi*qi+qj*qj))

def quaternion_pitch(q):
  """Get the pitch of a quaternion."""
  qr, qi, qj, qk = q
  return -math.atan2(2*(qi*qj+qk*qr),1-2*(qj*qj+qk*qk))

def quaternion_yaw(q):
  """Get the yaw of a quaternion."""
  qr, qi, qj, qk = q
  return -math.atan2(2*(qi*qk-qj*qr),1-2*(qj*qj+qk*qk))

class SensorReadings(object):
  def __init__(self, accel, gyro, magneto, interval):
    self.accel = accel
    self.gyro = gyro
    self.magneto = magneto
    self.interval = interval

class Differentiator(object):
  def __init__(self):
    self.prev = 0.0

  def update(self, x, dt):
    deriv = (x - self.prev) / dt
    self.prev = x
    return deriv

class Integrator(object):
  def __init__(self):
    self.integral = 0.0

  def clamp(self, min, max):
    if self.integral < min:
      self.integral = min
    if self.integral > max:
      self.integral = max

  def update(self, x, dt, min=float("-inf"), max=float("inf")):
    self.integral += x * dt
    return self.integral

class Trajectory(object):
  def __init__(self):
    pass

  def xvel(self, t):
    return 0.0

  def yvel(self, t):
    return 1.0 if t > 2.0 and t < 3.0 else 0.0

  def zvel(self, t):
    return 0.0

  def yawvel(self, t):
    return 0.0

class Controller(object):
  def __init__(self, trajectory):
    self.trajectory = trajectory
    self.time = 0.0

    self.xvel_i = Integrator()
    self.yvel_i = Integrator()

    self.xvel_error_d = Differentiator()
    self.xvel_error_dd = Differentiator()
    self.xvel_error_ddd = Differentiator()
    self.yvel_error_i = Integrator()

  def update(self, readings, dt, drone):
    """Takes sensor readings and returns PWM duty cycle applied to each rotor"""

    quaternion = drone.getQuaternion()
    accel = quaternion_rotate(quaternion, readings.accel)

    xfactor = 0.0

    yaccel_measured = accel[1]
    yvel_measured = self.yvel_i.update(yaccel_measured, dt)
    yvel_desired = self.trajectory.yvel(self.time)
    yvel_error = yvel_desired - yvel_measured
    yvel_error_i = self.yvel_error_i.update(yvel_error, dt, min=0.0, max=1/100.0)
    yfactor = 20.0*yvel_error + 100.0*yvel_error_i

    zfactor = 0.0

    yawfactor = 0.0

    duty_cycles = map(clamp_duty_cycle, [
      zfactor - xfactor + yawfactor + yfactor,
      -zfactor - xfactor - yawfactor + yfactor,
      -zfactor + xfactor + yawfactor + yfactor,
      zfactor + xfactor - yawfactor + yfactor,
    ])

    self.time += dt

    return tuple(duty_cycles)

def main():
  world = ode.World()
  world.setGravity((0.0, -9.81, 0.0))

  drone = ode.Body(world)
  drone_mass = ode.Mass()
  drone_mass.setSphere(150, 0.01) # model as a sphere with a radius of 10cm
  drone_mass.adjust(0.150) # adjust mass to be exactly 150g
  drone.setMass(drone_mass)

  drone.setPosition((0.0, 0.0, 0.0))
  #drone.setQuaternion((0.8, 0.0, 0.0, -0.2))

  controller = Controller(Trajectory())

  imax = 60
  jmax = 100
  dt = 1e-3
  prev_vel = (0.0, 0.0, 0.0)
  rotor_duty_cycles = (0.0, 0.0, 0.0, 0.0)
  for i in range(imax):
    pos = drone.getPosition()
    vel = drone.getLinearVel()
    quaternion = drone.getQuaternion()
    forward = drone.vectorToWorld((1.0, 0.0, 0.0))
    up = drone.vectorToWorld((0.0, 1.0, 0.0))
    avel = drone.getAngularVel()
    roll = quaternion_roll(quaternion)
    pitch = quaternion_pitch(quaternion)
    yaw = quaternion_yaw(quaternion)
    print "t=%7.4f x=(%7.3f,%7.3f,%7.3f) v=(%7.3f,%7.3f,%7.3f) roll=%7.3f pitch=%7.3f yaw=%7.3f av=(%7.3f,%7.3f,%7.3f) out=(%.3f,%.3f,%.3f,%.3f)" % (dt*i*jmax, pos[0], pos[1], pos[2], vel[0], vel[1], vel[2], roll, pitch, yaw, avel[0], avel[1], avel[2], rotor_duty_cycles[0], rotor_duty_cycles[1], rotor_duty_cycles[2], rotor_duty_cycles[3])

    for j in range(jmax):
      vel = drone.getLinearVel()
      avel = drone.getAngularVel()

      # Estimate the acceleration and angular acceleration.
      accel = differentiate(prev_vel, vel, dt)
      prev_vel = vel

      # Take sensor readings.
      magneto = drone.vectorFromWorld((1.0, 0.0, 0.0))
      readings = SensorReadings(drone.vectorFromWorld(accel), drone.vectorFromWorld(avel), magneto, dt)

      # Run control algorithm.
      rotor_duty_cycles = controller.update(readings, dt, drone)

      # Apply rotor thrust to drone.
      for duty_cycle, displacement in zip(rotor_duty_cycles, rotor_displacements):
        # Assume thrust is proportional to duty cycle
        thrust = duty_cycle
        drone.addRelForceAtRelPos((0.0, thrust, 0.0), displacement)

      # Apply drag to drone.
      drag_factor = -0.1
      drone.addForce((vel[0]*drag_factor, vel[1]*drag_factor, vel[2]*drag_factor))

      world.step(dt)

if __name__ == "__main__":
  main()
