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
  def __init__(self, initial=0.0):
    self.prev = initial

  def update(self, x, dt):
    deriv = (x - self.prev) / dt
    self.prev = x
    return deriv

class VectorDifferentiator(object):
  def __init__(self):
    self.prev = (0.0, 0.0, 0.0)

  def update(self, x, dt):
    deriv = (
      (x[0] - self.prev[0]) / dt,
      (x[1] - self.prev[1]) / dt,
      (x[2] - self.prev[2]) / dt,
    )
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

class PID(object):
  def __init__(self, kp, ki, kd):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.integrator = Integrator()
    self.differentiator = Differentiator()

  def update(self, desired, measured, dt):
    error = desired - measured
    integral = self.integrator.update(error, dt)
    derivative = self.differentiator.update(error, dt)
    return self.kp*error + self.ki*integral + self.kd*derivative

class Trajectory(object):
  def __init__(self):
    pass

  def xvel(self, t):
    return 0.1

  def yvel(self, t):
    return 0.0 #1.0 if t > 2.0 and t < 3.0 else 0.0

  def zvel(self, t):
    return 0.0

  def yawvel(self, t):
    return 0.0

class Controller(object):
  def __init__(self, trajectory):
    self.trajectory = trajectory
    self.time = 0.0

    self.pitchaccel_d = Differentiator()
    self.xvel_i = Integrator()
    self.yvel_i = Integrator()

    self.xfactor_pid = PID(0.000005, 0.0, 0.0)
    self.yfactor_pid = PID(20.0, 100.0, 0.0)

  def update(self, readings, dt, drone):
    """Takes sensor readings and returns PWM duty cycle applied to each rotor"""

    quaternion = drone.getQuaternion()
    accel = quaternion_rotate(quaternion, readings.accel)

    # xaccel_measured = accel[0]
    # xvel_measured = self.xvel_i.update(xaccel_measured, dt)
    # xvel_desired = self.trajectory.xvel(self.time)
    # xvel_error = xvel_desired - xvel_measured
    # xvel_error_d = self.xvel_error_d.update(xvel_error, dt)
    # xvel_error_dd = self.xvel_error_dd.update(xvel_error_d, dt)
    # xvel_error_ddd = self.xvel_error_ddd.update(xvel_error_dd, dt)
    # print xvel_error, xvel_error_d, xvel_error_dd, xvel_error_ddd
    # #xfactor = 0.0005*xvel_error - 0.0000001*xvel_error_ddd
    # w=100
    # if self.time < math.pi*2/w:
    #   xfactor = 0.015*(math.cos(w*self.time) + math.cos(2*w*self.time) + math.cos(3*w*self.time))
    # else:
    #   xfactor = 0.0
    # print xfactor

    pitchvel_measured = readings.gyro[2]
    pitchaccel_measured = self.pitchaccel_d.update(pitchvel_measured, dt)
    pitchvel_desired = 1.0
    xfactor = self.xfactor_pid.update(pitchvel_desired, pitchvel_measured, dt)

    yaccel_measured = accel[1]
    yvel_measured = self.yvel_i.update(yaccel_measured, dt)
    yvel_desired = self.trajectory.yvel(self.time)
    yfactor = 0.0#self.yfactor_pid.update(yvel_desired, yvel_measured, dt)

    zfactor = 0.0

    yawfactor = 0.0

    duty_cycles = map(clamp_duty_cycle, [
      zfactor - xfactor + yawfactor + yfactor,
      -zfactor - xfactor - yawfactor + yfactor,
      -zfactor + xfactor + yawfactor + yfactor,
      zfactor + xfactor - yawfactor + yfactor,
    ])

    self.time += dt

    return (xfactor, yfactor, zfactor, yawfactor), tuple(duty_cycles)

class Simulation(object):
  def __init__(self):
    self.world = ode.World()
    self.world.setGravity((0.0, -9.81, 0.0))

    self.drone = ode.Body(self.world)
    drone_mass = ode.Mass()
    drone_mass.setSphere(150, 0.01) # model as a sphere with a radius of 10cm
    drone_mass.adjust(0.700) # adjust mass to be exactly 700g
    self.drone.setMass(drone_mass)

    self.drone.setPosition((0.0, 0.0, 0.0))
    #self.drone.setQuaternion((0.8, 0.0, 0.0, -0.2))

    self.controller = Controller(Trajectory())

    self.vel_d = VectorDifferentiator()

  def update(self, dt):
    vel = self.drone.getLinearVel()
    avel = self.drone.getAngularVel()

    # Estimate the acceleration and angular acceleration.
    accel = self.vel_d.update(vel, dt)

    # Take sensor readings.
    magneto = self.drone.vectorFromWorld((1.0, 0.0, 0.0))
    readings = SensorReadings(self.drone.vectorFromWorld(accel), self.drone.vectorFromWorld(avel), magneto, dt)

    # Run control algorithm.
    output_factors, rotor_duty_cycles = self.controller.update(readings, dt, self.drone)

    # Apply rotor thrust to drone.
    for duty_cycle, displacement in zip(rotor_duty_cycles, rotor_displacements):
      # Thrust is 0.240*9.81 N at 50% throttle
      thrust = 4.7088*duty_cycle
      self.drone.addRelForceAtRelPos((0.0, thrust, 0.0), displacement)

    # Apply drag to drone.
    drag_factor = -0.1
    self.drone.addForce((vel[0]*drag_factor, vel[1]*drag_factor, vel[2]*drag_factor))

    self.world.step(dt)

    return output_factors, rotor_duty_cycles

def main():
  sim = Simulation()

  accel_d = VectorDifferentiator()
  aaccel = (0.0, 0.0, 0.0)

  imax = 50
  jmax = 10
  dt = 1e-3
  output_factors = (0.0, 0.0, 0.0, 0.0)
  outputs = (0.0, 0.0, 0.0, 0.0)
  for i in range(imax):
    pos = sim.drone.getPosition()
    vel = sim.drone.getLinearVel()
    quaternion = sim.drone.getQuaternion()
    forward = sim.drone.vectorToWorld((1.0, 0.0, 0.0))
    up = sim.drone.vectorToWorld((0.0, 1.0, 0.0))
    avel = sim.drone.getAngularVel()
    roll = quaternion_roll(quaternion)
    pitch = quaternion_pitch(quaternion)
    yaw = quaternion_yaw(quaternion)
    print "t=%7.4f x=(%7.3f,%7.3f,%7.3f) v=(%7.3f,%7.3f,%7.3f) roll=%7.3f pitch=%7.3f yaw=%7.3f av=(%7.3f,%7.3f,%7.3f) aa=(%7.3f,%7.3f,%7.3f) out=(%11.8f,%5.2f,%11.8f,%11.8f)=>(%5.3f,%5.3f,%5.3f,%5.3f)" % (dt*i*jmax, pos[0], pos[1], pos[2], vel[0], vel[1], vel[2], roll, pitch, yaw, avel[0], avel[1], avel[2], aaccel[0], aaccel[1], aaccel[2], output_factors[0], output_factors[1], output_factors[2], output_factors[3], outputs[0], outputs[1], outputs[2], outputs[3])

    for j in range(jmax):
      output_factors, outputs = sim.update(dt)
      up = sim.drone.vectorToWorld((0.0, 1.0, 0.0))
      # if up[1] < 0.8:
      #   print "Terminated due to instability."
      #   return
      avel = sim.drone.getAngularVel()
      aaccel = accel_d.update(avel, dt)

if __name__ == "__main__":
  main()
