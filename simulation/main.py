import math
import ode

rotor_displacements = [
  (0.5, 0.0, -0.5),
  (0.5, 0.0, 0.5),
  (-0.5, 0.0, 0.5),
  (-0.5, 0.0, -0.5),
]

rotor_directions = [1, -1, 1, -1]

def clamp_duty_cycle(x):
  if x < 0.0:
    return 0.0
  if x > 1.0:
    return 1.0
  return x

def normalise(v):
  mag = math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
  return (v[0] / mag, v[1] / mag, v[2] / mag)

def apply_pitch(v, pitch):
  c, s = math.cos(pitch), math.sin(pitch)
  return (
    c*v[0] + s*v[1],
    -s*v[0] + c*v[1],
    v[2],
  )

def apply_roll(v, roll):
  c, s = math.cos(roll), math.sin(roll)
  return (
    v[0],
    c*v[1] - s*v[2],
    s*v[1] + c*v[2],
  )

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
  def __init__(self, accel, gyro, magneto):
    self.accel = accel
    self.gyro = gyro
    self.magneto = magneto

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

class Trajectory(object):
  def __init__(self):
    pass

  def pitch(self, t):
    if t < 5.0:
      return 0.02*t/5.0
    if t < 25.0:
      return 0.02
    if t < 30.0:
      return 0.02*(30.0 - t)/5.0
    return 0.0

  def yvel(self, t):
    return t/20.0

  def roll(self, t):
    if t < 5.0:
      return 0.02*t/5.0
    if t < 25.0:
      return 0.02
    if t < 30.0:
      return 0.02*(30.0 - t)/5.0
    return 0.0

  def yawvel(self, t):
    return 0.0

class Controller(object):
  def __init__(self, trajectory):
    self.trajectory = trajectory
    self.time = 0.0

    self.magneto_offset = None

    self.roll = 0.0
    self.pitch = 0.0
    self.yvel = 0.0

    self.yvel_error_i = Integrator()
    self.pitch_error_i = Integrator()
    self.pitch_error_d = Differentiator()
    self.pitchfactor_d = Differentiator()
    self.roll_error_i = Integrator()
    self.roll_error_d = Differentiator()
    self.rollfactor_d = Differentiator()
    self.yawvel_error_i = Integrator()
    self.yawvel_error_d = Differentiator()

  def update(self, readings, dt):
    """Takes sensor readings and returns PWM duty cycle applied to each rotor"""

    # Normalise the magnetometer vector and record its offset if necessary.
    magneto = normalise(readings.magneto)
    if self.magneto_offset is None:
      self.magneto_offset = magneto

    # Rotate acceleration vector from body frame to world frame.
    accel_world = apply_pitch(apply_roll(readings.accel, self.roll), self.pitch)

    # Integrate y acceleration to get y velocity.
    self.yvel += accel_world[1] * dt

    # Measure roll and pitch using complementary filter.
    if abs(accel_world[1]) < 1e-4:
      accel_weight_roll = accel_weight_pitch = 0.0
    else:
      accel_weight_roll = accel_weight_pitch = 0.0
    roll_from_accel = math.atan2(readings.accel[1], readings.accel[2])
    pitch_from_accel = math.atan2(readings.accel[1], readings.accel[0])
    roll_from_gyro = self.roll + readings.gyro[0] * dt
    pitch_from_gyro = self.pitch - readings.gyro[2] * dt
    self.roll = roll_from_gyro*(1-accel_weight_roll) + roll_from_accel*accel_weight_roll
    self.pitch = pitch_from_gyro*(1-accel_weight_pitch) + pitch_from_accel*accel_weight_pitch

    yvel_desired = self.trajectory.yvel(self.time)
    yvel_error = yvel_desired - self.yvel
    yvel_error_i = self.yvel_error_i.update(yvel_error, dt)
    yfactor = 6.772 * yvel_error + 100 * yvel_error_i

    pitch_desired = self.trajectory.pitch(self.time)
    pitch_error = pitch_desired - self.pitch
    pitch_error_i = self.pitch_error_i.update(pitch_error, dt)
    pitch_error_d = self.pitch_error_d.update(pitch_error, dt)
    pitchfactor_out = 4.2637e-5 * pitch_error + 2.493e-5 * pitch_error_i
    pitchfactor = self.pitchfactor_d.update(pitchfactor_out, dt)

    roll_desired = self.trajectory.roll(self.time)
    roll_error = roll_desired - self.roll
    roll_error_i = self.roll_error_i.update(roll_error, dt)
    roll_error_d = self.roll_error_d.update(roll_error, dt)
    rollfactor_out = 4.2637e-5 * roll_error + 2.493e-5 * roll_error_i
    rollfactor = self.rollfactor_d.update(rollfactor_out, dt)

    yawvel_desired = self.trajectory.yawvel(self.time)
    yawvel_error = yawvel_desired - readings.gyro[1]
    yawvel_error_i = self.yawvel_error_i.update(yawvel_error, dt)
    yawvel_error_d = self.yawvel_error_d.update(yawvel_error, dt)
    yawfactor = 0.08738 * yawvel_error + 0.16883 * yawvel_error_i + 0.0011317 * yawvel_error_d

    duty_cycles = map(clamp_duty_cycle, [
      yfactor - pitchfactor + rollfactor + yawfactor,
      yfactor - pitchfactor - rollfactor - yawfactor,
      yfactor + pitchfactor - rollfactor + yawfactor,
      yfactor + pitchfactor + rollfactor - yawfactor,
    ])

    self.time += dt

    return (yfactor, pitchfactor, rollfactor, yawfactor), tuple(duty_cycles)

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
    #self.drone.setQuaternion((0.8, 0.0, 0.0, -0.045))

    self.controller = Controller(Trajectory())

    self.vel_d = VectorDifferentiator()

  def update(self, dt):
    vel = self.drone.getLinearVel()
    avel = self.drone.getAngularVel()

    # Estimate the acceleration and angular acceleration.
    accel = self.vel_d.update(vel, dt)

    # Take sensor readings.
    magneto = self.drone.vectorFromWorld((1.0, -0.5, 0.0))
    readings = SensorReadings(self.drone.vectorFromWorld(accel), self.drone.vectorFromWorld(avel), magneto)

    # Run control algorithm.
    output_factors, rotor_duty_cycles = self.controller.update(readings, dt)

    # Apply rotor thrust to drone.
    for duty_cycle, displacement, direction in zip(rotor_duty_cycles, rotor_displacements, rotor_directions):
      # Thrust is 0.240*9.81 N at 50% throttle
      thrust = 4.7088*duty_cycle
      torque = 0.001*duty_cycle
      self.drone.addRelForceAtRelPos((0.0, thrust, 0.0), displacement)
      self.drone.addRelTorque((0.0, direction*torque, 0.0))

    # Apply drag to drone.
    linear_drag_factor = -1e-1
    self.drone.addForce((vel[0]*linear_drag_factor, vel[1]*linear_drag_factor, vel[2]*linear_drag_factor))
    angular_drag_factor = -1e-5
    self.drone.addTorque((avel[0]*angular_drag_factor, avel[1]*angular_drag_factor, avel[2]*angular_drag_factor))

    self.world.step(dt)

    return output_factors, rotor_duty_cycles

def main():
  sim = Simulation()

  accel_d = VectorDifferentiator()
  aaccel = (0.0, 0.0, 0.0)

  imax = 40
  jmax = 1000
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
    print "t=%7.4f out=(%5.3f,%11.8f,%11.8f,%11.8f)=>(%5.3f,%5.3f,%5.3f,%5.3f) x=(%7.3f,%7.3f,%7.3f) v=(%7.3f,%7.3f,%7.3f) roll=%7.3f pitch=%7.3f yaw=%7.3f av=(%7.3f,%7.3f,%7.3f) aa=(%7.3f,%7.3f,%7.3f)" % (dt*i*jmax, output_factors[0], output_factors[1], output_factors[2], output_factors[3], outputs[0], outputs[1], outputs[2], outputs[3], pos[0], pos[1], pos[2], vel[0], vel[1], vel[2], roll, pitch, yaw, avel[0], avel[1], avel[2], aaccel[0], aaccel[1], aaccel[2])

    for j in range(jmax):
      output_factors, outputs = sim.update(dt)
      up = sim.drone.vectorToWorld((0.0, 1.0, 0.0))
      # if up[1] < 0.8:
      #   print "Terminated due to instability."
      #  return
      avel = sim.drone.getAngularVel()
      aaccel = accel_d.update(avel, dt)

if __name__ == "__main__":
  main()
