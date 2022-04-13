# TO-DO: Change repository name to 'space-rocket-controller-dimensionalization'
# TO-DO: Containerize to microservice
import numpy as np

# Takes the aerodynamic coefficients of the flight body to 
# Requires: Aerodynamic coefficients
# Required by: Equations of motion and nonlinear motion
# Outputs: Aerodynamic forces and moments
class Dimensionalizer():
  def __init__(self, metadata_distributor):
    self.metadata_distributor = metadata_distributor
    self.q_inf = self.metadata_distributor.get_var('q_inf')
    self.flightbody_sa = self.metadata_distributor.get_var('flightbody_sa')
    self.flightbody_l = self.metadata_distributor.get_var('flightbody_l')
    self.flightbody_w = self.metadata_distributor.get_var('flightbody_w')
    self.alpha = self.metadata_distributor.get_var('alpha')
    self.c_LDY = self.metadata_distributor.get_var('c_LDY')
    self.c_lmn = self.metadata_distributor.get_var('c_lmn')
    self.xyz_cg = self.metadata_distributor.get_var('xyz_cg')
    self.xyz_a = self.metadata_distributor.get_var('xyz_a')

  def get_aerodynamic_force(self):
    c_L, c_D, c_Y = self.c_LDY
    lift = c_L*self.q_inf*self.flightbody_sa
    drag = c_D*self.q_inf*self.flightbody_sa
    side_force = c_Y*self.q_inf*self.flightbody_sa
    self.metadata_distributor.set({'lift': lift})
    self.metadata_distributor.set({'drag': drag})
    self.metadata_distributor.set({'side_force': side_force})
    return lift, drag, side_force
  
  def get_fb_aerodynamic_force(self):
    drag = self.metadata_distributor.get_var('drag')
    x_a = -np.sin(self.alpha)*self.flightbody_l - np.cos(self.alpha)*drag
    y_a = self.metadata_distributor.get_var('side_force')
    z_a = -np.cos(self.alpha)*self.flightbody_l + np.sin(self.alpha)*drag
    xyz_a = np.array([x_a, y_a, z_a])
    self.metadata_distributor.set({'xyz_a': xyz_a})
    return xyz_a

  def get_aerodynamic_moment(self):
    c_l, c_m, c_n = self.c_lmn
    x_cg, y_cg, z_cg = self.xyz_cg
    x_a, y_a, z_a = self.xyz_a
    l_a = c_l*self.q_inf*self.flightbody_s*self.flightbody_w - z_cg*y_a - y_cg*z_a
    m_a = c_m*self.q_inf*self.flightbody_s*self.flightbody_l + z_cg*x_a - x_cg*z_a
    n_a = c_n*self.q_inf*self.flightbody_s*self.flightbody_w + y_cg*x_a + x_cg*y_a
    lmn_a = np.array([l_a, m_a, n_a])
    self.metadata_distributor.set({'lmn_a': lmn_a})
    return lmn_a