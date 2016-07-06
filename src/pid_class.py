#############################################################################
# Classe PID controller
# Nome: Armando Alves Neto
# VeRLab - Laboratorio de visao computacional e robotica
# DCC
#############################################################################
import time

BETA  = 1.0
ALPHA = 0.1
GAMMA = 0.0


class PID:
	def __init__(self, kp, ti, td, umin, umax):
		# tempo corrente
		self.t = time.time().real
		
		# ganhos de controle
		self.Kp = kp
		self.Ti = ti
		self.Td = td
		
		# referencia
		self.rn = 0.0
		
		# erros
		self.ePn1  = 0.0
		self.eDfn1 = 0.0
		self.eDfn2 = 0.0
		
		# saida de controle
		self.umin = umin
		self.umax = umax
		self.un = 0.0
	
	#########################################################################
	# set reference
	#########################################################################
	def reference(self, r):
		self.rn = r
	
	#########################################################################
	# get output
	#########################################################################
	def u(self, yn):
		
		# tempo de amostragem
		Ts = time.time().real - self.t
		# trata casos de mudanca brusca no timer do sistema
		if Ts < 0.0:
			Ts = 0.0
		
		# Proportional error with reference weighing
		ePn = (BETA*self.rn) - yn
	
		# Calculates error
		en = self.rn - yn
		
		# Calculates filter time
		Tf = ALPHA * self.Td
		if (Tf != 0.0) and (Ts != -Tf):
			# Calculates derivate error
			eDn = (GAMMA*self.rn) - yn
			# Filters the derivate error
			eDfn = (self.eDfn1/((Ts/Tf) + 1)) + (eDn*(Ts/Tf)/((Ts/Tf) + 1))
		else:
			eDfn = 0.0
		
		# delta de atuacao Proporcional
		delta_un = self.Kp*(ePn - self.ePn1)

		# TESTING ONLY PROPORTIONAL, UNCOMMENT THE OTHERS FOR INTEGRATIVE AND DERIVATIVE
		self.un = delta_un
		if self.un > self.umax:
			self.un = self.umax
		if self.un < self.umin:
			self.un = self.umin
		return self.un
	
		# # termo integral
		# if (self.Ti != 0.0):
		# 	delta_un = delta_un + self.Kp*(Ts/self.Ti)*en
		
		# # termo derivativo
		# if (Ts != 0.0):
		# 	delta_un = delta_un + self.Kp*(self.Td/Ts)*(eDfn - 2*self.eDfn1 + self.eDfn2)
	
		# # incrementa saida
		# self.un = self.un + delta_un
	
		# # integrator anti-windup logic
		# if self.un > self.umax:
		# 	self.un = self.umax
		# if self.un < self.umin:
		# 	self.un = self.umin
		
		# # update indexed values
		# self.ePn1 = ePn
		# self.eDfn2 = self.eDfn1
		# self.eDfn1 = eDfn
		
		# # novo tempo corrente
		# self.t = self.t + Ts
	
		# # retorna a nova saida			
		# return self.un

if __name__ == "__main__":
	
	# Test linear
	KP_VEL = 30.0 
	TI_VEL = 0#80.0
	TD_VEL = 0#0*0.00005

	pid = PID(KP_VEL, TI_VEL, TD_VEL, 0, 100)
	for i in [x * 0.1 for x in range(300, -1, -1)]:
		pid.reference(0.0)
		print 'dist:', i, 'vel:', pid.u(i)
		time.sleep(0.05)

	# Test angular
	# KP_PSI = 0.7
	# TI_PSI = 70.0
	# TD_PSI = 0*0.00007
	# pid = PID(KP_PSI, TI_PSI, TD_PSI, -95, 95)
	# pid.reference(0.0)
	# print pid.u(360)