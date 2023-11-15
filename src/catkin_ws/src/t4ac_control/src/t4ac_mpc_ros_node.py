#!/usr/bin/env python3.8
# Importamos lo que necesitamos
import rospy
import MPClass
import numpy as np
import csv
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from scipy import sparse

class MPCNode():
    def __init__(self):
        
        #__________________________
        # CSV PARA GRÁFICAS
        #__________________________
        self.de_graph = open('/home/docker_robesafe/shared_home/de_graph.csv', 'w', newline='')
        self.de_writer = csv.writer(self.de_graph, delimiter=',')
        #__________________________
        # VARIABLES
        #__________________________
        self.de = 0
        self.de_prev = 0
        self.v_de = 0
        self.W = 4.5
        self.min_error = 1.1
        self.lane_change_init = False
        self.Ts = 0.1

        #__________________________
        # INICIALIZACIÓN DE LOS MPC's
        #__________________________

        # Horizonte de predicción 
        self.N_lat   = 100

        # Definimos el MODELO LATERAL, incluyendo restricciones y condiciones iniciales.
        self.A_lat = sparse.csc_matrix([[1, self.Ts],
                                [0, 1]])

        self.B_lat = sparse.csc_matrix([[(self.Ts**2/2)],
                                [self.Ts]])

        self.u0_lat = 0
        self.umin_lat = np.array([-10])
        self.umax_lat = np.array([10])

        self.x0_lat = np.array([0.0, 0.0]) # OJO -> de_sub
        self.xmin_lat = np.array([-200.8, -200.2])
        self.xmax_lat = np.array([200.8, 200.2])

        self.xr_lat = np.array([0.0, 0.0])

        self.Q_lat = sparse.diags([1000000, 1000000])
        self.R_lat = 5*sparse.eye(1)

        self.mpc_lateral = MPClass.MPC( A=self.A_lat,B=self.B_lat,u0=self.u0_lat,umin=self.umin_lat,umax=self.umax_lat,x0=self.x0_lat,
                                        xmin=self.xmin_lat,xmax=self.xmax_lat,Q=self.Q_lat,R=self.R_lat,N=self.N_lat,xr=self.xr_lat)

        #__________________________
        # SUBSCRIBERS Y PUBLISHERS
        #__________________________

        rospy.init_node('t4ac_mpc_ros_node', anonymous=True)
        self.de_sub = rospy.Subscriber("/t4ac/control/de", Float64, self.mpc_lat_callback)
        self.lane_change_sub = rospy.Subscriber('/t4ac/decision_making/lane_change', Bool, self.lane_change_callback)
        self.y_lat_pub = rospy.Publisher('/t4ac/control/mpc_de', Float64, queue_size=1)
        self.lane_change_done = rospy.Publisher('/t4ac/control/lane_change_done', Bool, queue_size=1)

        # Tiempo de muestreo
        self.fs = rospy.Rate(10)

    def mpc_lat_callback(self, data):

        self.de_prev = self.de
        self.de = data.data
        self.v_de = (self.de-self.de_prev)/self.Ts
        self.de_writer.writerow([self.de, self.v_de, rospy.Time.now()])

    def lane_change_callback(self, data):

        lane_change = data.data

        if lane_change:
            self.mpc_lateral.set_x_ref(self.mpc_lateral.xr + np.array([self.W, 0.0]))
        else:
            self.mpc_lateral.set_x_ref(self.mpc_lateral.xr - np.array([self.W, 0.0]))

        self.lane_change_init = True

    def check_lane_change(self):
        
        lane_change_done_msg = Bool((abs(abs(self.de) - abs(self.mpc_lateral.xr[0])) < self.min_error))
        
        if lane_change_done_msg.data and self.lane_change_init: 
            self.lane_change_done.publish(lane_change_done_msg)
            self.lane_change_init = False

if __name__=="__main__": 

    #__________________________
    # BUCLE
    #__________________________

    y_lat = Float64()
    y_aux = np.array([0.0, 0.0])
    mpc_node = MPCNode()

    while not rospy.is_shutdown():
        
        # Control lateral
        mpc_node.u0_lat = mpc_node.mpc_lateral.mpc_move(y_aux) 
        y_aux = mpc_node.A_lat.dot(mpc_node.mpc_lateral.x0) + mpc_node.B_lat.dot(mpc_node.u0_lat)
        y_lat.data = y_aux.item(0)

        mpc_node.y_lat_pub.publish(y_lat)
        mpc_node.check_lane_change()

        # Tiempo de muestreo
        mpc_node.fs.sleep()

