import  cmd_list
import  driver
import  packets
import time
import ParticleFilter as pf
from hokuyolx import HokuyoLX
import matplotlib.pyplot as plt
import logging
import numpy as np



lvl = logging.INFO
logging.basicConfig(filename='Eurobot.log', filemode='w',format='%(levelname)s:%(asctime)s %(message)s', datefmt='%m/%d/%Y %I:%M:%S %p',level=lvl)

console = logging.StreamHandler()
console.setLevel(lvl)
# set a format which is simpler for console use
formatter = logging.Formatter('%(levelname)s: %(message)s')
console.setFormatter(formatter)
# add the handler to the root logger
logging.getLogger('').addHandler(console)
logger = logging.getLogger(__name__)

class Robotishe():
    def __init__(self,lidar_on=True):
        if lidar_on:
            logging.debug('lidar is connected')
            self.lidar = HokuyoLX(tsync=False)
            self.lidar.convert_time = False
        self.lidar_on = lidar_on
        self.x = 170  # mm
        self.y = 1830  # mm
        self.angle = 0.0  # pi
        self.particles = [pf.Robot(self.x,self.y,self.angle) for i in range(pf.particle_number)]
        self.dr = driver.Driver(1, 2, 3)
        self.dr.connect()
        command = {'source': 'fsm', 'cmd': 'echo', 'params': 'ECHO'}
        logging.info(self.dr.process_cmd(command))
        command = {'source': 'fsm', 'cmd': 'setCoordinates', 'params': [self.x/1000., self.y/1000., self.angle]}
        logging.info(self.dr.process_cmd(command))


    def lidar_sense(self):
        """Function returns landmark data"""
        timestamp, scan = self.lidar.get_intens()
        #scan =  np.load('scan.npy')
        # distances to landmarks(up to 3)!
        ans = []
        ideals = pf.ideal(self.x,self.y)
        #return ideals
        #print ideals
        beacons = pf.get_beacons(scan)
        print beacons
        return beacons
        for i in beacons:
            di = i[1] + 40
            for j in ideals:  # small corectness checker
                if abs(di-j) < 200:
                    ans.append(di)
                    break
        return ans

    def beacon_check(self):
        timestamp, scan = self.lidar.get_intens()
        print "2"
        print pf.get_beacons_2(scan)
        #print "1"
        #print pf.get_beacons(scan)



    def get_raw_lidar(self):
        #return np.load('scan.npy')
        #return np.load('scan.npy')[::-1]
        timestamp, scan = self.lidar.get_intens()
        #np.save('scan',scan)
        #return scan
        return scan[::-1]


    def stm_test(self):
        command = {'source': 'fsm', 'cmd': 'echo', 'params': 'ECHO'}
        print self.dr.process_cmd(command)
        #command = {'source': 'fsm', 'cmd': 'switchOnPneumo','params': ''}
        #print d.process_cmd(command)
        command = {'source': 'fsm', 'cmd': 'setCoordinates', 'params': [0.0, 0.0, 0.0]}
        print self.dr.process_cmd(command)
        command = {'source': 'fsm', 'cmd': 'addPointToStack', 'params': [0.8, 0.0, 0.0, 4]}
        print self.dr.process_cmd(command)
        # time.sleep(5)
        # command = {'source': 'fsm', 'cmd': 'switchOffPneumo','params': ''}
        # print d.process_cmd(command)

    def stm_local(self):
        particles = [pf.Robot() for i in range(pf.particle_number)]
        main_robot = pf.calculate_main(particles)
        command = {'source': 'fsm', 'cmd': 'setCoordinates', 'params': [0.0, 0.0, 0.0]}
        print self.dr.process_cmd(command)
        ax = plt.subplot(111)
        plot = ax.plot([], [], 'ro')[0]
        plt.axis([0, 2000, 0, 3000])
        plt.show()
        cord = [0, 0]
        plot.set_data(cord[0], cord[1])
        # make movement
        parameters = [0.15, 0, 0, 4]
        self.make_mov(parameters)
        cord = [main_robot.x, main_robot.y]
        plot.set_data(cord[0], cord[1])
        print main_robot

    def make_move(self,parameters):
        # in param MM
        pm = [(self.x+parameters[0])/1000.,(self.y+parameters[1])/1000.,parameters[2],parameters[3]]
        command = {'source': 'fsm', 'cmd': 'addPointToStack', 'params': pm}
        print self.dr.process_cmd(command)
        self.particles = pf.particles_move(self.particles,parameters )#[i/2. for i in parameters]
        # After movement
        # stamp = time.time()
        # while not stm_driver('is_point_was_reached'):
        #     time.sleep(0.3)
        #     if (time.time() - stamp) > 30:
        #         return False
        time.sleep(3)
        lidar_data = self.lidar_sense()
        print lidar_data
        for i in range(30):
            self.particles = pf.particles_sense(self.particles, lidar_data)
            self.particles = pf.particles_sense(self.particles, lidar_data)
            self.particles = pf.particles_sense(self.particles, lidar_data)
            main_robot = pf.calculate_main(self.particles)
            print main_robot
        main_robot = pf.calculate_main(self.particles)
        self.x = main_robot.x
        self.y = main_robot.y
        print main_robot

    def go_to_coord(self):
        #command = {'source': 'fsm', 'cmd': 'addPointToStack', 'params': pm}
        #print self.dr.process_cmd(command)
        return

    def go_to_coord_rotation(self,parameters): #  parameters [x,y,angle,speed]
        pm = [parameters[0] / 1000., parameters[1] / 1000., parameters[2], parameters[3]]
        command = {'source': 'fsm', 'cmd': 'addPointToStack', 'params': pm}
        logging.info(self.dr.process_cmd(command))
        # After movement
        stamp = time.time()
        cmd = {'source': 'fsm', 'cmd': 'is_point_was_reached'}
        while not self.dr.process_cmd(cmd)['data']:
            time.sleep(0.3)
            if (time.time() - stamp) > 30:
                return False # Error
        logging.info('point reached')
        self.particles = pf.particles_move(self.particles, [parameters[0]-self.x,parameters[1]-self.y,parameters[2]-self.angle])
        self.x = parameters[0]
        self.y = parameters[1]
        self.angle = parameters[2]
        print 'Before Calculation:'
        pf.calculate_main(self.particles)
        logging.info('3 second debug sleep')
        time.sleep(0.5)
        if self.lidar_on:
            #lidar_data = self.lidar_sense()
            lidar_data = self.get_raw_lidar()
            #print lidar_data
            self.particles = pf.particle_sense2(self.particles,lidar_data)
            #self.particles = pf.particles_sense(self.particles, lidar_data)
            print 'After Calculation:'
            main_robot = pf.calculate_main(self.particles)
            #for i in range(30):
            #   self.particles = pf.particles_sense(self.particles, lidar_data)
            #   self.particles = pf.particles_sense(self.particles, lidar_data)
            #   self.particles = pf.particles_sense(self.particles, lidar_data)
            #    main_robot = pf.calculate_main(self.particles)
            #    print main_robot
            self.x = main_robot.x
            self.y = main_robot.y
        command = {'source': 'fsm', 'cmd': 'setCoordinates', 'params': [self.x / 1000., self.y / 1000., self.angle]}
        logging.info(self.dr.process_cmd(command))
        # TODO add move correction


    def go_to_virtual(self,parameters):
        pm = [parameters[0] / 1000., parameters[1] / 1000., parameters[2], parameters[3]]
        # After movement
        logging.info('point reached')
        self.particles = pf.particles_move(self.particles,
                                           [parameters[0] - self.x, parameters[1] - self.y, parameters[2] - self.angle])
        self.x = parameters[0]
        self.y = parameters[1]
        self.angle = parameters[2]
        print 'Before Calculation'
        main_robot = pf.calculate_main(self.particles)
        logging.info('3 second debug sleep')
        if self.lidar_on:
            #lidar_data = self.lidar_sense()
            lidar_data = self.get_raw_lidar()
            # print lidar_data
            self.particles = pf.particle_sense2(self.particles, lidar_data)
            self.particles = pf.particle_sense2(self.particles, lidar_data)
            self.particles = pf.particle_sense2(self.particles, lidar_data)
            self.particles = pf.particle_sense2(self.particles, lidar_data)
            #self.particles = pf.particle_sense2(self.particles, lidar_data)
            #self.particles = pf.particle_sense2(self.particles, lidar_data)
            #self.particles = pf.particle_sense2(self.particles, lidar_data)
            #self.particles = pf.particle_sense2(self.particles, lidar_data)
            # self.particles = pf.particles_sense(self.particles, lidar_data)
            #for i in range(10):
               #self.particles = pf.particles_sense2(self.particles, lidar_data)
               #pf.calculate_main(self.particles)
            #   self.particles = pf.particles_sense(self.particles, lidar_data)
            #   self.particles = pf.particles_sense(self.particles, lidar_data)
            #    main_robot = pf.calculate_main(self.particles)
            #    print main_robot
            self.x = main_robot.x
            self.y = main_robot.y
        #command = {'source': 'fsm', 'cmd': 'setCoordinates', 'params': [self.x / 1000., self.y / 1000., self.angle]}
        #logging.info(self.dr.process_cmd(command))
        # TODO add move correction


    def demo(self):
        # goal 170 620
        # move out the cargo, go to cylinders, take cylinder, go with cylinder to base
        #1830
        #parameters = [250, 1780, 0.0, 4]#250,1780
        #self.go_to_virtual(parameters)

        parameters = [500, 1830, 0.0, 4]
        self.go_to_coord_rotation(parameters)
        parameters = [870, 1830, 0.0, 4]
        self.go_to_coord_rotation(parameters)
        parameters = [1000, 1000, 0.0, 4]
        self.go_to_coord_rotation(parameters)
        parameters = [500, 1000, 0.0, 4]
        self.go_to_coord_rotation(parameters)
        parameters = [500, 650, 0.0, 4]
        self.go_to_coord_rotation(parameters)
        parameters = [250, 650, 0.0, 4]
        self.go_to_coord_rotation(parameters)
        parameters = [250, 650, 3.14, 4]
        self.go_to_coord_rotation(parameters)
        #return




def test():
    rb = Robotishe(True)
    rb.make_move([100.0, 0.0, 0.0, 4])

#test()

#

#rb = Robotishe(True)
#print rb.lidar_sense()

def func_test():
    dr = driver.Driver(1, 2, 3)
    dr.connect()
    command = {'source': 'fsm', 'cmd': 'is_point_was_reached'}
    print (dr.process_cmd(command))

def lidar_test():
    rb = Robotishe(True)
    while True:
        time.sleep(3)
        print rb.lidar_sense()



#func_test()
def t():
    rb = Robotishe(True)
    rb.demo()

t()

def Marko_test():
    rb = Robotishe(True)
    t = rb.get_raw_lidar()
    angle, distance = pf.get_beacons_Marko(t)
    print (angle)
    print (distance)
    x_coords, y_coords = pf.p_trans(angle, distance)
    plt.plot(x_coords, y_coords, 'ro', color='r')
    plt.show()
    print (x_coords)
    print (y_coords)

def myTest():
    rb = Robotishe(True)
    scan = rb.get_raw_lidar()
    pf.get_beacons_2(scan)
    print pf.get_beacons_2(scan)
    print pf.get_beacons(scan)

#myTest()


def beacont():
    rb = Robotishe(True)
    while(True):
        rb.beacon_check()
        time.sleep(3)
#beacont()
#[(-524.7488169045432, -845.4629402431659), (2527.213663918089, 274.14637360294694), (2581.2125160857854, -1805.1525765157153)]
#Marko_test()
#rb = Robotishe(True)
#print(rb.lidar_sense())




